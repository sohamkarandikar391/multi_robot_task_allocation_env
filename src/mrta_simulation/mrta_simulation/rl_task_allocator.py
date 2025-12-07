import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import json
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import torch
import sys
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage 
import subprocess
from rclpy.qos import qos_profile_sensor_data 

# paper_code_path = os.path.join(os.path.dirname(__file__), '..', '..', 'HeteroMRTA')
paper_code_path = "/home/sohamk3901/mrta_ws/src/mrta_simulation/HeteroMRTA"
sys.path.insert(0, paper_code_path)

try:
    from attention import AttentionNet
    from worker import Worker
    from env.task_env import TaskEnv
    from parameters import EnvParams, TrainParams
    PAPER_CODE_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Could not import paper code: {e}")
    PAPER_CODE_AVAILABLE = False

class RLTaskAllocator(Node):
    def __init__(self):
        super().__init__('rl_task_allocator')
        pkg_dir = get_package_share_directory('mrta_simulation')
        config_path = os.path.join(pkg_dir, 'config', 'scenario_simple.yaml')

        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        self.parse_scenario()
        self.use_rl = self.load_policy()
        self.task_assignment_pub = self.create_publisher(
            String,
            '/task_assignments',
            10
        )

        # self.color_client = self.create_client(
        #     SetVisualProperties,
        #     '/gazebo/set_visual_properties'
        # )

        self.robot_positions = {}
        # self.create_subscription(
        #     PoseStamped,
        #     '/robot_positions',
        #     self.robot_position_callback,
        #     10
        # )
        self.setup_robot_subscribers()
        # In __init__
        self.trigger_pub = self.create_publisher(String, '/task_triggers', 10)
        self.completed_tasks = set()
        self.current_assignments = {}
        self.task_start_times = {}
        self.robot_routes = {}
        self.robot_at_task = {}
        self.robot_positions = {}
        self.waiting_at_task = {}
        self.ongoing_tasks = {}
        self.create_paper_env()
        self.create_timer(5.0, self.run_initial_allocation)
        self.create_timer(0.5, self.check_task_completion)
        self.get_logger().info('RL Task Allocator Initialized')
        self.initial_allocation_done = False

    def parse_scenario(self):
        self.robots = []
        self.robot_skills = {}
        self.robot_species = {}
        self.species_info = {}

        for species_name, species_data in self.config['species'].items():
            count = species_data['count']
            skills = species_data['skills']
            depot = species_data['depot_position'][:2]

            self.species_info[species_name] = {
                'skills': skills,
                'depot': depot,
                'count': count
            }

            for i in range(count):
                robot_id = f"{species_name}_{i}"
                self.robots.append(robot_id)
                self.robot_skills[robot_id] = skills
                self.robot_species[robot_id] = species_name

        self.tasks = {}
        for task in self.config['tasks']:
            task_id = task['id']
            self.tasks[task_id] = {
                'position': task['position'][:2],
                'requirements': task['requirements'],
                'duration': task['duration']
            }
        self.get_logger().info(
            f'Parsed: {len(self.robots)} robots, {len(self.tasks)} tasks, '
            f'{len(self.species_info)} species' 
        )

    def create_paper_env(self):
        if not PAPER_CODE_AVAILABLE:
            self.paper_env = None
            return
        
        try:
            EnvParams.TASKS_RANGE = (len(self.tasks), len(self.tasks))
            EnvParams.SPECIES_RANGE = (len(self.species_info), len(self.species_info))
            EnvParams.TRAIT_DIM = 5
            EnvParams.MAX_TIME = 200

            self.paper_env = TaskEnv()
            self.inject_scenario_into_env()
            self.get_logger().info('Created Paper TaskEnv')

        except Exception as e:
            self.get_logger().error(f'Failed to create paper env: {e}')
            self.paper_env = None

    def inject_scenario_into_env(self):
        """Inject our scenario into paper's TaskEnv"""
        if self.paper_env is None:
            return

        # Normalize positions
        all_positions = []
        for species_data in self.species_info.values():
            all_positions.append(species_data['depot'])
        for task_data in self.tasks.values():
            all_positions.append(task_data['position'])

        all_positions = np.array(all_positions)
        self.world_min = all_positions.min(axis=0)
        self.world_max = all_positions.max(axis=0)
        self.world_range = self.world_max - self.world_min

        def normalize(pos):
            return (np.array(pos) - self.world_min) / (self.world_range + 1e-6)

        # ===== CRITICAL: Rebuild the ENTIRE environment manually =====
        
        # 1. Build species list
        self.paper_env.num_species = len(self.species_info)
        self.paper_env.species = []
        species_list = list(self.species_info.items())
        
        for species_idx, (species_name, species_data) in enumerate(species_list):
            species_dict = {
                'num_agents': species_data['count'],
                'depot': normalize(species_data['depot']).tolist(),
                'trait': species_data['skills']
            }
            self.paper_env.species.append(species_dict)

        # 2. Build tasks list
        self.paper_env.num_tasks = len(self.tasks)
        self.paper_env.tasks = []
        
        for task_id, task_data in self.tasks.items():
            task_dict = {
                'requirement': task_data['requirements'],
                'pos': normalize(task_data['position']).tolist(),
                'duration': task_data['duration']
            }
            self.paper_env.tasks.append(task_dict)

        # 3. CRITICAL: Manually rebuild task_dic
        self.paper_env.task_dic = {}
        for i, task_data in enumerate(self.tasks.values()):
            norm_pos = normalize(task_data['position'])
            self.paper_env.task_dic[i] = {
                'ID': i,
                'requirements': np.array(task_data['requirements']),
                'members': [],
                'cost': [],
                'location': norm_pos,
                'feasible_assignment': False,
                'finished': False,
                'time_start': 0,
                'time_finish': 0,
                'status': np.array(task_data['requirements']),
                'time': float(task_data['duration']),
                'sum_waiting_time': 0,
                'efficiency': 0,
                'abandoned_agent': [],
                'optimized_ability': None,
                'optimized_species': []
            }

        # 4. CRITICAL: Manually rebuild agent_dic and depot_dic
        self.paper_env.agent_dic = {}
        self.paper_env.depot_dic = {}
        self.paper_env.species_dict = {
            'abilities': [],
            'number': []
        }
        
        agent_id = 0
        for species_idx, (species_name, species_data) in enumerate(species_list):
            # Add to species_dict
            self.paper_env.species_dict['abilities'].append(species_data['skills'])
            self.paper_env.species_dict['number'].append(species_data['count'])
            self.paper_env.species_dict[species_idx] = []
            
            # Create depot
            norm_depot = normalize(species_data['depot'])
            self.paper_env.depot_dic[species_idx] = {
                'location': norm_depot,
                'members': [],
                'ID': -species_idx - 1
            }
            
            # Create agents for this species
            for j in range(species_data['count']):
                self.paper_env.agent_dic[agent_id] = {
                    'ID': agent_id,
                    'species': species_idx,
                    'abilities': np.array(species_data['skills']),
                    'location': norm_depot.copy(),
                    'route': [-species_idx - 1],
                    'current_task': -species_idx - 1,
                    'contributed': False,
                    'arrival_time': [0.],
                    'cost': [0.1],  # Default cost
                    'travel_time': 0,
                    'velocity': 0.2,
                    'next_decision': 0,
                    'depot': norm_depot.copy(),
                    'travel_dist': 0,
                    'sum_waiting_time': 0,
                    'current_action_index': 0,
                    'decision_step': 0,
                    'task_waiting_ratio': 1,
                    'trajectory': [],
                    'angle': 0,
                    'returned': False,
                    'assigned': False,
                    'pre_set_route': None,
                    'no_choice': False
                }
                
                self.paper_env.species_dict[species_idx].append(agent_id)
                self.paper_env.depot_dic[species_idx]['members'].append(agent_id)
                agent_id += 1

        # Convert species_dict lists to numpy arrays
        self.paper_env.species_dict['abilities'] = np.array(self.paper_env.species_dict['abilities'])
        
        # 5. Update counts
        self.paper_env.tasks_num = len(self.paper_env.task_dic)
        self.paper_env.agents_num = len(self.paper_env.agent_dic)
        self.paper_env.species_num = len(self.paper_env.species_dict['number'])
        
        # 6. Regenerate distance matrices with new structure
        self.paper_env.species_distance_matrix, self.paper_env.species_neighbor_matrix = \
            self.paper_env.generate_distance_matrix()
        
        # 7. Initialize state
        self.paper_env.init_state()
        
        self.get_logger().info(
            f'✓ Injected scenario: {self.paper_env.agents_num} agents, '
            f'{self.paper_env.tasks_num} tasks, {self.paper_env.species_num} species'
        )


    # def inject_scenario_into_env(self):
    #     if self.paper_env is None:
    #         return

    #     all_positions = []
    #     for species_data in self.species_info.values():
    #         all_positions.append(species_data['depot'])
    #     for task_data in self.tasks.values():
    #         all_positions.append(task_data['position'])

    #     all_positions = np.array(all_positions)
    #     self.world_min = all_positions.min(axis=0)
    #     self.world_max = all_positions.max(axis=0)
    #     self.world_range = self.world_max - self.world_min

    #     def normalize(pos):
    #         return (np.array(pos) - self.world_min) / (self.world_range + 1e-6)

    #     self.paper_env.num_species = len(self.species_info)
    #     self.paper_env.species = []

    #     for species_name, species_data in self.species_info.items():
    #         species_dict = {
    #             'num_agents': species_data['count'],
    #             'depot': normalize(species_data['depot']).tolist(),
    #             'trait': species_data['skills']
    #         }
    #         self.paper_env.species.append(species_dict)

    #     self.paper_env.num_tasks = len(self.tasks)
    #     self.paper_env.tasks = []

    #     for task_id, task_data in self.tasks.items():
    #         task_dict = {
    #             'requirement': task_data['requirements'],
    #             'pos': normalize(task_data['position']).tolist(),
    #             'duration': task_data['duration']
    #         }
    #         self.paper_env.tasks.append(task_dict)

    #     self.paper_env.init_state()
    #     self.get_logger().info('Injected scenario into paper env')

    def change_task_color(self, task_id, completed=False):
        """Change task marker color: yellow->green when completed"""
        if not self.color_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Color service not available')
            return
        
        request = SetVisualProperties.Request()
        request.visual_name = f'task_{task_id}::link::visual'  # Adjust to match your model name
        
        if completed:
            # Green for completed
            request.ambient = Color(r=0.0, g=1.0, b=0.0, a=1.0)
            request.diffuse = Color(r=0.0, g=1.0, b=0.0, a=1.0)
        else:
            # Yellow for pending
            request.ambient = Color(r=1.0, g=1.0, b=0.0, a=1.0)
            request.diffuse = Color(r=1.0, g=1.0, b=0.0, a=1.0)
        
        future = self.color_client.call_async(request)

    def load_policy(self):
        if not PAPER_CODE_AVAILABLE:
            self.get_logger().warn('Paper code not available')
            return False

        try:
            # --- ALIGNMENT WITH TEST.PY ---
            # Set these EXACTLY as they appear in test.py
            EnvParams.TASKS_RANGE = (len(self.tasks), len(self.tasks))
            EnvParams.SPECIES_RANGE = (len(self.species_info), len(self.species_info))
            # EnvParams.SPECIES_AGENTS_RANGE is handled by the specific counts in your config
            EnvParams.MAX_TIME = 200
            EnvParams.TRAIT_DIM = 5
            
            TrainParams.EMBEDDING_DIM = 128
            # Exact formula from test.py
            TrainParams.AGENT_INPUT_DIM = 6 + EnvParams.TRAIT_DIM
            TrainParams.TASK_INPUT_DIM = 5 + 2 * EnvParams.TRAIT_DIM
            
            # Use CPU for inference in ROS to save GPU for Gazebo/Rviz if needed
            device = torch.device('cpu') 
            
            self.global_network = AttentionNet(
                TrainParams.AGENT_INPUT_DIM,
                TrainParams.TASK_INPUT_DIM,
                TrainParams.EMBEDDING_DIM
            ).to(device)

            model_path = os.path.join(paper_code_path, 'model', 'save_5_task_2', 'checkpoint.pth')

            if os.path.exists(model_path):
                # Map location is crucial when moving from GPU training to CPU inference
                checkpoint = torch.load(model_path, map_location=device, weights_only=False)
                self.global_network.load_state_dict(checkpoint['best_model'])
                self.global_network.eval()

                self.worker = Worker(0, self.global_network, self.global_network, 0, device)

                self.get_logger().info(f'Loaded RL policy from {model_path}')
                return True
            else:
                self.get_logger().warn(f'Model not found at {model_path}')
                return False

        except Exception as e:
            self.get_logger().error(f'Failed to load policy: {e}')
            import traceback
            traceback.print_exc()
            return False

    # Old self written load_policy
    # def load_policy(self):
    #     if not PAPER_CODE_AVAILABLE:
    #         self.get_logger().warn('Paper code not available')
    #         return False

    #     try:
    #         TrainParams.EMBEDDING_DIM = 128
    #         TrainParams.AGENT_INPUT_DIM = 6 + EnvParams.TRAIT_DIM
    #         TrainParams.TASK_INPUT_DIM = 5 + 2 * EnvParams.TRAIT_DIM

    #         device = torch.device('cpu')
    #         self.global_network = AttentionNet(
    #             TrainParams.AGENT_INPUT_DIM,
    #             TrainParams.TASK_INPUT_DIM,
    #             TrainParams.EMBEDDING_DIM
    #         ).to(device)

    #         model_path = os.path.join(paper_code_path, 'model', 'save_5_task', 'checkpoint.pth')

    #         if os.path.exists(model_path):
    #             checkpoint = torch.load(model_path, map_location=device, weights_only=False)
    #             self.global_network.load_state_dict(checkpoint['best_model'])
    #             self.global_network.eval()

    #             self.worker = Worker(0, self.global_network, self.global_network, 0, device)

    #             self.get_logger().info(f'Loaded RL policy from {model_path}')
    #             return True
    #         else:
    #             self.get_logger().warn(f'Model not found at {model_path}')
    #             return False

    #     except Exception as e:
    #         self.get_logger().error(f'Failed to load policy: {e}')
    #         import traceback
    #         traceback.print_exc()
    #         return False
    
    def robot_position_callback(self, msg):
        robot_id = msg.header.frame_id
        pos = np.array([msg.pose.position.x, msg.pose.position.y])
        self.robot_positions[robot_id] = pos

    def setup_robot_subscribers(self):
        self.robot_subs = []
        for robot_id in self.robots:
            topic = f'/{robot_id}/ground_truth' 
            self.get_logger().info(f'Subscribing to {topic} (TFMessage)')

            self.robot_subs.append(self.create_subscription(
                TFMessage,
                topic,
                lambda msg, rid=robot_id: self.gps_callback(msg, rid), 
                qos_profile_sensor_data 
            ))


    def gps_callback(self, msg, robot_id):
        # Parse the TFMessage to find this robot's transform
        for t in msg.transforms:
            # We only care about transforms relative to "world" (Global Coordinates)
            if "world" not in t.header.frame_id:
                continue
            
            name = t.child_frame_id
            
            if robot_id in name:
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                
                # Update the global position
                self.robot_positions[robot_id] = np.array([tx, ty])
                return

    def odom_callback(self, msg, robot_id):
        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.robot_positions[robot_id] = pos

    def run_initial_allocation(self):
        if self.initial_allocation_done:
            return
        
        self.get_logger().info('Running RL allocation to compute full routes ... ')

        if self.use_rl and self.paper_env is not None:
            success = self.compute_full_routes_with_rl()
        else:
            success = False

        if not success:
            self.get_logger().warn('RL failed, using greedy fallback')
            # self.compute_greedy_routes()

        idle_robots = [r for r in self.robots if len(self.robot_routes.get(r, [])) == 0]
        if idle_robots:
            self.get_logger().info(f"Finding work for {len(idle_robots)} idle robots...")
            # self.assign_idle_robots_greedy(idle_robots)

        self.get_logger().info('Route computation complete')
        # elapsed = time.time() - start_time

        for robot_id in self.robots:
            if robot_id in self.robot_routes and len(self.robot_routes[robot_id]) > 0:
                next_task = self.robot_routes[robot_id][0]
                self.assign_task(robot_id, next_task)

        self.initial_allocation_done = True

    def send_robot_to_depot(self, robot_id):
        # 1. Find where this robot belongs
        species_name = self.robot_species[robot_id]
        depot_pos = self.species_info[species_name]['depot']
        
        # 2. Construct a "Task" message pointing to home
        msg = String()
        msg.data = json.dumps({
            'robot_id': robot_id,
            'task_id': 'DEPOT',  # Special ID
            'task_position': [float(depot_pos[0]), float(depot_pos[1])],
            'duration': 0 # No work needed at depot
        })
        
        self.task_assignment_pub.publish(msg)
        self.current_assignments[robot_id] = 'DEPOT'
        
        self.get_logger().info(f'🏠 {robot_id} returning to Depot at {depot_pos}')

    def compute_full_routes_with_rl(self):
        # 1. Safety Check: Do we have data from ROS yet?
        if len(self.robot_positions) < len(self.robots):
            self.get_logger().warn(f"Waiting for robot states... ({len(self.robot_positions)}/{len(self.robots)})")
            return False

        self.get_logger().info('Using RL policy to compute routes')

        try:
            # 2. Reset the abstract environment
            self.paper_env.init_state()
            
            # 3. SYNC: Inject REAL robot positions into the paper_env
            # The RL model needs to know where the robots ACTUALLY are
            for i, robot_id in enumerate(self.robots):
                if robot_id in self.robot_positions:
                    # Normalize real position to 0-1 range expected by RL
                    real_pos = self.robot_positions[robot_id]
                    norm_pos = (real_pos - self.world_min) / (self.world_range + 1e-6)
                    
                    # Update the specific agent in the environment dictionary
                    # Note: We assume the order of self.robots matches agent_dic keys 0..N
                    if i in self.paper_env.agent_dic:
                        self.paper_env.agent_dic[i]['pos'] = norm_pos.tolist()

            # 4. Run Inference
            self.worker.env = self.paper_env
            _, _, results = self.worker.run_episode(
                training = False,
                sample = False,
                max_waiting = True 
            )
            
            makespan_val = results.get("makespan", [0])[0]
            self.get_logger().info(f'RL Policy makespan: {makespan_val:.2f}')

            self.extract_full_routes_from_paper()

            return True

        except Exception as e:
            self.get_logger().error(f'RL route computation failed: {e}')
            import traceback
            traceback.print_exc()
            return False
    # def compute_full_routes_with_rl(self):
    #     self.get_logger().info('Using RL policy to compute routes')

    #     try:
    #         self.paper_env.init_state()
    #         self.worker.env =self.paper_env

    #         _, _, results = self.worker.run_episode(
    #             training = False,
    #             sample = False,
    #             max_waiting = True
    #         )
    #         # self.get_logger().info(results)
    #         makespan_val = results.get("makespan", [0])[0]
    #         self.get_logger().info(f'RL Policy makespan: {makespan_val:.2f}')

    #         self.extract_full_routes_from_paper()

    #         return True

    #     except Exception as e:
    #         self.get_logger().error(f'RL route computation failed: {e}')
    #         import traceback

    #         traceback.print_exc()
    #         return False
    def extract_full_routes_from_paper(self):
        """Extract complete task sequence for each robot from paper's solution"""
        self.robot_routes = {}
        
        # DEBUG: Log paper env state
        self.get_logger().info(f'Paper env has {self.paper_env.num_tasks} tasks')
        self.get_logger().info(f'Paper env has {self.paper_env.num_species} species')
        self.get_logger().info(f'Paper env has {len(self.paper_env.agent_dic)} agents')
        
        # DEBUG: Log task structure
        for i, task in enumerate(self.paper_env.tasks):
            self.get_logger().info(f'Paper task {i}: req={task["requirement"]}, pos={task["pos"]}')
        
        # DEBUG: Log species structure
        for i, species in enumerate(self.paper_env.species):
            self.get_logger().info(f'Paper species {i}: agents={species["num_agents"]}, trait={species["trait"]}')
        
        for agent_idx, agent_data in enumerate(self.paper_env.agent_dic.values()):
            if agent_idx < len(self.robots):
                robot_id = self.robots[agent_idx]
                
                route = agent_data.get('route', [])
                self.get_logger().info(f'Raw route for {robot_id}: {route}')
                
                # The route contains indices into self.paper_env.tasks
                # -1 means depot, >= 0 means task index
                task_sequence = []
                for node in route:
                    if node >= 0:  # Skip depot markers
                        # This is an index into paper_env.tasks
                        if node < len(self.paper_env.tasks):
                            # Map back to YOUR task IDs
                            real_task_keys = list(self.tasks.keys())
                            if node < len(real_task_keys):
                                real_task_id = real_task_keys[node]
                                task_sequence.append(real_task_id)
                        else:
                            self.get_logger().error(
                                f'Invalid task index {node} (paper_env has {len(self.paper_env.tasks)} tasks)'
                            )
                
                self.robot_routes[robot_id] = task_sequence
                self.get_logger().info(f'Mapped route for {robot_id}: {task_sequence}')
    # def extract_full_routes_from_paper(self):
    #     self.robot_routes = {}

    #     for agent_idx, agent_data  in enumerate(self.paper_env.agent_dic.values()):
    #         if agent_idx < len(self.robots):
    #             robot_id  = self.robots[agent_idx]

    #             route = agent_data.get('route', [])
    #             self.get_logger().info(f'Route for {robot_id} is : {route} -- in the loop')
    #             real_task_keys = list(self.tasks.keys())

    #             task_sequence = []

    #             for node in route:
    #                 if node >= 0 and node < len(real_task_keys):
    #                     real_task_id = real_task_keys[node]
    #                     task_sequence.append(real_task_id)

    #             self.robot_routes[robot_id] = task_sequence
    #             self.get_logger().info(f'Route for {robot_id}: {task_sequence}')

    def run_allocation_once(self):
        self.get_logger().info('Running RL allocation')
        start_time = time.time()
        if self.use_rl and self.paper_env is not None:
            assignments = self.allocate_with_rl()
        else:
            assignments = self.allocate_greedy()

        elapsed = time.time() - start_time
        self.get_logger().info(f'Allocation complete in {elapsed:.3f}s: {len(assignments)} assignments')

        for robot_id, task_id in assignments.items():
            self.assign_task(robot_id, task_id)

    def allocate_with_rl(self):
        self.get_logger().info('Using RL policy:')
        try:
            self.paper_env.init_state()
            self.worker.env = self.paper_env
            _, _, results = self.worker.run_episode(
                training=False,
                sample=False,
                max_waiting=False
            )
            assignments = self.extract_assignments_from_paper()

            makespan_val = results.get("makespan", [0])[0]
            self.get_logger().info(f'RL Policy result: makespan={makespan_val:.2f}')

            return assignments
        except Exception as e:
            self.get_logger().error(f'RL allocation failed: {e}')
            import traceback
            traceback.print_exc()
            return self.allocate_greedy()

    def extract_assignments_from_paper(self):
        assignments = {}

        for agent_idx, agent_data in enumerate(self.paper_env.agent_dic.values()):
            if agent_idx < len(self.robots):
                robot_id = self.robots[agent_idx]

                route = agent_data.get('route', [])
                target_task_id = None

                for node in route:
                    if node >= 0:
                        target_task_id = node
                        break

                if target_task_id is not None:
                    real_task_keys = list(self.tasks.keys())
                    if target_task_id < len(real_task_keys):
                        real_task_id = real_task_keys[target_task_id]
                        assignments[robot_id] = real_task_id
        
        return assignments

    def compute_greedy_routes(self):
        self.get_logger().info('Computing greedy routes (Complete Coverage)...')
        self.robot_routes = {robot_id: [] for robot_id in self.robots}
        
        # Track unsatisfied tasks
        # Dictionary: {task_id: remaining_requirements_array}
        task_needs = {
            tid: np.array(self.tasks[tid]['requirements']) 
            for tid in self.tasks
        }

        # Keep going until no more robots can help with any task
        while True:
            made_assignment = False
            
            for robot_id in self.robots:
                # Skip if robot is "full" (e.g. has > 3 tasks queued) to balance load
                if len(self.robot_routes[robot_id]) >= 3: continue

                robot_skills = np.array(self.robot_skills[robot_id])
                species_name = self.robot_species[robot_id]

                # Find best task for this robot
                best_task = None
                best_score = -1.0

                for task_id, needs in task_needs.items():
                    # Check if task is already done
                    if np.sum(needs) <= 0: continue

                    # Check contribution
                    contribution = np.sum(np.minimum(robot_skills, needs))
                    if contribution <= 0: continue

                    # Score = Contribution / Distance
                    # (Simplified distance calculation from depot)
                    depot_pos = np.array(self.species_info[species_name]['depot'])
                    task_pos = np.array(self.tasks[task_id]['position'])
                    dist = np.linalg.norm(depot_pos - task_pos) + 0.1
                    
                    score = contribution / dist

                    if score > best_score:
                        best_score = score
                        best_task = task_id

                # Assign
                if best_task is not None:
                    self.robot_routes[robot_id].append(best_task)
                    # Reduce needs so other robots don't pile on unnecessarily
                    task_needs[best_task] = np.maximum(0, task_needs[best_task] - robot_skills)
                    made_assignment = True

            if not made_assignment:
                break


    def check_simulation_end(self):
        # 1. Check if all tasks are completed
        if len(self.completed_tasks) < len(self.tasks):
            return

        # 2. Check if all robots are back at their depots
        all_home = True
        
        for robot_id in self.robots:
            # Get current position
            curr_pos = self.robot_positions.get(robot_id)
            if curr_pos is None:
                all_home = False
                break
            
            # Get depot position
            species_name = self.robot_species[robot_id]
            depot_pos = np.array(self.species_info[species_name]['depot'])
            
            # Check distance
            dist = np.linalg.norm(curr_pos - depot_pos)
            
            # Tolerance: 1.5 meters (generous to ensure it triggers)
            if dist > 3.5:
                all_home = False
                break

        # 3. If everyone is home, shutdown
        if all_home:
            self.get_logger().info("==================================================")
            self.get_logger().info("🎉 SIMULATION COMPLETE - ALL TASKS DONE & ROBOTS HOME")
            self.get_logger().info("==================================================")
            
            # Wait a split second for the log to print
            time.sleep(5.0)
            # subprocess.run(["pkill", "-9", "-f", "gzclient"])
                
            # subprocess.run(["pkill", "-9", "-f", "gzserver"])
            
            # subprocess.run(["pkill", "-9", "-f", "gazebo"])
            
            # subprocess.run(["pkill", "-9", "-f", "ign gazebo"])
            # subprocess.run(["pkill", "-9", "-f", "gz sim"])
            # subprocess.run(["pkill", "-9", "-f", "ruby"])
            
            # subprocess.run(["pkill", "-9", "-f", "robot_controller"])
            
            # self.get_logger().info("Kill commands sent.")
            # self.get_logger().info("Closing Gazebo...")
            # os.system("pkill -f gazebo")     
            # os.system("pkill -f gzserver")    
            # os.system("pkill -f gzclient")   
            # os.system("pkill -f robot_controller") 
            
            # rclpy.shutdown()
            # sys.exit(0)

    def allocate_greedy(self):
        self.get_logger().info('Using greedy allocation (fallsafe)')

        assignments = {}
        available_tasks = list(self.tasks.keys())

        for robot_id in self.robots:
            robot_skills = self.robot_skills[robot_id]
            species_name = self.robot_species[robot_id]

            robot_pos = self.robot_positions.get(
                robot_id,
                np.array(self.species_info[species_name]['depot'])
            )

            best_task = None
            best_score = -float('inf')

            for task_id in available_tasks:
                task_data = self.tasks[task_id]
                task_pos = np.array(task_data['position'])
                task_req = task_data['requirements']

                can_contribute = sum(
                    1 for rs, tr in zip(robot_skills, task_req)
                    if rs > 0 and tr > 0
                )

                if can_contribute == 0:
                    continue

                dist = np.linalg.norm(robot_pos - task_pos) + 0.1
                score = can_contribute / dist

                if score > best_score:
                    best_score = score
                    best_task = task_id

            if best_task is not None:
                assignments[robot_id] = best_task

                available_tasks.remove(best_task)

        return assignments

    # def assign_task(self, robot_id, task_id):
    #     task_data = self.tasks[task_id]
    #     msg = String()

    #     msg.data = json.dumps({
    #         'robot_id': robot_id,
    #         'task_id': task_id,
    #         'task_position': task_data['position']
    #     })

    #     self.task_assignment_pub.publish(msg)

    #     self.current_assignments[robot_id] = task_id

    #     self.get_logger().info(
    #         f"{robot_id} -> Task {task_id} at {task_data['position']}"
    #     )


    # def check_task_completion(self):
    #     current_time = time.time()

    #     active_robots = [r for r in self.current_assignments.keys() if not self.is_robot_waiting_or_working(r)]

    #     for robot_id in active_robots:
    #         task_id = self.current_assignments[robot_id]
    #         robot_pos = self.robot_positions.get(robot_id)

    #         if robot_pos is None: continue
            
    #         task_pos = np.array(self.tasks[task_id]['position'])
    #         dist = np.linalg.norm(robot_pos - task_pos)

    #         if dist < 2.0:
    #             if task_id not in self.waiting_at_task:
    #                 self.waiting_at_task[task_id] = []

    #             self.waiting_at_task[task_id].append(robot_id)
    #             self.get_logger().info(f"{robot_id} arrived at Task {task_id}.")

    #     for task_id in list(self.waiting_at_task.keys()):
    #         if task_id in self.ongoing_tasks: continue

    #         if self.check_team_skills(task_id):
    #             duration = self.tasks[task_id]['duration']
    #             self.ongoing_tasks[task_id] = current_time + duration
    #             self.get_logger().info(f"Task {task_id} Ongoing!")
    #             msg = String()
    #             msg.data = json.dumps({'task_id': task_id, 'command': 'START'})
    #             self.trigger_pub.publish(msg)

    #     for task_id, finish_time in list(self.ongoing_tasks.items()):
    #         if current_time >= finish_time:
    #             self.complete_task(task_id)

    def check_task_completion(self):
        current_time = time.time()
        self.check_simulation_end()
        # Check ALL robots with assignments to see if they've arrived
        for robot_id in list(self.current_assignments.keys()):


            task_id = self.current_assignments[robot_id]
            
            if task_id == 'DEPOT':
                continue
            
            robot_pos = self.robot_positions.get(robot_id)



            if robot_pos is None: 
                continue
            
            task_pos = np.array(self.tasks[task_id]['position'])
            dist = np.linalg.norm(robot_pos - task_pos)

            # Robot arrived at task
            if dist < 2.0:
                # Initialize waiting list if needed
                if task_id not in self.waiting_at_task:
                    self.waiting_at_task[task_id] = []

                # Add robot to waiting list if not already there
                if robot_id not in self.waiting_at_task[task_id]:
                    self.waiting_at_task[task_id].append(robot_id)
                    self.get_logger().info(f"{robot_id} arrived at Task {task_id}.")

        # Check if any waiting tasks now have enough skills to start
        for task_id in list(self.waiting_at_task.keys()):
            # Skip if already ongoing
            if task_id in self.ongoing_tasks: 
                continue

            # Check if team has sufficient skills
            if self.check_team_skills(task_id):
                duration = self.tasks[task_id]['duration']
                self.ongoing_tasks[task_id] = current_time + duration
                self.get_logger().info(f"✅ Task {task_id} STARTED! Team complete.")
                
                # Publish start trigger
                msg = String()
                msg.data = json.dumps({'task_id': task_id, 'command': 'START'})
                self.trigger_pub.publish(msg)

        # Check if any ongoing tasks are finished
        for task_id, finish_time in list(self.ongoing_tasks.items()):
            if current_time >= finish_time:
                self.complete_task(task_id)
        
    def is_robot_waiting_or_working(self, robot_id):
        for waiters in self.waiting_at_task.values():
            if robot_id in waiters: return True

        return False

    def check_team_skills(self, task_id):
        required = np.array(self.tasks[task_id]['requirements'])
        current_skills = np.zeros_like(required)
        waiters = self.waiting_at_task[task_id]
        for robot_id in waiters:
            r_skills = np.array(self.robot_skills[robot_id])
            current_skills += r_skills

        if np.all(current_skills >= required):
            return True

        return False

    def complete_task(self, task_id):
        self.get_logger().info(f"Task {task_id} COMPLETED")
        self.completed_tasks.add(task_id)
        # self.change_task_color(task_id, completed=True)

        robots_to_release = self.waiting_at_task.pop(task_id)
        del self.ongoing_tasks[task_id]

        for robot_id in robots_to_release:
            if robot_id in self.current_assignments:
                del self.current_assignments[robot_id]

            route = self.robot_routes.get(robot_id, [])
            if task_id in route:
                route.remove(task_id)

            self.assign_next_task(robot_id, route)

        if len(self.completed_tasks) == len(self.tasks):
            self.get_logger().info('All tasks completed')

    def assign_idle_robots_greedy(self, idle_robots):
        """Force assign idle robots to help the nearest active task"""
        for robot_id in idle_robots:
            robot_skills = np.array(self.robot_skills[robot_id])
            species_name = self.robot_species[robot_id]
            
            # Get robot position (Depot)
            robot_pos = self.robot_positions.get(
                robot_id,
                np.array(self.species_info[species_name]['depot'])
            )

            best_task = None
            min_dist = float('inf')

            # Look at ALL tasks to see where this robot can help
            for task_id, task_data in self.tasks.items():
                # Logic: Assign to closest task where it has RELEVANT skills
                # (Or just closest task if you want them to move regardless)
                task_req = np.array(task_data['requirements'])
                
                # Check if robot has ANY matching skill for this task
                # (Even if the task is already fully staffed by the RL, extra help doesn't hurt)
                contribution = np.sum(np.minimum(robot_skills, task_req))
                
                if contribution > 0:
                    dist = np.linalg.norm(robot_pos - np.array(task_data['position']))
                    if dist < min_dist:
                        min_dist = dist
                        best_task = task_id
            
            # If we found a match, update the route!
            if best_task is not None:
                self.robot_routes[robot_id] = [best_task]
                self.get_logger().info(f"➕ Forced {robot_id} to assist with Task {best_task}")
            else:
                 self.get_logger().warn(f"⚠️ Could not find useful work for {robot_id}")

    def assign_next_task(self, robot_id, route):
        while len(route) > 0 and route[0] in self.completed_tasks:
            route.pop(0)
        if len(route) > 0:
            next_task = route[0]
            self.assign_task(robot_id, next_task)
        else:
            if self.current_assignments.get(robot_id) != 'DEPOT':
                self.send_robot_to_depot(robot_id)

    def assign_task(self, robot_id, task_id):
        task_data = self.tasks[task_id]
        
        msg = String()
        msg.data = json.dumps({
            'robot_id': robot_id,
            'task_id': task_id,
            'task_position': task_data['position'],
            'duration': task_data['duration']
        })
        
        self.task_assignment_pub.publish(msg)
        self.current_assignments[robot_id] = task_id
        self.task_start_times[robot_id] = time.time()
        
        self.get_logger().info(
            f'📋 {robot_id} → Task {task_id} @ {task_data["position"]}'
        )

def main(args = None):
    rclpy.init(args=args)
    allocator = RLTaskAllocator()

    try:
        rclpy.spin(allocator)
    except KeyboardInterrupt:
        pass
    finally:
        allocator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()