#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import numpy as np
import json
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import torch
import sys
import time

# --- HARDCODE PATH ---
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

        # 1. Parse Config
        self.parse_scenario()
        
        # 2. Setup ROS Communication
        self.task_assignment_pub = self.create_publisher(String, '/task_assignments', 10)
        self.trigger_pub = self.create_publisher(String, '/task_triggers', 10)
        self.create_subscription(PoseStamped, '/robot_positions', self.robot_position_callback, 10)
        self.create_subscription(String, '/task_status', self.task_status_callback, 10)

        # 3. State Tracking
        self.robot_positions = {}
        self.completed_tasks = set()
        self.current_assignments = {}
        self.robot_routes = {}
        self.waiting_at_task = {} 
        self.ongoing_tasks = {}   

        # 4. Initialize RL (The Test Script Way)
        self.init_rl_environment()
        
        # 5. Timers
        # Wait 5s for robots to spawn, then calculate routes
        self.create_timer(5.0, self.run_allocation_once)
        # Run sync logic constantly
        self.create_timer(0.5, self.check_sync_status)
        
        self.initial_allocation_done = False
        self.get_logger().info('RL Task Allocator Initialized')

    def parse_scenario(self):
        """Reads YAML and builds internal lists"""
        self.robots = []
        self.robot_skills = {}
        self.species_info = {}
        self.tasks = {}

        # Parse Species
        for species_name, species_data in self.config['species'].items():
            count = species_data['count']
            skills = species_data['skills']
            depot = species_data['depot_position'][:2]
            
            self.species_info[species_name] = {'skills': skills, 'depot': depot, 'count': count}
            
            for i in range(count):
                robot_id = f"{species_name}_{i}"
                self.robots.append(robot_id)
                self.robot_skills[robot_id] = skills

        # Parse Tasks
        for task in self.config['tasks']:
            t_id = task['id']
            self.tasks[t_id] = {
                'position': task['position'][:2],
                'requirements': task['requirements'],
                'duration': task['duration']
            }

    def init_rl_environment(self):
        """
        Mimics main() from test.py to set up the environment and model.
        """
        if not PAPER_CODE_AVAILABLE: return

        # 1. Set Parameters to Match Scenario
        EnvParams.TASKS_RANGE = (len(self.tasks), len(self.tasks))
        EnvParams.SPECIES_RANGE = (len(self.species_info), len(self.species_info))
        EnvParams.TRAIT_DIM = 5 # Must match training
        EnvParams.MAX_TIME = 200
        
        # 2. Update TrainParams (Input Dimensions)
        TrainParams.AGENT_INPUT_DIM = 6 + EnvParams.TRAIT_DIM
        TrainParams.TASK_INPUT_DIM = 5 + 2 * EnvParams.TRAIT_DIM
        TrainParams.EMBEDDING_DIM = 128

        # 3. Load Model
        device = torch.device('cpu')
        self.global_network = AttentionNet(
            TrainParams.AGENT_INPUT_DIM, TrainParams.TASK_INPUT_DIM, TrainParams.EMBEDDING_DIM
        ).to(device)
        
        # POINT TO YOUR NEW MODEL
        model_path = os.path.join(paper_code_path, 'model', 'save_5_task', 'checkpoint.pth')
        
        try:
            # Weights_only=False fixes the PyTorch 2.6 error
            checkpoint = torch.load(model_path, map_location=device, weights_only=False)
            self.global_network.load_state_dict(checkpoint['best_model'])
            self.global_network.eval()
            self.get_logger().info(f'Loaded Model: {model_path}')
        except Exception as e:
            self.get_logger().error(f"Model Load Failed: {e}")
            return

        # 4. Create Environment (The Digital Twin)
        # We define ranges to force the generator to make exactly 9 robots and 5 tasks
        per_species_count = [s['count'] for s in self.species_info.values()]
        total_robots = sum(per_species_count)
        
        # This init generates a random environment of the correct size
        self.paper_env = TaskEnv(
            per_species_range=(min(per_species_count), max(per_species_count)),
            species_range=EnvParams.SPECIES_RANGE,
            tasks_range=EnvParams.TASKS_RANGE,
            traits_dim=EnvParams.TRAIT_DIM,
            seed=42 # Consistent seed
        )

        # 5. Setup Worker
        self.worker = Worker(0, self.global_network, self.global_network, 0, device)
        self.worker.env = self.paper_env

    def sync_digital_twin(self):
        """
        Takes the generic TaskEnv created above and injects REAL ROS data.
        This replaces the random generation with your actual scenario.
        """
        if self.paper_env is None: return

        # 1. Coordinate Normalization
        all_pos = [s['depot'] for s in self.species_info.values()] + [t['position'] for t in self.tasks.values()]
        all_pos = np.array(all_pos)
        world_min = all_pos.min(axis=0)
        world_range = all_pos.max(axis=0) - world_min
        def norm(p): return (np.array(p) - world_min) / (world_range + 1e-6)

        # 2. Inject Tasks (Sorted by ID)
        # We overwrite self.paper_env.task_dic directly
        sorted_task_ids = sorted(self.tasks.keys())
        for idx, t_id in enumerate(sorted_task_ids):
            t_data = self.tasks[t_id]
            
            # Update existing dictionary entry or create new if missing
            if idx not in self.paper_env.task_dic:
                self.paper_env.task_dic[idx] = {}

            # Pad requirements to match TRAIT_DIM
            req = np.zeros(EnvParams.TRAIT_DIM)
            req[:len(t_data['requirements'])] = t_data['requirements']

            self.paper_env.task_dic[idx].update({
                'ID': idx,
                'requirements': req,
                'status': req.copy(), # Reset status
                'location': norm(t_data['position']),
                'time': t_data['duration'],
                'feasible_assignment': False,
                'finished': False,
                'members': []
            })
        
        # 3. Inject Agents (Sorted by Species -> ID)
        # We need a deterministic mapping: RL Agent 0 -> ROS Robot "red_0"
        self.ordered_robot_ids = []
        sorted_species = sorted(self.species_info.keys())
        
        agent_idx = 0
        for s_idx, s_name in enumerate(sorted_species):
            info = self.species_info[s_name]
            count = info['count']
            
            # Pad skills
            skills = np.zeros(EnvParams.TRAIT_DIM)
            skills[:len(info['skills'])] = info['skills']
            
            depot_pos = norm(info['depot'])
            
            for i in range(count):
                robot_name = f"{s_name}_{i}"
                self.ordered_robot_ids.append(robot_name)
                
                # Get current position from ROS (or default to depot)
                current_ros_pos = self.robot_positions.get(robot_name, info['depot'])
                
                if agent_idx not in self.paper_env.agent_dic:
                    self.paper_env.agent_dic[agent_idx] = {}

                self.paper_env.agent_dic[agent_idx].update({
                    'ID': agent_idx,
                    'species': s_idx,
                    'abilities': skills,
                    'location': norm(current_ros_pos), # Use REAL position
                    'depot': depot_pos,
                    'route': [-s_idx - 1], # Reset route to start at depot
                    'current_task': -s_idx - 1,
                    'velocity': 0.5,
                    'assigned': False
                })
                agent_idx += 1

        self.paper_env.agents_num = agent_idx
        self.paper_env.tasks_num = len(sorted_task_ids)
        
        # 4. Finalize Environment State
        # Re-calculate distance matrices based on new positions
        self.paper_env.species_distance_matrix, self.paper_env.species_neighbor_matrix = \
            self.paper_env.generate_distance_matrix()
        
        self.paper_env.init_state()
        self.get_logger().info("Digital Twin Synced with ROS State.")

    def run_allocation_once(self):
        if self.initial_allocation_done or self.paper_env is None: return
        
        self.get_logger().info('Running RL Inference...')
        
        # 1. Sync State
        self.sync_digital_twin()
        
        # 2. Run Episode (Inference)
        # Mimic test.py: sampling=False (Deterministic), max_task=False
        try:
            _, _, results = self.worker.run_episode(training=False, sample=False, max_waiting=False)
            makespan = results.get('makespan', [0])[0]
            self.get_logger().info(f"RL Result: Makespan {makespan:.2f}")
        except Exception as e:
            self.get_logger().error(f"RL Inference Failed: {e}")
            return

        # 3. Extract Routes
        # The RL updates agent_dic['route'] in place. We just read it out.
        self.robot_routes = {}
        sorted_tasks = sorted(self.tasks.keys()) # Must match sync order
        
        for agent_idx, agent_data in self.paper_env.agent_dic.items():
            if agent_idx < len(self.ordered_robot_ids):
                robot_id = self.ordered_robot_ids[agent_idx]
                raw_route = agent_data.get('route', [])
                
                # Convert RL indices to Task IDs
                # RL Route looks like: [-1 (Depot), 0 (Task), 2 (Task), -1 (Depot)]
                real_route = []
                for node in raw_route:
                    if node >= 0 and node < len(sorted_tasks):
                        real_route.append(sorted_tasks[node])
                
                self.robot_routes[robot_id] = real_route
                self.get_logger().info(f"Route {robot_id}: {real_route}")

        # 4. Assign First Tasks
        for robot_id, route in self.robot_routes.items():
            if len(route) > 0:
                self.assign_task(robot_id, route[0])

        self.initial_allocation_done = True

    
    def assign_task(self, r_id, t_id):
        t_data = self.tasks[t_id]
        msg = String()
        msg.data = json.dumps({
            'robot_id': r_id, 'task_id': t_id, 
            'task_position': t_data['position'],
            'duration': t_data['duration']
        })
        self.task_assignment_pub.publish(msg)
        self.current_assignments[r_id] = t_id
        self.get_logger().info(f"📋 {r_id} -> Task {t_id}")

    def robot_position_callback(self, msg):
        self.robot_positions[msg.header.frame_id] = np.array([msg.pose.position.x, msg.pose.position.y])

    def task_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data['status'] == 'ARRIVED':
                self.handle_arrival(data['robot_id'], data['task_id'])
        except: pass

    
    def check_sync_status(self):
        current_time = time.time()
        
        # 1. Check Arrivals (Distance fallback)
        for r_id, t_id in list(self.current_assignments.items()):
            if self.is_busy(r_id): continue
            r_pos = self.robot_positions.get(r_id)
            if r_pos is not None:
                t_pos = np.array(self.tasks[t_id]['position'])
                if np.linalg.norm(r_pos - t_pos) < 2.0:
                    self.handle_arrival(r_id, t_id)

        # 2. Check Team & Start
        for t_id in list(self.waiting_at_task.keys()):
            if t_id in self.ongoing_tasks: continue
            if self.check_skills(t_id):
                duration = self.tasks[t_id]['duration']
                self.ongoing_tasks[t_id] = current_time + duration
                self.get_logger().info(f"⚙️ Task {t_id} START! ({duration}s)")
                self.trigger_pub.publish(String(data=json.dumps({'task_id': t_id, 'command': 'START'})))

        # 3. Check Done
        for t_id, end_time in list(self.ongoing_tasks.items()):
            if current_time >= end_time:
                self.finish_task(t_id)

    def handle_arrival(self, r_id, t_id):
        if t_id not in self.waiting_at_task: self.waiting_at_task[t_id] = []
        if r_id not in self.waiting_at_task[t_id]:
            self.waiting_at_task[t_id].append(r_id)
            self.get_logger().info(f"⏳ {r_id} waiting at Task {t_id}")

    def is_busy(self, r_id):
        for waiters in self.waiting_at_task.values():
            if r_id in waiters: return True
        return False

    def check_skills(self, t_id):
        req = np.zeros(EnvParams.TRAIT_DIM)
        raw = self.tasks[t_id]['requirements']
        req[:len(raw)] = raw
        
        curr = np.zeros_like(req)
        for r_id in self.waiting_at_task[t_id]:
            r_raw = self.robot_skills[r_id]
            r_skill = np.zeros(EnvParams.TRAIT_DIM)
            r_skill[:len(r_raw)] = r_raw
            curr += r_skill
        return np.all(curr >= req)

    def finish_task(self, t_id):
        self.get_logger().info(f"✅ Task {t_id} DONE")
        self.completed_tasks.add(t_id)
        robots = self.waiting_at_task.pop(t_id)
        del self.ongoing_tasks[t_id]
        
        for r_id in robots:
            if r_id in self.current_assignments: del self.current_assignments[r_id]
            route = self.robot_routes.get(r_id, [])
            if t_id in route: route.remove(t_id)
            
            if route and route[0] not in self.completed_tasks:
                self.assign_task(r_id, route[0])
            else:
                self.get_logger().info(f"🏁 {r_id} finished route")

def main(args=None):
    rclpy.init(args=args)
    node = RLTaskAllocator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()