#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
from geometry_msgs.msg import Pose, Point, Quaternion
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET

class ScenarioSpawner(Node):
    def __init__(self):
        super().__init__('scenario_spawner')
        
        # Service client
        self.spawn_client = self.create_client(SpawnEntity, '/world/simple_world/create')
        
        # Wait for Gazebo services
        self.get_logger().info('Waiting for Gazebo spawn service...')
        timeout_counter = 0
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')
            timeout_counter += 1
            if timeout_counter > 30:
                self.get_logger().error('Spawn service timeout! Check if Gazebo is running.')
                return
        
        self.get_logger().info('Spawn service is ready!')
        
        # Load configuration
        pkg_dir = get_package_share_directory('mrta_simulation')
        config_path = os.path.join(pkg_dir, 'config', 'scenario_simple.yaml')
        
        self.get_logger().info(f'Loading config from: {config_path}')
        
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.get_logger().info('Configuration loaded successfully')
        
        # Load model SDFs
        self.depot_sdf = self.load_model_sdf('depot_marker')
        self.task_sdf = self.load_model_sdf('task_marker')
        
        # Spawn everything in order
        self.spawn_depots()
        self.spawn_tasks()
        self.spawn_obstacles()  # ADD THIS LINE
        self.spawn_robots()
        self.get_logger().info('Scenario setup complete!')
    
    def load_model_sdf(self, model_name):
        """Load SDF file content from models directory"""
        pkg_dir = get_package_share_directory('mrta_simulation')
        model_path = os.path.join(pkg_dir, 'models', model_name, 'model.sdf')
        
        with open(model_path, 'r') as f:
            return f.read()
    
    def spawn_entity(self, name, sdf, pose):
        """Generic spawn function for any entity"""
        request = SpawnEntity.Request()
        
        # Fill in the entity_factory structure
        request.entity_factory.name = name
        request.entity_factory.sdf = sdf
        request.entity_factory.allow_renaming = False
        request.entity_factory.pose = pose
        request.entity_factory.relative_to = "world"
        
        # Call service
        self.get_logger().info(f'Spawning: {name}')
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'✓ Successfully spawned: {name}')
            else:
                self.get_logger().error(f'✗ Failed to spawn {name}')
        else:
            self.get_logger().error(f'✗ Service call timeout for {name}')
    
    def spawn_depots(self):
        """Spawn depot markers for each species"""
        self.get_logger().info('Spawning depot markers...')
        
        for species_name, species_data in self.config['species'].items():
            pos = species_data['depot_position']
            
            pose = Pose()
            pose.position = Point(x=pos[0], y=pos[1], z=0.3)
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            name = f"depot_{species_name}"
            self.spawn_entity(name, self.depot_sdf, pose)
    
    def spawn_tasks(self):
        """Spawn task markers"""
        self.get_logger().info('Spawning task markers...')
        
        for task in self.config['tasks']:
            pos = task['position']
            task_id = task['id']
            
            pose = Pose()
            pose.position = Point(x=pos[0], y=pos[1], z=0.25)
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            name = f"task_{task_id}"
            self.spawn_entity(name, self.task_sdf, pose)

    def spawn_obstacles(self):
        """Spawn obstacles from configuration"""
        self.get_logger().info(f'Config keys: {list(self.config.keys())}')
        self.get_logger().info(f'Full config: {self.config}')
        
        obstacles_config = self.config.get('obstacles', {})
        self.get_logger().info(f'Obstacles found: {len(obstacles_config)}')
        self.get_logger().info(f'Obstacles: {obstacles_config}')
        obstacles_config = self.config.get('obstacles', {})
        
        if not obstacles_config:
            self.get_logger().info('No obstacles in configuration')
            return
        
        self.get_logger().info(f'Spawning {len(obstacles_config)} obstacles...')
        
        for obstacle_name, obstacle_info in obstacles_config.items():
            model_type = obstacle_info['model']
            position = obstacle_info['position']
            
            try:
                # Load the SDF file for this obstacle type
                obstacle_sdf = self.load_model_sdf(model_type)
                
                # Create pose
                pose = Pose()
                pose.position = Point(x=position[0], y=position[1], z=position[2])
                pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                
                # Spawn it
                self.spawn_entity(obstacle_name, obstacle_sdf, pose)
                
            except FileNotFoundError:
                self.get_logger().error(f'Could not find model: {model_type}')
            except Exception as e:
                self.get_logger().error(f'Error spawning obstacle {obstacle_name}: {str(e)}')

    def spawn_robots(self):
        """Spawn robots for each species"""
        import math
        
        self.get_logger().info('Spawning robots...')
        
        for species_name, species_data in self.config['species'].items():
            robot_type = species_data['type']
            count = species_data['count']
            depot_pos = species_data['depot_position']
            
            # Circle radius around depot
            radius = 1.0  # 1 meter radius circle
            
            for i in range(count):
                # Calculate angle for circular arrangement
                angle = (i / count) * 2 * math.pi  # Divide circle evenly
                
                # Calculate position on circle
                offset_x = radius * math.cos(angle)
                offset_y = radius * math.sin(angle)
                
                pose = Pose()
                pose.position = Point(
                    x=depot_pos[0] + offset_x,
                    y=depot_pos[1] + offset_y,
                    z=0.15 if robot_type == "turtlebot3_burger" else (depot_pos[2]+1.0)
                )
                pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                
                robot_name = f"{species_name}_{i}"
                
                if robot_type == "turtlebot3_burger":
                    self.spawn_turtlebot(robot_name, pose)
                elif robot_type == "crazyflie":
                    self.spawn_quadrotor(robot_name, pose)
                else:
                    self.get_logger().warn(f"Unknown robot type: {robot_type} for {robot_name}")
    
    def spawn_turtlebot(self, name, pose):
        """
        Spawn a turtlebot and REMAP its topics so it can be moved individually.
        """

        sdf_content = self.load_model_sdf('simple_turtlebot')

        new_cmd_vel = f"<topic>/model/{name}/cmd_vel</topic>"
        sdf_content = sdf_content.replace("<topic>cmd_vel</topic>", new_cmd_vel)
        new_odom = f"<topic>/model/{name}/odometry</topic>"
        sdf_content = sdf_content.replace("<topic>odometry</topic>", new_odom)
        sdf_content = sdf_content.replace("<odom_topic>odometry</odom_topic>", f"<odom_topic>/model/{name}/odometry</odom_topic>")
        self.get_logger().info(f"Remapped topics for {name} to /model/{name}/...")
        self.spawn_entity(name, sdf_content, pose)

    def spawn_quadrotor(self, name, pose):
        """Spawn a quadrotor from model file"""
        quadrotor_sdf = self.load_model_sdf('simple_quadrotor')
        self.spawn_entity(name, quadrotor_sdf, pose)

def main(args=None):
    rclpy.init(args=args)
    spawner = ScenarioSpawner()
    
    # Keep node alive briefly
    rclpy.spin_once(spawner, timeout_sec=2.0)
    
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()