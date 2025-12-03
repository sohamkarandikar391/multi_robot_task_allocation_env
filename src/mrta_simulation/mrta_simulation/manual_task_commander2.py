#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys

class ManualTaskCommander(Node):
    def __init__(self):
        super().__init__('manual_task_commander')
        self.task_pub = self.create_publisher(
            String,
            '/task_assignments',
            10
        )

        self.get_logger().info('Manual Task Commander started')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Commands:')
        self.get_logger().info('  <robot_id> <task_id>  - Assign task to robot')
        self.get_logger().info('  list                  - List available robots/tasks')
        self.get_logger().info('  quit                  - Exit')
        self.get_logger().info('=' * 60)

        self.load_scenario_config()

        self.timer = self.create_timer(0.1, self.command_loop)
        self.running = True

    def load_scenario_config(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            import yaml
            import os
            pkg_dir = get_package_share_directory('mrta_simulation')
            config_path = os.path.join(pkg_dir, 'config', 'scenario_simple.yaml')

            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            self.robots = []

            for species_name, species_data in config['species'].items():
                count = species_data['count']
                for i in range(count):
                    robot_id = f"{species_name}_{i}"
                    self.robots.append(robot_id)

            self.tasks = config['tasks']

            self.get_logger().info(f'Loaded {len(self.robots)} robots and {len(self.tasks)} tasks')

        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
            self.robots = []
            self.tasks = []

    def command_loop(self):
        if not self.running:
            return

        try:
            print("\n> ", end='', flush=True)

            import select
            if select.select([sys.stdin], [], [], 0.0)[0]:
                command = sys.stdin.readline().strip()
                self.process_command(command)

        except Exception as e:
            self.get_logger().error(f'Command loop error: {e}')

    def process_command(self, command):
        parts = command.split()

        if not parts:
            return 

        cmd = parts[0].lower()

        if cmd == 'quit' or cmd == 'exit':
            self.get_logger().info('Shutting down...')
            self.running = False
            rclpy.shutdown()

        elif cmd == 'list':
            self.print_available_entities()

        elif len(parts) == 2:
            robot_id = parts[0]
            try:
                task_id = int(parts[1])
                self.send_task_assignment(robot_id, task_id)

            except ValueError:
                self.get_logger().error('Task ID must be an integer')
        else:
            self.get_logger().warn('Unknown command. Use: <robot_id> <task_id>, list, or quit')

    def print_available_entities(self):
        print("\n" + "="*60)
        print("Available Robots:")
        for robot in self.robots:
            print(f" - {robot}")

        print("\n Available Tasks:")
        for task in self.tasks:
            pos = task['position']
            req = task['requirements']
            print(f" - Task {task['id']}: pos={pos}, requirements={req}")

        print("="*60)

    def send_task_assignment(self, robot_id, task_id):
        if robot_id not in self.robots:
            self.get_logger().error(f'Unknown robot: {robot_id}')
            return

        tasks_ids = [t['id'] for t in self.tasks]

        if task_id not in task_ids:
            self.get_logger().error(f'Unknown task: {task_id}')
            return

        task_pos = None
        for task in self.tasks:
            if task['id'] == task_id:
                task_pos = task['position']
                break

        msg_data = {
            'robot_id': robot_id,
            'task_id': task_id,
            'task_position': task_pos
        }

        msg = String()
        msg.data = json.dumps(msg_data)

        self.task_pub.publish(msg)
        self.get_logger().info(f'Assigned Task {task_id} to {robot_id}')
    

def main(args=None):
    rclpy.init(args=args)

    commander = ManualTaskCommander()

    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        pass
    finally:
        commander.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()