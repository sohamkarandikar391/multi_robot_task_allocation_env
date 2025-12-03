#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import json
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from mrta_simulation.potential_field_controller import PotentialFieldController
import math

def get_yaw_from_quaternion(q):
    """
    Convert orientation quaternion to Euler Yaw angle
    """
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)

class RobotControllerNode(Node):

    def __init__(self, robot_id, is_aerial=False):
        super().__init__(f'robot_controller_{robot_id}')

        self.robot_id = robot_id
        self.is_aerial = is_aerial

        self.current_pos = None
        self.current_goal = None
        self.at_goal = False
        self.goal_threshold = 0.3

        self.pf_controller = PotentialFieldController(
            k_att = 1.5,
            k_rep = 8.0,
            d_safe = 0.8,
            d_influence = 3.0,
            max_vel = 0.5 if not is_aerial else 0.8
        )

        self.load_environment()
        self.other_robot_positions = {}
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{robot_id}/cmd_vel',
            10
        )
        
        self.pos_pub = self.create_publisher(
            PoseStamped,
            '/robot_positions',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{robot_id}/odometry',
            self.odom_callback,
            10
        )

        self.task_sub = self.create_subscription(
            String, 
            '/task_assignments',
            self.task_assignment_callback,
            10
        )

        

        self.create_subscription(
            PoseStamped,
            '/robot_positions',
            self.robot_positions_callback,
            10
        )

        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.broadcast_timer = self.create_timer(0.2, self.broadcast_position)
        self.get_logger().info(f'Robot Controller {robot_id} initialized.')

        import numpy as np
        # Give it a goal 2 meters forward and 2 meters left
        self.current_goal = np.array([2.0, 2.0]) 
        self.at_goal = False
        self.get_logger().info(f"TEST MODE: Force goal set to {self.current_goal}")
        # -------------------------------------

        self.control_loop_timer = self.create_timer(0.1, self.control_loop)

    def load_environment(self):
        try:
            pkg_dir = get_package_share_directory('mrta_simulation')
            config_path = os.path.join(pkg_dir, 'config', 'scenario_simple.yaml')

            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            self.obstacles = []
            obstacles_config = config.get('obstacles', {})
            for obs_name, obs_info in obstacles_config.items():
                pos = obs_info['position']
                self.obstacles.append([pos[0], pos[1]])

            self.task_positions = {}
            for task in config['tasks']:
                task_id = task['id']
                pos = task['position']
                self.task_positions[task_id] = [pos[0], pos[1]]

            self.get_logger().info(f'Loaded {len(self.obstacles)} obstacles, {len(self.task_positions)} tasks')

        except Exception as e:
            self.get_logger().error(f'Failed to load environment: {e}')
            self.obstacles = []
            self.task_positions = {}

    def odom_callback(self, msg):
        self.current_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.current_z = msg.pose.pose.position.z  
        self.current_yaw = get_yaw_from_quaternion(msg.pose.pose.orientation)

        self.get_logger().info(f'Odom received: {self.current_pos}', throttle_duration_sec=1.0)

    def task_assignment_callback(self, msg):
        try:
            data = json.loads(msg.data)

            if data['robot_id'] != self.robot_id:
                return

            task_id = data['task_id']
            task_pos = data['task_position']

            self.current_goal = np.array([task_pos[0], task_pos[1]])
            self.at_goal = False

            self.get_logger().info(f'New task assigned: Task {task_id} at {self.current_goal}')

        except Exception as e:
            self.get_logger().error(f'Failed to parse task assignment : {e}')

    def robot_positions_callback(self, msg):
        robot_id = msg.header.frame_id

        if robot_id == self.robot_id:
            return

        pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y
        ])


        self.other_robot_positions[robot_id] = pos

    def broadcast_position(self):
        if self.current_pos is None:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_id
        msg.pose.position.x = float(self.current_pos[0])
        msg.pose.position.y = float(self.current_pos[1])
        msg.pose.position.z = 0.0

        self.pos_pub.publish(msg)

    # def control_loop(self):
    #     velocity_global = self.pf_controller.compute_velocity_command(
    #         robot_pos=self.current_pos,
    #         goal_pos=self.current_goal,
    #         obstacles=self.obstacles,
    #         other_robots=list(self.other_robot_positions.values())
    #     )
    #     desired_heading = math.atan2(velocity_global[1], velocity_global[0])
    #     heading_error = desired_heading - self.current_yaw
    #     while heading_error > math.pi: heading_error -= 2 * math.pi
    #     while heading_error < -math.pi: heading_error += 2 * math.pi
    #     twist = Twist()
    #     velocity_magnitude = np.linalg.norm(velocity_global)
    #     twist.linear.x = velocity_magnitude * math.cos(heading_error)
    #     k_angular = 2.0 
    #     twist.angular.z = k_angular * heading_error
    #     twist.linear.y = 0.0
    #     if self.current_pos is None:
    #         return

    #     if self.current_goal is None or self.at_goal:
    #         self.stop_robot()
    #         return

    #     distance_to_goal = np.linalg.norm(self.current_pos - self.current_goal)
    #     if distance_to_goal < self.goal_threshold:
    #         self.get_logger().info(f'Reached goal at {self.current_goal}')
    #         self.at_goal = True
    #         self.stop_robot()
    #         return

    #     other_positions = list(self.other_robot_positions.values())
    #     velocity_2d = self.pf_controller.compute_velocity_command(
    #         robot_pos = self.current_pos,
    #         goal_pos = self.current_goal,
    #         obstacles = self.obstacles,
    #         other_robots = other_positions
    #     )

    #     if self.pf_controller.is_stuck():
    #         self.get_logger().warn('Robot appears stuck! Adding perturbation ...')
    #         velocity_2d += self.pf_controller.get_random_perturbation()

    #     twist = self.pf_controller.create_twist_msg(velocity_2d, self.is_aerial)
    #     self.cmd_vel_pub.publish(twist)
    def control_loop(self):
        if self.current_pos is None: return
        if self.current_goal is None or self.at_goal:
            self.stop_robot()
            return

        # 1. Get Global Velocity (2D)
        velocity_global = self.pf_controller.compute_velocity_command(
            robot_pos=self.current_pos,
            goal_pos=self.current_goal,
            obstacles=self.obstacles,
            other_robots=list(self.other_robot_positions.values())
        )

        # 2. Calculate Heading & Error
        desired_heading = math.atan2(velocity_global[1], velocity_global[0])
        heading_error = desired_heading - self.current_yaw
        while heading_error > math.pi: heading_error -= 2 * math.pi
        while heading_error < -math.pi: heading_error += 2 * math.pi

        twist = Twist()
        velocity_magnitude = np.linalg.norm(velocity_global)

        if self.is_aerial:
            # === DRONE LOGIC (3D) ===
            
            # A. Altitude Control (Simple P-Controller to fly at 1.5m)
            target_height = 1.5
            current_z = 0.0 # You need to update odom_callback to read msg.pose.pose.position.z
            # (Assuming you update odom_callback to store self.current_z)
            
            # If we don't have Z yet, just fly up blind
            twist.linear.z = 1.0 if hasattr(self, 'current_z') and self.current_z < target_height else 0.0

            # B. Holonomic Movement (Drones can slide!)
            # We rotate the global velocity vector into the drone's body frame
            # V_body_x = V_global_x * cos(-yaw) - V_global_y * sin(-yaw)
            c = math.cos(self.current_yaw)
            s = math.sin(self.current_yaw)
            
            # Rotate global vector to match robot frame
            twist.linear.x = velocity_global[0] * c + velocity_global[1] * s
            twist.linear.y = -velocity_global[0] * s + velocity_global[1] * c
            
            # C. Yaw Control (Face the goal)
            twist.angular.z = 1.0 * heading_error

        else:
            # === TURTLEBOT LOGIC (2D Non-Holonomic) ===
            twist.linear.x = velocity_magnitude * math.cos(heading_error)
            twist.linear.y = 0.0 # No sliding
            twist.linear.z = 0.0 # No flying
            twist.angular.z = 2.0 * heading_error

        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    import sys
    if len(sys.argv) < 2:
        print('Usage ros2 run mrta_simulation robot_contoller <robot_id> [--aerial]')
        sys.exit(1)

    robot_id = sys.argv[1]
    is_aerial = '--aerial' in sys.argv

    controller = RobotControllerNode(robot_id, is_aerial)

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()