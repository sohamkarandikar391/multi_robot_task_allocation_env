#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
import numpy as np
import json
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from tf2_msgs.msg import TFMessage
from mrta_simulation.potential_field_controller import PotentialFieldController
import math
from rclpy.qos import qos_profile_sensor_data


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
        self.state = "IDLE"
        self.task_duration = 0.0
        self.work_timer = None
        self.current_task_id = None
        self.current_pos = None
        self.current_yaw = 0.0
        self.current_z = 0.0
        self.current_goal = None
        self.at_goal = False
        self.goal_threshold = 1.2  # Tighter threshold

        self.pf_controller = PotentialFieldController(
            k_att=1.5,
            k_rep=15,
            d_safe=0.8,
            d_influence=3.0,
            max_vel=2.5, 
            max_acc=5.0,
            dt=0.1,
            mass=10
        )

        self.load_environment()
        self.other_robot_positions = {}
        
        # Publishers
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

        # Subscribers
        self.task_sub = self.create_subscription(
            String, 
            '/task_assignments',
            self.task_assignment_callback,
            10
        )

        self.gps_sub = self.create_subscription(
            TFMessage,
            f'/{robot_id}/ground_truth',
            self.gps_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            PoseStamped,
            '/robot_positions',
            self.robot_positions_callback,
            10
        )

        self.create_subscription(
            String,
            '/task_triggers',
            self.trigger_callback,
            10
        )

        self.status_pub = self.create_publisher(String, '/task_status', 10)

        # Drone-specific setup
        if self.is_aerial:
            self.enable_pub = self.create_publisher(Bool, f'/{robot_id}/enable', 10)
            self.drone_enabled = False
            self.target_altitude = 2.0
            
            # Altitude PID state
            self.altitude_integral = 0.0
            self.altitude_previous_error = 0.0
            
            # Enable timer - sends enable signal
            self.enable_timer = self.create_timer(0.5, self.enable_drone)
        
        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.broadcast_timer = self.create_timer(0.2, self.broadcast_position)
        
        self._last_log_time = 0.0
        
        self.get_logger().info(f'Robot Controller {robot_id} initialized (aerial={is_aerial})')

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

    def gps_callback(self, msg):
        for t in msg.transforms:
            if "world" not in t.header.frame_id:
                continue
            name = t.child_frame_id
            if (self.robot_id in name or "differential" in name) and ("wheel" not in name) and ("caster" not in name):
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                tz = t.transform.translation.z

                self.current_pos = np.array([tx, ty])
                self.current_z = tz
                
                q = t.transform.rotation
                t3 = +2.0 * (q.w * q.z + q.x * q.y)
                t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self.current_yaw = math.atan2(t3, t4)
                return

    def task_assignment_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data['robot_id'] != self.robot_id: return

            self.current_task_id = data['task_id']
            self.task_duration = data.get('duration', 5)
            
            task_pos = data['task_position']
            self.current_goal = np.array([task_pos[0], task_pos[1]])
            
            
            self.state = "MOVING"
            self.at_goal = False
            
            self.pf_controller.reset_velocity()
            if self.is_aerial:
                self.altitude_integral = 0.0
                self.altitude_previous_error = 0.0

            self.get_logger().info(f'Accepted Task {self.current_task_id} (Duration: {self.task_duration}s)')

        except Exception as e:
            self.get_logger().error(f'Task parse error: {e}')

    def robot_positions_callback(self, msg):
        robot_id = msg.header.frame_id
        if robot_id == self.robot_id:
            return

        pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y
        ])
        self.other_robot_positions[robot_id] = pos

    def trigger_callback(self, msg):
        try:
            data = json.loads(msg.data)
            
            if data['task_id'] == self.current_task_id:
                if data['command'] == 'START':
                    if self.state == "WAITING_FOR_TEAM":
                        self.get_logger().info("Starting work...")
                        self.start_working()
                    elif self.state == "MOVING":
                        self.get_logger().warn("haven't arrived yet!")
                        
        except Exception as e:
            self.get_logger().error(f"Trigger error: {e}")

    def start_working(self):
        self.state = "WORKING"
        
        duration = getattr(self, 'task_duration', 5.0)
        self.get_logger().info(f"Working for {duration} seconds...")
        
        self.work_timer = self.create_timer(duration, self.finish_working)

    def finish_working(self):
        self.get_logger().info(f"Task {self.current_task_id} Complete!")
        
        if self.work_timer:
            self.work_timer.destroy()
            self.work_timer = None
            
        self.send_status("COMPLETED")
        
        self.state = "IDLE"
        self.current_goal = None
        self.at_goal = True 

    def send_status(self, status_str):
        msg = String()
        msg.data = json.dumps({
            'robot_id': self.robot_id,
            'task_id': self.current_task_id,
            'status': status_str
        })
        self.status_pub.publish(msg)

    def broadcast_position(self):
        if self.current_pos is None:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_id
        msg.pose.position.x = float(self.current_pos[0])
        msg.pose.position.y = float(self.current_pos[1])
        msg.pose.position.z = float(self.current_z)

        self.pos_pub.publish(msg)

    def enable_drone(self):
        """Continuously send enable signal to drone"""
        if not self.is_aerial:
            return
            
        msg = Bool()
        msg.data = True
        self.enable_pub.publish(msg)

    def compute_altitude_command(self):
        """
        PID controller for altitude with anti-windup
        Returns: vertical velocity command
        """
        # REDUCED GAINS - Start conservative!
        Kp = 0.8   # Was 1.5
        Ki = 0.05  # Was 0.1
        Kd = 0.3   # Was 0.5
        
        dt = 0.1
        
        # Error
        error = self.target_altitude - self.current_z
        
        # Deadband - ignore tiny errors
        if abs(error) < 0.05:
            error = 0.0
        
        # Integral with anti-windup (clamp)
        self.altitude_integral += error * dt
        self.altitude_integral = max(min(self.altitude_integral, 1.0), -1.0)  # Tighter clamp
        
        # Derivative
        derivative = (error - self.altitude_previous_error) / dt
        self.altitude_previous_error = error
        
        # PID output
        output = Kp * error + Ki * self.altitude_integral + Kd * derivative
        
        # Clamp output to safe range
        MAX_CLIMB_RATE = 0.15  # m/s - REDUCED from 1.0
        output = max(min(output, MAX_CLIMB_RATE), -MAX_CLIMB_RATE)
        
        return output
    
    def check_altitude_safety(self):
        """Emergency stop if altitude goes crazy"""
        if not self.is_aerial:
            return True
        
        MAX_SAFE_ALTITUDE = 5.0
        
        if self.current_z > MAX_SAFE_ALTITUDE:
            self.get_logger().error(
                f"⚠️ EMERGENCY: Altitude {self.current_z:.2f}m exceeds safe limit! STOPPING!"
            )
            # Force stop
            twist = Twist()
            twist.linear.z = -1.0  # Descend rapidly
            self.cmd_vel_pub.publish(twist)
            return False
        
        return True
    
    def control_loop(self):
        # Wait for GPS lock
        twist = Twist()

        if self.current_pos is None:
            self.stop_robot()
            return
        if not self.check_altitude_safety():
            return
        # === IDLE STATE: No Goal ===
        if self.current_goal is None:
            if self.is_aerial:
                # Hover at target altitude
                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = self.compute_altitude_command()
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_robot()
            return
        
        # === GOAL REACHED STATE ===
        distance_to_goal = np.linalg.norm(self.current_goal - self.current_pos)
        
        if distance_to_goal < self.goal_threshold:

            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if self.is_aerial: twist.linear.z = self.compute_altitude_command()
            self.cmd_vel_pub.publish(twist)
            
            # Transition to Waiting
            if self.state == "MOVING":
                self.state = "WAITING_FOR_TEAM"
                self.get_logger().info(f"⏳ Arrived at Task {self.current_task_id}. Waiting for team...")
                self.send_status("ARRIVED")
            
            return
            # if not self.at_goal:
            #     self.get_logger().info(f"✓ GOAL REACHED! Distance: {distance_to_goal:.2f}m")
            #     self.at_goal = True
            #     self.pf_controller.reset_velocity()
            
            # # Stay at goal
            # if self.is_aerial:
            #     twist = Twist()
            #     twist.linear.x = 0.0
            #     twist.linear.y = 0.0
            #     twist.linear.z = self.compute_altitude_command()
            #     twist.angular.z = 0.0
            #     self.cmd_vel_pub.publish(twist)
            # else:
            #     self.stop_robot()
            # return
        
        # === MOVING TO GOAL STATE ===
        
        # Get potential field velocity (2D)
        velocity_global = self.pf_controller.compute_velocity_command(
            robot_pos=self.current_pos,
            goal_pos=self.current_goal,
            obstacles=self.obstacles,
            other_robots=list(self.other_robot_positions.values())
        )

        # Calculate heading
        desired_heading = math.atan2(velocity_global[1], velocity_global[0])
        heading_error = desired_heading - self.current_yaw
        
        # Normalize heading error to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        
        if self.is_aerial:
            # === DRONE CONTROL ===
            
            # 1. Altitude (Z-axis) - PID Controller
            twist.linear.z = self.compute_altitude_command()
            
            # 2. XY Velocity - Transform to body frame
            c = math.cos(self.current_yaw)
            s = math.sin(self.current_yaw)
            
            twist.linear.x = velocity_global[0] * c + velocity_global[1] * s
            twist.linear.y = -velocity_global[0] * s + velocity_global[1] * c
            
            # 3. Yaw control
            Kp_yaw = 1.0
            twist.angular.z = Kp_yaw * heading_error
            
            # Logging
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self._last_log_time > 1.0:
                self.get_logger().info(
                    f"Drone: dist={distance_to_goal:.2f}m, "
                    f"alt={self.current_z:.2f}/{self.target_altitude}m, "
                    f"vel_xy=({twist.linear.x:.2f}, {twist.linear.y:.2f}), "
                    f"vel_z={twist.linear.z:.2f}"
                )
                self._last_log_time = current_time

        else:
            # === TURTLEBOT CONTROL (Non-holonomic) ===
            MAX_SPEED = 0.5
            MAX_TURN_SPEED = 1.0
            GAIN_ANGULAR = 0.8
            
            turn_cmd = GAIN_ANGULAR * heading_error
            twist.angular.z = max(min(turn_cmd, MAX_TURN_SPEED), -MAX_TURN_SPEED)
            
            # Only move forward if facing the goal
            if abs(heading_error) < (math.pi / 2):
                throttle = math.cos(heading_error)
                velocity_magnitude = np.linalg.norm(velocity_global)
                target_speed = min(velocity_magnitude, MAX_SPEED)
                twist.linear.x = target_speed * throttle
            else:
                twist.linear.x = 0.0

            twist.linear.y = 0.0
            twist.linear.z = 0.0
            
            # Logging
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self._last_log_time > 1.0:
                # self.get_logger().info(
                #     f"Turtlebot: dist={distance_to_goal:.2f}m, "
                #     f"heading_error={math.degrees(heading_error):.1f}°, "
                #     f"vel={twist.linear.x:.2f}m/s"
                # )
                self._last_log_time = current_time

        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """Send zero velocity command"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

# class RobotControllerNode(Node):

#     def __init__(self, robot_id, is_aerial=False):
#         super().__init__(f'robot_controller_{robot_id}')

#         self.robot_id = robot_id
#         self.is_aerial = is_aerial

#         self.current_pos = None
#         self.current_goal = None
#         self.at_goal = False
#         self.goal_threshold = 0.3

#         self.pf_controller = PotentialFieldController(
#             k_att = 1.5,
#             k_rep = 8.0,
#             d_safe = 0.8,
#             d_influence = 3.0,
#             max_vel = 0.5 if not is_aerial else 0.8
#         )

#         self.load_environment()
#         self.other_robot_positions = {}
#         self.cmd_vel_pub = self.create_publisher(
#             Twist,
#             f'/{robot_id}/cmd_vel',
#             10
#         )
        
#         self.pos_pub = self.create_publisher(
#             PoseStamped,
#             '/robot_positions',
#             10
#         )

#         # self.odom_sub = self.create_subscription(
#         #     Odometry,
#         #     f'/{robot_id}/odometry',
#         #     self.odom_callback,
#         #     10
#         # )

#         self.task_sub = self.create_subscription(
#             String, 
#             '/task_assignments',
#             self.task_assignment_callback,
#             10
#         )

#         self.gps_sub = self.create_subscription(
#             TFMessage,
#             f'/{robot_id}/ground_truth',
#             self.gps_callback,
#             qos_profile_sensor_data
#         )

#         self.create_subscription(
#             PoseStamped,
#             '/robot_positions',
#             self.robot_positions_callback,
#             10
#         )

#         self.control_timer = self.create_timer(0.1, self.control_loop)
#         self.broadcast_timer = self.create_timer(0.2, self.broadcast_position)
#         self.get_logger().info(f'Robot Controller {robot_id} initialized.')

#         if self.is_aerial:
#             self.enable_pub = self.create_publisher(Bool, f'/{robot_id}/enable', 10)

#             self.create_timer(0.1, self.enable_drone)
#             self.drone_enabled = False


#     def load_environment(self):
#         try:
#             pkg_dir = get_package_share_directory('mrta_simulation')
#             config_path = os.path.join(pkg_dir, 'config', 'scenario_simple.yaml')

#             with open(config_path, 'r') as f:
#                 config = yaml.safe_load(f)

#             self.obstacles = []
#             obstacles_config = config.get('obstacles', {})
#             for obs_name, obs_info in obstacles_config.items():
#                 pos = obs_info['position']
#                 self.obstacles.append([pos[0], pos[1]])

#             self.task_positions = {}
#             for task in config['tasks']:
#                 task_id = task['id']
#                 pos = task['position']
#                 self.task_positions[task_id] = [pos[0], pos[1]]

#             self.get_logger().info(f'Loaded {len(self.obstacles)} obstacles, {len(self.task_positions)} tasks')

#         except Exception as e:
#             self.get_logger().error(f'Failed to load environment: {e}')
#             self.obstacles = []
#             self.task_positions = {}

#     def odom_callback(self, msg):
#         self.current_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
#         self.current_z = msg.pose.pose.position.z  
#         self.current_yaw = get_yaw_from_quaternion(msg.pose.pose.orientation)

#         self.get_logger().info(f'Odom received: {self.current_pos}', throttle_duration_sec=1.0)

#     def gps_callback(self, msg):
        
#         for t in msg.transforms:
#             if "world" not in t.header.frame_id:
#                 continue
#             name = t.child_frame_id
#             if (self.robot_id in name or "differential" in name) and ("wheel" not in name) and ("caster" not in name):
#                 tx = t.transform.translation.x
#                 ty = t.transform.translation.y
#                 tz = t.transform.translation.z

#                 self.current_pos = np.array([tx, ty])
#                 self.current_z = tz
                
#                 q = t.transform.rotation
#                 t3 = +2.0 * (q.w * q.z + q.x * q.y)
#                 t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#                 self.current_yaw = math.atan2(t3, t4)
#                 # self.get_logger().info(f"GPS Lock: {self.current_pos}")
                
#                 return

#     def task_assignment_callback(self, msg):
#         try:
#             data = json.loads(msg.data)

#             if data['robot_id'] != self.robot_id:
#                 return

#             task_id = data['task_id']
#             task_pos = data['task_position']

#             self.current_goal = np.array([task_pos[0], task_pos[1]])
#             self.at_goal = False

#             self.get_logger().info(f'New task assigned: Task {task_id} at {self.current_goal}')

#         except Exception as e:
#             self.get_logger().error(f'Failed to parse task assignment : {e}')

#     def robot_positions_callback(self, msg):
#         robot_id = msg.header.frame_id

#         if robot_id == self.robot_id:
#             return

#         pos = np.array([
#             msg.pose.position.x,
#             msg.pose.position.y
#         ])


#         self.other_robot_positions[robot_id] = pos

#     def broadcast_position(self):
#         if self.current_pos is None:
#             return

#         msg = PoseStamped()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = self.robot_id
#         msg.pose.position.x = float(self.current_pos[0])
#         msg.pose.position.y = float(self.current_pos[1])
#         msg.pose.position.z = 0.0

#         self.pos_pub.publish(msg)

#     def enable_drone(self):
#         msg = Bool()
#         msg.data = True
#         self.enable_pub.publish(msg)

#     def hover_robot(self):
#         """Make drone hover at target altitude"""
#         twist = Twist()
#         twist.linear.x = 0.0
#         twist.linear.y = 0.0
        
#         # Altitude hold
#         current_z = getattr(self, 'current_z', 0.0)
#         target_z = 2.0
        
#         error = target_z - current_z
#         Kp = 2.0
#         twist.linear.z = max(min(Kp * error, 1.0), -1.0)
        
#         twist.angular.x = 0.0
#         twist.angular.y = 0.0
#         twist.angular.z = 0.0
        
#         self.cmd_vel_pub.publish(twist)


#     def control_loop(self):
#         if self.current_pos is None: return
#         if self.current_goal is None or self.at_goal:
#             if self.is_aerial:
#                 self.hover_robot()
#             else:
#                 self.stop_robot()
#             return
#         diff = self.current_goal - self.current_pos
#         distance_to_goal = np.linalg.norm(diff)
#         # 1. Get Global Velocity (2D)
#         velocity_global = self.pf_controller.compute_velocity_command(
#             robot_pos=self.current_pos,
#             goal_pos=self.current_goal,
#             obstacles=self.obstacles,
#             other_robots=list(self.other_robot_positions.values())
#         )

#         # 2. Calculate Heading & Error
#         desired_heading = math.atan2(velocity_global[1], velocity_global[0])
#         heading_error = desired_heading - self.current_yaw
#         while heading_error > math.pi: heading_error -= 2 * math.pi
#         while heading_error < -math.pi: heading_error += 2 * math.pi

#         twist = Twist()
#         velocity_magnitude = np.linalg.norm(velocity_global)
#         STOPPING_DISTANCE = 1.5
#         if distance_to_goal < STOPPING_DISTANCE:
#             if not self.at_goal:
#                 self.get_logger().info(f"✓ GOAL REACHED at {self.current_goal}")
#                 self.at_goal = True
#                 self.pf_controller.reset_velocity()
            
#             if self.is_aerial:
#                 self.hover_robot()
#             else:
#                 self.stop_robot()
#             return

#         if self.is_aerial:
#             TARGET_ALTITUDE = 2.0
#             current_z = getattr(self, 'current_z', 0.0)
#             altitude_error = TARGET_ALTITUDE - current_z
#             Kp_altitude = 2.0
#             twist.linear.z = max(min(Kp_altitude * altitude_error, 1.0), -1.0)
#             c = math.cos(self.current_yaw)
#             s = math.sin(self.current_yaw)
            
#             twist.linear.x = velocity_global[0] * c + velocity_global[1] * s
#             twist.linear.y = -velocity_global[0] * s + velocity_global[1] * c

            

#             Kp_yaw = 1.5
#             twist.angular.z = Kp_yaw * heading_error

#             if not hasattr(self, '_last_log_time'):
#                 self._last_log_time = 0
#             current_time = self.get_clock().now().nanoseconds / 1e9
#             if current_time - self._last_log_time > 1.0:
#                 self.get_logger().info(
#                     f"Drone: dist={distance_to_goal:.2f}m, "
#                     f"alt={current_z:.2f}m, "
#                     f"target_alt={TARGET_ALTITUDE}m, "
#                     f"vel=({twist.linear.x:.2f}, {twist.linear.y:.2f}, {twist.linear.z:.2f})"
#                 )
#                 self._last_log_time = current_time

#         else:
#             # === TURTLEBOT LOGIC (2D Non-Holonomic) ===
#             MAX_SPEED = 0.5
#             MAX_TURN_SPEED = 1.0
#             GAIN_ANGULAR = 0.8
#             turn_cmd = GAIN_ANGULAR * heading_error
#             twist.angular.z = max(min(turn_cmd, MAX_TURN_SPEED), -MAX_TURN_SPEED)
#             if abs(heading_error) < (math.pi / 2):
#                 # We are facing generally forward. Drive!
#                 throttle = math.cos(heading_error)
                
#                 # Use the Potential Field magnitude, but cap it at MAX_SPEED
#                 target_speed = min(velocity_magnitude, MAX_SPEED)
#                 twist.linear.x = target_speed * throttle
#             else:
#                 # We are facing backwards. STOP driving and just Turn.
#                 # This prevents the "Reverse Spiral" of death.
#                 twist.linear.x = 0.0

#             twist.linear.y = 0.0 # No sliding
#             twist.linear.z = 0.0 # No 

#         self.cmd_vel_pub.publish(twist)

#     def stop_robot(self):
#         twist = Twist()
#         twist.linear.x = 0.0
#         twist.linear.y = 0.0
#         twist.angular.z = 0.0

#         self.cmd_vel_pub.publish(twist)


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