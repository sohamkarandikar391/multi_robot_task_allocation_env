#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import Twist

class PotentialFieldController:
    def __init__(self, k_att=1.5, k_rep=8.0, d_safe=1, d_influence=3.0, max_vel=0.5, max_acc=2.0, dt = 0.1):
        self.k_att = k_att
        self.k_rep = k_rep
        self.d_safe = d_safe
        self.d_influence = d_influence
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.dt = dt
        self.current_velocity = np.zeros(2)
        self.prev_positions = []
        self.stuck_threshold = 0.05
        self.stuck_history_length = 20


    def compute_attractive_force(self, current_pos, goal_pos):
        diff = goal_pos - current_pos
        distance = np.linalg.norm(diff)

        if distance < 0.01:
            return np.zeros(2)

        force = self.k_att * diff
        return force

    def compute_repulsive_force(self, current_pos, obstacle_pos, is_robot=False):
        diff = current_pos - obstacle_pos
        distance = np.linalg.norm(diff)

        if distance > self.d_influence:
            return np.zeros(2)

        if distance < 0.01:
            distance = 0.01
            diff = np.random.randn(2) * 0.1

        if distance <= self.d_safe:
            magnitude = self.k_rep * (1.0 / self.d_safe - 1.0/self.d_influence) * (1.0 / (distance**2))
        else:
            magnitude = self.k_rep * (1.0 / distance - 1.0/self.d_influence) * (1.0 / (distance**2))

        if is_robot:
            magnitude *= 0.7

        direction = diff/np.linalg.norm(diff)
        force = magnitude*direction

        return force
        return zeros(len(force))

    def compute_velocity_command(self, robot_pos, goal_pos, obstacles, other_robots):
        robot_pos = np.array(robot_pos)
        goal_pos = np.array(goal_pos)

        f_attractive = self.compute_attractive_force(robot_pos, goal_pos)

        f_repulsive_obs = np.zeros(2)

        for obs_pos in obstacles:
            obs_pos = np.array(obs_pos)
            f_repulsive_obs += self.compute_repulsive_force(robot_pos, obs_pos, is_robot=False)

        f_repulsive_robots = np.zeros(2)

        for other_pos in other_robots:
            other_pos = np.array(other_pos)
            f_repulsive_robots += self.compute_repulsive_force(robot_pos, other_pos, is_robot=True)

        f_total = f_attractive + f_repulsive_obs + f_repulsive_robots
        # f_total = f_attractive
        acceleration = f_total

        acc_magnitude = np.linalg.norm(acceleration)
        if acc_magnitude > self.max_acc:
            acceleration = acceleration/acc_magnitude * self.max_acc

        self.current_velocity  = self.current_velocity + acceleration * self.dt

        # damping_factor = 0.9
        # self.current_velocity *= damping_factor

        vel_magnitude = np.linalg.norm(self.current_velocity)
        if vel_magnitude > self.max_vel:
            self.current_velocity = (self.current_velocity / vel_magnitude) * self.max_vel

        self._update_stuck_detection(robot_pos)

        return self.current_velocity.copy()

    def reset_velocity(self):
        self.current_velocity = np.zeros(2)

    def _update_stuck_detection(self, current_pos):

        self.prev_positions.append(current_pos.copy())

        if len(self.prev_positions) > self.stuck_history_length:
            self.prev_positions.pop(0)

    def is_stuck(self):
        if len(self.prev_positions) < self.stuck_history_length:
            return False

        positions = np.array(self.prev_positions)
        variance = np.var(positions, axis=0).sum()

        return variance < self.stuck_threshold

    def get_random_perturbation(self):

        angle = np.random.uniform(0, 2*np.pi)
        magnitude = self.max_vel * 0.5

        return magnitude * np.array([np.cos(angle), np.sin(angle)])

    def create_twist_msg(self, velocity_2d, is_aerial=False):
        twist = Twist()
        if is_aerial:
            twist.linear.x = float(velocity_2d[0])
            twist.linear.y = float(velocity_2d[1])
            twist.linear.z = 0.0

        else:
            twist.linear.x = float(velocity_2d[0])
            twist.linear.y = float(velocity_2d[1])
            twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        return twist