#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time
from tf_transformations import euler_from_quaternion

class LocalPlanner(Node):
    def __init__(self):
        super().__init__('local_planner')
        self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/path_points', self.path_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.k_a = 1.0
        self.k_r = 0.05
        self.k_ro = 1.0
        self.rho_0 = 1.0
        self.position_x = None
        self.position_y = None
        self.orientation = None
        self.path_worldframe = None
        self.path_index = 0
        self.goal_pos_threshold = 0.25
        self.distance_threshold = 0.4
        self.speed_up_lim = 0.25
        self.speed_low_lim = 0.1
        self.repulsive_speed_lim = -0.3
        self.rotate_speed = 0.1
        self.is_adjusting_orientation = False
        self.last_scan = None
        self.goal_orientation = None
        self.rotation_threshold = 0.1

    def laser_callback(self, msg):
        self.last_scan = msg

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.position_x = pos.x
        self.position_y = pos.y
        _, _, self.orientation = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

    def path_callback(self, msg):
        self.path_worldframe = [(pose.position.x, pose.position.y) for pose in msg.poses]
        self.path_index = 0

    def follow_path(self):
        if self.is_adjusting_orientation:
            self.adjust_final_orientation()
            return

        if self.path_worldframe is None:
            return

        if self.path_index >= len(self.path_worldframe):
            self.is_adjusting_orientation = True
            self.adjust_final_orientation()
            return

        goal_x, goal_y = self.path_worldframe[self.path_index]
        angle_diff = self.calculate_angular_difference(goal_x, goal_y)

        if self.path_index == len(self.path_worldframe) - 1:
            dist = math.sqrt((self.position_x - goal_x)**2 + (self.position_y - goal_y)**2)
            if dist < self.goal_pos_threshold:
                self.path_index += 1
                return
        else:
            dist = math.sqrt((self.position_x - goal_x)**2 + (self.position_y - goal_y)**2)
            if dist < self.distance_threshold:
                self.path_index += 1
                return

        attraction_x, attraction_y = self.calculate_attractive_force(goal_x, goal_y)

        if self.last_scan:
            repulsion_x, repulsion_y, repulsion_angular = self.calculate_repulsive_forces(self.last_scan)
        else:
            repulsion_x, repulsion_y, repulsion_angular = 0.0, 0.0, 0.0

        if abs(attraction_y) > abs(attraction_x):
            total_force_x = abs(attraction_y) + repulsion_x
        else:
            total_force_x = abs(attraction_x) + repulsion_x
        
        total_force_y = repulsion_y
        angular_force = self.calculate_angular_correction(goal_x, goal_y) + repulsion_angular

        if total_force_x > self.speed_up_lim:
            total_force_x = self.speed_up_lim
        elif 0.0 < total_force_x < self.speed_low_lim:
            total_force_x = self.speed_low_lim
        elif total_force_x < self.repulsive_speed_lim:
            total_force_x = self.repulsive_speed_lim

        twist = Twist()
        if abs(angle_diff) > 0.785:
            twist.linear.x = 0.0
            twist.linear.y = total_force_y
            twist.angular.z = 0.6 * angular_force
        else:
            twist.linear.x = total_force_x
            twist.linear.y = total_force_y
            twist.angular.z = angular_force

        self.vel_publisher.publish(twist)

    def calculate_attractive_force(self, goal_x, goal_y):
        force_x = self.k_a * (goal_x - self.position_x)
        force_y = self.k_a * (goal_y - self.position_y)
        return force_x, force_y

    def calculate_repulsive_forces(self, msg):
        force_x, force_y, angular_force = 0.0, 0.0, 0.0
        min_force_threshold = 0.1
        
        for i, distance in enumerate(msg.ranges):
            if distance > self.rho_0 or distance == float('inf') or math.isnan(distance):
                continue

            intensity = (self.rho_0 - distance) / self.rho_0
            angle = msg.angle_min + i * msg.angle_increment

            obstacle_x = distance * math.cos(angle)
            obstacle_y = distance * math.sin(angle)
            dist = max(distance, 0.01)

            repulsion = self.k_r * intensity
            force_x += repulsion * (-obstacle_x / dist)
            force_y += repulsion * (-obstacle_y / dist)

            ang_contrib = - (intensity * (i - len(msg.ranges)/2) /
                             (len(msg.ranges)/2))
            angular_force += ang_contrib * repulsion

        if abs(force_x) < min_force_threshold:
            force_x = min_force_threshold * (1 if force_x >= 0 else -1)
        if abs(force_y) < min_force_threshold:
            force_y = min_force_threshold * (1 if force_y >= 0 else -1)

        return (force_x, force_y, angular_force)

    def calculate_angular_correction(self, goal_x, goal_y):
        goal_dir = math.atan2(goal_y - self.position_y, goal_x - self.position_x)
        angle_diff = self.normalize_angle(goal_dir - self.orientation)
        return self.k_ro * angle_diff

    def calculate_angular_difference(self, goal_x, goal_y):
        goal_dir = math.atan2(goal_y - self.position_y, goal_x - self.position_x)
        angle_diff = self.normalize_angle(goal_dir - self.orientation)
        return angle_diff

    def adjust_final_orientation(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0

        angle_diff = abs(self.normalize_angle(self.goal_orientation - self.orientation))
        if angle_diff > self.rotation_threshold:
            self.final_rotation_direction = self.determine_rotation_direction(self.goal_orientation)
            if self.final_rotation_direction == "right":
                twist.angular.z = -self.rotate_speed
            else:
                twist.angular.z = self.rotate_speed
        else:
            twist.angular.z = 0.0
            self.path_index = 0
            self.path_worldframe = None
            self.is_adjusting_orientation = False
            self.get_logger().info('Goal reached !!!')

        self.vel_publisher.publish(twist)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def determine_rotation_direction(self, goal_orientation):
        curr = self.normalize_angle(self.orientation)
        goal = self.normalize_angle(goal_orientation)
        diff = self.normalize_angle(goal - curr)
        return "left" if diff > 0 else "right"

def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()

    while rclpy.ok():
        rclpy.spin_once(node)
        node.follow_path()
        time.sleep(0.01)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()