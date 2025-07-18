#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
import random
import math
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class Particle:
    def __init__(self, x, y, theta, weight):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

class AMCL(Node):
    def __init__(self):
        super().__init__('amcl')
        self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.pose_publisher = self.create_publisher(PoseArray, 'particle_cloud', 10)
        self.estimated_pose_publisher = self.create_publisher(PoseStamped, 'estimated_pose', 10)

        self.particles = []
        self.num_particles = 100
        self.map_data = None
        self.odom_pose = None
        self.laser_scan = None
        self.init_particles()

    def init_particles(self):
        """Initialize particles with random positions and orientations."""
        for _ in range(self.num_particles):
            x = random.uniform(-2, 2)
            y = random.uniform(-2, 2)
            theta = random.uniform(-math.pi, math.pi)
            weight = 1.0 / self.num_particles
            self.particles.append(Particle(x, y, theta, weight))

    def map_callback(self, msg):
        """Callback function to handle map data."""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def odom_callback(self, msg):
        """Callback function to handle odometry data."""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.odom_pose = (pos.x, pos.y, theta)

    def laser_callback(self, msg):
        """Callback function to handle laser scan data."""
        self.laser_scan = msg
        self.update_particles()
        self.resample_particles()
        self.publish_particles()
        self.publish_estimated_pose()

    def update_particles(self):
        """Update particles based on odometry and laser scan data."""
        if self.odom_pose is None or self.laser_scan is None:
            return

        '''
        Else case:
        1. Update particle positions based on odometry data
        2. Calculate the weight of each particle based on the laser scan data
        '''

    def calculate_weight(self, particle):
        """Calculate the weight of a particle based on the laser scan data."""
        if self.map_data is None or self.laser_scan is None:
            return 1.0

        weight = 1.0
        '''
        Calculate the weight of the particle based on the laser scan data
        Need to implement this:
        FUNCTION calculate_weight(particle):
            IF map_data IS None OR laser_scan IS None:
                RETURN 1.0

            SET weight TO 1.0

            FOR EACH distance IN laser_scan.ranges:
                IF distance IS infinity OR distance IS NaN:
                    CONTINUE

                SET x,y,ANGLE tO the position of the laser scan point in the map frame

                IF map_x IS within map bounds AND map_y IS within map bounds:
                    IF map_data[map_y, map_x] IS 100:
                        SET weight TO weight * 0.1
                    ELSE:
                        SET weight TO weight * 0.9

            RETURN weight
        '''

    def resample_particles(self):
        """Resample particles based on their weights."""
        weights = [particle.weight for particle in self.particles]
        '''
        Need to implement this:
        FUNCTION resample_particles():
            SET weights TO a list of weights of all particles

            SET new_particles TO an empty list

            FOR i FROM 0 TO num_particles:
                SET particle TO a random particle based on the weights
                APPEND particle TO new_particles

            SET self.particles TO new_particles
        '''

    def publish_particles(self):
        """Publish the particle cloud."""
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"

        for particle in self.particles:
            pose = Pose()
            pose.position.x = particle.x
            pose.position.y = particle.y
            pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, particle.theta)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            pose_array.poses.append(pose)

        self.pose_publisher.publish(pose_array)

    def publish_estimated_pose(self):
        """Publish the estimated pose of the robot."""
        x = np.mean([p.x for p in self.particles])
        y = np.mean([p.y for p in self.particles])
        theta = np.mean([p.theta for p in self.particles])

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, theta)
        pose_stamped.pose.orientation.x = q[0]
        pose_stamped.pose.orientation.y = q[1]
        pose_stamped.pose.orientation.z = q[2]
        pose_stamped.pose.orientation.w = q[3]

        self.estimated_pose_publisher.publish(pose_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = AMCL()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()