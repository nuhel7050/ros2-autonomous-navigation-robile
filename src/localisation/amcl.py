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

        for particle in self.particles:
            particle.x += self.odom_pose[0] + random.gauss(0, 0.1)
            particle.y += self.odom_pose[1] + random.gauss(0, 0.1)
            particle.theta += self.odom_pose[2] + random.gauss(0, 0.1)
            particle.weight = self.calculate_weight(particle)

    def calculate_weight(self, particle):
        """Calculate the weight of a particle based on the laser scan data."""
        if self.map_data is None or self.laser_scan is None:
            return 1.0

        weight = 1.0
        for i, distance in enumerate(self.laser_scan.ranges):
            if distance == float('inf') or math.isnan(distance):
                continue

            angle = self.laser_scan.angle_min + i * self.laser_scan.angle_increment + particle.theta
            x = particle.x + distance * math.cos(angle)
            y = particle.y + distance * math.sin(angle)

            map_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            map_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

            if 0 <= map_x < self.map_data.shape[1] and 0 <= map_y < self.map_data.shape[0]:
                if self.map_data[map_y, map_x] == 100:
                    weight *= 0.1
                else:
                    weight *= 0.9

        return weight

    def resample_particles(self):
        """Resample particles based on their weights."""
        weights = [particle.weight for particle in self.particles]
        new_particles = random.choices(self.particles, weights, k=self.num_particles)
        self.particles = [Particle(p.x, p.y, p.theta, 1.0 / self.num_particles) for p in new_particles]

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