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
    """
    Local Planner for autonomous mobile robot navigation.
    
    Implements a potential field approach for local path following with 
    obstacle avoidance using laser scan data and odometry feedback.
    """
    
    def __init__(self):
        """Initialize the local planner node with subscribers, publisher, and parameters."""
        super().__init__('local_planner')
        
        # Initialize subscribers and publisher
        self._init_communication()
        
        # Initialize control parameters
        self._init_control_params()
        
        # Initialize state variables
        self._init_state()

    def _init_communication(self):
        """Initialize subscribers and publisher."""
        # Subscribers
        self.create_subscription(LaserScan, 'scan', self._laser_callback, 10)
        self.create_subscription(Odometry, 'odom', self._odom_callback, 10)
        self.create_subscription(PoseArray, '/path_points', self._path_callback, 10)
        
        # Publisher
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def _init_control_params(self):
        """Initialize control parameters for potential field navigation."""
        # Force field parameters
        self.k_a = 1.0            # Attractive force gain
        self.k_r = 0.05           # Repulsive force gain
        self.k_ro = 1.0           # Angular correction gain
        self.rho_0 = 1.0          # Obstacle influence threshold
        
        # Distance thresholds
        self.goal_pos_threshold = 0.25    # Final goal position threshold
        self.distance_threshold = 0.4     # Waypoint distance threshold
        
        # Speed limits
        self.speed_up_lim = 0.25          # Maximum forward speed
        self.speed_low_lim = 0.1          # Minimum forward speed
        self.repulsive_speed_lim = -0.3   # Maximum backward speed
        self.rotate_speed = 0.1           # Rotation speed
        
        # Angular threshold
        self.rotation_threshold = 0.1     # Final orientation threshold

    def _init_state(self):
        """Initialize state variables."""
        # Robot pose
        self.position_x = None
        self.position_y = None
        self.orientation = None
        
        # Path tracking
        self.path_worldframe = None
        self.path_index = 0
        self.is_adjusting_orientation = False
        self.goal_orientation = None
        
        # Sensor data
        self.last_scan = None

    # Callback methods
    def _laser_callback(self, msg):
        """Store the latest laser scan data."""
        self.last_scan = msg

    def _odom_callback(self, msg):
        """Update robot pose from odometry data."""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.position_x = pos.x
        self.position_y = pos.y
        _, _, self.orientation = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

    def _path_callback(self, msg):
        """Update path waypoints from received path message."""
        self.path_worldframe = [(pose.position.x, pose.position.y) for pose in msg.poses]
        self.path_index = 0

    # Path following methods
    def follow_path(self):
        """Main path following control loop."""
        # Check if robot is in final orientation adjustment phase
        if self.is_adjusting_orientation:
            self._adjust_final_orientation()
            return

        # Check if path exists
        if self.path_worldframe is None:
            return

        # Check if path is completed
        if self.path_index >= len(self.path_worldframe):
            self.is_adjusting_orientation = True
            self._adjust_final_orientation()
            return

        # Get current goal from path
        goal_x, goal_y = self.path_worldframe[self.path_index]
        
        # Calculate angular difference to goal
        angle_diff = self._calculate_angular_difference(goal_x, goal_y)

        # Check if goal is reached
        if self._check_goal_reached(goal_x, goal_y):
            return

        # Calculate forces
        attraction_x, attraction_y = self._calculate_attractive_force(goal_x, goal_y)
        
        # Include obstacle avoidance if scan data available
        if self.last_scan:
            repulsion_x, repulsion_y, repulsion_angular = self._calculate_repulsive_forces(self.last_scan)
        else:
            repulsion_x, repulsion_y, repulsion_angular = 0.0, 0.0, 0.0

        # Determine total forces
        total_force_x = self._calculate_total_force_x(attraction_x, attraction_y, repulsion_x)
        total_force_y = repulsion_y
        angular_force = self._calculate_angular_correction(goal_x, goal_y) + repulsion_angular

        # Apply speed limits
        total_force_x = self._apply_speed_limits(total_force_x)

        # Create and publish velocity command
        self._publish_velocity_command(total_force_x, total_force_y, angular_force, angle_diff)

    def _check_goal_reached(self, goal_x, goal_y):
        """Check if current goal waypoint is reached and update path index if needed."""
        dist = math.sqrt((self.position_x - goal_x)**2 + (self.position_y - goal_y)**2)
        
        # Use different thresholds for final goal vs waypoints
        if self.path_index == len(self.path_worldframe) - 1:
            # Final goal
            if dist < self.goal_pos_threshold:
                self.path_index += 1
                return True
        else:
            # Intermediate waypoint
            if dist < self.distance_threshold:
                self.path_index += 1
                return True
                
        return False

    def _calculate_total_force_x(self, attraction_x, attraction_y, repulsion_x):
        """Determine the combined forward force considering attraction and repulsion."""
        # Use the dominant attractive component (x or y)
        if abs(attraction_y) > abs(attraction_x):
            total_force_x = abs(attraction_y) + repulsion_x
        else:
            total_force_x = abs(attraction_x) + repulsion_x
            
        return total_force_x

    def _apply_speed_limits(self, force_x):
        """Apply speed limits to the calculated force."""
        if force_x > self.speed_up_lim:
            return self.speed_up_lim
        elif 0.0 < force_x < self.speed_low_lim:
            return self.speed_low_lim
        elif force_x < self.repulsive_speed_lim:
            return self.repulsive_speed_lim
        return force_x

    def _publish_velocity_command(self, force_x, force_y, angular_force, angle_diff):
        """Create and publish velocity command based on calculated forces."""
        twist = Twist()
        
        # If angle to goal is large, prioritize rotation over translation
        if abs(angle_diff) > 0.785:  # ~45 degrees
            twist.linear.x = 0.0
            twist.linear.y = force_y
            twist.angular.z = 0.6 * angular_force
        else:
            twist.linear.x = force_x
            twist.linear.y = force_y
            twist.angular.z = angular_force

        self.vel_publisher.publish(twist)

    # Force calculation methods
    def _calculate_attractive_force(self, goal_x, goal_y):
        """Calculate attractive force toward goal position."""
        force_x = self.k_a * (goal_x - self.position_x)
        force_y = self.k_a * (goal_y - self.position_y)
        return force_x, force_y

    def _calculate_repulsive_forces(self, scan):
        """Calculate repulsive forces from obstacles detected by laser scan."""
        force_x, force_y, angular_force = 0.0, 0.0, 0.0
        min_force_threshold = 0.1
        
        # Process each laser scan point
        for i, distance in enumerate(scan.ranges):
            # Skip invalid readings or beyond influence threshold
            if distance > self.rho_0 or distance == float('inf') or math.isnan(distance):
                continue

            # Calculate repulsive force intensity
            intensity = (self.rho_0 - distance) / self.rho_0
            angle = scan.angle_min + i * scan.angle_increment

            # Convert polar to cartesian coordinates
            obstacle_x = distance * math.cos(angle)
            obstacle_y = distance * math.sin(angle)
            dist = max(distance, 0.01)  # Avoid division by zero

            # Calculate force components
            repulsion = self.k_r * intensity
            force_x += repulsion * (-obstacle_x / dist)
            force_y += repulsion * (-obstacle_y / dist)

            # Calculate angular contribution
            ang_contrib = -(intensity * (i - len(scan.ranges)/2) / (len(scan.ranges)/2))
            angular_force += ang_contrib * repulsion

        # Apply minimum thresholds to ensure responsiveness
        if abs(force_x) < min_force_threshold:
            force_x = min_force_threshold * (1 if force_x >= 0 else -1)
        if abs(force_y) < min_force_threshold:
            force_y = min_force_threshold * (1 if force_y >= 0 else -1)

        return force_x, force_y, angular_force

    def _calculate_angular_correction(self, goal_x, goal_y):
        """Calculate angular correction to align robot with goal direction."""
        goal_dir = math.atan2(goal_y - self.position_y, goal_x - self.position_x)
        angle_diff = self._normalize_angle(goal_dir - self.orientation)
        return self.k_ro * angle_diff

    def _calculate_angular_difference(self, goal_x, goal_y):
        """Calculate angular difference between robot heading and goal direction."""
        goal_dir = math.atan2(goal_y - self.position_y, goal_x - self.position_x)
        angle_diff = self._normalize_angle(goal_dir - self.orientation)
        return angle_diff

    # Final orientation adjustment
    def _adjust_final_orientation(self):
        """Adjust the robot's orientation to match the goal orientation."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0

        # Check if orientation is within threshold
        angle_diff = abs(self._normalize_angle(self.goal_orientation - self.orientation))
        if angle_diff > self.rotation_threshold:
            # Determine rotation direction
            final_rotation_direction = self._determine_rotation_direction(self.goal_orientation)
            if final_rotation_direction == "right":
                twist.angular.z = -self.rotate_speed
            else:
                twist.angular.z = self.rotate_speed
        else:
            # Goal orientation reached, reset state
            twist.angular.z = 0.0
            self.path_index = 0
            self.path_worldframe = None
            self.is_adjusting_orientation = False
            self.get_logger().info('Goal reached !!!')

        self.vel_publisher.publish(twist)

    # Utility methods
    def _normalize_angle(self, angle):
        """Normalize angle to range [-π, π]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _determine_rotation_direction(self, goal_orientation):
        """Determine shortest rotation direction to reach goal orientation."""
        curr = self._normalize_angle(self.orientation)
        goal = self._normalize_angle(goal_orientation)
        diff = self._normalize_angle(goal - curr)
        return "left" if diff > 0 else "right"


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = LocalPlanner()

    # Main control loop
    while rclpy.ok():
        rclpy.spin_once(node)
        node.follow_path()
        time.sleep(0.01)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()