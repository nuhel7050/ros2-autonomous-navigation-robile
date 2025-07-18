#!/usr/bin/env python3

# =========================
#         IMPORTS
# =========================
import rclpy
from rclpy.node import Node

# Messages for geometry, sensor, and navigation
from geometry_msgs.msg import Twist, PoseStamped, PoseArray, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path

# ROS 2 QoS settings
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Math and other Python libraries
import math
import sys
import time
import numpy as np
import heapq
from math import atan2, sqrt, pi
from tf_transformations import euler_from_quaternion

# For map inflation
from scipy.ndimage import binary_dilation
from skimage.morphology import disk

# Configure printing options for numpy
np.set_printoptions(threshold=sys.maxsize)


# =========================
#    FIRST CLASS
# =========================
class PotentialFieldPathPlanning(Node):
    def __init__(self):
        super().__init__('potential_node')

        """
        Subscriptions for LaserScan and Odometry data.
        - Subscribes to laser scan information ('scan') to gather details about obstacles.
        - Subscribes to odometry data ('odom') to determine the robot's current position.
        """
        self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        """
        Publishers for velocity commands and goal pose.
        - Publishes velocity commands ('cmd_vel') to move the robot.
        - Publishes the goal pose ('goal_pose') to relay the target destination.
        """
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)

        """
        Constants for the potential field method:
        - k_a: Attractive force constant (guides the robot toward the goal).
        - k_r: Repulsive force constant (pushes the robot away from obstacles).
        - rho_0: Threshold distance for repulsive force activation.
        """
        self.k_a = 1.8   # Attractive force constant
        self.k_r = 3.0   # Repulsive force constant
        self.rho_0 = 1.5 # Threshold distance for repulsive force

        self.position_x = None
        self.position_y = None
        self.orientation_z = None
        self.orientation_w = None

        self.max_linear_velocity = 3.0   # Maximum linear velocity
        self.max_angular_velocity = 6.0  # Maximum angular velocity
        self.goal_pose = PoseStamped()

    def laser_callback(self, msg):
        """Callback for handling incoming LaserScan messages."""
        self.get_logger().info('LaserScan received!')
        # TODO: Incorporate logic here for obstacle avoidance using LaserScan data

    def odom_callback(self, msg):
        """Callback for receiving incoming Odometry messages."""
        self.get_logger().info('Odometry data received!')
        # TODO: Update self.position_x, self.position_y, self.orientation_z, etc., from msg
    
    def send_goal(self, goal_x, goal_y):
        """Publishes a desired goal for the robot to travel toward."""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'odom'
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        # Adjust orientation if desired, e.g., goal_msg.pose.orientation.w = 1.0
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Goal sent: x={goal_x}, y={goal_y}")


# =========================
#    SECOND CLASS
# =========================
class OdometryMotionModel(Node):
    def __init__(self):
        """Initializes the OdometryMotionModel node."""
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=5
        )
        super().__init__('odometry_motion_model')

        # Subscriptions
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_profile
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        # self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10) # Update Datatype if needed
        self.goal_subscriber = self.create_subscription(
            PoseStamped, '/goalpose', self.goal_pose_callback, 10
        )
        self.laser_subscriber = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10
        )
        
        # Publishers
        self.path_publisher = self.create_publisher(Path, '/path_line', 10)
        self.path_point_publisher = self.create_publisher(PoseArray, '/path_points', 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initial Variables
        self.current_pos_x, self.current_pos_y, self.orientation = 0.0, 0.0, 0.0
        self.goal_x, self.goal_y, self.goal_orientation = None, None, None
        self.map_data = None
        self.grid = None
        self.path = None
        self.update_grid = None
        self.path_worldframe = None
        self.path_index = 0  # Current waypoint index in the path
        # self.threshold = 10  # Adjusted threshold for map padding (Simulation)
        self.threshold = 2   # Adjusted threshold for map padding (Real)
        # self.max_distance = 0.5 # A* replanning and obstacle avoidance start (Simulation)
        self.max_distance = 0.5 # A* replanning and obstacle avoidance start (Real)

        # Movement Parameters
        self.move_speed = 3.0
        self.rotation_threshold = 0.1    # radians (~2.86 degrees)
        self.goal_pos_threshold = 0.25   # meters
        self.goal_orientation_threshold = 0.05  # radians, ~3 degrees

        # Thresholds for path following
        self.distance_threshold = 0.4
        self.orientation_threshold = 0.1  # Threshold for orientation checks

        # Potential Field Constants
        self.k_a = 1.0   # Attractive force constant
        self.k_r = 0.05  # Repulsive force constant
        self.k_ro = 1.0  # Rotation factor for facing goal direction
        self.rho_0 = 1.0 # Distance threshold for obstacles
        self.last_scan = None

        # Speed Limits
        # self.speed_up_lim = 0.5
        # self.speed_low_lim = 0.2
        # self.repulsive_speed_lim = -0.5
        # self.rotate_speed = 1.0
        self.speed_up_lim = 0.25
        self.speed_low_lim = 0.1
        self.repulsive_speed_lim = -0.3
        self.rotate_speed = 0.1

        # Replanning Control
        self.last_replan_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.replan_interval = 1.0  # seconds

        # Orientation Adjustment State
        self.is_adjusting_orientation = False  # New state variable

    ########################## Call_Back Functions Start ###############################
    def map_callback(self, msg):
        """Callback for '/map' topic. Processes incoming map data and inflates obstacles."""
        print("Received Map data...")
        self.map_data = {
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
            'origin': {
                'x': msg.info.origin.position.x,
                'y': msg.info.origin.position.y,
                'z': msg.info.origin.position.z
            },
            'data': msg.data
        }
        self.grid = np.array(self.map_data['data']).reshape(
            self.map_data['height'], self.map_data['width']
        )

        # Inflate obstacles
        structure = disk(self.threshold)  
        binary_occupancy_grid = np.where(self.grid == 100, 1, 0)
        dilated_grid = binary_dilation(binary_occupancy_grid, structure=structure)
        self.grid = np.where(dilated_grid == 1, 100, 0)  # Convert back to occupancy format
        print(f"Dilated Grid: \n{self.grid}")
        self.get_logger().info('Map data updated with threshold.')
        self.update_grid = np.copy(self.grid)

    def odom_callback(self, msg):
        """Callback for '/odom' topic. Updates robot's position and orientation."""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.current_pos_x = pos.x
        self.current_pos_y = pos.y
        _, _, self.orientation = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

    def laser_callback(self, msg):
        """Callback for '/scan' topic. Updates the latest LIDAR data."""
        self.last_scan = msg  # Store the scan data

    def goal_pose_callback(self, msg):
        """Callback for '/goal_pose' topic. Receives goal position and plans the path."""
        if self.map_data is None:
            self.get_logger().info('GoalPose_CB: Map data is not yet available.')
            return

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        _, _, self.goal_orientation = euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

        start_pos = self.map_to_grid(self.current_pos_x, self.current_pos_y, self.map_data, "Start")
        goal_pos = self.map_to_grid(self.goal_x, self.goal_y, self.map_data, "Goal")
        path = self.a_star_search(start_pos, goal_pos, self.grid)

        # Simplify path and update self.path
        self.path = self.simplify_path(path)
        self.path_worldframe = self.grid_to_map(self.path, self.map_data)
        self.publish_path_points(self.path_worldframe)
        self.publish_path(self.path_worldframe)
        self.get_logger().info(f'A* Path in grid: {self.path}')
        self.get_logger().info(f'A* Path in world frame: {self.path_worldframe}')

    ########################## Call_Back Functions End ###############################
    ######################## User-Defined Functions Start ##################################
    def simplify_path(self, path):
        """Removes unnecessary waypoints from the path."""
        if not path or len(path) < 3:
            return path

        simplified_path = [path[0]]
        for i in range(1, len(path) - 1):
            prev = path[i - 1]
            current = path[i]
            nxt = path[i + 1]

            # If there's a shift in x or y, or a change in direction
            if not ((prev[0] == current[0] == nxt[0]) or (prev[1] == current[1] == nxt[1])):
                simplified_path.append(current)

        simplified_path.append(path[-1])
        return simplified_path

    def further_simplify_path(self, path, segment_step=5):
        """Segments and selects every nth point for further path simplification."""
        if not path or len(path) < 3:
            return path

        final_path = [path[0]]
        segment = [path[0]]

        for i in range(1, len(path)):
            prev = path[i - 1]
            current = path[i]

            # If current point is roughly one step away
            if abs(current[0] - prev[0]) <= 1 and abs(current[1] - prev[1]) <= 1:
                segment.append(current)
            else:
                if len(segment) > 1:
                    segment_simplified = [segment[j] for j in range(0, len(segment), segment_step)]
                    if segment[-1] not in segment_simplified:
                        segment_simplified.append(segment[-1])
                    final_path.extend(segment_simplified)
                else:
                    final_path.extend(segment)
                segment = [current]

        # Handle the last segment
        if len(segment) > 1:
            segment_simplified = [segment[j] for j in range(0, len(segment), segment_step)]
            if segment[-1] not in segment_simplified:
                segment_simplified.append(segment[-1])
            final_path.extend(segment_simplified)
        else:
            final_path.extend(segment)

        final_path.append(path[-1])
        return final_path

    def map_to_grid(self, x, y, map_data, msg):
        """Transforms world coordinates to grid indices."""
        mx = int(round((x - map_data['origin']['x']) / map_data['resolution']))
        my = int(round((y - map_data['origin']['y']) / map_data['resolution']))
        self.get_logger().info(f"Position of robot for {msg} on map: x: {mx}, y: {my}")
        return (mx, my)

    def grid_to_map(self, path_, map_data):
        """Converts grid indices back to world coordinates."""
        path_new = []
        origin_x = map_data['origin']['x']
        origin_y = map_data['origin']['y']
        resolution = map_data['resolution']
        for i in range(len(path_)):
            wx = origin_x + path_[i][0] * resolution
            wy = origin_y + path_[i][1] * resolution
            path_new.append((wx, wy))
        return path_new

    ######################## User-Defined Functions End ##################################
    ############################# A star Algorithm Start ####################################
    def a_star_search(self, start, goal, grid):
        """Uses the A* algorithm for path finding."""
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while open_set:
            current = heapq.heappop(open_set)[1]
            if current == goal:
                break

            for nxt in self.get_neighbors(current, grid):
                new_cost = cost_so_far[current] + self.heuristic(current, nxt)
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + self.heuristic(goal, nxt)
                    heapq.heappush(open_set, (priority, nxt))
                    came_from[nxt] = current

        print(f"came_from len = {len(came_from)}")
        print(f"came_from = {came_from}")
        return self.reconstruct_path(came_from, start, goal)

    def heuristic(self, a, b):
        """Heuristic using Euclidean distance."""
        return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def get_neighbors(self, pos, grid):
        """Retrieves valid neighboring cells for A* exploration."""
        directions = [(-1, -1), (-1, 0), (-1, 1),
                      (0, -1),         (0, 1),
                      (1, -1),  (1, 0),  (1, 1)]
        neighbors = []
        x, y = pos
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]:
                if grid[ny, nx] != 100:
                    neighbors.append((nx, ny))
        return neighbors

    def reconstruct_path(self, came_from, start, goal):
        """Reconstructs the path found by A*."""
        current = goal
        path = []
        print(f"current = {current}, start = {start}")
        while current != start:
            path.append(current)
            current = came_from.get(current)
        path.append(start)
        path.reverse()
        return path

    ############################# A star Algorithm End ####################################
    ###################### Publishing Data start #######################################
    def publish_path_points(self, path):
        """Publishes the path as a PoseArray."""
        if not path:
            self.get_logger().info('No path to publish.')
            return

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"

        for (x, y) in path:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.path_point_publisher.publish(pose_array)
        self.get_logger().info('Path published successfully.')

    def publish_path(self, path):
        """Publishes the path as a Path message."""
        if not path:
            self.get_logger().info('No path to publish.')
            return

        path_msg = Path()
        current_time = self.get_clock().now().to_msg()
        path_msg.header.stamp = current_time
        path_msg.header.frame_id = "map"

        for (x, y) in path:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = current_time
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)

        self.path_publisher.publish(path_msg)
        self.get_logger().info('Path published successfully.')
    ###################### Publishing Data End #######################################
    ###################### Potential Field Algorithm Start #######################################
    def update_map_with_lidar(self):
        """Updates the map with new LIDAR-detected obstacles and potentially replans."""
        
        if self.map_data is None or self.last_scan is None:
            self.get_logger().info('Map or Lidar data not ready for local planning.')
            return

        # Convert robot pos to grid
        robot_pos = self.map_to_grid(self.current_pos_x, self.current_pos_y, self.map_data, "Robot")

        # Insert obstacles
        new_obstacle_positions = set()

        for i, distance in enumerate(self.last_scan.ranges):
            if distance < self.max_distance or distance == float('inf') or math.isnan(distance):
                continue

            angle = self.last_scan.angle_min + i * self.last_scan.angle_increment

            obstacle_x = robot_pos[0] + int(distance * math.cos(
                angle + self.orientation) / self.map_data['resolution'])
            obstacle_y = robot_pos[1] + int(distance * math.sin(
                angle + self.orientation) / self.map_data['resolution'])

            if 0 <= obstacle_x < self.update_grid.shape[1] and 0 <= obstacle_y < self.update_grid.shape[0]:
                new_obstacle_positions.add((obstacle_x, obstacle_y))
                self.update_grid[obstacle_y, obstacle_x] = 100

        # Remove old obstacles not in new set
        for y in range(self.update_grid.shape[0]):
            for x in range(self.update_grid.shape[1]):
                if self.update_grid[y, x] == 100 and (x, y) not in new_obstacle_positions:
                    self.update_grid[y, x] = 0

        # Inflate
        struct = disk(self.threshold)
        binary_occupancy_grid = np.where(self.update_grid == 100, 1, 0)
        dilated_grid = binary_dilation(binary_occupancy_grid, structure=struct)
        updated_dilated_grid = np.where(dilated_grid == 1, 100, 0)

        path_blocked = False
        if self.path:
            for (gx, gy) in self.path:
                if 0 <= gx < updated_dilated_grid.shape[1] and 0 <= gy < updated_dilated_grid.shape[0]:
                    if updated_dilated_grid[gy, gx] == 100:
                        path_blocked = True
                        break

        if path_blocked:
            self.get_logger().info('Path blocked by an obstacle. Replanning...')
            self.grid = updated_dilated_grid

            if self.goal_x is not None and self.goal_y is not None:
                start_pos = self.map_to_grid(self.current_pos_x, self.current_pos_y, self.map_data, "Start")
                goal_pos = self.map_to_grid(self.goal_x, self.goal_y, self.map_data, "Goal")
                path = self.a_star_search(start_pos, goal_pos, self.grid)

                self.path = self.simplify_path(path)
                self.path_worldframe = self.grid_to_map(self.path, self.map_data)
                self.path_index = 0
                self.publish_path(self.path_worldframe)
                self.publish_path_points(self.path_worldframe)

                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                print("Stopping to re-route")
        else:
            print('Current path is unobstructed. No replanning required.')

    def follow_path(self):
        """Controls the robot with potential fields to follow the path."""
        if self.is_adjusting_orientation:
            self.adjust_final_orientation()
            return

        if self.path_worldframe is None:
            return

        if self.path_index >= len(self.path_worldframe):
            self.is_adjusting_orientation = True
            self.adjust_final_orientation()
            return

        # Replanning frequency
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.last_replan_time > self.replan_interval:
            self.update_map_with_lidar()
            self.last_replan_time = current_time

        print("Moving along the path")
        goal_x, goal_y = self.path_worldframe[self.path_index]
        angle_diff = self.calculate_angular_difference(goal_x, goal_y)

        # If near the final or intermediate goal
        if self.path_index == len(self.path_worldframe) - 1:
            dist = math.sqrt((self.current_pos_x - goal_x)**2 + (self.current_pos_y - goal_y)**2)
            if dist < self.goal_pos_threshold:
                self.path_index += 1
                return
        else:
            dist = math.sqrt((self.current_pos_x - goal_x)**2 + (self.current_pos_y - goal_y)**2)
            if dist < self.distance_threshold:
                self.path_index += 1
                return

        distance = self.calculate_distance(goal_x, goal_y)
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

        # Speed limiting
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

        self.publisher_.publish(twist)

    def calculate_distance(self, goal_x, goal_y):
        """Computes the Euclidean distance to the goal."""
        return math.sqrt((goal_x - self.current_pos_x)**2 + (goal_y - self.current_pos_y)**2)

    def calculate_attractive_force(self, goal_x, goal_y):
        """Computes the attractive force that pulls the robot toward the goal."""
        force_x = self.k_a * (goal_x - self.current_pos_x)
        force_y = self.k_a * (goal_y - self.current_pos_y)
        return force_x, force_y

    def calculate_angular_correction(self, goal_x, goal_y):
        """Calculates angular correction needed to face the goal direction."""
        goal_dir = math.atan2(goal_y - self.current_pos_y, goal_x - self.position_x)
        angle_diff = self.normalize_angle(goal_dir - self.orientation)
        return self.k_ro * angle_diff

    def calculate_angular_difference(self, goal_x, goal_y):
        """Calculates the angular difference between current orientation and the goal."""
        goal_dir = math.atan2(goal_y - self.current_pos_y, goal_x - self.position_x)
        angle_diff = self.normalize_angle(goal_dir - self.orientation)
        return angle_diff

    def adjust_final_orientation(self):
        """Adjusts the robot's orientation when it reaches the goal position."""
        print("Positional goal reached. Stopping linear motion.")
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0

        angle_diff = abs(self.normalize_angle(self.goal_orientation - self.orientation))
        if angle_diff > self.rotation_threshold:
            self.final_rotation_direction = self.determine_rotation_direction(self.goal_orientation)
            print(f"Adjusting orientation towards: {self.final_rotation_direction}")
            if self.final_rotation_direction == "right":
                twist.angular.z = -self.rotate_speed
                print("here1")
                print(self.rotate_speed)
            else:
                twist.angular.z = self.rotate_speed
                print("here")
                print(self.rotate_speed)
        else:
            print("Orientation goal reached. Halting rotation.")
            twist.angular.z = 0.0
            self.path_index = 0
            self.path = None
            self.path_worldframe = None
            self.is_adjusting_orientation = False
            self.get_logger().info('Goal reached !!!')

        self.publisher_.publish(twist)

    def normalize_angle(self, angle):
        """Normalizes an angle into the range [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def determine_rotation_direction(self, goal_orientation):
        """Determines whether to rotate left or right to achieve the goal orientation."""
        curr = self.normalize_angle(self.orientation)
        goal = self.normalize_angle(goal_orientation)
        diff = self.normalize_angle(goal - curr)
        return "left" if diff > 0 else "right"

    def calculate_repulsive_forces(self, msg):
        """Computes repulsive forces from LIDAR data to avoid obstacles."""
        force_x, force_y, angular_force = 0.0, 0.0, 0.0
        min_force_threshold = 0.1
        
        for i, distance in enumerate(msg.ranges):
            if distance > self.max_distance or distance == float('inf') or math.isnan(distance):
                continue

            intensity = (self.max_distance - distance) / self.max_distance
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
    ###################### Potential Field Algorithm End #######################################


def main(args=None):
    """Main function to initialize and spin the node."""
    rclpy.init(args=args)
    node = OdometryMotionModel()

    while rclpy.ok():
        rclpy.spin_once(node)
        node.follow_path()
        time.sleep(0.01)  # Sleep for 10ms

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
