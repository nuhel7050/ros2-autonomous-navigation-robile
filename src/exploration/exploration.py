import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener, LookupException, TransformException
from std_msgs.msg import String
import numpy as np
import time
import os


class IntegratedExplorer(Node):
    def __init__(self):
        super().__init__('integrated_explorer')
        self.get_logger().info("Initializing Integrated Explorer...")
        
        # Use QoS settings for map subscription
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)

        # Publisher for goal pose (sends goals to A* planner)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Subscribe to A* planner result
        self.create_subscription(String, '/progress_result', self.astar_result_callback, 10)
        
        # Subscribe to the path from A* planner
        self.create_subscription(Path, '/path', self.path_callback, 10)
        
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_callback, 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # TF buffer to get robot's current pose in the map frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # State tracking
        self.exploring = False
        self.tried_goals = []  # already-attempted frontier goals
        self.current_pose = None
        self.current_map = None
        self.frontier_goals = []
        self.current_goal = None
        self.map_resolution = 0.05  # default, will be updated from map
        self.map_origin = [0, 0, 0]  # default, will be updated from map
        self.exploration_timeout = 60.0  # seconds
        self.exploration_start_time = None
        self.map_counter = 0  # To reduce logging frequency
        self.received_first_map = False
        self.received_first_pose = False
        
        # Timer for periodic exploration updates - start with faster updates initially
        self.create_timer(2.0, self.exploration_loop)
        
        # Timer to check subscriptions
        self.create_timer(5.0, self.check_connections)
        
        self.get_logger().info("Integrated Explorer initialized and waiting for map and pose...")

    def check_connections(self):
        """Periodically check if we're receiving expected data"""
        if not self.received_first_map:
            self.get_logger().warn("Still waiting for first map message...")
        if not self.received_first_pose:
            self.get_logger().warn("Still waiting for first pose message...")
        
        # If we have both but haven't started exploring, try to kickstart
        if self.received_first_map and self.received_first_pose and not self.exploring:
            self.get_logger().info("Got map and pose. Starting exploration...")
            self.find_and_publish_frontier()

    def map_callback(self, map_msg):
        """Stores the latest map and analyzes frontiers if not currently exploring."""
        self.received_first_map = True
        self.current_map = map_msg
        self.map_resolution = map_msg.info.resolution
        self.map_origin = [
            map_msg.info.origin.position.x,
            map_msg.info.origin.position.y,
            map_msg.info.origin.position.z
        ]
        
        # Reduce logging frequency - only log every 5th map update
        self.map_counter += 1
        if self.map_counter % 5 == 0:
            self.get_logger().info(f"Map received: {map_msg.info.width}x{map_msg.info.height}, resolution: {self.map_resolution}")
            self.get_logger().info(f"Map origin: {self.map_origin}")
    
        # Only analzye map for frontiers if not currently navigating AND we have a pose
        if not self.exploring and self.current_pose is not None:
            self.get_logger().info("Finding frontiers...")
            self.find_and_publish_frontier()

    def pose_callback(self, msg):
        """Updates the robot's current pose."""
        self.received_first_pose = True
        self.current_pose = msg.pose.pose
        
        # Limit pose logging to reduce console spam
        if hasattr(self, 'pose_counter'):
            self.pose_counter += 1
        else:
            self.pose_counter = 0
            
        if self.pose_counter % 10 == 0:
            self.get_logger().info(f"Received pose: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})")
    
        # If this is our first pose and we have a map, start exploration
        if self.current_map is not None and not self.exploring:
            self.find_and_publish_frontier()
            
    def find_frontier_clusters(self, map_msg):
        """
        Analyzes an OccupancyGrid to find frontier clusters.
        Returns a list of frontier cluster centroids in world coordinates.
        """
        width = map_msg.info.width
        height = map_msg.info.height
        
        # Make sure we actually have data
        if len(map_msg.data) != width * height:
            self.get_logger().error(f"Map data length {len(map_msg.data)} doesn't match dimensions {width}x{height}")
            return []
        
        data = np.array(map_msg.data).reshape(height, width)
        self.get_logger().info(f"Processing map data array of shape {data.shape}")
        
        # Count different cell types for debugging
        unknown_cells = np.sum(data == -1)
        free_cells = np.sum(data == 0)
        occupied_cells = np.sum(data > 0)
        
        self.get_logger().info(f"Map contains: {unknown_cells} unknown, {free_cells} free, {occupied_cells} occupied cells")
        
        if free_cells == 0:
            self.get_logger().warn("No free cells in map - cannot find frontiers!")
            return []

        # Directions for 8-connected neighbors
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

        # Find all frontier cells: unknown cell with at least one adjacent free cell
        self.get_logger().info("Starting frontier cell detection...")
        frontier_set = set()
        
        # Performance optimization: only check the edges between known and unknown space
        for y in range(height):
            for x in range(width):
                if data[y, x] == -1:  # unknown cell
                    # Check neighbors for free space
                    for dx, dy in directions:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            if data[ny, nx] == 0:  # neighbor is free
                                frontier_set.add((x, y))
                                break

        # Debug the number of frontier cells found
        self.get_logger().info(f"Found {len(frontier_set)} frontier cells")
        
        if len(frontier_set) == 0:
            self.get_logger().warn("No frontiers found in map!")
            return []

        # Cluster frontier cells into regions
        clusters = []
        while frontier_set:
            # Start a new cluster
            x, y = frontier_set.pop()
            cluster = [(x, y)]
            stack = [(x, y)]
            
            # Depth-first search to find connected frontier cells
            while stack:
                cx, cy = stack.pop()
                for dx, dy in directions:
                    nx, ny = cx + dx, cy + dy
                    if (nx, ny) in frontier_set:
                        frontier_set.remove((nx, ny))
                        stack.append((nx, ny))
                        cluster.append((nx, ny))
            
            # Only keep clusters with enough cells (filter out noise)
            if len(cluster) >= 5:
                clusters.append(cluster)

        self.get_logger().info(f"Found {len(clusters)} frontier clusters")
        
        if len(clusters) == 0:
            self.get_logger().warn("No significant frontier clusters - exploration may be complete")
            return []

        # Compute centroid for each cluster and convert to world coordinates
        frontier_goals = []
        for cluster in clusters:
            if not cluster:
                continue
            avg_x = sum(p[0] for p in cluster) / len(cluster)
            avg_y = sum(p[1] for p in cluster) / len(cluster)
            
            # Convert grid coords to world coords
            wx = map_msg.info.origin.position.x + (avg_x) * map_msg.info.resolution
            wy = map_msg.info.origin.position.y + (avg_y) * map_msg.info.resolution
            
            # Track cluster size for potential prioritization
            cluster_size = len(cluster)
            frontier_goals.append((wx, wy, cluster_size))
            self.get_logger().debug(f"Frontier centroid at grid ({avg_x:.1f}, {avg_y:.1f}) -> world ({wx:.2f}, {wy:.2f})")

        # Filter for safety: ensure the goals are not too close to obstacles
        safety_threshold = 0.5  # meters
        safety_cells = int(safety_threshold / map_msg.info.resolution)
        filtered_goals = []
        
        for wx, wy, size in frontier_goals:
            # Convert world coordinates back to grid coordinates
            gx = int((wx - map_msg.info.origin.position.x) / map_msg.info.resolution)
            gy = int((wy - map_msg.info.origin.position.y) / map_msg.info.resolution)
            
            # Check if this point is at least safety_threshold from any obstacle
            safe = True
            for dx in range(-safety_cells, safety_cells+1):
                for dy in range(-safety_cells, safety_cells+1):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        if data[ny, nx] > 20:  # Occupied cell (with threshold)
                            safe = False
                            break
                if not safe:
                    break
                    
            if safe:
                filtered_goals.append((wx, wy, size))
                self.get_logger().debug(f"Safe frontier at ({wx:.2f}, {wy:.2f})")
            else:
                self.get_logger().debug(f"Filtered out unsafe frontier at ({wx:.2f}, {wy:.2f})")
        
        self.get_logger().info(f"Found {len(filtered_goals)} safe frontier goals")
        return filtered_goals
            
    def find_and_publish_frontier(self):
        """Find frontiers and publish the best one as a goal."""
        if self.current_map is None:
            self.get_logger().warn("No map data available yet.")
            return
        
        if self.current_pose is None:
            self.get_logger().warn("No pose data available yet.")
            return
            
        # Find frontier goals
        self.get_logger().info("Finding frontier clusters...")
        self.frontier_goals = self.find_frontier_clusters(self.current_map)
        self.get_logger().info(f"Found {len(self.frontier_goals)} frontier clusters")
        
        if not self.frontier_goals:
            self.get_logger().info("No unexplored frontiers found. Will try again later.")
            return
        
        # Filter out any goals that were already attempted (within 0.5m)
        if self.tried_goals:
            prev_count = len(self.frontier_goals)
            self.frontier_goals = [
                g for g in self.frontier_goals if not any(
                    math.hypot(g[0]-t[0], g[1]-t[1]) < 0.5 for t in self.tried_goals
                )
            ]
            self.get_logger().info(f"Filtered out {prev_count - len(self.frontier_goals)} already-tried frontiers")
        
        if not self.frontier_goals:
            self.get_logger().info("No unexplored frontiers remaining. Exploration complete!")
            self.save_map()
            return
            
        # Get robot's current position
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        # Choose the closest frontier goal to the robot's current position
        self.frontier_goals.sort(key=lambda g: math.hypot(g[0]-robot_x, g[1]-robot_y))
        
        # Print all frontiers with distances for debugging
        for i, (gx, gy, size) in enumerate(self.frontier_goals):
            dist = math.hypot(gx-robot_x, gy-robot_y)
            self.get_logger().info(f"Frontier {i}: ({gx:.2f}, {gy:.2f}) - dist: {dist:.2f}m, size: {size}")
        
        nearest_goal = self.frontier_goals[0]
        gx, gy, _ = nearest_goal
        
        self.get_logger().info(f"Selected frontier at ({gx:.2f}, {gy:.2f}). Sending to A* planner...")
        
        # Create a PoseStamped for the goal
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = gx
        goal_pose.pose.position.y = gy
        goal_pose.pose.orientation.w = 1.0  # Default orientation
        
        # Publish the goal for A* planner
        self.goal_pub.publish(goal_pose)
        self.current_goal = (gx, gy)
        self.exploring = True
        self.exploration_start_time = time.time()
        
    def astar_result_callback(self, msg):
        """Called when A* planner reports success/failure."""
        result = msg.data.lower() == "true"
        
        if not result:
            self.get_logger().warn("A* planner could not find path to frontier.")
            # Mark this goal as tried so we don't attempt it again
            if self.current_goal:
                self.tried_goals.append(self.current_goal)
            # Allow choosing a new frontier
            self.exploring = False
            
            # Emergency stop the robot
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            
            # Immediately try to find a new frontier
            self.find_and_publish_frontier()
        else:
            self.get_logger().info("A* planner found path to frontier.")
            # A* found a path - now we wait for the path_callback to track progress
    
    def path_callback(self, msg):
        """Received a path from A* planner."""
        if not msg.poses:
            self.get_logger().warn("Received empty path from A* planner.")
            self.exploring = False
            return
            
        self.get_logger().info(f"Received path with {len(msg.poses)} waypoints")
        # Log first and last waypoints
        if len(msg.poses) > 0:
            first = msg.poses[0].pose.position
            last = msg.poses[-1].pose.position
            self.get_logger().info(f"Path from ({first.x:.2f}, {first.y:.2f}) to ({last.x:.2f}, {last.y:.2f})")
    
    def exploration_loop(self):
        """Periodic check to see if we've reached goals or need to select new ones."""
        # Check if we have the necessary data to explore
        if not self.received_first_map:
            self.get_logger().debug("Waiting for first map data...")
            return
            
        if not self.received_first_pose:
            self.get_logger().debug("Waiting for first pose data...")
            return
        
        if not self.exploring and not self.frontier_goals:
            self.get_logger().info("Not currently exploring. Finding new frontiers...")
            self.find_and_publish_frontier()
            return
            
        if not self.exploring:
            return
            
        # Check for exploration timeout
        if self.exploration_start_time and time.time() - self.exploration_start_time > self.exploration_timeout:
            self.get_logger().warn(f"Exploration timeout after {self.exploration_timeout} seconds!")
            # Stop the robot
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            
            # Mark current goal as tried
            if self.current_goal:
                self.tried_goals.append(self.current_goal)
                
            self.exploring = False
            # Try finding a new frontier immediately
            self.find_and_publish_frontier()
            return
            
        if self.current_pose is None or self.current_goal is None:
            return
            
        # Check if we've reached the current goal
        dist_to_goal = math.hypot(
            self.current_pose.position.x - self.current_goal[0],
            self.current_pose.position.y - self.current_goal[1]
        )
        
        self.get_logger().debug(f"Distance to goal: {dist_to_goal:.2f}m")
        
        if dist_to_goal < 0.5:  # Within 0.5m of goal
            self.get_logger().info(f"Reached frontier goal! Distance: {dist_to_goal:.2f}m")
            self.tried_goals.append(self.current_goal)
            self.current_goal = None
            self.exploring = False
            # Find a new frontier immediately
            self.find_and_publish_frontier()
    
    def save_map(self):
        """Save the final map when exploration is complete."""
        self.get_logger().info("Saving final map...")
        
        # Use ROS2 map_saver to save the map
        # Need to save it explicitly - log the instruction to save the map
        map_dir = os.path.expanduser("~/maps")
        os.makedirs(map_dir, exist_ok=True)
        
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        map_name = f"map_{timestamp}"
        
        self.get_logger().info(f"Map would be saved to {map_dir}/{map_name}")
        self.get_logger().info("To save the map, run: ros2 run nav2_map_server map_saver_cli -f ~/maps/map_name")


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()