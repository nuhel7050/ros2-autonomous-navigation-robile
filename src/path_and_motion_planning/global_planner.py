#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import numpy as np
import heapq
from math import sqrt
from skimage.morphology import disk
from scipy.ndimage import binary_dilation
from tf_transformations import euler_from_quaternion

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(PoseStamped, '/goalpose', self.goal_pose_callback, 10)
        self.create_subscription(PoseStamped, '/startpose', self.start_pose_callback, 10)
        self.path_publisher = self.create_publisher(Path, '/path_line', 10)
        self.path_point_publisher = self.create_publisher(PoseArray, '/path_points', 10)

        self.map_data = None
        self.grid = None
        self.update_grid = None
        self.path = None
        self.path_worldframe = None
        self.threshold = 2
        self.goal_x, self.goal_y, self.goal_orientation = None, None, None
        self.start_x, self.start_y, self.start_orientation = None, None, None

    def map_callback(self, msg):
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
        structure = disk(self.threshold)
        binary_occupancy_grid = np.where(self.grid == 100, 1, 0)
        dilated_grid = binary_dilation(binary_occupancy_grid, structure=structure)
        self.grid = np.where(dilated_grid == 1, 100, 0)
        self.update_grid = np.copy(self.grid)

    def start_pose_callback(self, msg):
        self.start_x = msg.pose.position.x
        self.start_y = msg.pose.position.y
        _, _, self.start_orientation = euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

    def goal_pose_callback(self, msg):
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

        start_pos = self.map_to_grid(self.start_x, self.start_y, self.map_data, "Start")
        goal_pos = self.map_to_grid(self.goal_x, self.goal_y, self.map_data, "Goal")
        path = self.a_star_search(start_pos, goal_pos, self.grid)

        self.path = self.simplify_path(path)
        self.path_worldframe = self.grid_to_map(self.path, self.map_data)
        self.publish_path_points(self.path_worldframe)
        self.publish_path(self.path_worldframe)
        self.get_logger().info(f'A* Path in grid: {self.path}')
        self.get_logger().info(f'A* Path in world frame: {self.path_worldframe}')

    def map_to_grid(self, x, y, map_data, msg):
        mx = int(round((x - map_data['origin']['x']) / map_data['resolution']))
        my = int(round((y - map_data['origin']['y']) / map_data['resolution']))
        self.get_logger().info(f"Position of robot for {msg} on map: x: {mx}, y: {my}")
        return (mx, my)

    def grid_to_map(self, path_, map_data):
        path_new = []
        origin_x = map_data['origin']['x']
        origin_y = map_data['origin']['y']
        resolution = map_data['resolution']
        for i in range(len(path_)):
            wx = origin_x + path_[i][0] * resolution
            wy = origin_y + path_[i][1] * resolution
            path_new.append((wx, wy))
        return path_new

    def simplify_path(self, path):
        if not path or len(path) < 3:
            return path

        simplified_path = [path[0]]
        for i in range(1, len(path) - 1):
            prev = path[i - 1]
            current = path[i]
            nxt = path[i + 1]

            if not ((prev[0] == current[0] == nxt[0]) or (prev[1] == current[1] == nxt[1])):
                simplified_path.append(current)

        simplified_path.append(path[-1])
        return simplified_path

    def a_star_search(self, start, goal, grid):
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

        return self.reconstruct_path(came_from, start, goal)

    def heuristic(self, a, b):
        return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def get_neighbors(self, pos, grid):
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
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from.get(current)
        path.append(start)
        path.reverse()
        return path

    def publish_path_points(self, path):
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

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()