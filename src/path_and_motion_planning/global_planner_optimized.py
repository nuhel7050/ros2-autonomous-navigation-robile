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
    """
    A* Global Path Planner for ROS2
    Subscribes to map data and start/goal poses to generate optimal paths
    using the A* search algorithm, with path simplification and dilation
    for obstacle avoidance.
    """
    def __init__(self):
        super().__init__('global_planner')
        self._init_subscribers()
        self._init_publishers()
        self._init_state()

    def _init_subscribers(self):
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, 10)
        self.create_subscription(PoseStamped, '/goalpose', self._goal_pose_callback, 10)
        self.create_subscription(PoseStamped, '/startpose', self._start_pose_callback, 10)

    def _init_publishers(self):
        self.path_publisher = self.create_publisher(Path, '/path_line', 10)
        self.path_point_publisher = self.create_publisher(PoseArray, '/path_points', 10)

    def _init_state(self):
        self.map_data = None
        self.grid = None
        self.update_grid = None
        self.path = None
        self.path_worldframe = None
        self.threshold = 2  # Dilation threshold for obstacle avoidance
        self.goal_x, self.goal_y, self.goal_orientation = None, None, None
        self.start_x, self.start_y, self.start_orientation = None, None, None

    def _map_callback(self, msg):
        self.map_data = {
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
            'origin': {'x': msg.info.origin.position.x, 'y': msg.info.origin.position.y, 'z': msg.info.origin.position.z},
            'data': msg.data
        }
        self.grid = np.array(self.map_data['data']).reshape(self.map_data['height'], self.map_data['width'])
        self._dilate_obstacles()

    def _dilate_obstacles(self):
        structure = disk(self.threshold)
        binary_occupancy_grid = np.where(self.grid == 100, 1, 0)
        dilated_grid = binary_dilation(binary_occupancy_grid, structure=structure)
        self.grid = np.where(dilated_grid == 1, 100, 0)
        self.update_grid = np.copy(self.grid)

    def _start_pose_callback(self, msg):
        self.start_x = msg.pose.position.x
        self.start_y = msg.pose.position.y
        _, _, self.start_orientation = euler_from_quaternion([
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        ])

    def _goal_pose_callback(self, msg):
        if self.map_data is None:
            self.get_logger().info('GoalPose_CB: Map data is not yet available.')
            return
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        _, _, self.goal_orientation = euler_from_quaternion([
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        ])
        self._generate_path()

    def _generate_path(self):
        start_pos = self._map_to_grid(self.start_x, self.start_y, "Start")
        goal_pos = self._map_to_grid(self.goal_x, self.goal_y, "Goal")
        raw_path = self._a_star_search(start_pos, goal_pos)
        self.path = self._simplify_path(raw_path)
        self.path_worldframe = self._grid_to_map(self.path)
        self._publish_path_points(self.path_worldframe)
        self._publish_path(self.path_worldframe)
        self.get_logger().info(f'A* Path in grid: {self.path}')
        self.get_logger().info(f'A* Path in world frame: {self.path_worldframe}')

    def _map_to_grid(self, x, y, msg_type):
        mx = int(round((x - self.map_data['origin']['x']) / self.map_data['resolution']))
        my = int(round((y - self.map_data['origin']['y']) / self.map_data['resolution']))
        self.get_logger().info(f"Position of robot for {msg_type} on map: x: {mx}, y: {my}")
        return (mx, my)

    def _grid_to_map(self, path_):
        path_new = [(self.map_data['origin']['x'] + x * self.map_data['resolution'],
                     self.map_data['origin']['y'] + y * self.map_data['resolution']) for x, y in path_]
        return path_new

    def _a_star_search(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from, cost_so_far = {start: None}, {start: 0}
        while open_set:
            current = heapq.heappop(open_set)[1]
            if current == goal:
                break
            for nxt in self._get_neighbors(current):
                new_cost = cost_so_far[current] + self._heuristic(current, nxt)
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + self._heuristic(goal, nxt)
                    heapq.heappush(open_set, (priority, nxt))
                    came_from[nxt] = current
        return self._reconstruct_path(came_from, start, goal)

    def _heuristic(self, a, b):
        return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def _get_neighbors(self, pos):
        directions = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
        return [(pos[0] + dx, pos[1] + dy) for dx, dy in directions if 0 <= pos[0] + dx < self.grid.shape[1] and 0 <= pos[1] + dy < self.grid.shape[0] and self.grid[pos[1] + dy, pos[0] + dx] != 100]

    def _reconstruct_path(self, came_from, start, goal):
        path, current = [], goal
        while current != start:
            path.append(current)
            current = came_from.get(current)
        return path[::-1]

    def _simplify_path(self, path):
        return path if len(path) < 3 else [p for i, p in enumerate(path) if i == 0 or i == len(path)-1 or not ((path[i-1][0] == p[0] == path[i+1][0]) or (path[i-1][1] == p[1] == path[i+1][1]))]

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
