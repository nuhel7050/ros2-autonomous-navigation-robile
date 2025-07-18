import numpy as np
from heapq import heappop, heappush
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Header, String
import math
import tf_transformations

class AStarNode:
    def __init__(self, state, g, f, parent=None):
        self.state = state
        self.g = g
        self.f = f
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f


class AStarPathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # Initialize subscriptions and publishers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.slam_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.slam_pose_callback, 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)

        self.occupancy_grid = None
        self.grid_width = 0
        self.grid_height = 0
        self.resolution = 0.05
        self.origin = (0.0, 0.0)
        self.curr_pose = None
        self.goal_pose = None

    def map_callback(self, msg):
        self.grid_width = msg.info.width
        self.grid_height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)

        data = np.array(msg.data).reshape((self.grid_height, self.grid_width))
        self.occupancy_grid = np.rot90(data, 2)
        self.get_logger().info(f'Map Loaded: {self.grid_width}x{self.grid_height}')

    def slam_pose_callback(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.start_pose = self.rescale_pose_to_mapsize(x=x, y=y)

    def goal_callback(self, goal_msg):
        self.goal_pose_world = (goal_x := goal_msg.pose.position.x, goal_msg.pose.position.y)
        self.goal_pose = self.rescale_to_grid(goal_msg.pose.position)

        self.get_logger().info(f'Goal_pose in grid: {self.goal_pose}')
        self.get_logger().info(f'Getting goal pose: {self.goal_pose}')

        self.plan()

    def rescale_to_grid(self, position):
        grid_x = int((position.x - self.origin[0]) / self.resolution)
        grid_y = self.grid_height - int((position.y - self.origin[1]) / self.resolution)
        grid_pose = (grid_x, grid_y)
        return grid_pose

    def slam_pose_callback(self, msg):
        self.curr_pose = self.rescale_to_grid(msg.pose.pose.position)

    def is_valid(self, pose):
        x, y = pose
        return 0 <= x < self.grid_width and 0 <= y < self.grid_height and self.occupancy_grid[y, x] == 0

    def heuristic(self, current, goal):
        return np.hypot(goal[0] - current[0], goal[1] - current[1])

    def get_neighbors(self, state):
        moves = [(dx, dy) for dx in [-1, 0, 1] for dy in [-1, 0, 1] if not (dx == 0 and dy == 0)]
        neighbors = [(state[0] + dx, state[1] + dy) for dx, dy in moves]
        return [n for n in neighbors if self.is_valid(n)]

    def astar(self, start, goal):
        fringe = [AStarNode(start, 0, self.heuristic(start, goal))]
        explored = set()

        while fringe:
            current_node = heappop(fringe)

            if current_node.state == goal:
                path = []
                while current_node:
                    path.append(current_node.state)
                    current_node = current_node.parent
                path.reverse()
                return path

            for neighbor in self.get_neighbors(current_node.state):
                tentative_g = current_node.g + 1
                neighbor_node = AStarNode(neighbor, tentative_g, tentative_g + self.heuristic(neighbor, goal), current_node)
                heappush(fringe, neighbor_node)

        return None

    def publish_path(self, path):
        ros_path = Path()
        ros_path.header.frame_id = 'map'
        ros_time = self.get_clock().now().to_msg()

        for p in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = ros_time
            pose.pose.position.x = (p[0] * self.resolution) + self.origin[0]
            pose.pose.position.y = (p[1] * self.resolution) + self.origin[1]
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)

        self.path_pub.publish(ros_path)

    def plan(self):
        if self.occupancy_grid is None or self.curr_pose is None or self.goal_pose is None:
            self.get_logger().info('Missing map, start, or goal pose.')
            return

        if not self.is_valid(self.goal_pose):
            self.get_logger().info('Goal Pose is not valid!')
            return

        path = self.astar(self.curr_pose, self.goal_pose)
        if path:
            ros_path = self.convert_to_ros_path(path)
            self.path_pub.publish(path)
            self.get_logger().info('Path successfully published!')
        else:
            self.get_logger().info('No valid path found.')

    def get_neighbors(self, state):
        neighbors = [(state[0]+dx, state[1]+dy) for dx in [-1,0,1] for dy in [-1,0,1] if not (dx == 0 and dy == 0)]
        valid_neighbors = []
        for neighbor in neighbors:
            if self.is_valid(neighbor):
                valid_neighbors.append(neighbor)
        return valid_neighbors

    def heuristic(self, current, goal):
        return math.hypot(current[0] - goal[0], current[1] - goal[1])

    def astar(self, start, goal):
        fringe = []
        explored = set()
        heappush(fringe, AStarNode(start, 0, self.heuristic(start, goal)))

        while fringe:
            current_node = heappop(fringe)

            if current_node.state == goal:
                path = []
                while current_node:
                    path.append(current_node.state)
                    current_node = current_node.parent
                return path[::-1]

            explored.add(current_node.state)

            for neighbor in self.get_neighbors(current_node.state):
                if neighbor in explored_set:
                    continue
                neighbor_g = current_node.g + 1
                neighbor_node = AStarNode(neighbor, neighbor_g, neighbor_g + self.heuristic(neighbor, goal), current_node)
                heappush(fringe, neighbor_node)

        return []

    def publish_path(self, path):
        ros_path = Path()
        ros_path.header.frame_id = 'map'
        ros_path.header.stamp = self.get_clock().now().to_msg()

        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = ros_path.header.stamp
            pose.pose.position.x = (point[0] * self.resolution) + self.origin[0]
            pose.pose.position.y = (point[1] * self.resolution) + self.origin[1]
            ros_path.poses.append(pose)

        self.path_pub.publish(ros_path)


def main(args=None):
    rclpy.init(args=args)
    node = AStarPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
