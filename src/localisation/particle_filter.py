import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from geometry_msgs.msg import Quaternion, Pose, Point, PoseStamped, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
from math import atan2, asin, sqrt
import traceback
import sys

MS = 0.001

class ParticleEntity:
    def __init__(self, pose: Pose, weight: float):
        self.pose = pose
        self.weight = weight

class ParticleFilterLocalizer(Node):

    def __init__(self):
        try:
            super().__init__('particle_filter_localizer')
            self.get_logger().info('Launching Particle Filter Localizer node')
            
            # Enhanced logging
            self.get_logger().info('Configuring subscriptions...')
            
            self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                    self.handle_initial_pose, 10)
            self.get_logger().debug('Established /initialpose subscription')
            
            self.create_subscription(Odometry, '/odom',
                                    self.handle_odometry, 10)
            self.get_logger().debug('Established /odom subscription')
            
            self.create_subscription(LaserScan, '/scan',
                                    self.handle_scan, 10)
            self.get_logger().debug('Established /scan subscription')
            
            self.get_logger().info('Setting up map subscription...')
            map_quality = QoSProfile(
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                history=HistoryPolicy.KEEP_LAST,
                                depth=1)
            self.create_subscription(OccupancyGrid, '/map', 
                                    self.handle_map, map_quality)
            self.get_logger().debug('Established /map subscription')
            
            self.get_logger().info('Creating publishers...')
            self._filter_path_pub = self.create_publisher(Path, 
                                                    '/mcl_path', 
                                                    10)
            self._odometry_path_pub = self.create_publisher(Path, 
                                                        '/odom_path', 
                                                        10)
            self._particle_cloud_pub = self.create_publisher(PoseArray, 
                                                    '/particlecloud', 
                                                    10)
            self.pose_estimation_pub = self.create_publisher(Odometry, 
                                                    '/mcl_pose', 
                                                    10)  # Renamed from /odom to /mcl_pose to avoid topic clash
            self.get_logger().debug('Publishers established')
            
            # Higher frequency timer for more frequent particle updates
            self.create_timer(100 * MS, self.periodic_update)
            self.get_logger().debug('Update timer created')
            
            self._filter_path = Path()
            self._odometry_path = Path()
            
            self._particles = []  # For visualization in rviz
            self._particle_collection = []  # Contains data as ((x, y, yaw), weight)
            self._map_data = None  # Will store map information when received

            self._previous_used_odom = None  # Initialize with None instead of undefined Pose
            self._previous_odom = None
            self._current_position = None
            self._latest_scan = None
            self._performing_update = False

            # Motion detection flag
            self._robot_moving = False
            
            # Track particle initialization state
            self._particle_system_initialized = False
            
            # Initialize particles with default position after brief delay
            self.create_timer(2.0, self.initialize_default_particles_timer)
            self.get_logger().info('Setup complete')
        except Exception as e:
            self.get_logger().error(f'Setup error: {str(e)}')
            traceback.print_exc()
            raise

    def initialize_default_particles_timer(self):
        """Timer callback for default particle initialization"""
        try:
            if not self._particle_system_initialized:
                self.setup_default_particles()
                self._particle_system_initialized = True
                self.get_logger().info('Default particle initialization finished')
        except Exception as e:
            self.get_logger().error(f'Error in initialize_default_particles_timer: {str(e)}')

    def setup_default_particles(self):
        """Create particles with default position if none exist"""
        try:
            if not self._particles:
                self.get_logger().info('Creating particles with default position')
                default_position = Pose()
                default_position.position.x = 0.0
                default_position.position.y = 0.0
                default_position.position.z = 0.0
                default_position.orientation.w = 1.0
                self._current_position = default_position
                self._previous_used_odom = default_position
                self._previous_odom = default_position
                self._setup_gaussian_particles()
        except Exception as e:
            self.get_logger().error(f'Error in setup_default_particles: {str(e)}')

    ####################### Callback Methods Begin ###########################

    def handle_map(self, data):
        try:
            self._map_data = {
                'resolution': data.info.resolution,
                'width': data.info.width,
                'height': data.info.height,
                'origin': {
                    'x': data.info.origin.position.x,
                    'y': data.info.origin.position.y,
                    'z': data.info.origin.position.z
                },
                'data': data.data
            }
            self.get_logger().info(f'Map received: {data.info.width}x{data.info.height}, resolution: {data.info.resolution}')
        except Exception as e:
            self.get_logger().error(f'Error handling map data: {str(e)}')

    def periodic_update(self):
        try:
            if self._particles:
                self._publish_particle_cloud()
                self.get_logger().debug(f'Periodic update - particle count: {len(self._particles)}')
        except Exception as e:
            self.get_logger().error(f'Error in periodic update: {str(e)}')

    def handle_initial_pose(self, msg):
        try:
            self.get_logger().info('Initial pose received')
            position = Point(x=msg.pose.pose.position.x, 
                            y=msg.pose.pose.position.y, 
                            z=msg.pose.pose.position.z)
            orientation = Quaternion(x=msg.pose.pose.orientation.x, 
                                    y=msg.pose.pose.orientation.y, 
                                    z=msg.pose.pose.orientation.z, 
                                    w=msg.pose.pose.orientation.w if msg.pose.pose.orientation.w != 0 else 1.0)
            self._current_position = Pose(position=position, orientation=orientation)
            self._previous_used_odom = self._current_position
            self._previous_odom = self._current_position
            self._setup_gaussian_particles()
            publish_pose = Odometry()
            publish_pose.header.stamp = self.get_clock().now().to_msg()
            publish_pose.header.frame_id = 'odom'
            publish_pose.child_frame_id = 'base_link'
            publish_pose.pose.pose.position = position
            publish_pose.pose.pose.orientation = orientation
            self.pose_estimation_pub.publish(publish_pose)
        except Exception as e:
            self.get_logger().error(f'Error processing initial pose: {str(e)}')

    def handle_odometry(self, msg):
        try:
            if self._previous_odom is None:
                self._previous_odom = msg.pose.pose
                self.get_logger().debug('First odometry data received')
                return
                
            self._previous_used_odom = msg.pose.pose
            movement_x, movement_y, rotation = self.compute_pose_difference(self._previous_odom, self._previous_used_odom)
            
            # Detect robot motion
            if abs(movement_x) > 0.01 or abs(movement_y) > 0.01 or abs(rotation) > 0.01:
                self._robot_moving = True
                self.get_logger().debug(f'Motion detected: dx={movement_x:.4f}, dy={movement_y:.4f}, dθ={rotation:.4f}')
            else:
                self._robot_moving = False

            # Update particles only during motion
            if self._robot_moving and self._particles:
                self.update_particle_positions(movement_x, movement_y, rotation)
                self._update_odometry_path(self._previous_used_odom)

            self._previous_odom = self._previous_used_odom
        except Exception as e:
            self.get_logger().error(f'Error processing odometry: {str(e)}')
    
    def handle_scan(self, msg: LaserScan):
        try:
            self._latest_scan = msg
            self.get_logger().debug(f'Laser scan received: {len(msg.ranges)} measurements')

            # Update weights only during motion
            if self._robot_moving and self._performing_update and self._particles:
                self.recalculate_weights()
        except Exception as e:
            self.get_logger().error(f'Error processing laser scan: {str(e)}')
    
    ####################### Callback Methods End ############################
    ####################### Utility Methods Begin ############################

    def extract_yaw(self, orientation: Quaternion):
        try:
            _, _, yaw = self.quaternion_to_euler(orientation.x,
                                            orientation.y,
                                            orientation.z,
                                            orientation.w)
            return yaw
        except Exception as e:
            self.get_logger().error(f'Error extracting yaw: {str(e)}')
            return 0.0

    def quaternion_to_euler(self, x, y, z, w):
        try:
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = atan2(t0, t1)

            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = asin(t2)

            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = atan2(t3, t4)

            return roll_x, pitch_y, yaw_z  # in radians
        except Exception as e:
            self.get_logger().error(f'Error converting quaternion to euler: {str(e)}')
            return 0.0, 0.0, 0.0

    def euler_to_quaternion(self, yaw, pitch=0.0, roll=0.0) -> Quaternion:
        try:
            qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
            qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
            qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
            qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

            return Quaternion(x=qx, y=qy, z=qz, w=qw)
        except Exception as e:
            self.get_logger().error(f'Error converting euler to quaternion: {str(e)}')
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    ####################### Utility Methods End ############################

    def _setup_gaussian_particles(self, pose: Pose = None, scale: float = 0.05):
        try:
            if pose is None:
                pose = self._current_position
            
            # Increased particle count for better visualization
            num_particles = 100
            x_coords = list(np.random.normal(loc=pose.position.x, scale=scale, size=num_particles))
            y_coords = list(np.random.normal(loc=pose.position.y, scale=scale, size=num_particles))
            current_heading = self.extract_yaw(pose.orientation)
            heading_angles = list(np.random.normal(loc=current_heading, scale=0.01, size=num_particles))

            initial_weight = 1.0 / float(num_particles)
            
            # Clear existing particles
            self._particles = []
            random_samples = []
            
            for x, y, heading in zip(x_coords, y_coords, heading_angles):
                position = Point(x=x, y=y, z=0.0)
                orientation = self.euler_to_quaternion(heading, 0.0, 0.0)
                temp_pose = Pose(position=position, orientation=orientation)
                random_samples.append(((x, y, heading), initial_weight))
                self._particles.append(ParticleEntity(temp_pose, initial_weight))
            
            self._particle_collection = random_samples
            self._performing_update = True
            
            self.get_logger().info(f'Created {len(self._particles)} particles')
            
            # Immediately update visualization
            self._publish_particle_cloud()
        except Exception as e:
            self.get_logger().error(f'Error setting up gaussian particles: {str(e)}')

    def compute_pose_difference(self, initial_pose: Pose, final_pose: Pose):
        try:
            delta_x = final_pose.position.x - initial_pose.position.x
            delta_y = final_pose.position.y - initial_pose.position.y
            _, _, initial_heading = self.quaternion_to_euler(initial_pose.orientation.x,
                                                        initial_pose.orientation.y,
                                                        initial_pose.orientation.z,
                                                        initial_pose.orientation.w)
            _, _, final_heading = self.quaternion_to_euler(final_pose.orientation.x,
                                                        final_pose.orientation.y,
                                                        final_pose.orientation.z,
                                                        final_pose.orientation.w)
            delta_heading = final_heading - initial_heading
            return delta_x, delta_y, delta_heading
        except Exception as e:
            self.get_logger().error(f'Error computing pose difference: {str(e)}')
            return 0.0, 0.0, 0.0

    def update_particle_positions(self, delta_x, delta_y, delta_heading):
        try:
            noise_x = 0.002
            noise_y = 0.002
            noise_heading = 0.001
            minimum_movement = 0.0001

            for particle in self._particles:
                position_x = particle.pose.position.x 
                position_y = particle.pose.position.y
                heading = self.extract_yaw(particle.pose.orientation)
                
                updated_x = position_x + delta_x + np.random.normal(0, noise_x)
                updated_y = position_y + delta_y + np.random.normal(0, noise_y)
                updated_heading = heading + delta_heading + np.random.normal(0, noise_heading)
                updated_heading = (updated_heading + np.pi) % (2 * np.pi) - np.pi
                
                particle.pose.position.x = updated_x
                particle.pose.position.y = updated_y
                particle.pose.orientation = self.euler_to_quaternion(updated_heading, 0.0, 0.0)
        except Exception as e:
            self.get_logger().error(f'Error updating particle positions: {str(e)}')
        
    ########################## Raycasting Simulation Begin ############################

    def generate_simulated_scan(self, pose: Pose):
        try:
            if self._map_data is None:
                self.get_logger().warn('Map unavailable for scan simulation')
                return []
            
            if self._latest_scan is None:
                self.get_logger().warn('No reference scan data available')
                return []
            
            virtual_ranges = []
            max_range = self._latest_scan.range_max
            beam_count = len(self._latest_scan.ranges)
            angle_min = self._latest_scan.angle_min
            angle_step = self._latest_scan.angle_increment
            
            # Convert particle position to grid coordinates
            particle_x = pose.position.x
            particle_y = pose.position.y
            particle_heading = self.extract_yaw(pose.orientation)

            map_resolution = self._map_data['resolution']
            map_origin_x = self._map_data['origin']['x']
            map_origin_y = self._map_data['origin']['y']
            
            def check_cell_occupancy(x, y):
                """Determine if map cell is occupied"""
                grid_x = int((x - map_origin_x) / map_resolution)
                grid_y = int((y - map_origin_y) / map_resolution)
                if 0 <= grid_x < self._map_data['width'] and 0 <= grid_y < self._map_data['height']:
                    index = grid_y * self._map_data['width'] + grid_x
                    if 0 <= index < len(self._map_data['data']):
                        return self._map_data['data'][index] > 50
                return False

            # Generate simulated scan by casting rays
            for i in range(beam_count):
                angle = particle_heading + angle_min + i * angle_step
                distance = 0.0
                obstacle_detected = False
                
                # Step along ray until obstacle or max range
                while distance < max_range and not obstacle_detected:
                    distance += 0.05  # ray increment
                    ray_x = particle_x + distance * np.cos(angle)
                    ray_y = particle_y + distance * np.sin(angle)

                    # Check for obstacle collision
                    if check_cell_occupancy(ray_x, ray_y):
                        obstacle_detected = True
                
                # Record ray distance
                virtual_ranges.append(distance if obstacle_detected else max_range)
            
            return virtual_ranges
        except Exception as e:
            self.get_logger().error(f'Error in simulated scan generation: {str(e)}')
            return []

    ########################## Raycasting Simulation End ############################
    ########################## Weight Calculation Begin ############################
    def compute_particle_weight(self, real_scan: LaserScan, simulated_scan: list):
        try:
            weight = 1.0
            sigma = 0.4  # Sensitivity parameter
            scale_factor = 1.0 / (np.sqrt(2 * np.pi) * sigma)

            # Validate scan length compatibility
            if len(real_scan.ranges) != len(simulated_scan):
                self.get_logger().warn(f'Scan length mismatch: real scan: {len(real_scan.ranges)}, simulated scan: {len(simulated_scan)}')
                return weight

            for real_range, simulated_range in zip(real_scan.ranges, simulated_scan):
                # Skip invalid measurements
                if real_range < real_scan.range_min or real_range > real_scan.range_max or np.isnan(real_range) or np.isinf(real_range):
                    continue

                difference = real_range - simulated_range
                # Prevent numerical underflow
                probability = max(scale_factor * np.exp(-(difference ** 2) / (2 * sigma ** 2)), 1e-10)
                weight *= probability

            return weight
        except Exception as e:
            self.get_logger().error(f'Error computing particle weight: {str(e)}')
            return 1.0

    def recalculate_weights(self):
        try:
            if self._latest_scan is None or self._map_data is None:
                self.get_logger().warn('Missing scan or map data, weight update skipped')
                return

            # Update each particle's weight based on scan matching
            for particle in self._particles:
                previous_weight = particle.weight
                simulated_scan = self.generate_simulated_scan(particle.pose)
                if not simulated_scan:  # Empty scan data
                    continue
                new_weight = self.compute_particle_weight(self._latest_scan, simulated_scan)
                particle.weight = new_weight
                self.get_logger().debug(f"Particle previous weight: {previous_weight:.6f}, updated weight: {new_weight:.6f}")

            # Normalize all weights
            self.normalize_particle_weights()

            # Select representative particles
            self.perform_resampling()
        except Exception as e:
            self.get_logger().error(f'Error recalculating weights: {str(e)}')

    def normalize_particle_weights(self):
        try:
            weight_sum = sum(particle.weight for particle in self._particles)
            if weight_sum <= 0:
                self.get_logger().error('Total weight is zero or negative. Check weight calculation.')
                # Reset to uniform distribution
                uniform_weight = 1.0 / len(self._particles)
                for particle in self._particles:
                    particle.weight = uniform_weight
            else:
                for particle in self._particles:
                    particle.weight /= weight_sum
                
                # Verify normalization (should be close to 1.0)
                normalized_sum = sum(particle.weight for particle in self._particles)
                self.get_logger().debug(f"Sum of normalized weights: {normalized_sum:.6f}")
        except Exception as e:
            self.get_logger().error(f'Error normalizing weights: {str(e)}')

    def strengthen_best_particle(self):
        try:
            # Identify highest weight particle
            if not self._particles:
                self.get_logger().error('No particles to strengthen')
                return
                
            best_particle = max(self._particles, key=lambda p: p.weight)
            optimal_pose = best_particle.pose

            # Add additional particles near the best one
            for i in range(5):  # Add 5 reinforcement particles
                x = np.random.normal(loc=optimal_pose.position.x, scale=0.01)
                y = np.random.normal(loc=optimal_pose.position.y, scale=0.01)
                heading = self.extract_yaw(optimal_pose.orientation)
                heading += np.random.normal(scale=0.005)
                new_pose = Pose(
                    position=Point(x=x, y=y, z=0.0),
                    orientation=self.euler_to_quaternion(heading, 0.0, 0.0)
                )
                self._particles.append(ParticleEntity(new_pose, weight=1.0 / len(self._particles)))
            
            self.normalize_particle_weights()
        except Exception as e:
            self.get_logger().error(f'Error strengthening best particle: {str(e)}')

    ########################## Weight Calculation End ############################
    ########################## Resampling Begin ##############################
    def perform_resampling(self):
        try:
            if not self._particles:
                self.get_logger().error('No particles available for resampling')
                return
                
            resampled_particles = []
            weights = [particle.weight for particle in self._particles]
            
            # Validate weight integrity
            if any(np.isnan(w) for w in weights) or sum(weights) <= 0:
                self.get_logger().error('Invalid weight values detected. Reinitializing particle system.')
                self._setup_gaussian_particles()
                return
            
            cumulative_weights = np.cumsum(weights)
            if len(cumulative_weights) > 0:
                cumulative_weights[-1] = 1.0  # Ensure exact sum of 1.0
            else:
                self.get_logger().error('No weights available for cumulative sum')
                return

            interval = 1.0 / len(self._particles)
            starting_point = np.random.uniform(0, interval)
            current_index = 0
            for _ in range(len(self._particles)):
                while current_index < len(cumulative_weights) - 1 and starting_point > cumulative_weights[current_index]:
                    current_index += 1
                if current_index < len(self._particles):  # Bounds check
                    selected_pose = self._particles[current_index].pose
                    resampled_particles.append(ParticleEntity(selected_pose, weight=1.0 / len(self._particles)))
                starting_point += interval

            # Replace particle set with resampled version
            self._particles = resampled_particles
            
            # Add diversity around best particle
            self.strengthen_best_particle()
        except Exception as e:
            self.get_logger().error(f'Error during resampling: {str(e)}')
    
    ########################## Resampling End ##############################
    ################### Publishing Methods Begin ##################################################

    def _update_odometry_path(self, odom_pose: Pose):
        try:
            timestamp = self.get_clock().now().to_msg()
            self._odometry_path.header.frame_id = 'map'
            self._odometry_path.header.stamp = timestamp
            path_pose = PoseStamped()
            path_pose.header = self._odometry_path.header
            path_pose.pose = odom_pose
            self._odometry_path.poses.append(path_pose)
            self._odometry_path_pub.publish(self._odometry_path)
        except Exception as e:
            self.get_logger().error(f'Error updating odometry path: {str(e)}')

    def _publish_particle_cloud(self):
        try:
            if len(self._particles) == 0:
                self.get_logger().warn('No particles available to publish')
                return
                
            cloud_msg = PoseArray()
            cloud_msg.header.frame_id = 'map'
            cloud_msg.header.stamp = self.get_clock().now().to_msg()
            for particle in self._particles:
                cloud_msg.poses.append(particle.pose)
            self._particle_cloud_pub.publish(cloud_msg)
            self.get_logger().debug(f'Published cloud with {len(cloud_msg.poses)} particles')
            
            # Publish position estimate from best particle
            if self._particles:
                best_particle = max(self._particles, key=lambda p: p.weight)
                position_msg = Odometry()
                position_msg.header.frame_id = 'map'
                position_msg.header.stamp = self.get_clock().now().to_msg()
                position_msg.child_frame_id = 'base_link'
                position_msg.pose.pose = best_particle.pose
                self.pose_estimation_pub.publish(position_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing particle cloud: {str(e)}')
    
    ################### Publishing Methods End ##################################################

def main(args=None):
    print("Launching Particle Filter Localizer node...")
    try:
        rclpy.init(args=args)
        print("ROS 2 initialization successful")
    except Exception as e:
        print(f"ROS 2 initialization failed: {e}")
        traceback.print_exc()
        sys.exit(1)
        
    localizer_node = None
    
    try:
        print("Creating ParticleFilterLocalizer instance...")
        localizer_node = ParticleFilterLocalizer()
        print("ParticleFilterLocalizer created successfully, beginning execution...")
        rclpy.spin(localizer_node)
    except KeyboardInterrupt:
        print("Keyboard interruption detected, shutting down...")
    except Exception as e:
        print(f"Exception encountered: {e}")
        traceback.print_exc()
    finally:
        if localizer_node:
            print("Destroying node instance...")
            localizer_node.destroy_node()
        print("Shutting down ROS 2 client...")
        rclpy.shutdown()
        print("Node shutdown complete.")

if __name__ == '__main__':
    main()
