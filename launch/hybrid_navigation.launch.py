#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Launch the SLAM Toolbox
    slam_toolbox_cmd = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='async_slam_toolbox_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_laser_range': 5.6,
            'min_laser_range': 0.1,
            'resolution': 0.05,
            'map_update_interval': 1.0,
            'enable_interactive_mode': False,
        }]
    )
    
    # AMCL for localization
    amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.2,
            'base_frame_id': "base_link",
            'beam_skip_distance': 0.5,
            'beam_skip_error_threshold': 0.3,
            'beam_skip_threshold': 0.3,
            'do_beamskip': False,
            'global_frame_id': 'map',
            'lambda_short': 0.1,
            'laser_likelihood_max_dist': 2.0,
            'laser_max_range': 5.6,
            'laser_min_range': 0.1,
            'laser_model_type': 'likelihood_field',
            'max_beams': 60,
            'max_particles': 2000,
            'min_particles': 500,
            'odom_frame_id': 'odom',
            'pf_err': 0.05,
            'pf_z': 0.99,
            'recovery_alpha_fast': 0.0,
            'recovery_alpha_slow': 0.0,
            'resample_interval': 1,
            'robot_model_type': 'differential',
            'save_pose_rate': 0.5,
            'sigma_hit': 0.2,
            'tf_broadcast': True,
            'transform_tolerance': 2.0,
            'update_min_a': 0.2,
            'update_min_d': 0.25,
            'z_hit': 0.5,
            'z_max': 0.05,
            'z_rand': 0.5,
            'z_short': 0.05,
            'scan_topic': 'scan',
            'set_initial_pose': True,
            'initial_pose.x': 0.0,
            'initial_pose.y': 0.0,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': 0.0,
        }]
    )
    
    # A* Planner Node
    astar_cmd = Node(
            package='global_path_planner',
            executable='astar',
            name='astar',
            output='screen',
            parameters=[{'min_threshold': 5, 'use_sim_time': use_sim_time}],
        )
    
    # Local Planner (Potential Field)
    local_planner_cmd = Node(
            package='local_path_planner',
            executable='local_planner',
            name='local_planner',
            output='screen',
            parameters=[{'k_rep': 0.5, 'k_att': 0.8, 'use_sim_time': use_sim_time,}],
        )
    
    # Exploration Node
    explorer_cmd = Node(
            package='exploration',
            executable='explore',
            name='integrated_explorer',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        )
    
    # TF Monitor for debugging
    tf_monitor = Node(
        package="tf2_ros",
        executable="tf2_monitor",
        name='tf_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'buffer_size': 10.0
        }],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # RViz configuration
    rviz_config = os.path.join(
        get_package_share_directory('robile_navigation'),
        'rviz',
        'navigation.rviz'
    )

    ld = LaunchDescription()
    
    ld.add_action(slam_toolbox_cmd)
    ld.add_action(amcl_cmd)
    ld.add_action(astar_cmd)
    ld.add_action(local_planner_cmd)
    ld.add_action(explorer_cmd)
    ld.add_action(tf_monitor)
    
    return ld