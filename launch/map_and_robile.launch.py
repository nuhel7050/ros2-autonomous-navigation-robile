import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robile_launch_file = os.path.join(get_package_share_directory('robile_gazebo'), 'launch', 'gazebo_4_wheel.launch.py')

    map_file_path = os.path.join(get_package_share_directory('map_and_robile'), 'maps', 'map_01.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robile_launch_file),
        ),

        # No need to launch rviz since, gazebo_4_wheel.launch.py already launches rviz
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{"yaml_filename": map_file_path}]
        ),
    ]) 
