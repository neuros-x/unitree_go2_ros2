"""Online 2D SLAM for the Go2: perception (cloud -> /scan) + slam_toolbox (async).

slam_toolbox publishes map -> odom and /map; run the sim with
publish_map_odom_tf:=false. Save the map with
    ros2 run nav2_map_server map_saver_cli -f <name> --ros-args -p use_sim_time:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    here = get_package_share_directory('unitree_go2_navigation')
    slam_share = get_package_share_directory('slam_toolbox')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('self_filter', default_value='true'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(here, 'config', 'slam_toolbox.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(here, 'launch', 'perception.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'self_filter': LaunchConfiguration('self_filter')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, 'launch', 'online_async_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'slam_params_file': LaunchConfiguration('slam_params_file'),
                              'autostart': 'true'}.items()),
    ])
