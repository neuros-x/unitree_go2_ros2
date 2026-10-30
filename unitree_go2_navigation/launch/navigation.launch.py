"""Nav2 for the Go2 against a map from SLAM (default) or a saved map (map:=<yaml>).

map:=''        -> SLAM (slam.launch.py) supplies /map and map -> odom.
map:=<yaml>    -> map_server + AMCL (nav2_bringup localization_launch.py).

Nav2 itself is nav2_bringup's navigation_launch.py with config/nav2_params.yaml:
MPPI controller, NavFn planner, scan-based costmaps, velocity smoother and
collision monitor, all managed by one lifecycle manager. Its final output is a
plain geometry_msgs/Twist on /cmd_vel, which is what CHAMP subscribes to.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    here = get_package_share_directory('unitree_go2_navigation')
    nav2 = get_package_share_directory('nav2_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = LaunchConfiguration('params_file')
    nav_map = LaunchConfiguration('map')
    use_slam = PythonExpression(["'", nav_map, "' == ''"])
    use_amcl = PythonExpression(["'", nav_map, "' != ''"])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value='',
                              description="Saved map yaml for AMCL; '' runs SLAM instead"),
        DeclareLaunchArgument('self_filter', default_value='true'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(here, 'config', 'nav2_params.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(here, 'launch', 'slam.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'self_filter': LaunchConfiguration('self_filter')}.items(),
            condition=IfCondition(use_slam)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(here, 'launch', 'perception.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'self_filter': LaunchConfiguration('self_filter')}.items(),
            condition=IfCondition(use_amcl)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'localization_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time, 'map': nav_map,
                              'params_file': params, 'autostart': 'true'}.items(),
            condition=IfCondition(use_amcl)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time, 'params_file': params,
                              'autostart': 'true', 'use_composition': 'False'}.items()),
    ])
