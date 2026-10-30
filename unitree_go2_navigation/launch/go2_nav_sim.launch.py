"""One command: Go2 in Gazebo + SLAM (or AMCL on a saved map) + Nav2.

    ros2 launch unitree_go2_navigation go2_nav_sim.launch.py               # indoor, SLAM
    ros2 launch unitree_go2_navigation go2_nav_sim.launch.py world:=outdoor
    ros2 launch unitree_go2_navigation go2_nav_sim.launch.py \
        map:=$(ros2 pkg prefix unitree_go2_navigation)/share/unitree_go2_navigation/maps/go2_indoor.yaml

world accepts 'indoor', 'outdoor', 'default' or a path to an .sdf file. Navigation
starts once the effort controller is up (nav_delay, wall seconds), so Nav2 never
sees a robot that is still lying on the floor.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

WORLDS = {'indoor': 'go2_indoor.sdf', 'outdoor': 'go2_outdoor.sdf', 'default': 'default.sdf'}


def _setup(context):
    desc = get_package_share_directory('unitree_go2_description')
    sim = get_package_share_directory('unitree_go2_sim')
    here = get_package_share_directory('unitree_go2_navigation')
    world = LaunchConfiguration('world').perform(context)
    world = os.path.join(desc, 'worlds', WORLDS[world]) if world in WORLDS else world
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim, 'launch', 'unitree_go2_launch.py')),
            launch_arguments={
                'world': world,
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
                'world_init_x': LaunchConfiguration('x'),
                'world_init_y': LaunchConfiguration('y'),
                'world_init_heading': LaunchConfiguration('yaw'),
                'publish_map_odom_tf': 'false',
            }.items()),
        TimerAction(
            period=float(LaunchConfiguration('nav_delay').perform(context)),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(here, 'launch', 'navigation.launch.py')),
                launch_arguments={'map': LaunchConfiguration('map'),
                                  'self_filter': LaunchConfiguration('self_filter')}.items())]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='indoor'),
        DeclareLaunchArgument('map', default_value='',
                              description="Saved map yaml for AMCL; '' maps with SLAM"),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('self_filter', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('nav_delay', default_value='40.0'),
        OpaqueFunction(function=_setup),
    ])
