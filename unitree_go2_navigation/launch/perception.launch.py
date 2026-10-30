"""Velodyne PointCloud2 -> /scan for SLAM and Nav2, with an optional self-filter.

    /velodyne_points/points --pointcloud_to_laserscan--> /scan_raw
    /scan_raw --laser_filters box (Go2 body envelope)--> /scan

With self_filter:=false the converter publishes /scan directly.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('unitree_go2_navigation'),
                          'config', 'perception.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cloud = LaunchConfiguration('cloud_topic')
    self_filter = LaunchConfiguration('self_filter')

    def converter(scan_out, condition):
        return Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[params, {'use_sim_time': use_sim_time}],
            remappings=[('cloud_in', cloud), ('scan', scan_out)],
            condition=condition,
        )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cloud_topic', default_value='/velodyne_points/points'),
        DeclareLaunchArgument('self_filter', default_value='true',
                              description='Drop scan points inside the Go2 body envelope'),
        converter('/scan_raw', IfCondition(self_filter)),
        converter('/scan', UnlessCondition(self_filter)),
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_self_filter',
            parameters=[params, {'use_sim_time': use_sim_time}],
            remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan')],
            condition=IfCondition(self_filter),
        ),
    ])
