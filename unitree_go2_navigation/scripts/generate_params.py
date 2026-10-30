#!/usr/bin/env python3
"""Write config/nav2_params.yaml and config/slam_toolbox.yaml for the Go2.

Both start from the stock Jazzy files (nav2_bringup's nav2_params.yaml and
slam_toolbox's mapper_params_online_async.yaml) and override only what the Go2
needs, so a Nav2 upgrade can be absorbed by re-running this script:

    python3 scripts/generate_params.py

Go2 specifics: CHAMP's planar base frame is base_footprint; the body is about
0.70 x 0.31 m (footprint padded to 0.76 x 0.40); CHAMP walks comfortably up to
about 0.4 m/s and 0.8 rad/s; the obstacle source is the 2D /scan made from the
Velodyne cloud (see config/perception.yaml), so the 3D voxel layer of the stock
local costmap is replaced by a scan obstacle layer.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, '..', 'config')

BASE = 'base_footprint'
FOOTPRINT = '[[0.38, 0.20], [0.38, -0.20], [-0.38, -0.20], [-0.38, 0.20]]'
SCAN_SOURCE = {
    'topic': '/scan', 'data_type': 'LaserScan', 'clearing': True, 'marking': True,
    'max_obstacle_height': 2.0, 'raytrace_max_range': 10.0, 'raytrace_min_range': 0.0,
    'obstacle_max_range': 8.0, 'obstacle_min_range': 0.0,
}
INFLATION = {'plugin': 'nav2_costmap_2d::InflationLayer', 'cost_scaling_factor': 3.0,
             'inflation_radius': 0.6}


def nav2():
    src = os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')
    d = yaml.safe_load(open(src))
    d.pop('loopback_simulator', None)

    d['amcl']['ros__parameters'].update({
        'base_frame_id': BASE, 'scan_topic': 'scan', 'laser_max_range': 30.0,
        'max_beams': 120, 'set_initial_pose': True,
        'initial_pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}})

    d['bt_navigator']['ros__parameters'].update({'robot_base_frame': BASE, 'odom_topic': '/odom'})

    c = d['controller_server']['ros__parameters']
    c['progress_checker'].update({'required_movement_radius': 0.3, 'movement_time_allowance': 15.0})
    c['general_goal_checker'].update({'xy_goal_tolerance': 0.3, 'yaw_goal_tolerance': 0.5})
    # MPPI at 10 Hz with 1000 x 40 rollouts: the stock 20 Hz x 2000 x 56 missed
    # its rate (9.5 Hz) on a shared CPU, and the legged gait gains nothing from it.
    c['controller_frequency'] = 10.0
    c['FollowPath'].update({'batch_size': 1000, 'time_steps': 40, 'model_dt': 0.1,
                            'prune_distance': 2.0})
    c['FollowPath'].update({
        'motion_model': 'DiffDrive', 'vx_max': 0.4, 'vx_min': -0.2, 'vy_max': 0.0,
        'wz_max': 0.8, 'ax_max': 1.0, 'ax_min': -1.0, 'ay_max': 0.0, 'ay_min': 0.0,
        'az_max': 1.5, 'visualize': False})

    for name in ('local_costmap', 'global_costmap'):
        p = d[name][name]['ros__parameters']
        p.pop('robot_radius', None)
        p.update({'robot_base_frame': BASE, 'footprint': FOOTPRINT, 'footprint_padding': 0.03})
        p['inflation_layer'] = dict(INFLATION)
        p['obstacle_layer'] = {'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True,
                               'observation_sources': 'scan', 'scan': dict(SCAN_SOURCE)}
    lc = d['local_costmap']['local_costmap']['ros__parameters']
    lc.pop('voxel_layer', None)
    lc.update({'width': 5, 'height': 5, 'plugins': ['obstacle_layer', 'inflation_layer']})
    gc = d['global_costmap']['global_costmap']['ros__parameters']
    gc['plugins'] = ['static_layer', 'obstacle_layer', 'inflation_layer']

    d['behavior_server']['ros__parameters'].update({'robot_base_frame': BASE,
                                                   'max_rotational_vel': 0.8})
    d['velocity_smoother']['ros__parameters'].update({
        'max_velocity': [0.4, 0.0, 0.8], 'min_velocity': [-0.2, 0.0, -0.8],
        'max_accel': [1.0, 0.0, 1.5], 'max_decel': [-1.0, 0.0, -1.5]})
    d['collision_monitor']['ros__parameters'].update({'base_frame_id': BASE})
    d['collision_monitor']['ros__parameters']['scan']['topic'] = '/scan'
    d['docking_server']['ros__parameters'].update({'base_frame': BASE})
    return d, src


def slam():
    src = os.path.join(get_package_share_directory('slam_toolbox'), 'config',
                       'mapper_params_online_async.yaml')
    d = yaml.safe_load(open(src))
    d['slam_toolbox']['ros__parameters'].update({
        'base_frame': BASE, 'odom_frame': 'odom', 'map_frame': 'map', 'scan_topic': '/scan',
        'mode': 'mapping', 'resolution': 0.05, 'max_laser_range': 20.0,
        'minimum_travel_distance': 0.2, 'minimum_travel_heading': 0.2,
        'map_update_interval': 2.0, 'transform_timeout': 0.2, 'tf_buffer_duration': 30.0,
        'do_loop_closing': True, 'use_sim_time': True})
    return d, src


if __name__ == '__main__':
    for fname, (data, src) in (('nav2_params.yaml', nav2()), ('slam_toolbox.yaml', slam())):
        path = os.path.normpath(os.path.join(CONFIG, fname))
        with open(path, 'w') as f:
            f.write(f'# Generated by scripts/generate_params.py from {src}\n'
                    '# with Go2 overrides. Edit the script, not this file.\n')
            yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)
        print('wrote', path)
