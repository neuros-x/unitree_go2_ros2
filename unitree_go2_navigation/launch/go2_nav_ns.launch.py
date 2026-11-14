"""Nav2 for a Go2 that runs under its own namespace (NEUROS-X's /go2_0).

    ros2 launch neuros_sim gazebo_sim.launch.py world:=go2_indoor 'robot:={go2,1}'
    ros2 launch unitree_go2_navigation go2_nav_ns.launch.py namespace:=go2_0

go2_nav_sim.launch.py runs the Go2 and Nav2 in the root namespace. A namespaced
Go2 (neuros_sim/launch/go2.launch.py) differs in three ways this launch handles:

- its TF frames carry the namespace (go2_0/base_footprint, go2_0/odom) on the
  shared /tf, so every frame in the Nav2 params is prefixed and nothing remaps /tf;
- its topics are under the namespace, so the scan pipeline, map server and Nav2
  run there too and Nav2's NavigateToPose is /<ns>/navigate_to_pose;
- its <ns>/odom is Gazebo's ground truth. Nav2 is given the EKF's <ns>/odom/filtered
  instead, so it navigates on the robot's own odometry, as on hardware.

Localisation (localization:=amcl, the default): AMCL on the saved map publishes
map -> <ns>/odom, starting from the spawn pose (x, y, yaw). Start the sim with
localization:=amcl so neuros_sim does not publish that transform too. CHAMP's leg
odometry under-reads distance by about half in sim, so localization:=odom (Nav2 on
neuros_sim's fixed anchor and odometry alone) is only for a quick look.
The params come from config/nav2_params.yaml and config/perception.yaml, rewritten
per namespace into ~/.cache/neuros/nav2/<ns>.yaml.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_FRAMES = ('base_footprint', 'base_link', 'odom')
NAV2_NODES = [('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
              ('nav2_smoother', 'smoother_server', []),
              ('nav2_planner', 'planner_server', []),
              ('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
              ('nav2_velocity_smoother', 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav')]),
              ('nav2_collision_monitor', 'collision_monitor', []),
              ('nav2_bt_navigator', 'bt_navigator', []),
              ('nav2_waypoint_follower', 'waypoint_follower', [])]


def namespaced(params, ns: str, key: str = ''):
    """Rewrite one node's parameters for a robot under /<ns> with prefixed frames."""
    if isinstance(params, dict):
        return {k: namespaced(v, ns, str(k)) for k, v in params.items()}
    if not isinstance(params, str):
        return params
    if key.endswith(('frame', 'frame_id')) and params in ROBOT_FRAMES:
        return f'{ns}/{params}'
    if key.endswith('topic') and params.lstrip('/') == 'odom':
        return f'/{ns}/odom/filtered'          # not <ns>/odom: that is ground truth
    if key.endswith('topic') and params.startswith('/'):
        return f'/{ns}{params}'
    return params


def _setup(context):
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    nav_map = LaunchConfiguration('map').perform(context)
    here = get_package_share_directory('unitree_go2_navigation')
    with open(os.path.join(here, 'config', 'nav2_params.yaml')) as f:
        nav2 = yaml.safe_load(f)
    with open(os.path.join(here, 'config', 'perception.yaml')) as f:
        perception = yaml.safe_load(f)
    amcl = LaunchConfiguration('localization').perform(context) == 'amcl'
    if amcl:
        x, y, yaw = (float(LaunchConfiguration(k).perform(context)) for k in ('x', 'y', 'yaw'))
        # CHAMP's leg odometry under-reads distance (0.91 m for 1.25 m walked, 27%,
        # measured against Gazebo on 2026-11-14) while its IMU heading is right.
        # With Nav2's 0.2 translation noise AMCL lost the Go2 crossing the building
        # (7 m off); the motion model is widened to the error it actually has.
        nav2['amcl']['ros__parameters'].update(
            set_initial_pose=True, initial_pose={'x': x, 'y': y, 'z': 0.0, 'yaw': yaw},
            alpha3=0.5, alpha4=0.3)
    else:
        nav2.pop('amcl', None)
    nav2.pop('docking_server', None)
    nav2.pop('route_server', None)
    nodes = {**namespaced(nav2, ns), **namespaced(perception, ns),
             'map_server': {'ros__parameters': {'yaml_filename': nav_map}}}
    for p in nodes.values():
        if isinstance(p, dict) and 'ros__parameters' in p:
            p['ros__parameters']['use_sim_time'] = use_sim_time
    for sub in ('global_costmap', 'local_costmap'):
        nodes[sub][sub]['ros__parameters']['use_sim_time'] = use_sim_time
    out_dir = os.path.expanduser('~/.cache/neuros/nav2')
    os.makedirs(out_dir, exist_ok=True)
    params = os.path.join(out_dir, f'{ns}.yaml')
    with open(params, 'w') as f:
        yaml.safe_dump({ns: nodes}, f, sort_keys=False)

    def node(package, executable, name=None, remappings=()):
        return Node(package=package, executable=executable, name=name or executable,
                    namespace=ns, output='screen', parameters=[params],
                    remappings=list(remappings))

    actions = [
        node('pointcloud_to_laserscan', 'pointcloud_to_laserscan_node', 'pointcloud_to_laserscan',
             [('cloud_in', f'/{ns}/velodyne_points/points'), ('scan', 'scan_raw')]),
        node('laser_filters', 'scan_to_scan_filter_chain', 'scan_self_filter',
             [('scan', 'scan_raw'), ('scan_filtered', 'scan')]),
        node('nav2_map_server', 'map_server'),
        *([node('nav2_amcl', 'amcl')] if amcl else []),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', namespace=ns, output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': ['map_server'] + (['amcl'] if amcl else [])}]),
    ]
    actions += [node(pkg, exe, remappings=remap) for pkg, exe, remap in NAV2_NODES]
    actions.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', namespace=ns, output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': [exe for _, exe, _ in NAV2_NODES]}]))
    return actions


def generate_launch_description():
    here = get_package_share_directory('unitree_go2_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='go2_0'),
        DeclareLaunchArgument('map', default_value=os.path.join(here, 'maps', 'go2_indoor.yaml'),
                              description='Saved map for the global costmap'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('localization', default_value='amcl',
                              description="'amcl' on the saved map, or 'odom' (no AMCL)"),
        DeclareLaunchArgument('x', default_value='0.0', description='AMCL initial pose'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=_setup),
    ])
