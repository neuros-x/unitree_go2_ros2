# Go2 simulation and navigation: how the pieces connect

## Processes and topics

```
Gazebo Harmonic (gz sim, bullet-featherstone 1 ms)
  model: unitree_go2_description (xacro)
    gz_ros2_control  -> /controller_manager (runs inside gz; cycles on ROS /clock)
    sensors          -> gz /imu/data, /gps/fix, /velodyne_points/points,
                        /unitree_lidar/points, /rgb_image, /d455/*, /*_foot_contacts
    OdometryPublisher-> gz /odom/ground_truth
        |
  ros_gz_bridge (started after the spawner exits; see "Clock" below)
        |
ROS 2 Jazzy
  /clock, /imu/data, /gps/fix, /odom/ground_truth, /velodyne_points/points, ...
  joint_states_controller, joint_group_effort_controller (ros2_control)
  CHAMP quadruped_controller_node : /cmd_vel -> joint trajectories (gait)
  CHAMP state_estimation_node     : joint states + IMU -> odom/raw, foot poses
  base_to_footprint_ekf           : base_footprint -> base_link (roll/pitch/height)
  footprint_to_odom_ekf           : odom -> base_footprint, /odom
  unitree_application foot_contacts_bridge : gz contacts -> champ_msgs
```

Navigation (`unitree_go2_navigation`) adds:

```
/velodyne_points/points
  -> pointcloud_to_laserscan (target base_footprint, 0.12-1.2 m above floor) -> /scan_raw
  -> laser_filters LaserScanBoxFilter (body box around base_link)          -> /scan
/scan -> slam_toolbox (online async)  -> /map, map -> odom
      or map_server + AMCL            -> /map, map -> odom
/scan + /map -> Nav2: planner_server (NavFn) -> controller_server (MPPI, 10 Hz)
             -> velocity_smoother -> collision_monitor -> /cmd_vel (Twist) -> CHAMP
```

## TF tree

```
map --(slam_toolbox | AMCL | static identity when publish_map_odom_tf:=true)--> odom
odom --(footprint_to_odom_ekf)--> base_footprint
base_footprint --(base_to_footprint_ekf)--> base_link
base_link --(robot_state_publisher)--> trunk, legs, imu_link, velodyne, gps_link, camera_link, ...
```

`base_footprint` is level and sits on the floor under the body (base_link is
0.22 m above it when standing), so it is the frame for 2D navigation and for
the scan's height window.

## Clock

Everything except Gazebo runs on simulation time, so ROS `/clock` is required.
gz advertises `/clock` while the server starts. A `ros_gz_bridge` started at
the same moment can miss that advertisement (gz-transport discovery) and never
receive the topic, while topics advertised later (the sensors) still work.
The symptom: sensors publish at full rate, `/clock` is silent,
`controller_manager` logs "No clock received", CHAMP never ticks and the robot
lies on the floor (sim time stays at zero). This used to be described as
"the Go2 deadlocks Gazebo". "Detected jump back in time" warnings are a
different fault: sim time going backwards, i.e. two clock sources. Starting the bridge after the spawn
reduces the chance but did not remove it (it failed again with three other
Gazebo instances on the host). ROS `/clock` therefore comes from
`unitree_go2_sim/scripts/gz_clock_relay.py`: it relays gz `/clock` and, when no
message has arrived for 2 s, discards its gz-transport node and subscribes from
a new one, forcing a new discovery round. The sensor bridges still start from
an `OnProcessExit` handler on the spawner.

A second trap: signalling only the `ros2 launch` process (`kill <pid>`) stops
`ros_gz_sim`'s ruby wrapper but leaves the `gz sim` server running; Ctrl-C in a
terminal signals the whole process group and is fine. Two servers in one
`GZ_PARTITION` publish interleaved clocks. Check `pgrep -af "gz sim"` between runs.

## Namespaces (NEUROS-X)

Under a namespace (`/go2_0` in NEUROS-X) the description must be built with
`ros_namespace:=go2_0`. gz_ros2_control then starts `/go2_0/controller_manager`
and reads `/go2_0/robot_description`. Without it, the manager sits at the root
and waits for a root `/robot_description`. The wait happens inside the plugin's
Configure, which holds the Gazebo server, so the whole simulation freezes.
CHAMP's command topic is the relative `cmd_vel/smooth`, so remap rules must be
relative too.

## Measuring

- `tools/go2_sim_check.py`: per-topic rates, real-time factor, and ground-truth
  distance walked for a `/cmd_vel` command (`--ns /go2_0 --gt-topic /odom` for
  the NEUROS-X launch).
- `unitree_go2_navigation/scripts/go2_nav_check.py`: sends NavigateToPose goals
  and reports status, time, path length and ground-truth arrival error.
