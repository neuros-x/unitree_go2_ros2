#!/usr/bin/env python3
"""Measure a running Go2 simulation: sensor rates, sim-time progress, walking.

Run it next to `ros2 launch unitree_go2_sim unitree_go2_launch.py` (same
ROS_DOMAIN_ID). It listens for --listen seconds, reports the rate of each
topic and how far /clock advanced, then (unless --no-walk) commands a forward
walk on /cmd_vel and reports the ground-truth displacement.

    python3 tools/go2_sim_check.py --json /tmp/go2_check.json

Exit code 0 only when every required topic published and the robot moved
at least --min-distance metres.
"""
import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, NavSatFix, PointCloud2

TOPICS = {
    '/clock': Clock,
    '/imu/data': Imu,
    '/gps/fix': NavSatFix,
    '/velodyne_points/points': PointCloud2,
    '/odom/ground_truth': Odometry,
    '/odom': Odometry,
    '/joint_states': JointState,
}


class Check(Node):
    def __init__(self, ns, gt_topic):
        super().__init__('go2_sim_check')
        self.ns = ns.rstrip('/')
        self.gt_topic = gt_topic
        self.counts = {t: 0 for t in TOPICS}
        self.first = {}
        self.last_clock = None
        self.first_clock = None
        self.pose = None
        for topic, typ in TOPICS.items():
            name = '/clock' if topic == '/clock' else self.ns + topic   # /clock is global
            if topic == '/odom/ground_truth':
                name = self.ns + gt_topic
            self.create_subscription(typ, name,
                                     lambda m, t=topic: self._on(t, m),
                                     qos_profile_sensor_data)
        self.cmd = self.create_publisher(Twist, self.ns + '/cmd_vel', 10)

    def _on(self, topic, msg):
        self.counts[topic] += 1
        self.first.setdefault(topic, msg)
        if topic == '/clock':
            t = msg.clock.sec + msg.clock.nanosec * 1e-9
            if self.first_clock is None:
                self.first_clock = t
            self.last_clock = t
        elif topic == '/odom/ground_truth':
            p = msg.pose.pose.position
            self.pose = (p.x, p.y, p.z)


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='', help="robot namespace, e.g. /go2_0 (neuros_sim)")
    ap.add_argument('--gt-topic', default='/odom/ground_truth',
                    help="Gazebo ground-truth odometry under --ns (neuros_sim bridges it to /odom)")
    ap.add_argument('--listen', type=float, default=10.0)
    ap.add_argument('--walk-speed', type=float, default=0.3)
    ap.add_argument('--walk-time', type=float, default=10.0)
    ap.add_argument('--min-distance', type=float, default=0.5)
    ap.add_argument('--no-walk', action='store_true')
    ap.add_argument('--optional', nargs='*', default=['/odom'],
                    help='topics that may be silent without failing the check')
    ap.add_argument('--json')
    a = ap.parse_args()

    rclpy.init()
    node = Check(a.ns, a.gt_topic)
    spin_for(node, 2.0)                       # discovery
    node.counts = {t: 0 for t in TOPICS}
    node.first_clock = None
    wall0 = time.monotonic()
    spin_for(node, a.listen)
    wall = time.monotonic() - wall0
    res = {'listen_wall_s': round(wall, 2),
           'rates_hz': {t: round(c / wall, 2) for t, c in node.counts.items()}}
    if node.first_clock is not None:
        res['sim_s'] = round(node.last_clock - node.first_clock, 2)
        res['rtf'] = round(res['sim_s'] / wall, 3)
    pc = node.first.get('/velodyne_points/points')
    if pc is not None:
        res['velodyne_points_per_msg'] = pc.width * pc.height
    fix = node.first.get('/gps/fix')
    if fix is not None:
        res['gps_first_fix'] = [round(fix.latitude, 7), round(fix.longitude, 7),
                                round(fix.altitude, 2)]

    if not a.no_walk:
        start = node.pose
        tw = Twist()
        tw.linear.x = a.walk_speed
        end = time.monotonic() + a.walk_time
        while time.monotonic() < end:
            node.cmd.publish(tw)
            spin_for(node, 0.1)
        node.cmd.publish(Twist())
        spin_for(node, 1.0)
        if start and node.pose:
            dx, dy = node.pose[0] - start[0], node.pose[1] - start[1]
            res['walk'] = {'cmd_mps': a.walk_speed, 'cmd_s': a.walk_time,
                           'start': [round(v, 3) for v in start],
                           'end': [round(v, 3) for v in node.pose],
                           'distance_m': round(math.hypot(dx, dy), 3),
                           'base_z_end': round(node.pose[2], 3)}
        else:
            res['walk'] = {'error': 'no ground-truth odometry'}

    missing = [t for t, r in res['rates_hz'].items()
               if r == 0 and t not in a.optional]
    res['missing'] = missing
    walked = res.get('walk', {}).get('distance_m', 0.0) if not a.no_walk else None
    res['ok'] = not missing and (a.no_walk or walked >= a.min_distance)
    out = json.dumps(res, indent=2)
    print(out)
    if a.json:
        with open(a.json, 'w') as f:
            f.write(out + '\n')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if res['ok'] else 1)


if __name__ == '__main__':
    main()
