#!/usr/bin/env python3
"""Check several Go2s in one world: each one walks alone, the others stay put.

Walks each robot in turn on <ns>/cmd_vel and reports every robot's ground-truth
displacement, so crossed topics show up as the wrong robot moving. Also reports
per-robot sensor rates and whether sensor frames carry the robot's namespace.

    python3 tools/go2_fleet_check.py --ns /go2_0 /go2_1 --json /tmp/fleet.json

Exit code 0 only if every walker moved at least --min-distance and no bystander
moved more than --max-bystander.
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
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2

SENSORS = {'/imu/data': Imu, '/gps/fix': NavSatFix, '/velodyne_points/points': PointCloud2}


class FleetCheck(Node):
    def __init__(self, namespaces):
        super().__init__('go2_fleet_check')
        self.pose = {}
        self.counts = {(ns, t): 0 for ns in namespaces for t in SENSORS}
        self.frames = {}
        self.clock_sec = None
        self.clock_back = 0
        self.pubs = {ns: self.create_publisher(Twist, f'{ns}/cmd_vel', 10) for ns in namespaces}
        self.create_subscription(Clock, '/clock', self._on_clock, 100)
        for ns in namespaces:
            self.create_subscription(Odometry, f'{ns}/odom',
                                     lambda m, n=ns: self._odom(n, m), 10)
            for topic, typ in SENSORS.items():
                self.create_subscription(typ, ns + topic,
                                         lambda m, n=ns, t=topic: self._sensor(n, t, m),
                                         qos_profile_sensor_data)

    def _on_clock(self, msg):
        now = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.clock_sec is not None and now < self.clock_sec:
            self.clock_back += 1
        self.clock_sec = now

    def _odom(self, ns, msg):
        p = msg.pose.pose.position
        self.pose[ns] = (p.x, p.y)

    def _sensor(self, ns, topic, msg):
        self.counts[(ns, topic)] += 1
        self.frames[(ns, topic)] = msg.header.frame_id

    def spin_for(self, seconds, every=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if every:
                every()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', nargs='+', required=True)
    ap.add_argument('--listen', type=float, default=10.0)
    ap.add_argument('--walk-speed', type=float, default=0.3)
    ap.add_argument('--walk-time', type=float, default=10.0)
    ap.add_argument('--min-distance', type=float, default=1.0)
    ap.add_argument('--max-bystander', type=float, default=0.25)
    ap.add_argument('--json')
    a = ap.parse_args()

    rclpy.init()
    node = FleetCheck(a.ns)
    t0, c0 = time.monotonic(), None
    node.spin_for(1.0)
    c0 = node.clock_sec
    node.spin_for(a.listen)
    wall = time.monotonic() - t0 - 1.0
    res = {'rtf': round((node.clock_sec - c0) / wall, 3) if c0 and node.clock_sec else None,
           'rates_hz': {f'{ns}{t}': round(n / (wall + 1.0), 1)
                        for (ns, t), n in node.counts.items()},
           'frames': {f'{ns}{t}': f for (ns, t), f in node.frames.items()},
           'walks': {}}
    ok = bool(node.pose) and len(node.pose) == len(a.ns)
    tw, stop = Twist(), Twist()
    tw.linear.x = a.walk_speed
    for walker in a.ns:
        start = dict(node.pose)
        node.spin_for(a.walk_time, every=lambda w=walker: node.pubs[w].publish(tw))
        node.pubs[walker].publish(stop)
        node.spin_for(3.0, every=lambda w=walker: node.pubs[w].publish(stop))
        moved = {ns: round(math.dist(node.pose[ns], start[ns]), 3)
                 for ns in a.ns if ns in start and ns in node.pose}
        res['walks'][walker] = moved
        for ns, dist in moved.items():
            ok = ok and (dist >= a.min_distance if ns == walker else dist <= a.max_bystander)
    res['clock_backward_jumps'] = node.clock_back
    res['ok'] = ok
    print(json.dumps(res, indent=1))
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(res, f, indent=1)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
