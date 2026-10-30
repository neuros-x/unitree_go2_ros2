#!/usr/bin/env python3
"""Relay Gazebo's /clock to ROS /clock, and recover if the subscription is lost.

ros_gz_bridge subscribes to gz /clock once. If gz-transport discovery loses the
server's advertisement (it happens when the bridge and the server start
together, and intermittently on a host running several Gazebo instances), the
bridge never receives a clock message and never retries. Without ROS /clock,
controller_manager and the CHAMP nodes never tick and the Go2 lies on the floor
while its sensors keep publishing.

This node does the same job as the bridge's /clock entry, but on a wall-clock
watchdog: when no clock message has arrived for --timeout seconds it throws
away its gz-transport node and subscribes again from a new one, which forces a
fresh discovery round (a new bridge process always receives the clock).

    ros2 run unitree_go2_sim gz_clock_relay.py [--topic /clock] [--timeout 2.0]
"""
import argparse
import threading
import time

import rclpy
from gz.msgs10.clock_pb2 import Clock as GzClock
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class ClockRelay(Node):
    def __init__(self, topic, timeout):
        # Wall time on purpose: this node is what makes sim time exist.
        super().__init__('gz_clock_relay', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=False)])
        self.topic, self.timeout = topic, timeout
        self.pub = self.create_publisher(
            Clock, '/clock', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.lock = threading.Lock()
        self.last = None
        self.started = time.monotonic()
        self.resubscribes = 0
        self.gz = None
        self._subscribe()
        self.create_timer(0.5, self._watchdog)

    def _subscribe(self):
        self.gz = GzNode()     # a new node means a new discovery round
        if not self.gz.subscribe(GzClock, self.topic, self._on_clock):
            self.get_logger().error(f'could not subscribe to gz {self.topic}')

    def _on_clock(self, msg):
        out = Clock()
        out.clock.sec, out.clock.nanosec = msg.sim.sec, msg.sim.nsec
        self.pub.publish(out)
        with self.lock:
            first = self.last is None
            self.last = time.monotonic()
        if first:
            self.get_logger().info(
                f'receiving gz {self.topic} (after {self.resubscribes} resubscription(s))')

    def _watchdog(self):
        with self.lock:
            last = self.last
        silent = time.monotonic() - (last if last is not None else self.started)
        if silent > self.timeout:
            self.resubscribes += 1
            self.get_logger().warn(
                f'no gz {self.topic} for {silent:.1f} s; resubscribing '
                f'(attempt {self.resubscribes})')
            with self.lock:
                self.last = None
                self.started = time.monotonic()
            self._subscribe()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/clock')
    ap.add_argument('--timeout', type=float, default=2.0)
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = ClockRelay(a.topic, a.timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
