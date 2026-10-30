#!/usr/bin/env python3
"""Send the Go2 a sequence of Nav2 goals and measure how each one went.

    ros2 run unitree_go2_navigation go2_nav_check.py --goals "7,-4 -7,1 0,0" --json out.json

Goals are x,y in the map frame. With SLAM the map frame starts where the robot
starts, so world = map + start pose; the ground-truth error of each arrival is
measured from Gazebo's /odom/ground_truth under that assumption (spawn yaw 0).
Exit code 0 only if every goal succeeded and ended within --tolerance metres of
the goal in ground truth.
"""
import argparse
import json
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
          GoalStatus.STATUS_CANCELED: 'canceled'}


class NavCheck(Node):
    def __init__(self):
        super().__init__('go2_nav_check')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.gt = None
        self.anchor = None
        self.path_len = 0.0
        self.create_subscription(Odometry, '/odom/ground_truth', self._on_gt,
                                 qos_profile_sensor_data)

    def _on_gt(self, msg):
        p = msg.pose.pose.position
        self.gt = (p.x, p.y)
        # Path length from 10 cm steps: summing every 50 Hz sample would add the
        # gait's lateral sway (about 2 mm per sample) and overstate it severalfold.
        if self.anchor is None:
            self.anchor = self.gt
        step = math.hypot(p.x - self.anchor[0], p.y - self.anchor[1])
        if step >= 0.10:
            self.path_len += step
            self.anchor = self.gt

    def wait(self, fut, timeout):
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return fut.done()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--goals', default='7,-4 -7,1 0,0')
    ap.add_argument('--timeout', type=float, default=180.0, help='seconds per goal')
    ap.add_argument('--tolerance', type=float, default=0.6)
    ap.add_argument('--activation-wait', type=float, default=120.0,
                    help='seconds to keep retrying a goal that Nav2 rejects as inactive')
    ap.add_argument('--json')
    a = ap.parse_args()
    goals = [tuple(float(v) for v in g.split(',')) for g in a.goals.split()]

    rclpy.init()
    node = NavCheck()
    if not node.client.wait_for_server(timeout_sec=120.0):
        print('navigate_to_pose action server not available', file=sys.stderr)
        sys.exit(2)
    end = time.monotonic() + 10.0
    while node.gt is None and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    start = node.gt or (0.0, 0.0)
    results = []
    for gx, gy in goals:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
        goal.pose.pose.orientation.w = 1.0
        # bt_navigator rejects goals until its lifecycle manager has activated
        # it (with AMCL there are two managers, and localization comes up
        # first), so retry a rejection for up to --activation-wait seconds.
        deadline = time.monotonic() + a.activation_wait
        while True:
            t0, len0 = time.monotonic(), node.path_len
            send = node.client.send_goal_async(goal)
            accepted = node.wait(send, 15.0) and send.result().accepted
            if accepted or time.monotonic() > deadline:
                break
            end = time.monotonic() + 5.0
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        if not accepted:
            results.append({'goal': [gx, gy], 'status': 'rejected'})
            continue
        res = send.result().get_result_async()
        done = node.wait(res, a.timeout)
        status = STATUS.get(res.result().status, str(res.result().status)) if done else 'timeout'
        if not done:
            send.result().cancel_goal_async()
        wx, wy = gx + start[0], gy + start[1]
        err = math.hypot(node.gt[0] - wx, node.gt[1] - wy) if node.gt else None
        results.append({'goal': [gx, gy], 'status': status,
                        'time_s': round(time.monotonic() - t0, 1),
                        'walked_m': round(node.path_len - len0, 2),
                        'gt_error_m': None if err is None else round(err, 3)})
        print(json.dumps(results[-1]), flush=True)
    ok = all(r.get('status') == 'succeeded' and (r.get('gt_error_m') or 1e9) <= a.tolerance
             for r in results)
    out = {'start_gt': [round(v, 3) for v in start], 'goals': results, 'ok': ok}
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(out, f, indent=2)
    print(json.dumps({'ok': ok}))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
