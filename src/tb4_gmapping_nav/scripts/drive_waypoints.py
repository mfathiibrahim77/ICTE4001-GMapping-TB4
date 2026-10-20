#!/usr/bin/env python3
"""
drive_waypoints - send a list of goals to Nav2 one after another (repeatable experiments).

    ros2 run tb4_gmapping_nav drive_waypoints.py <waypoints.yaml> [--loops 2] [--return-home]

Waypoints are in the MAP frame. With GMapping the map origin is the robot's pose when SLAM
started, so the same waypoint file reproduces the same route on every run if the robot always
starts from the same spot (e.g. the dock). Example: config/waypoints_example.yaml

Uses the nav2_msgs/action/NavigateToPose action of bt_navigator directly (works with SLAM:
unlike nav2_simple_commander.BasicNavigator it does not wait for AMCL).
"""
import argparse
import math
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


def make_pose(node, x, y, yaw, frame='map'):
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(float(yaw) / 2.0)
    p.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return p


class WaypointDriver(Node):
    def __init__(self):
        super().__init__('drive_waypoints')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def go(self, x, y, yaw, timeout):
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(self, x, y, yaw)
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
        result = handle.get_result_async()
        t0 = time.time()
        while not result.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - t0 > timeout:
                self.get_logger().warn('Timeout - cancelling goal')
                rclpy.spin_until_future_complete(self, handle.cancel_goal_async())
                return False
        ok = result.result().status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(f'  -> {"reached" if ok else "FAILED"} in {time.time() - t0:.1f} s')
        return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('waypoints', help='YAML file with a list "waypoints: [[x, y, yaw], ...]"')
    ap.add_argument('--loops', type=int, default=1)
    ap.add_argument('--return-home', action='store_true', help='finish at (0, 0, 0) = SLAM start pose')
    ap.add_argument('--timeout', type=float, default=120.0, help='[s] per goal')
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    with open(args.waypoints) as f:
        wps = yaml.safe_load(f)['waypoints']

    rclpy.init()
    node = WaypointDriver()
    node.get_logger().info('Waiting for Nav2 navigate_to_pose action server ...')
    node.client.wait_for_server()

    route = [tuple(w) for w in wps] * args.loops
    if args.return_home:
        route.append((0.0, 0.0, 0.0))
    n_ok = 0
    t_start = time.time()
    for i, (x, y, yaw) in enumerate(route, 1):
        node.get_logger().info(f'[{i}/{len(route)}] goal x={x:.2f} y={y:.2f} yaw={yaw:.2f}')
        n_ok += node.go(x, y, yaw, args.timeout)
    node.get_logger().info(f'Done: {n_ok}/{len(route)} goals reached in {time.time() - t_start:.0f} s')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
