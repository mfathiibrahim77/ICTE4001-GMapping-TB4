#!/usr/bin/env python3
"""
slam_eval_recorder - logs everything needed to evaluate GMapping to CSV files.

    ros2 run tb4_gmapping_nav slam_eval_recorder.py --ros-args -p run_name:=office1 -p use_sim_time:=true

Writes to ~/gmapping_runs/<run_name>_<YYYYmmdd_HHMMSS>/
  poses.csv  (10 Hz)
      t                  [s]   ROS time (sim time in Gazebo)
      slam_x/y/yaw       [m, m, rad]  pose of base_link in the map frame  (GMapping estimate)
      odom_x/y/yaw       [m, m, rad]  pose of base_link in the odom frame (dead reckoning only)
      gt_x/y/yaw         [m, m, rad]  ground truth in the simulator world frame (empty if not available)
      entropy            [nats]       entropy of the normalised particle weights (from /entropy)
      neff               [-]          effective sample size N_eff = 1/sum(w~^2) (from /slam_gmapping/neff)
      particle_spread    [m]          sqrt(var_x + var_y) of the particle positions (from ~/particles)
  map_log.csv (one line per /map update)
      t, width, height, resolution, known_area_m2, free_area_m2, occupied_cells

Ground truth (simulation only, optional):
  gt_type:=odometry  gt_topic:=/ground_truth/odom          (nav_msgs/Odometry, e.g. Gazebo p3d plugin)
  gt_type:=tf        gt_topic:=/world/<world>/dynamic_pose/info gt_child_frame:=turtlebot4
                     (tf2_msgs/TFMessage bridged from Ignition with ros_gz_bridge, see README)
"""
import csv
import math
import os
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage


def yaw_from_quat(q):
    """Yaw (rotation about z) from a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SlamEvalRecorder(Node):

    def __init__(self):
        super().__init__('slam_eval_recorder')
        self.declare_parameter('run_name', 'run')
        self.declare_parameter('output_root', os.path.expanduser('~/gmapping_runs'))
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('entropy_topic', '/entropy')
        self.declare_parameter('particles_topic', '/slam_gmapping/particles')
        self.declare_parameter('neff_topic', '/slam_gmapping/neff')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('gt_topic', '')
        self.declare_parameter('gt_type', 'odometry')
        self.declare_parameter('gt_child_frame', 'turtlebot4')

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.map_frame, self.odom_frame, self.base_frame = gp('map_frame'), gp('odom_frame'), gp('base_frame')
        self.gt_child = gp('gt_child_frame')

        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.out_dir = os.path.join(gp('output_root'), f"{gp('run_name')}_{stamp}")
        os.makedirs(self.out_dir, exist_ok=True)

        self.pose_file = open(os.path.join(self.out_dir, 'poses.csv'), 'w', newline='')
        self.pose_csv = csv.writer(self.pose_file)
        self.pose_csv.writerow(['t', 'slam_x', 'slam_y', 'slam_yaw', 'odom_x', 'odom_y', 'odom_yaw',
                                'gt_x', 'gt_y', 'gt_yaw', 'entropy', 'neff', 'particle_spread'])
        self.map_file = open(os.path.join(self.out_dir, 'map_log.csv'), 'w', newline='')
        self.map_csv = csv.writer(self.map_file)
        self.map_csv.writerow(['t', 'width', 'height', 'resolution', 'known_area_m2', 'free_area_m2',
                               'occupied_cells'])

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.entropy = float('nan')
        self.neff = float('nan')
        self.spread = float('nan')
        self.gt = None  # (x, y, yaw)

        best_effort = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        latched = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Float64, gp('entropy_topic'), self.on_entropy, 10)
        self.create_subscription(Float64, gp('neff_topic'), self.on_neff, 10)
        self.create_subscription(PoseArray, gp('particles_topic'), self.on_particles, best_effort)
        self.create_subscription(OccupancyGrid, gp('map_topic'), self.on_map, latched)

        gt_topic, gt_type = gp('gt_topic'), gp('gt_type')
        if gt_topic:
            if gt_type == 'tf':
                self.create_subscription(TFMessage, gt_topic, self.on_gt_tf, best_effort)
            else:
                self.create_subscription(Odometry, gt_topic, self.on_gt_odom, best_effort)
            self.get_logger().info(f'Ground truth: {gt_topic} ({gt_type})')
        else:
            self.get_logger().info('No ground-truth topic: only SLAM vs odometry will be compared.')

        self.create_timer(1.0 / float(gp('rate')), self.on_timer)
        self.n_rows = 0
        self.get_logger().info(f'Logging to {self.out_dir}')

    # ----------------------------------------------------------------- callbacks
    def on_entropy(self, msg):
        self.entropy = msg.data

    def on_neff(self, msg):
        self.neff = msg.data

    def on_particles(self, msg):
        if not msg.poses:
            return
        xy = np.array([[p.position.x, p.position.y] for p in msg.poses])
        self.spread = float(math.sqrt(np.var(xy[:, 0]) + np.var(xy[:, 1])))

    def on_map(self, msg):
        data = np.asarray(msg.data, dtype=np.int16)
        cell_area = msg.info.resolution ** 2
        known = int(np.count_nonzero(data >= 0))
        free = int(np.count_nonzero(data == 0))
        occ = int(np.count_nonzero(data >= 65))
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.map_csv.writerow([f'{t:.3f}', msg.info.width, msg.info.height, msg.info.resolution,
                               f'{known * cell_area:.3f}', f'{free * cell_area:.3f}', occ])
        self.map_file.flush()

    def on_gt_odom(self, msg):
        p = msg.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def on_gt_tf(self, msg):
        for tr in msg.transforms:
            name = tr.child_frame_id
            if name == self.gt_child or name.endswith('/' + self.gt_child):
                t = tr.transform.translation
                self.gt = (t.x, t.y, yaw_from_quat(tr.transform.rotation))
                return

    def lookup(self, parent):
        try:
            tr = self.tf_buffer.lookup_transform(parent, self.base_frame, Time())
        except TransformException:
            return None
        t = tr.transform.translation
        return (t.x, t.y, yaw_from_quat(tr.transform.rotation))

    def on_timer(self):
        slam = self.lookup(self.map_frame)
        odom = self.lookup(self.odom_frame)
        if slam is None or odom is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        gt = self.gt if self.gt is not None else ('', '', '')
        fmt = lambda v: f'{v:.4f}' if isinstance(v, float) else v  # noqa: E731
        self.pose_csv.writerow([f'{t:.3f}', *map(fmt, slam), *map(fmt, odom), *map(fmt, gt),
                                fmt(self.entropy), fmt(self.neff), fmt(self.spread)])
        self.n_rows += 1
        if self.n_rows % 50 == 0:
            self.pose_file.flush()

    def close(self):
        self.pose_file.close()
        self.map_file.close()
        print(f'[slam_eval_recorder] wrote {self.n_rows} pose rows to {self.out_dir}')


def main():
    rclpy.init()
    node = SlamEvalRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
