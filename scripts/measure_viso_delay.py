#!/usr/bin/env python3
import argparse
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class DelayMeasure(Node):
    def __init__(self, in_topic: str, out_topic: str, field: str, duration_s: float):
        super().__init__('viso_delay_measure')
        self.in_topic = in_topic
        self.out_topic = out_topic
        self.field = field
        self.deadline = time.time() + duration_s
        self.in_times = []
        self.in_vals = []
        self.out_times = []
        self.out_vals = []
        # Use sensor-data like QoS (BEST_EFFORT) to avoid RELIABILITY mismatch
        self.sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Subscriptions
        if self._is_pose(self.in_topic):
            self.create_subscription(PoseStamped, self.in_topic, self._in_pose_cb, self.sub_qos)
        else:
            self.create_subscription(Odometry, self.in_topic, self._in_odom_cb, self.sub_qos)

        if self._is_pose(self.out_topic):
            self.create_subscription(PoseStamped, self.out_topic, self._out_pose_cb, self.sub_qos)
        else:
            self.create_subscription(Odometry, self.out_topic, self._out_odom_cb, self.sub_qos)

    def _is_pose(self, topic: str) -> bool:
        return topic.endswith('/pose') and 'odometry' not in topic

    def _extract(self, msg) -> float:
        if isinstance(msg, PoseStamped):
            p = msg.pose.position
            o = msg.pose.orientation
        else:
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
        if self.field == 'x':
            return float(p.x)
        if self.field == 'y':
            return float(p.y)
        if self.field == 'z':
            return float(p.z)
        if self.field == 'yaw':
            # yaw from quaternion
            import math
            siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
            cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
            return float(math.atan2(siny_cosp, cosy_cosp))
        raise ValueError('unsupported field')

    def _in_pose_cb(self, msg: PoseStamped):
        self.in_times.append(self._to_sec(msg.header.stamp))
        self.in_vals.append(self._extract(msg))

    def _in_odom_cb(self, msg: Odometry):
        self.in_times.append(self._to_sec(msg.header.stamp))
        self.in_vals.append(self._extract(msg))

    def _out_pose_cb(self, msg: PoseStamped):
        self.out_times.append(self._to_sec(msg.header.stamp))
        self.out_vals.append(self._extract(msg))

    def _out_odom_cb(self, msg: Odometry):
        self.out_times.append(self._to_sec(msg.header.stamp))
        self.out_vals.append(self._extract(msg))

    @staticmethod
    def _to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def done(self) -> bool:
        return time.time() > self.deadline

    def estimate_delay_ms(self) -> float:
        # Resample both to uniform time base (10 Hz)
        def resample(ts, xs, rate=10.0):
            if len(ts) < 5:
                return None, None
            t0, t1 = ts[0], ts[-1]
            if t1 <= t0:
                return None, None
            grid = np.arange(t0, t1, 1.0 / rate)
            xi = np.interp(grid, ts, xs)
            # normalize
            xi = (xi - np.mean(xi))
            s = np.std(xi)
            if s > 1e-9:
                xi /= s
            return grid, xi

        ti, xi = resample(self.in_times, self.in_vals)
        to, xo = resample(self.out_times, self.out_vals)
        if ti is None or to is None:
            raise RuntimeError('insufficient data to estimate delay')

        # align overlapping window
        t_start = max(ti[0], to[0])
        t_end = min(ti[-1], to[-1])
        if t_end - t_start < 1.0:
            raise RuntimeError('not enough overlap for correlation')
        def crop(t, x, a, b):
            m = (t >= a) & (t <= b)
            return t[m], x[m]
        ti2, xi2 = crop(ti, xi, t_start, t_end)
        to2, xo2 = crop(to, xo, t_start, t_end)
        # resample out to input grid
        xo_i = np.interp(ti2, to2, xo2)

        # cross-correlation (positive lag means output lags input)
        corr = np.correlate(xi2, xo_i, mode='full')
        lags = np.arange(-len(xi2) + 1, len(xi2)) / 10.0  # seconds at 10 Hz
        lag_sec = lags[np.argmax(corr)]
        return float(lag_sec * 1000.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--in_topic', default='/mavros/odometry/out')
    parser.add_argument('--out_topic', default='/mavros/local_position/pose')
    parser.add_argument('--field', default='x', choices=['x', 'y', 'z', 'yaw'])
    parser.add_argument('--duration', type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    node = DelayMeasure(args.in_topic, args.out_topic, args.field, args.duration)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.2)
        delay_ms = node.estimate_delay_ms()
        node.get_logger().info(f'Estimated delay: {delay_ms:.1f} ms (field={args.field})')
        print(f'{delay_ms:.1f}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
