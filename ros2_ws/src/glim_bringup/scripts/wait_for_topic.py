#!/usr/bin/env python3
import argparse
import sys
import time

import rclpy
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', required=True)
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--min_publishers', type=int, default=1)
    args = parser.parse_args()

    rclpy.init()
    node = Node('wait_for_topic')

    deadline = time.time() + args.timeout
    topic = args.topic
    min_pub = max(0, args.min_publishers)

    try:
        while rclpy.ok():
            pubs = node.count_publishers(topic)
            if pubs >= min_pub:
                node.get_logger().info(f"Topic ready: {topic} (publishers={pubs})")
                rclpy.shutdown()
                return 0
            if time.time() > deadline:
                node.get_logger().error(f"Timeout waiting for topic: {topic}")
                rclpy.shutdown()
                return 1
            time.sleep(0.2)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 1


if __name__ == '__main__':
    sys.exit(main())


