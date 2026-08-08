#!/usr/bin/env python3
"""Directly check ROS graph publishers without using the ROS CLI daemon."""

from __future__ import annotations

import argparse
import sys
import time


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check whether a ROS topic has publishers.")
    parser.add_argument("topic")
    parser.add_argument("--timeout", type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if args.timeout <= 0.0:
        print("timeout must be positive", file=sys.stderr)
        return 2
    try:
        import rclpy
    except ImportError as error:
        print(f"ROS 2 Python modules are unavailable: {error}", file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node = rclpy.create_node("topic_readiness_probe")
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_publishers_info_by_topic(args.topic):
                return 0
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
