#!/usr/bin/env python3
"""
Set Initial Pose CLI Helper for AMCL / Nav2
===========================================
Publishes geometry_msgs/msg/PoseWithCovarianceStamped to /initialpose.

Usage:
    ros2 run risbot2_node set_initial_pose.py --x 1.5 --y 2.0 --yaw 1.57
    python3 set_initial_pose.py --x 0.0 --y 0.0 --yaw 0.0
"""

import argparse
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def euler_to_quaternion(yaw):
    return (
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0)
    )


class InitialPosePublisher(Node):
    def __init__(self, x, y, yaw, frame_id="map"):
        super().__init__('set_initial_pose_cli')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = euler_to_quaternion(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Standard initial pose covariance
        cov = [0.0] * 36
        cov[0] = 0.25   # x
        cov[7] = 0.25   # y
        cov[35] = 0.068 # yaw
        msg.pose.covariance = cov

        # Publish 3 times to ensure delivery over ROS 2 topic
        for _ in range(3):
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"✅ Published Initial Pose: x={x}, y={y}, yaw={yaw} rad ({math.degrees(yaw):.1f} deg) to frame '{frame_id}'")


def main():
    parser = argparse.ArgumentParser(description="Publish initial pose to /initialpose for AMCL localization.")
    parser.add_argument('--x', type=float, default=0.0, help="X position (meters)")
    parser.add_argument('--y', type=float, default=0.0, help="Y position (meters)")
    parser.add_argument('--yaw', type=float, default=0.0, help="Yaw angle (radians or degrees if specified)")
    parser.add_argument('--degrees', action='store_true', help="Specify yaw in degrees instead of radians")
    parser.add_argument('--frame', type=str, default='map', help="Frame ID (default: map)")
    args = parser.parse_args()

    yaw = math.radians(args.yaw) if args.degrees else args.yaw

    rclpy.init()
    node = InitialPosePublisher(x=args.x, y=args.y, yaw=yaw, frame_id=args.frame)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
