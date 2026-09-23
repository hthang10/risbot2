#!/usr/bin/env python3
"""
Navigation Goal Helper for RISBOT2 (Nav2 & Omniplanner)
======================================================
Allows sending 3DSG object goals or 2D coordinate goals via CLI.

Examples:
    # Send semantic object goal (Omniplanner 3DSG):
    ros2 run risbot2_node send_goal.py --objects O(64) O(73)

    # Send 2D pose coordinate goal (Nav2):
    ros2 run risbot2_node send_goal.py --x 2.0 --y 1.5 --yaw 0.0
"""

import argparse
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose, FollowPath


def euler_to_quaternion(yaw):
    return (
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0)
    )


class GoalSenderNode(Node):
    def __init__(self):
        super().__init__('send_goal_cli')

    def send_semantic_goal(self, robot_id, object_names):
        try:
            from omniplanner_msgs.msg import GotoPointsGoalMsg
        except ImportError:
            self.get_logger().error("Không tìm thấy omniplanner_msgs! Hãy chạy 'source install/setup.bash' trước.")
            return

        pub = self.create_publisher(GotoPointsGoalMsg, '/omniplanner_node/goto_points_planner/goto_points_goal', 10)
        msg = GotoPointsGoalMsg()
        msg.robot_id = robot_id
        msg.point_names_to_visit = object_names

        for _ in range(5):
            pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"✅ Đã gửi lệnh di chuyển 3DSG Omniplanner cho robot '{robot_id}': {object_names}")

    def send_nav2_goal(self, x, y, yaw, frame_id="map"):
        # 1. Thử gửi trực tiếp qua /follow_path (Do controller_server đang chạy quản lý)
        follow_client = ActionClient(self, FollowPath, '/follow_path')
        if follow_client.wait_for_server(timeout_sec=1.5):
            self.get_logger().info(f"Đang gửi mục tiêu 2D đến Nav2 Controller (/follow_path): x={x}, y={y}...")
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = frame_id

            pose_msg = PoseStamped()
            pose_msg.header = path_msg.header
            pose_msg.pose.position.x = float(x)
            pose_msg.pose.position.y = float(y)
            pose_msg.pose.position.z = 0.0

            qx, qy, qz, qw = euler_to_quaternion(yaw)
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw

            path_msg.poses = [pose_msg]

            goal_msg = FollowPath.Goal()
            goal_msg.path = path_msg
            goal_msg.controller_id = 'FollowPath'

            future = follow_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            if goal_handle and goal_handle.accepted:
                self.get_logger().info("✅ Mục tiêu 2D đã được Nav2 Controller chấp nhận! Robot đang di chuyển...")
                return

        # 2. Thử qua /navigate_to_pose (nếu có bt_navigator)
        nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        if nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f"Đang gửi mục tiêu 2D đến /navigate_to_pose: x={x}, y={y}...")
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = PoseStamped()
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.header.frame_id = frame_id
            goal_msg.pose.pose.position.x = float(x)
            goal_msg.pose.pose.position.y = float(y)
            qx, qy, qz, qw = euler_to_quaternion(yaw)
            goal_msg.pose.pose.orientation.x = qx
            goal_msg.pose.pose.orientation.y = qy
            goal_msg.pose.pose.orientation.z = qz
            goal_msg.pose.pose.orientation.w = qw

            future = nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            if goal_handle and goal_handle.accepted:
                self.get_logger().info("✅ Mục tiêu 2D đã được /navigate_to_pose chấp nhận!")
                return

        # 3. Fallback: Topic /goal_pose
        self.get_logger().info("Phát điểm mục tiêu qua topic /goal_pose...")
        pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        qx, qy, qz, qw = euler_to_quaternion(yaw)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        for _ in range(5):
            pub.publish(pose_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"✅ Đã phát điểm 2D ({x}, {y}) lên topic /goal_pose")


def main():
    parser = argparse.ArgumentParser(description="Send navigation goals to RISBOT2.")
    parser.add_argument('--objects', nargs='+', help="List of 3DSG object names to visit (e.g. O(64) O(73))")
    parser.add_argument('--x', type=float, help="Goal X coordinate (meters)")
    parser.add_argument('--y', type=float, help="Goal Y coordinate (meters)")
    parser.add_argument('--yaw', type=float, default=0.0, help="Goal Yaw orientation (radians)")
    parser.add_argument('--robot', type=str, default='risbot2', help="Robot ID (default: risbot2)")
    args = parser.parse_args()

    rclpy.init()
    node = GoalSenderNode()

    if args.objects:
        node.send_semantic_goal(args.robot, args.objects)
    elif args.x is not None and args.y is not None:
        node.send_nav2_goal(args.x, args.y, args.yaw)
    else:
        print("Vui lòng nhập --objects [tên vật thể] HOẶC --x [tọa độ X] --y [tọa độ Y]")
        parser.print_help()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
