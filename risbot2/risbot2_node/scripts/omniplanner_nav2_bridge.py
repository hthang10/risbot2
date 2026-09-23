#!/usr/bin/env python3
"""
Omniplanner → Nav2 Bridge Node
================================
Bridges the Omniplanner global planner (3DSG) with Nav2's controller server.

Subscribes to Omniplanner's compiled plan output (nav_msgs/Path) and forwards
it to Nav2's FollowPath action server for local planning with obstacle avoidance.

Topics:
  Subscribed:
    /{robot_name}/omniplanner_node/compiled_plan_out  (nav_msgs/Path)
  Action Client:
    /follow_path  (nav2_msgs/FollowPath)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath


class OmniplannerNav2Bridge(Node):
    """Bridge between Omniplanner's Path output and Nav2's FollowPath action."""

    def __init__(self):
        super().__init__('omniplanner_nav2_bridge')

        # Parameters
        self.declare_parameter('robot_name', 'risbot2')
        self.robot_name = self.get_parameter('robot_name').value

        # Publisher for RViz Global Plan visualization (/plan)
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.plan_pub = self.create_publisher(Path, '/plan', latching_qos)

        # Action client for Nav2 FollowPath
        self._follow_path_client = ActionClient(
            self, FollowPath, '/follow_path'
        )

        # Subscribe to Omniplanner's compiled plan
        reliable_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
        )

        plan_topic = f'/{self.robot_name}/omniplanner_node/compiled_plan_out'
        self.path_sub = self.create_subscription(
            Path,
            plan_topic,
            self.path_callback,
            reliable_qos,
        )

        # Track current goal handle for cancellation
        self._current_goal_handle = None

        self.get_logger().info(
            f'Omniplanner-Nav2 Bridge started. '
            f'Subscribing to: {plan_topic} → forwarding to /follow_path and publishing /plan'
        )

    def path_callback(self, msg: Path):
        """Receive path from Omniplanner and forward to Nav2."""
        if len(msg.poses) == 0:
            self.get_logger().warning('Received empty path from Omniplanner, ignoring')
            return

        self.get_logger().info(
            f'Received path with {len(msg.poses)} waypoints from Omniplanner'
        )

        # Publish to /plan for RViz Global Path display
        self.plan_pub.publish(msg)


        # Cancel any existing goal before sending new one
        if self._current_goal_handle is not None:
            self.get_logger().info('Cancelling previous Nav2 goal...')
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

        # Wait for action server (non-blocking check)
        if not self._follow_path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                'Nav2 FollowPath action server not available! '
                'Is controller_server running?'
            )
            return

        # Create and send FollowPath goal
        goal_msg = FollowPath.Goal()
        goal_msg.path = msg
        goal_msg.controller_id = 'FollowPath'

        self.get_logger().info('Sending path to Nav2 FollowPath...')
        send_goal_future = self._follow_path_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle Nav2's response to our goal request."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Nav2 FollowPath goal was rejected')
            return

        self.get_logger().info('Nav2 FollowPath goal accepted')
        self._current_goal_handle = goal_handle

        # Get result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle Nav2's result when path following is complete."""
        result = future.result()
        status = result.status

        if status == 4:  # SUCCEEDED
            self.get_logger().info('Nav2: Path following completed successfully!')
        elif status == 5:  # CANCELED
            self.get_logger().info('Nav2: Path following was cancelled')
        elif status == 6:  # ABORTED
            self.get_logger().warning('Nav2: Path following was aborted')
        else:
            self.get_logger().warning(f'Nav2: Path following ended with status {status}')

        self._current_goal_handle = None

    def feedback_callback(self, feedback_msg):
        """Handle feedback from Nav2 during path following."""
        feedback = feedback_msg.feedback
        dist = feedback.distance_to_goal
        # Only log occasionally to avoid spam
        if int(dist * 10) % 5 == 0:
            self.get_logger().debug(
                f'Nav2 feedback: {dist:.2f}m to goal'
            )


def main(args=None):
    rclpy.init(args=args)
    node = OmniplannerNav2Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cancel any active goal before shutdown
        if node._current_goal_handle is not None:
            node._current_goal_handle.cancel_goal_async()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
