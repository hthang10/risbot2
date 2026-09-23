#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data

def yaw_from_quaternion(q: Quaternion) -> float:
    """
    Extract yaw from geometry_msgs/Quaternion
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def quaternion_from_yaw(yaw: float) -> Quaternion:
    """
    Create quaternion from yaw only (2D robot)
    """
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def time_to_sec(t):
    """
    Convert builtin_interfaces/Time to float seconds
    """
    return t.sec + t.nanosec * 1e-9

class T265OdomNode(Node):
    def __init__(self):
        super().__init__('t265_odom_node')

        # ===== PARAMETERS =====
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # ===== SUBSCRIBER =====
        self.sub = self.create_subscription(
            Odometry,
            '/camera/pose/sample',
            self.odom_cb,
            qos_profile_sensor_data
        )

        # ===== PUBLISHER =====
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        # ===== TF BROADCASTER =====
        self.tf_broadcaster = TransformBroadcaster(self)

        # ===== STATE =====
        self.first_msg = True
        self.last_time = None
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_yaw = 0.0

        self.get_logger().info('T265 2D odom node started (Odometry input, Humble compatible)')

    def odom_cb(self, msg: Odometry):
        now = self.get_clock().now().to_msg()

        # ===== YAW =====
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        # ===== INIT =====
        if self.first_msg:
            self.last_time = now
            self.last_x = msg.pose.pose.position.x
            self.last_y = msg.pose.pose.position.y
            self.last_yaw = yaw
            self.first_msg = False
            return

        dt = time_to_sec(now) - time_to_sec(self.last_time)
        if dt <= 0.0:
            return

        # ===== VELOCITY 2D =====
        dx = msg.pose.pose.position.x - self.last_x
        dy = msg.pose.pose.position.y - self.last_y

        vx = dx / dt
        vy = dy / dt

        v = math.cos(yaw) * vx + math.sin(yaw) * vy
        wz = (yaw - self.last_yaw) / dt

        # ===== BUILD ODOM =====
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Position (2D)
        odom.pose.pose.position.x = msg.pose.pose.position.x
        odom.pose.pose.position.y = msg.pose.pose.position.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion_from_yaw(yaw)

        # Velocity (2D)
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = wz

        # ===== COVARIANCE =====
        for i in range(36):
            odom.pose.covariance[i] = 0.0
            odom.twist.covariance[i] = 0.0
        odom.pose.covariance[0]  = 0.01
        odom.pose.covariance[7]  = 0.01
        odom.pose.covariance[35] = 0.02
        odom.twist.covariance[0]  = 0.1
        odom.twist.covariance[35] = 0.1

        # ===== PUBLISH ODOM =====
        self.odom_pub.publish(odom)

        # ===== PUBLISH TF odom → base_link =====
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = odom.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = odom.pose.pose.position.x
            tf.transform.translation.y = odom.pose.pose.position.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

        # ===== SAVE STATE =====
        self.last_time = now
        self.last_x = msg.pose.pose.position.x
        self.last_y = msg.pose.pose.position.y
        self.last_yaw = yaw

def main():
    rclpy.init()
    node = T265OdomNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
