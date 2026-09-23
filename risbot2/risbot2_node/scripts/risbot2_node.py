#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
import math

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

DISTANCE_WHEEL = 0.3    # Khoảng cách giữa hai bánh xe (m)
DIAMETER_WHEEL = 0.14    # Đường kính bánh xe (m)
WHEEL_RADIUS = 0.07    # Bán kính bánh xe (m)
STEP_PER_REVOLUTION = 800  # Số bước mỗi vòng quay của động cơ

class CmdVelToStep(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_step')
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.serial_subscription = self.create_timer(0.1, self.receive_step_feedback)
        self.last_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Giới hạn dt nếu là tin nhắn đầu tiên hoặc bị trễ quá lâu
        if dt <= 0 or dt > 0.5:
            dt = 0.1

        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # 1. Tính vận tốc dài từng bánh xe (m/s)
        v_right = linear_velocity + angular_velocity * (DISTANCE_WHEEL / 2.0)
        v_left  = linear_velocity - angular_velocity * (DISTANCE_WHEEL / 2.0)

        # 2. Vận tốc bước (bước/giây)
        steps_per_sec_right = (v_right / (2 * math.pi * WHEEL_RADIUS)) * STEP_PER_REVOLUTION
        steps_per_sec_left  = (v_left  / (2 * math.pi * WHEEL_RADIUS)) * STEP_PER_REVOLUTION

        # 3. Tính số bước tương đối cần đi trong khoảng dt (khớp với moveTo của Arduino)
        step_right = int(steps_per_sec_right * dt)
        step_left  = int(steps_per_sec_left * dt)

        send_data = f"{step_right},{step_left}\n"
        ser.write(send_data.encode('utf-8'))

    def receive_step_feedback(self):
        if ser.in_waiting:
            feedback = ser.readline().decode('utf-8').strip()
            # self.get_logger().info(f"Received feedback: {feedback}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToStep()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
