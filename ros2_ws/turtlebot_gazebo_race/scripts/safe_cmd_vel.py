#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SafeCmdVel(Node):
    def __init__(self):
        super().__init__('safe_cmd_vel')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_safe')
        self.declare_parameter('timeout', 0.35)
        self.declare_parameter('max_linear', 0.22)
        self.declare_parameter('max_angular', 0.85)
        self.declare_parameter('publish_rate', 20.0)
        self.timeout = float(self.get_parameter('timeout').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        rate = float(self.get_parameter('publish_rate').value)
        self.last_msg = Twist()
        self.last_time = self.get_clock().now()
        self.received_once = False
        self.pub = self.create_publisher(Twist, self.get_parameter('output_topic').value, 10)
        self.sub = self.create_subscription(Twist, self.get_parameter('input_topic').value, self.on_cmd, 10)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(f'SafeCmdVel activo: /cmd_vel -> /cmd_vel_safe, timeout={self.timeout}s')

    @staticmethod
    def clamp(value, limit):
        if not math.isfinite(value):
            return 0.0
        return max(-limit, min(limit, value))

    def on_cmd(self, msg):
        safe = Twist()
        safe.linear.x = self.clamp(msg.linear.x, self.max_linear)
        safe.angular.z = self.clamp(msg.angular.z, self.max_angular)
        self.last_msg = safe
        self.last_time = self.get_clock().now()
        self.received_once = True

    def on_timer(self):
        age = (self.get_clock().now() - self.last_time).nanoseconds / 1e9
        self.pub.publish(self.last_msg if self.received_once and age <= self.timeout else Twist())

def main(args=None):
    rclpy.init(args=args)
    node = SafeCmdVel()
    try:
        rclpy.spin(node)
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
