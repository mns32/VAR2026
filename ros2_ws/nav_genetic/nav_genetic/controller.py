#!/usr/bin/env python3
"""
WallFollower para el circuito de la practica.

Usa /scan con QoS BEST_EFFORT y publica /cmd_vel. La parte importante es que
no asume que ranges[0] sea siempre el frente: convierte angulos a indices a
partir de angle_min/angle_increment, que es lo que cambia entre simuladores.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


DEFAULT_GENOME = {
    'target_distance': 0.55,
    'kp': 0.9,
    'ki': 0.0,
    'kd': 0.0,
    'linear_speed': 0.14,
    'max_angular': 0.45,
}

def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class WallFollower(Node):
    def __init__(self, genome=None, side='right'):
        super().__init__('wall_follower')

        if genome is None:
            self.declare_parameter('target_distance', DEFAULT_GENOME['target_distance'])
            self.declare_parameter('kp', DEFAULT_GENOME['kp'])
            self.declare_parameter('ki', DEFAULT_GENOME['ki'])
            self.declare_parameter('kd', DEFAULT_GENOME['kd'])
            self.declare_parameter('linear_speed', DEFAULT_GENOME['linear_speed'])
            self.declare_parameter('max_angular', DEFAULT_GENOME['max_angular'])
            self.declare_parameter('side', side)
            genome = {
                'target_distance': self.get_parameter('target_distance').value,
                'kp': self.get_parameter('kp').value,
                'ki': self.get_parameter('ki').value,
                'kd': self.get_parameter('kd').value,
                'linear_speed': self.get_parameter('linear_speed').value,
                'max_angular': self.get_parameter('max_angular').value,
            }
            self.side = str(self.get_parameter('side').value)
        else:
            self.side = side

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()
        self.last_scan_time = None
        self.last_cmd = Twist()
        self.enabled = True
        self.update_genome(genome)

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, scan_qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.watchdog = self.create_timer(1.0, self.on_watchdog)
        self.control_timer = self.create_timer(0.10, self.on_control)
        self.get_logger().info(
            f"Reactive wall follower activo: side={self.side}, d*={self.target:.2f}, "
            f"v={self.speed:.2f}, wmax={self.max_angular:.2f}"
        )

    def update_genome(self, genome):
        self.target = float(genome.get('target_distance', DEFAULT_GENOME['target_distance']))
        self.kp = float(genome.get('kp', DEFAULT_GENOME['kp']))
        self.ki = float(genome.get('ki', DEFAULT_GENOME['ki']))
        self.kd = float(genome.get('kd', DEFAULT_GENOME['kd']))
        self.speed = float(genome.get('linear_speed', DEFAULT_GENOME['linear_speed']))
        self.max_angular = float(genome.get('max_angular', DEFAULT_GENOME['max_angular']))
        self.prev_error = 0.0
        self.integral = 0.0

    def set_enabled(self, value):
        self.enabled = value
        if not value:
            self.publish_zero()

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def on_watchdog(self):
        if self.last_scan_time is None:
            self.get_logger().warn(
                'No llega /scan. Si Gazebo esta abierto, el robot no esta spawneado '
                'o el bridge del laser no esta funcionando.',
                throttle_duration_sec=3.0,
            )
            self.publish_zero()
            return

        age = (self.get_clock().now() - self.last_scan_time).nanoseconds * 1e-9
        if age > 1.0:
            self.get_logger().warn(
                f'/scan esta parado desde hace {age:.1f}s; paro por seguridad.',
                throttle_duration_sec=3.0,
            )
            self.publish_zero()

    def angle_to_index(self, msg, angle_rad):
        if not msg.ranges or abs(msg.angle_increment) < 1e-9:
            return 0

        angle = angle_rad
        if msg.angle_min >= -0.01 and msg.angle_max > 6.0 and angle < 0.0:
            angle += 2.0 * math.pi

        angle = clamp(angle, msg.angle_min, msg.angle_max)
        idx = int(round((angle - msg.angle_min) / msg.angle_increment))
        return max(0, min(idx, len(msg.ranges) - 1))

    def sector_distance(self, msg, deg_min, deg_max):
        samples = []
        steps = max(4, int(abs(deg_max - deg_min) / 2))
        for i in range(steps + 1):
            t = i / steps
            deg = deg_min + (deg_max - deg_min) * t
            idx = self.angle_to_index(msg, math.radians(deg))
            r = msg.ranges[idx]
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                samples.append(float(r))

        if not samples:
            return msg.range_max

        samples.sort()
        return samples[min(len(samples) - 1, max(0, int(0.20 * len(samples))))]

    def on_scan(self, msg):
        if not self.enabled or not msg.ranges:
            return
        self.last_scan_time = self.get_clock().now()
        self.scan = msg

    def on_control(self):
        if not self.enabled or not hasattr(self, 'scan'):
            return

        msg = self.scan

        front = self.sector_distance(msg, -35, 35)
        front_left = self.sector_distance(msg, 20, 75)
        front_right = self.sector_distance(msg, -75, -20)
        left = self.sector_distance(msg, 75, 115)
        right = self.sector_distance(msg, -115, -75)

        cmd = Twist()

        if front < 0.48:
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_angular if front_left >= front_right else -self.max_angular
        else:
            sees_left = left < 2.8
            sees_right = right < 2.8

            if sees_left and sees_right:
                turn = 0.55 * clamp(left - right, -1.2, 1.2)
            elif sees_right:
                turn = 0.95 * clamp(self.target - right, -0.8, 0.8)
            elif sees_left:
                turn = 0.75 * clamp(left - self.target, -0.8, 0.8)
            else:
                turn = -0.12 if self.side == 'right' else 0.12

            if front < 1.15:
                gap_turn = clamp(front_left - front_right, -0.6, 0.6)
                turn += 0.55 * gap_turn

            turn = 0.70 * turn + 0.30 * self.last_cmd.angular.z
            cmd.angular.z = clamp(turn, -self.max_angular, self.max_angular)
            cmd.linear.x = self.speed

            if front < 0.85:
                cmd.linear.x = min(cmd.linear.x, 0.08)
            if abs(cmd.angular.z) > 0.30:
                cmd.linear.x = min(cmd.linear.x, 0.10)

        self.get_logger().info(
            f"scan f={front:.2f} fl={front_left:.2f} fr={front_right:.2f} "
            f"l={left:.2f} r={right:.2f} "
            f"cmd v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}",
            throttle_duration_sec=2.0,
        )

        self.cmd_pub.publish(cmd)
        self.last_cmd = cmd

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
