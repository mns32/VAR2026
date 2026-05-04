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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


DEFAULT_GENOME = {
    'target_distance': 0.55,
    'kp': 0.9,
    'ki': 0.0,
    'kd': 0.0,
    'linear_speed': 0.14,
    'max_angular': 0.45,
}

SPAWN_X = 9.05
SPAWN_Y = 9.00
SPAWN_YAW = 3.14

WORLD_WAYPOINTS = [
    (9.05, 9.0),
    (-8.4, 9.0),
    (-9.4, 5.5),
    (-8.4, 2.0),
    (-9.4, -2.0),
    (-9.4, -5.5),
    (-8.4, -9.0),
    (8.4, -9.0),
    (9.4, -6.0),
    (9.4, -2.0),
    (7.6, 0.4),
    (8.5, 1.0),
    (9.0, 3.0),
    (9.0, 5.2),
    (7.2, 5.4),
    (7.2, 7.2),
    (9.05, 7.2),
    (9.05, 9.0),
]


def world_to_odom(point):
    dx = point[0] - SPAWN_X
    dy = point[1] - SPAWN_Y
    c = math.cos(SPAWN_YAW)
    s = math.sin(SPAWN_YAW)
    return (
        c * dx + s * dy,
        -s * dx + c * dy,
    )


ROUTE_WAYPOINTS = [world_to_odom(point) for point in WORLD_WAYPOINTS]


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


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
        self.last_odom_time = None
        self.pose = None
        self.current_waypoint = 1
        self.enabled = True
        self.update_genome(genome)

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, scan_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.watchdog = self.create_timer(1.0, self.on_watchdog)
        self.control_timer = self.create_timer(0.10, self.on_control)
        self.get_logger().info(
            f"Waypoint follower activo: {len(ROUTE_WAYPOINTS)} waypoints, "
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

        if self.last_odom_time is None:
            self.get_logger().warn(
                'No llega /odom. Sin odometria no puedo seguir la ruta.',
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

        odom_age = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1e-9
        if odom_age > 1.0:
            self.get_logger().warn(
                f'/odom esta parado desde hace {odom_age:.1f}s; paro por seguridad.',
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

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pose = (p.x, p.y, yaw)
        self.last_odom_time = self.get_clock().now()

    def on_control(self):
        if not self.enabled or self.pose is None or not hasattr(self, 'scan'):
            return

        msg = self.scan

        front = self.sector_distance(msg, -35, 35)
        front_left = self.sector_distance(msg, 15, 75)
        front_right = self.sector_distance(msg, -75, -15)

        x, y, yaw = self.pose
        target = ROUTE_WAYPOINTS[self.current_waypoint]
        dist = math.hypot(target[0] - x, target[1] - y)
        if dist < 0.65:
            self.current_waypoint = (self.current_waypoint + 1) % len(ROUTE_WAYPOINTS)
            target = ROUTE_WAYPOINTS[self.current_waypoint]
            dist = math.hypot(target[0] - x, target[1] - y)

        desired_yaw = math.atan2(target[1] - y, target[0] - x)
        heading_error = normalize_angle(desired_yaw - yaw)

        cmd = Twist()

        if front < 0.50:
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_angular if front_left >= front_right else -self.max_angular
        else:
            cmd.angular.z = clamp(1.25 * heading_error, -self.max_angular, self.max_angular)
            if abs(heading_error) > 0.55:
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = self.speed * clamp(1.0 - abs(heading_error) / 1.0, 0.35, 1.0)

            if front < 0.80:
                gap_turn = clamp(front_left - front_right, -0.6, 0.6)
                cmd.angular.z = clamp(cmd.angular.z + 0.35 * gap_turn, -self.max_angular, self.max_angular)
                cmd.linear.x = min(cmd.linear.x, 0.08)

            if abs(cmd.angular.z) > 0.30:
                cmd.linear.x = min(cmd.linear.x, 0.06)

        self.get_logger().info(
            f"wp={self.current_waypoint} target=({target[0]:.1f},{target[1]:.1f}) "
            f"pos=({x:.1f},{y:.1f}) err={heading_error:.2f} scan f={front:.2f} "
            f"cmd v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}",
            throttle_duration_sec=2.0,
        )

        self.cmd_pub.publish(cmd)

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
