#!/usr/bin/env python3

import math
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


DEFAULT_GENOME = {
    'target_distance': 0.45,
    'kp': 0.9,
    'ki': 0.0,
    'kd': 0.0,
    'linear_speed': 0.22,
    'max_angular': 0.72,
}

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
        self.first_scan_time = None
        self.current_yaw = None
        self.last_cmd = Twist()
        self.turn_direction = 0.0
        self.turn_start_time = None
        self.turn_start_yaw = None
        self.turn_target_angle = math.radians(68.0)
        self.turn_reason = ''
        self.turn_cooldown_until = None
        self.enabled = True
        self.update_genome(genome)

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, scan_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
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

        #Controla que el LiDAR funcione
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

    #Almacena el último escaneo recibido y actualiza el instante temporal en el que se recibió
    def on_scan(self, msg):  
        if not self.enabled or not msg.ranges:
            return
        self.last_scan_time = self.get_clock().now()
        if self.first_scan_time is None:
            self.first_scan_time = self.last_scan_time
        self.scan = msg

    #Proporciona información sobre la posición y orientación del robot.
    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def preferred_turn_direction(self):
        return -1.0 if self.side == 'right' else 1.0

    def latch_turn(self, direction, reason):
        self.turn_direction = 1.0 if direction >= 0.0 else -1.0
        self.turn_start_time = self.get_clock().now()
        self.turn_start_yaw = self.current_yaw
        self.turn_reason = reason

    def in_turn_cooldown(self):
        return (
            self.turn_cooldown_until is not None
            and self.get_clock().now() < self.turn_cooldown_until
        )

    #Determina el tiempo de giro del robot
    def latched_turn_finished(self, front):
        if self.turn_start_time is None:
            return True

        elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds * 1e-9
        if elapsed > 2.4:
            return True

        if elapsed < 0.35:
            return False

        if self.current_yaw is None or self.turn_start_yaw is None:
            return front > 1.05 and elapsed > 1.2

        turned = abs(normalize_angle(self.current_yaw - self.turn_start_yaw))
        return turned >= self.turn_target_angle and front > 0.65

    #Controla que el LiDAR funcione
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
        now = self.get_clock().now()
        initial_elapsed = 999.0
        if self.first_scan_time is not None:
            initial_elapsed = (now - self.first_scan_time).nanoseconds * 1e-9

        if initial_elapsed < 2.6 and front > 0.55:
            cmd.linear.x = self.speed
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.last_cmd = cmd
            self.get_logger().info(
                f"initial_straight t={initial_elapsed:.1f}s f={front:.2f} "
                f"l={left:.2f} r={right:.2f} cmd v={cmd.linear.x:.2f} w=0.00",
                throttle_duration_sec=1.0,
            )
            return

        if self.turn_direction != 0.0:
            if self.latched_turn_finished(front):
                self.turn_direction = 0.0
                self.turn_start_time = None
                self.turn_start_yaw = None
                self.turn_reason = ''
                self.turn_cooldown_until = now + Duration(seconds=0.9)
            else:
                cmd.linear.x = 0.0 if front < 0.42 else 0.075
                cmd.angular.z = self.turn_direction * self.max_angular
                self.cmd_pub.publish(cmd)
                self.last_cmd = cmd
                turned = 0.0
                if self.current_yaw is not None and self.turn_start_yaw is not None:
                    turned = abs(normalize_angle(self.current_yaw - self.turn_start_yaw))
                self.get_logger().info(
                    f"turn_latch={self.turn_reason} f={front:.2f} fl={front_left:.2f} "
                    f"fr={front_right:.2f} turned={math.degrees(turned):.0f} "
                    f"cmd v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}",
                    throttle_duration_sec=1.0,
                )
                return

        left_path_open = front_left > 1.05 or left > 1.10
        right_path_open = front_right > 1.05 or right > 1.10
        both_paths_open = left_path_open and right_path_open
        preferred_path_strong = (
            (front_right > 1.30 or right > 1.45)
            if self.side == 'right'
            else (front_left > 1.30 or left > 1.45)
        )
        intersection = (
            front < 0.62
            and both_paths_open
            and preferred_path_strong
            and not self.in_turn_cooldown()
        )
        if intersection and front > 0.55:
            self.latch_turn(self.preferred_turn_direction(), 'intersection')
            cmd.linear.x = 0.07
            cmd.angular.z = self.turn_direction * self.max_angular
            self.cmd_pub.publish(cmd)
            self.last_cmd = cmd
            return

        if front < 0.48:
            if both_paths_open:
                direction = self.preferred_turn_direction()
            else:
                direction = 1.0 if front_left >= front_right else -1.0
            if not self.in_turn_cooldown():
                self.latch_turn(direction, 'front_blocked')
            cmd.linear.x = 0.0
            cmd.angular.z = direction * self.max_angular
        else:
            sees_left = left < 2.8
            sees_right = right < 2.8

            if self.side == 'right' and sees_right:
                error = self.target - right
                turn = -0.22 if error < -0.30 else 1.25 * clamp(error, -0.30, 0.8)
            elif self.side == 'left' and sees_left:
                error = left - self.target
                turn = 0.22 if error > 0.30 else 1.25 * clamp(error, -0.8, 0.30)
            elif sees_right:
                error = self.target - right
                turn = -0.18 if error < -0.30 else 0.85 * clamp(error, -0.30, 0.8)
            elif sees_left:
                error = left - self.target
                turn = 0.18 if error > 0.30 else 0.85 * clamp(error, -0.8, 0.30)
            else:
                turn = -0.12 if self.side == 'right' else 0.12

            if front < 1.15:
                gap_turn = clamp(front_left - front_right, -0.6, 0.6)
                turn += 0.55 * gap_turn

            turn = 0.70 * turn + 0.30 * self.last_cmd.angular.z
            cmd.angular.z = clamp(turn, -self.max_angular, self.max_angular)
            cmd.linear.x = self.speed

            if front < 0.85:
                cmd.linear.x = min(cmd.linear.x, 0.12)
            if abs(cmd.angular.z) > 0.30:
                cmd.linear.x = min(cmd.linear.x, 0.14)

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
