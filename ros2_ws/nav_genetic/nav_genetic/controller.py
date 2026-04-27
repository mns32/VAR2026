#!/usr/bin/env python3
"""
WallFollower – Controlador de seguimiento de pared para el Turtlebot3.

Mantiene una distancia objetivo respecto a la pared derecha usando un
controlador PID. Incluye:
  - Anticipación de curvas (rayo derecho-delantero a -45°)
  - Maniobra de emergencia (giro agresivo si hay pared al frente)
  - Marcha atrás si está pegado a un muro (anti-atasco)
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# Defaults sin GA: completan vuelta despacio pero seguros
DEFAULT_GENOME = {
    'target_distance': 0.40,
    'kp':              4.0,
    'ki':              0.0,
    'kd':              1.5,
    'linear_speed':    0.18,    # ↓ un pelín más lento que antes
    'max_angular':     1.8,     # ↑ más giro disponible para curvas
}

# Umbrales de detección frontal (ajusta si tu circuito es muy cerrado)
EMERGENCY_FRONT = 0.40   # m, gira agresivo hacia fuera
STUCK_FRONT     = 0.22   # m, marcha atrás
SLOWDOWN_FRONT  = 0.70   # m, empieza a frenar


class WallFollower(Node):
    def __init__(self, genome=None, side='right'):
        super().__init__('wall_follower')

        if genome is None:
            self.declare_parameter('target_distance', DEFAULT_GENOME['target_distance'])
            self.declare_parameter('kp',              DEFAULT_GENOME['kp'])
            self.declare_parameter('ki',              DEFAULT_GENOME['ki'])
            self.declare_parameter('kd',              DEFAULT_GENOME['kd'])
            self.declare_parameter('linear_speed',    DEFAULT_GENOME['linear_speed'])
            self.declare_parameter('max_angular',     DEFAULT_GENOME['max_angular'])
            self.declare_parameter('side', side)
            self._load_from_params()
        else:
            self.update_genome(genome)
            self.side = side

        # Estado PID
        self.prev_error = 0.0
        self.integral   = 0.0
        self.last_time  = self.get_clock().now()
        self.enabled    = True

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, scan_qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._log_params()

    # ── API pública para el GA ───────────────────────────────────────────────
    def update_genome(self, g):
        self.target       = float(g['target_distance'])
        self.kp           = float(g['kp'])
        self.ki           = float(g['ki'])
        self.kd           = float(g['kd'])
        self.linear_speed = float(g['linear_speed'])
        self.max_angular  = float(g['max_angular'])
        self.prev_error   = 0.0
        self.integral     = 0.0

    def set_enabled(self, value: bool):
        self.enabled = bool(value)
        if not value:
            self.publish_zero()

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    # ── Internos ─────────────────────────────────────────────────────────────
    def _load_from_params(self):
        self.target       = self.get_parameter('target_distance').value
        self.kp           = self.get_parameter('kp').value
        self.ki           = self.get_parameter('ki').value
        self.kd           = self.get_parameter('kd').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.max_angular  = self.get_parameter('max_angular').value
        self.side         = self.get_parameter('side').value

    def _log_params(self):
        self.get_logger().info(
            f"WallFollower: side={self.side} d*={self.target:.2f} "
            f"Kp={self.kp:.2f} Ki={self.ki:.2f} Kd={self.kd:.2f} "
            f"v={self.linear_speed:.2f} ω_max={self.max_angular:.2f}")

    def _ray_at(self, msg, angle_rad):
        n = len(msg.ranges)
        rel = (angle_rad - msg.angle_min) % (2.0 * math.pi)
        i = int(round(rel / msg.angle_increment)) % n
        r = msg.ranges[i]
        if math.isinf(r) or math.isnan(r) or r <= 0.0:
            return msg.range_max
        return r

    def _min_in_window(self, msg, center_deg, halfwidth_deg=10):
        center = math.radians(center_deg)
        hw = math.radians(halfwidth_deg)
        n = max(3, int(2 * hw / msg.angle_increment))
        vals = [self._ray_at(msg, center + k * msg.angle_increment)
                for k in range(-n // 2, n // 2 + 1)]
        return min(vals)

    def on_scan(self, msg):
        if not self.enabled:
            return

        # ── Lecturas direccionales ───────────────────────────────────────────
        if self.side == 'right':
            d_side       = self._min_in_window(msg, -90, 10)   # lateral
            d_side_front = self._min_in_window(msg, -45, 18)   # derecho-delantero
            sign = -1.0   # error positivo (lejos) → ω<0 (girar derecha)
        else:
            d_side       = self._min_in_window(msg,  90, 10)
            d_side_front = self._min_in_window(msg,  45, 18)
            sign = +1.0
        d_front = self._min_in_window(msg, 0, 22)

        now = self.get_clock().now()
        dt = max((now - self.last_time).nanoseconds * 1e-9, 1e-3)
        self.last_time = now

        # ── 1) Atasco grave: marcha atrás girando hacia fuera ────────────────
        if d_front < STUCK_FRONT:
            cmd = Twist()
            cmd.linear.x  = -0.06
            cmd.angular.z = float(-sign * self.max_angular)  # gira lejos de la pared
            self.cmd_pub.publish(cmd)
            self.integral = 0.0
            self.prev_error = self.target - d_side
            return

        # ── 2) Pared al frente: giro agresivo hacia fuera, sin avanzar ───────
        if d_front < EMERGENCY_FRONT:
            cmd = Twist()
            cmd.linear.x  = 0.03
            cmd.angular.z = float(-sign * self.max_angular)
            self.cmd_pub.publish(cmd)
            self.integral = 0.0
            self.prev_error = self.target - d_side
            return

        # ── 3) PID normal con anticipación de curvas ─────────────────────────
        # El min de (lateral, lateral-delantero) hace que el controlador
        # "vea" la curva ANTES de que el rayo lateral pierda la pared.
        d_effective = min(d_side, d_side_front * 0.85)
        error = self.target - d_effective

        self.integral += error * dt
        self.integral = max(-1.0, min(1.0, self.integral))
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        u = -sign * (self.kp * error + self.ki * self.integral + self.kd * derivative)
        u = max(-self.max_angular, min(self.max_angular, u))

        # Slowdown progresivo si hay obstáculo al frente
        v = self.linear_speed
        if d_front < SLOWDOWN_FRONT:
            v *= max(0.15, (d_front - 0.30) / 0.40)

        cmd = Twist()
        cmd.linear.x  = float(v)
        cmd.angular.z = float(u)
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
