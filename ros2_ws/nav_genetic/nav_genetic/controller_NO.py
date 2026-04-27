#!/usr/bin/env python3
"""
WallFollower – Controlador de seguimiento de pared para el Turtlebot3.

Mantiene una distancia objetivo respecto a la pared derecha usando un
controlador PID sobre el error de distancia. La velocidad lineal se reduce
automáticamente si hay un obstáculo cerca al frente (slowdown de seguridad).

El nodo es parametrizable y se usa de tres formas:
  1. Standalone:    ros2 run nav_genetic wall_follower
  2. Con genoma:    instanciado desde ga_train.py / run_best.py
  3. Con params:    ros2 run nav_genetic wall_follower --ros-args -p kp:=5.0 ...
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# Parámetros por defecto (sin GA estos ya completan vuelta, despacito)
DEFAULT_GENOME = {
    'target_distance': 0.40,   # m, distancia deseada a la pared derecha
    'kp':              4.0,    # ganancia proporcional
    'ki':              0.0,    # ganancia integral
    'kd':              1.5,    # ganancia derivativa
    'linear_speed':    0.20,   # m/s, velocidad de avance base
    'max_angular':     1.5,    # rad/s, máximo giro permitido
}


class WallFollower(Node):
    def __init__(self, genome=None, side='right'):
        super().__init__('wall_follower')

        # Si no nos pasan genoma, leemos de parámetros ROS (con defaults)
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

        # Estado interno del PID
        self.prev_error = 0.0
        self.integral   = 0.0
        self.last_time  = self.get_clock().now()
        # Flag para des/activar la publicación (lo usa el GA entre trials)
        self.enabled = True

        # /scan en BEST_EFFORT (así está configurado el bridge en el launch)
        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, scan_qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._log_params()

    # ── API pública para el GA ───────────────────────────────────────────────
    def update_genome(self, g):
        """Actualiza los parámetros sin recrear el nodo y resetea el PID."""
        self.target       = float(g['target_distance'])
        self.kp           = float(g['kp'])
        self.ki           = float(g['ki'])
        self.kd           = float(g['kd'])
        self.linear_speed = float(g['linear_speed'])
        self.max_angular  = float(g['max_angular'])
        self.prev_error   = 0.0
        self.integral     = 0.0

    def set_enabled(self, value: bool):
        """Activa/desactiva la publicación de cmd_vel."""
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
        """Devuelve la lectura del láser en el ángulo dado (en rad, frame robot)."""
        n = len(msg.ranges)
        # Normalizar a rango [angle_min, angle_min + 2π)
        rel = (angle_rad - msg.angle_min) % (2.0 * math.pi)
        i = int(round(rel / msg.angle_increment)) % n
        r = msg.ranges[i]
        if math.isinf(r) or math.isnan(r) or r <= 0.0:
            return msg.range_max
        return r

    def _min_in_window(self, msg, center_deg, halfwidth_deg=5):
        """Mínima distancia en una ventana angular alrededor de un ángulo."""
        center = math.radians(center_deg)
        hw = math.radians(halfwidth_deg)
        n = max(3, int(2 * hw / msg.angle_increment))
        vals = []
        for k in range(-n // 2, n // 2 + 1):
            vals.append(self._ray_at(msg, center + k * msg.angle_increment))
        return min(vals)

    def on_scan(self, msg):
        if not self.enabled:
            return

        # Lateral (pared a seguir) y frontal (para frenar)
        if self.side == 'right':
            d_side = self._min_in_window(msg, -90, 8)
            sign = -1.0   # error positivo (lejos de pared) → girar derecha (ω<0)
        else:
            d_side = self._min_in_window(msg, +90, 8)
            sign = +1.0
        d_front = self._min_in_window(msg, 0, 18)

        # PID sobre el error de distancia lateral
        error = self.target - d_side
        now = self.get_clock().now()
        dt = max((now - self.last_time).nanoseconds * 1e-9, 1e-3)
        self.last_time = now

        self.integral += error * dt
        self.integral = max(-1.0, min(1.0, self.integral))   # anti-windup
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        u = -sign * (self.kp * error + self.ki * self.integral + self.kd * derivative)
        u = max(-self.max_angular, min(self.max_angular, u))

        # Reducir velocidad si hay obstáculo al frente (curvas cerradas)
        v = self.linear_speed
        if d_front < 0.50:
            v *= max(0.0, (d_front - 0.20) / 0.30)

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
