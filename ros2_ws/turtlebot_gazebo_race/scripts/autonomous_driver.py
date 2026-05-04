#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class AutonomousDriver(Node):
    """Controlador autonomo para el circuito usando LaserScan.

    Lee /scan y publica /cmd_vel. No usa algoritmo genetico.
    La politica intenta ir centrada en el pasillo y esquivar paredes frontales.
    """

    def __init__(self):
        super().__init__('autonomous_driver')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('target_side_distance', 0.55)
        self.declare_parameter('linear_speed', 0.16)
        self.declare_parameter('slow_linear_speed', 0.08)
        self.declare_parameter('max_angular', 0.75)
        self.declare_parameter('kp_center', 0.75)
        self.declare_parameter('kp_wall', 1.15)
        self.declare_parameter('kp_obstacle', 1.25)
        self.declare_parameter('front_stop_distance', 0.42)
        self.declare_parameter('front_slow_distance', 0.95)
        self.declare_parameter('publish_rate', 10.0)

        self.target_side_distance = float(self.get_parameter('target_side_distance').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.slow_linear_speed = float(self.get_parameter('slow_linear_speed').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.kp_center = float(self.get_parameter('kp_center').value)
        self.kp_wall = float(self.get_parameter('kp_wall').value)
        self.kp_obstacle = float(self.get_parameter('kp_obstacle').value)
        self.front_stop_distance = float(self.get_parameter('front_stop_distance').value)
        self.front_slow_distance = float(self.get_parameter('front_slow_distance').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.scan = None
        self.last_cmd = Twist()
        self.last_scan_time = self.get_clock().now()

        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.sub = self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.on_scan, qos)
        self.timer = self.create_timer(1.0 / float(self.get_parameter('publish_rate').value), self.on_timer)

        self.get_logger().info('AutonomousDriver activo: /scan -> /cmd_vel')

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def angle_to_index(self, scan, angle):
        """Convierte un angulo robotico (-pi..pi, 0=frente) a indice del LaserScan."""
        # El SDF suele publicar 0..2pi. Otros lasers publican -pi..pi.
        a = angle
        amin = scan.angle_min
        amax = scan.angle_max
        if amin >= -0.01 and amax > 6.0 and a < 0.0:
            a += 2.0 * math.pi
        a = clamp(a, amin, amax)
        idx = int(round((a - amin) / scan.angle_increment))
        return max(0, min(idx, len(scan.ranges) - 1))

    def sector_min(self, scan, deg_min, deg_max):
        """Distancia minima valida en un sector. Sectores en grados, 0=frente."""
        if scan is None or not scan.ranges:
            return float('inf')

        a1 = math.radians(deg_min)
        a2 = math.radians(deg_max)

        # Si el sector cruza 0 grados, se parte en dos.
        samples = []
        steps = max(3, int(abs(deg_max - deg_min) / 2))
        for i in range(steps + 1):
            t = i / steps
            a = a1 + (a2 - a1) * t
            idx = self.angle_to_index(scan, self.normalize_angle(a))
            r = scan.ranges[idx]
            if math.isfinite(r) and scan.range_min < r < scan.range_max:
                samples.append(r)

        if not samples:
            return float('inf')
        samples.sort()
        # Percentil bajo en lugar del minimo absoluto para ignorar ruido puntual.
        return samples[min(len(samples) - 1, max(0, int(0.2 * len(samples))))]

    def on_scan(self, msg):
        self.scan = msg
        self.last_scan_time = self.get_clock().now()

    def publish_stop(self):
        self.last_cmd = Twist()
        self.pub.publish(self.last_cmd)

    def on_timer(self):
        if self.scan is None:
            self.publish_stop()
            return

        age = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if age > 0.6:
            self.get_logger().warn('No llegan datos recientes de /scan; paro el robot.', throttle_duration_sec=2.0)
            self.publish_stop()
            return

        s = self.scan
        front = min(self.sector_min(s, -18, 18), self.sector_min(s, 342, 359), self.sector_min(s, 0, 18))
        front_left = self.sector_min(s, 25, 65)
        front_right = self.sector_min(s, -65, -25)
        left = self.sector_min(s, 75, 110)
        right = self.sector_min(s, -110, -75)

        cmd = Twist()

        # 1) Pared de frente: gira hacia el lado con mas espacio.
        if front < self.front_stop_distance:
            cmd.linear.x = 0.0
            turn_left = left >= right
            cmd.angular.z = self.max_angular if turn_left else -self.max_angular
            self.pub.publish(cmd)
            self.last_cmd = cmd
            return

        # 2) Obstaculo/curva delante: anticipa el giro hacia el hueco.
        obstacle_turn = 0.0
        if front < self.front_slow_distance:
            # Si hay mas espacio por la izquierda, angular positivo. Si por derecha, negativo.
            obstacle_turn = self.kp_obstacle * clamp(front_left - front_right, -0.8, 0.8)
            cmd.linear.x = self.slow_linear_speed
        else:
            cmd.linear.x = self.linear_speed

        # 3) Mantenerse centrado. Si solo ve una pared, la sigue a distancia objetivo.
        side_turn = 0.0
        sees_left = math.isfinite(left) and left < 2.8
        sees_right = math.isfinite(right) and right < 2.8

        if sees_left and sees_right:
            # Si esta cerca de la derecha: left > right => gira izquierda (+).
            side_turn = self.kp_center * clamp(left - right, -1.0, 1.0)
        elif sees_right:
            # Si right < target => gira izquierda (+). Si right > target => gira derecha (-).
            side_turn = self.kp_wall * clamp(self.target_side_distance - right, -0.7, 0.7)
        elif sees_left:
            # Si left < target => gira derecha (-). Si left > target => gira izquierda (+).
            side_turn = self.kp_wall * clamp(left - self.target_side_distance, -0.7, 0.7)
        else:
            # Sin paredes: avanza despacio y busca suavemente.
            cmd.linear.x = self.slow_linear_speed
            side_turn = 0.15

        angular = side_turn + obstacle_turn
        # Suavizado para que no oscile ni entre en circulos agresivos.
        angular = 0.65 * angular + 0.35 * self.last_cmd.angular.z
        cmd.angular.z = clamp(angular, -self.max_angular, self.max_angular)

        # Si va girando mucho, reduce velocidad.
        if abs(cmd.angular.z) > 0.45:
            cmd.linear.x = min(cmd.linear.x, self.slow_linear_speed)

        self.pub.publish(cmd)
        self.last_cmd = cmd


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDriver()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
