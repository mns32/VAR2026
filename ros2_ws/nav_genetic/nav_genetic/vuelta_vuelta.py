#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollower(Node):
    def __init__(self, genome=None):
        super().__init__('wall_follower')
        # Ajustamos el target y las ganancias para un giro más agresivo
        self.target = 0.45 
        self.kp = 6.0
        self.kd = 2.0
        self.speed = 0.15
        
        self.prev_error = 0.0
        self.last_time = self.get_clock().now()
        self.enabled = True

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.on_scan, scan_qos)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def set_enabled(self, value):
        self.enabled = value
        if not value: self.publish_zero()

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def on_scan(self, msg):
        if not self.enabled: return
        
        num_rays = len(msg.ranges)
        if num_rays == 0: return

        # --- LÓGICA DE TRES PUNTOS ---
        # 1. Derecha pura (270°)
        d_side = msg.ranges[int(270 * num_rays / 360)]
        # 2. Diagonal delantera-derecha (315°) - ESTA ES LA CLAVE PARA GIRAR
        d_diag = msg.ranges[int(315 * num_rays / 360)]
        # 3. Frente (0°)
        d_front = msg.ranges[0]

        # Limpiar valores
        def clean(d): return d if (math.isfinite(d) and d > 0.05) else 10.0
        d_side, d_diag, d_front = clean(d_side), clean(d_diag), clean(d_front)

        # PID
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0: dt = 0.033
        self.last_time = now

        # Usamos la distancia más corta entre el lado y la diagonal para el error
        # Esto hace que si la diagonal detecta que la pared se acaba o se acerca, el robot reaccione
        dist_actual = min(d_side, d_diag * 0.8) 
        error = self.target - dist_actual
        deriva = (error - self.prev_error) / dt
        self.prev_error = error

        cmd = Twist()
        
        # SI HAY OBSTÁCULO DELANTE: Girar sobre el eje (Tu idea de parar y girar, pero automática)
        if d_front < 0.5:
            cmd.linear.x = 0.02 # Casi parado
            cmd.angular.z = 1.2  # Giro rápido a la izquierda
        else:
            # SEGUIMIENTO NORMAL
            cmd.linear.x = self.speed
            u = (self.kp * error) + (self.kd * deriva)
            cmd.angular.z = max(-2.0, min(2.0, float(u)))

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(WallFollower())
    rclpy.shutdown()