#!/usr/bin/env python3
"""
TrialMonitor – Nodo que observa al robot durante una evaluación del GA.

Mide:
  - distancia recorrida (path_length, integrando /odom)
  - colisiones (/scan con mínima por debajo de un umbral)
  - distancia al punto de salida (para detectar vuelta completa)
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


COLLISION_THRESHOLD = 0.13   # m, lectura láser por debajo = chocaste
LAP_RETURN_RADIUS   = 1.5    # m, "estás cerca del punto de salida"
LAP_FAR_AWAY        = 6.0    # m, hay que alejarse antes para considerar vuelta


class TrialMonitor(Node):
    def __init__(self):
        super().__init__('trial_monitor')

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, scan_qos)

        self.reset()

    def reset(self):
        """Reinicia las métricas para una nueva evaluación."""
        self.start_xy        = None
        self.last_xy         = None
        self.path_length     = 0.0
        self.collisions      = 0
        self.min_clearance   = float('inf')
        self.went_far        = False    # se alejó del punto de salida en algún momento
        self.lap_completed   = False
        self.start_time      = self.get_clock().now()
        self.last_seen_time  = None

    # ── Callbacks ────────────────────────────────────────────────────────────
    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.start_xy is None:
            self.start_xy = (x, y)
            self.last_xy = (x, y)
            return

        dx = x - self.last_xy[0]
        dy = y - self.last_xy[1]
        # Filtrar saltos enormes (un teleport "perdido")
        step = math.hypot(dx, dy)
        if step < 1.0:
            self.path_length += step
        self.last_xy = (x, y)
        self.last_seen_time = self.get_clock().now()

        # ¿Se ha alejado lo suficiente? (para luego detectar vuelta)
        d_start = math.hypot(x - self.start_xy[0], y - self.start_xy[1])
        if d_start > LAP_FAR_AWAY:
            self.went_far = True
        # ¿Ha vuelto al punto de salida tras alejarse?
        if self.went_far and d_start < LAP_RETURN_RADIUS:
            self.lap_completed = True

    def on_scan(self, msg):
        valid = [r for r in msg.ranges
                 if not math.isinf(r) and not math.isnan(r) and r > 0.0]
        if not valid:
            return
        m = min(valid)
        if m < self.min_clearance:
            self.min_clearance = m
        if m < COLLISION_THRESHOLD:
            self.collisions += 1

    # ── Fitness ──────────────────────────────────────────────────────────────
    def fitness(self, eval_seconds: float) -> float:
        """
        Función de fitness para el GA.
          + distancia recorrida   (queremos ir lejos)
          - penalización por colisiones
          - penalización si min_clearance es bajo (rozó pared)
          + bonus grande si completó vuelta
          + bonus por vuelta rápida (tiempo restante al volver al inicio)
        """
        f = self.path_length

        # Penalización por colisiones (cada lectura por debajo de umbral)
        f -= 0.5 * self.collisions

        # Si en algún momento estuvo MUY cerca de chocar
        if self.min_clearance < 0.10:
            f -= 5.0

        # Vuelta completa = recompensa fuerte
        if self.lap_completed:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            f += 50.0
            f += max(0.0, eval_seconds - elapsed) * 2.0  # cuanto antes mejor

        return f

    def summary(self) -> dict:
        return {
            'path_length':   round(self.path_length, 2),
            'collisions':    self.collisions,
            'min_clearance': round(self.min_clearance, 3),
            'lap_completed': self.lap_completed,
        }
