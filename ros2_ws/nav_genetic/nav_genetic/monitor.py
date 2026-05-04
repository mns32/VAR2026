#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

COLLISION_THRESHOLD = 0.15   
LAP_RETURN_RADIUS   = 1.2    
LAP_FAR_AWAY        = 5.0    

class TrialMonitor(Node):
    def __init__(self):
        super().__init__('trial_monitor')
        
        # QoS compatible con Gazebo
        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, scan_qos)
        self.reset()

    def reset(self):
        self.start_xy = None
        self.last_xy = None
        self.path_length = 0.0
        self.collisions = 0
        self.min_clearance = 10.0
        self.lap_completed = False
        self.far_enough = False
        self.start_time = self.get_clock().now()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        curr = (p.x, p.y)
        if self.start_xy is None:
            self.start_xy = curr
            self.last_xy = curr
            return
        
        self.path_length += math.hypot(curr[0]-self.last_xy[0], curr[1]-self.last_xy[1])
        self.last_xy = curr
        
        dist_to_start = math.hypot(curr[0]-self.start_xy[0], curr[1]-self.start_xy[1])
        if dist_to_start > LAP_FAR_AWAY: self.far_enough = True
        if self.far_enough and dist_to_start < LAP_RETURN_RADIUS: self.lap_completed = True

    def on_scan(self, msg):
        valid = [r for r in msg.ranges if math.isfinite(r) and r > 0.01]
        if not valid: return
        m = min(valid)
        if m < self.min_clearance: self.min_clearance = m
        if m < COLLISION_THRESHOLD: self.collisions += 1

    def fitness(self, eval_seconds: float) -> float:
        f = self.path_length
        f -= 0.5 * self.collisions
        if self.min_clearance < 0.12: f -= 5.0
        if self.lap_completed:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            f += 100.0 + max(0.0, eval_seconds - elapsed) * 2.0
        return f

    def summary(self) -> dict:
        return {
            'path_length': round(self.path_length, 2),
            'collisions': self.collisions,
            'min_dist': round(self.min_clearance, 3),
            'lap': self.lap_completed,
            'lap_completed': self.lap_completed,
        }
