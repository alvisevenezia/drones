#!/usr/bin/env python3
"""
Evaluation du drift : compare l'odometrie VIO (/odom, approximee) a la verite
terrain PX4 (/fmu/out/vehicle_local_position_v1, quasi-vraie position en SITL).

- Aligne les deux au 1er echantillon commun (offset), puis mesure le drift = ||vio - truth||.
- Publie /vio_drift (Float64, metres) + un marqueur ROUGE de la vraie position
  (/truth_marker, repere 'odom') a comparer a la fleche BLEUE du VIO (/drone_marker).
- Log periodique : drift courant, drift max, distance parcourue -> drift/parcouru en %.

NB verite : en SITL, vehicle_local_position est l'estimee EKF2 (GPS simule) = quasi-vérité
(cm). Suffisant comme reference pour juger le drift de la VIO GPS-denied.
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleLocalPosition
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import Marker


class DriftEval(Node):
    def __init__(self):
        super().__init__('drift_eval')
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1', self.on_truth, px4_qos)
        self.create_subscription(Odometry, '/odom', self.on_vio, 10)
        self.drift_pub = self.create_publisher(Float64, '/vio_drift', 10)
        self.mk_pub = self.create_publisher(Marker, '/truth_marker', 1)
        self.vio_mk_pub = self.create_publisher(Marker, '/vio_marker', 1)

        self.vio = None          # derniere position VIO (ENU, repere odom)
        self.vio_quat = (0.0, 0.0, 0.0, 1.0)
        self.truth = None        # derniere verite terrain (ENU)
        self.truth_yaw = 0.0
        self.vio0 = None         # alignement initial (repere odom)
        self.truth0 = None       # alignement initial (verite)
        self.max_drift = 0.0
        self.path_len = 0.0
        self.last_truth = None

        self.create_timer(2.0, self.report)
        self.get_logger().info('Drift eval : /odom (VIO) vs verite PX4 -> /vio_drift + /truth_marker')

    def on_vio(self, m: Odometry):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.vio = np.array([p.x, p.y, p.z])
        self.vio_quat = (q.x, q.y, q.z, q.w)
        self.publish_vio_marker()

    def publish_vio_marker(self):
        # fleche BLEUE = position VIO (FAST-LIO /odom), repere 'odom' (comme /truth_marker)
        if self.vio is None:
            return
        mk = Marker()
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.header.frame_id = 'odom'
        mk.ns = 'vio'
        mk.id = 0
        mk.type = Marker.ARROW
        mk.action = Marker.ADD
        mk.pose.position = Point(x=float(self.vio[0]), y=float(self.vio[1]), z=float(self.vio[2]))
        mk.pose.orientation = Quaternion(x=self.vio_quat[0], y=self.vio_quat[1],
                                         z=self.vio_quat[2], w=self.vio_quat[3])
        mk.scale.x, mk.scale.y, mk.scale.z = 0.8, 0.15, 0.15
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.1, 0.5, 1.0, 1.0
        self.vio_mk_pub.publish(mk)

    def on_truth(self, m: VehicleLocalPosition):
        if not (m.xy_valid and m.z_valid):
            return
        # NED -> ENU (comme px4_odom.py)
        self.truth = np.array([float(m.y), float(m.x), float(-m.z)])
        self.truth_yaw = (math.pi / 2.0) - float(m.heading)
        if self.last_truth is not None:
            self.path_len += float(np.linalg.norm(self.truth - self.last_truth))
        self.last_truth = self.truth.copy()
        self.ensure_align()
        self.publish_truth_marker()
        self.compute_drift()

    def ensure_align(self):
        if self.vio0 is None and self.vio is not None and self.truth is not None:
            self.vio0 = self.vio.copy()
            self.truth0 = self.truth.copy()
            self.get_logger().info('Drift eval : alignement initial fait, mesure en cours.')

    def aligned_truth(self):
        # verite exprimee dans le repere odom du VIO (coincide au depart)
        return self.vio0 + (self.truth - self.truth0)

    def compute_drift(self):
        if self.vio0 is None or self.vio is None:
            return
        d = float(np.linalg.norm(self.vio - self.aligned_truth()))
        self.max_drift = max(self.max_drift, d)
        self.drift_pub.publish(Float64(data=d))

    def publish_truth_marker(self):
        if self.vio0 is None:
            return
        pos = self.aligned_truth()
        yaw = self.truth_yaw
        mk = Marker()
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.header.frame_id = 'odom'
        mk.ns = 'truth'
        mk.id = 0
        mk.type = Marker.ARROW
        mk.action = Marker.ADD
        mk.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        mk.pose.orientation = Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        mk.scale.x, mk.scale.y, mk.scale.z = 0.8, 0.15, 0.15
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.2, 0.2, 1.0   # ROUGE = verite
        self.mk_pub.publish(mk)

    def report(self):
        if self.vio0 is None:
            self.get_logger().info('Drift eval : en attente de /odom + verite PX4...')
            return
        d = float(np.linalg.norm(self.vio - self.aligned_truth())) if self.vio is not None else 0.0
        pct = (self.max_drift / self.path_len * 100.0) if self.path_len > 0.1 else 0.0
        self.get_logger().info(
            f'DRIFT actuel={d:.3f} m | max={self.max_drift:.3f} m | '
            f'parcouru={self.path_len:.2f} m | drift/parcouru={pct:.1f}%')


def main():
    rclpy.init()
    node = DriftEval()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
