#!/usr/bin/env python3
"""
Agregation des 5 VL53L9CX (avant, arriere, gauche, droite, dessous) en UN nuage.

Chaque capteur = depth_camera dont le nuage est x-avant dans SON repere oriente.
On applique l'extrinseque (rotation vers la direction de visee + petite translation
de montage) a chaque nuage, on concatene, et on publie /tof/points dans 'base_link'.
-> couverture ~360deg + sol => l'ICP a des contraintes dans toutes les directions
   (robuste en rotation, contrairement au seul capteur avant a FoV etroit).

Entrees : /tof_front|back|left|right|down/points  (PointCloud2, x-avant capteur)
Sortie  : /tof/points  (PointCloud2, repere 'base_link')  -> cloud_denan -> ICP
"""
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


HALF = np.pi / 2.0
# capteur -> (rotation capteur->base_link, translation de montage dans base_link)
EXTRINSICS = {
    '/tof_front/points': (np.eye(3),      np.array([0.10, 0.0, 0.0])),
    '/tof_back/points':  (Rz(np.pi),      np.array([-0.10, 0.0, 0.0])),
    '/tof_left/points':  (Rz(HALF),       np.array([0.0, 0.10, 0.0])),
    '/tof_right/points': (Rz(-HALF),      np.array([0.0, -0.10, 0.0])),
    '/tof_down/points':  (Ry(HALF),       np.array([0.0, 0.0, -0.03])),
}


class ToFAggregate(Node):
    def __init__(self):
        super().__init__('tof_aggregate')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.latest = {}   # topic -> Nx3 (base_link) du dernier nuage recu
        for topic in EXTRINSICS:
            self.create_subscription(PointCloud2, topic,
                                     lambda m, t=topic: self.on_cloud(m, t), qos)
        self.pub = self.create_publisher(PointCloud2, '/tof/points', qos)
        self.create_timer(0.05, self.publish_merged)   # 20 Hz
        self.get_logger().info('Agregation 5x VL53L9CX -> /tof/points (base_link)')

    def on_cloud(self, msg: PointCloud2, topic: str):
        pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            self.latest[topic] = np.empty((0, 3))
            return
        pts = pts[np.isfinite(pts).all(axis=1)].astype(np.float64)
        R, t = EXTRINSICS[topic]
        self.latest[topic] = pts @ R.T + t     # capteur -> base_link

    def publish_merged(self):
        clouds = [c for c in self.latest.values() if c.shape[0]]
        if not clouds:
            return
        merged = np.vstack(clouds)
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = 'base_link'
        self.pub.publish(pc2.create_cloud_xyz32(h, merged.tolist()))


def main():
    rclpy.init()
    node = ToFAggregate()
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
