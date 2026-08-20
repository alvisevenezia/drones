#!/usr/bin/env python3
"""
Filtre NaN/Inf du nuage ToF (depth_camera leogue/VL53L9CX) -> nuage DENSE.

Le depth_camera produit un nuage ORGANISE 54x42 dont les zones sans retour sont
NaN (realiste pour un dToF). Mais PCL/ICP (icp_odometry, rtabmap) plante sur les
NaN : "Invalid (NaN, Inf) point given to nearestKSearch". On republie ici un nuage
DENSE (uniquement les points valides) sur /tof/points_dense, consomme par l'odometrie.
"""
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2

# FAST-LIO fait pcl::fromROSMsg -> PointXYZI : il FAUT un champ intensity, sinon nuage vide.
_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]


class CloudDenan(Node):
    def __init__(self):
        super().__init__('cloud_denan')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.sub = self.create_subscription(PointCloud2, '/tof/points', self.cb, qos)
        self.pub = self.create_publisher(PointCloud2, '/tof/points_dense', qos)
        self.get_logger().info('Filtre NaN : /tof/points -> /tof/points_dense (dense, pour ICP/LIO)')

    def cb(self, msg: PointCloud2):
        pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0]:
            pts = pts[np.isfinite(pts).all(axis=1)]
        out = pc2.create_cloud_xyz32(msg.header, pts.tolist() if pts.shape[0] else [])
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CloudDenan()
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
