#!/usr/bin/env python3
"""
Carte ToF dense (carte A du double SLAM) — accumulee dans le repere 'map' CORRIGE.

Projection 6-DOF CORRECTE et VECTORISEE : le nuage /tof/points est exprime dans le
repere REEL du capteur (cloud.header.frame_id = 'tof_link', co-localise avec la camera
via une TF statique). On transforme TOUT le nuage d'un coup (numpy) vers 'map' via :

    rgbd_odometry (VIO) : odom -> base_link       (pose 6-DOF, roll/pitch inclus)
    rtabmap             : map  -> odom            (correction loop-closure = anti-drift)
    static_tf           : base_link -> camera_link -> tof_link (montage reel, FLU x-avant)

Points cles :
  - transformation VECTORISEE (numpy batch) : tient les 2268 pts a 15 Hz sans accumuler
    de retard -> la pose utilisee est fraiche -> projection correcte (avant : boucle
    Python par point -> backlog -> poses perimees -> points etales dans le sol).
  - la fleche drone est publiee sur un timer DECOUPLE (temps reel), pas dans le callback
    nuage -> plus de retard de plusieurs secondes.

Entree  : /tof/points   (PointCloud2, repere 'tof_link', VL53L9CX 54x42)
Sortie  : /slam_map_tof  (PointCloud2, repere 'map') + /drone_marker (fleche)

Fallback 'odom' si 'map' pas encore dispo ; reset de la carte au changement de repere.
"""
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import Marker

import tf2_ros

MAX_RANGE = 8.9   # m : au-dela = pas de retour (portee VL53L9CX ~9 m) -> rejete


def quat_rotate(q, v):
    """Applique le quaternion q=(x,y,z,w) au vecteur v. v peut etre (3,) ou (N,3)."""
    x, y, z, w = q
    u = np.array([x, y, z])
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


class SlamMapperToF(Node):
    def __init__(self):
        super().__init__('slam_mapper_tof')
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(PointCloud2, '/tof/points', self.on_cloud, sensor_qos)
        self.map_pub = self.create_publisher(PointCloud2, '/slam_map_tof', 1)
        self.marker_pub = self.create_publisher(Marker, '/drone_marker', 1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map_points = {}     # voxel -> (x,y,z)
        self.voxel = 0.08
        self.cur_frame = None    # 'map' ou 'odom'

        self.create_timer(0.1, self.publish_marker)   # fleche temps reel (decouplee du nuage)
        self.create_timer(1.0, self.publish_map)
        self.get_logger().info('Carte ToF active : /tof/points -> /slam_map_tof '
                               '(projection TF 6-DOF vectorisee, repere map corrige)')

    def lookup(self, target, source, stamp):
        """TF target<-source a 'stamp', sinon derniere connue ; None si absente."""
        for when in (stamp, Time()):
            try:
                return self.tf_buffer.lookup_transform(
                    target, source, when, timeout=Duration(seconds=0.03))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue
        return None

    def on_cloud(self, cloud: PointCloud2):
        stamp = Time.from_msg(cloud.header.stamp)
        src = cloud.header.frame_id or 'tof_link'

        tf = None
        frame = None
        for candidate in ('map', 'odom'):
            tf = self.lookup(candidate, src, stamp)
            if tf is not None:
                frame = candidate
                break
        if tf is None:
            return

        if frame != self.cur_frame:
            self.get_logger().info(f'Carte ToF : repere -> {frame} (reset accumulation)')
            self.map_points.clear()
            self.cur_frame = frame

        t = tf.transform.translation
        q = tf.transform.rotation
        trans = np.array([t.x, t.y, t.z])
        quat = np.array([q.x, q.y, q.z, q.w])

        pts = pc2.read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            return
        rng = np.linalg.norm(pts, axis=1)
        # nuage fusionne 360deg + retire le SOL (base_link z<-1.8 m : le drone vole a ~2.5 m)
        # -> ne garde que les structures verticales (etageres, poteaux, murs)
        keep = np.isfinite(rng) & (rng > 0.02) & (rng < MAX_RANGE) & (pts[:, 2] > -1.8)
        pts = pts[keep]
        if pts.shape[0] == 0:
            return

        w = trans + quat_rotate(quat, pts.astype(np.float64))            # (N,3) dans 'frame'
        keys = np.round(w / self.voxel).astype(np.int64)
        for k, wp in zip(map(tuple, keys.tolist()), w.tolist()):
            self.map_points[k] = (wp[0], wp[1], wp[2])

    def publish_marker(self):
        # fleche publiee dans 'odom' (repere brut) -> colle aux axes TF de la camera.
        # (la carte, elle, est dans 'map' corrige ; l'ecart map<->odom = le drift corrige)
        frame = 'odom'
        tf = self.lookup(frame, 'base_link', Time())   # derniere pose connue = temps reel
        if tf is None:
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        mk = Marker()
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.header.frame_id = frame
        mk.ns = 'drone'
        mk.id = 0
        mk.type = Marker.ARROW
        mk.action = Marker.ADD
        mk.pose.position = Point(x=t.x, y=t.y, z=t.z)
        mk.pose.orientation = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
        mk.scale.x = 0.8
        mk.scale.y = 0.15
        mk.scale.z = 0.15
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.1, 0.8, 1.0, 1.0
        self.marker_pub.publish(mk)

    def publish_map(self):
        if not self.map_points or self.cur_frame is None:
            return
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = self.cur_frame
        self.map_pub.publish(pc2.create_cloud_xyz32(h, list(self.map_points.values())))


def main():
    rclpy.init()
    node = SlamMapperToF()
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
