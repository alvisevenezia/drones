#!/usr/bin/env python3
"""
Odometrie fiable depuis la pose EKF de PX4 (plan B, remplace la VIO).

- Ecoute /fmu/out/vehicle_local_position_v1 (NED + heading).
- Publie /odom (nav_msgs/Odometry, repere ENU 'odom') + TF odom->base_link.
  -> slam_mapper (carte ToF), RTAB-Map (carte dense) et la fleche drone
     utilisent tous cette odometrie qui SUIT toujours le drone en simu.

Approximation : position + lacet (roll/pitch negliges, vol doux) -> robuste et sans
bug de conversion de quaternion.  NED(x=N,y=E,z=Bas) -> ENU(x=E,y=N,z=Haut).
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleLocalPosition
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class Px4Odom(Node):
    def __init__(self):
        super().__init__('px4_odom')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1', self.on_pos, qos)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.get_logger().info('Odometrie PX4 -> /odom + TF odom->base_link')

    def on_pos(self, m: VehicleLocalPosition):
        # NED -> ENU
        ex, ey, ez = float(m.y), float(m.x), float(-m.z)
        yaw = (math.pi / 2.0) - float(m.heading)   # heading NED -> yaw ENU
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = ex
        odom.pose.pose.position.y = ey
        odom.pose.pose.position.z = ez
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = ex
        t.transform.translation.y = ey
        t.transform.translation.z = ez
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = Px4Odom()
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
