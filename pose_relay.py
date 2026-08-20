#!/usr/bin/env python3
"""
Relais de pose : PX4 (px4_msgs/VehicleLocalPosition) -> geometry_msgs/PoseStamped.

Objectif : exposer la position du drone dans un type de message qu'Unity
(ROS-TCP-Connector) connait nativement, sans generer px4_msgs cote Unity.

Topic entree : /fmu/out/vehicle_local_position_v1   (QoS best_effort)
Topic sortie : /drone/pose                          (geometry_msgs/PoseStamped)

Position PX4 = NED (x=Nord, y=Est, z=Bas). On republie x/y/z bruts ;
la conversion vers le repere Unity (Y en haut) est faite dans le script Unity.

Lancer :  source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
          python3 ~/Documents/drones/pose_relay.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import PoseStamped


class PoseRelay(Node):
    def __init__(self):
        super().__init__('pose_relay')

        # PX4 publie en best_effort -> il faut un abonne best_effort
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.on_pose,
            px4_qos,
        )
        self.pub = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.n = 0
        self.get_logger().info('Relais actif : /fmu/out/vehicle_local_position_v1 -> /drone/pose')

    def on_pose(self, msg: VehicleLocalPosition):
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        # NED brut (converti cote Unity)
        out.pose.position.x = float(msg.x)
        out.pose.position.y = float(msg.y)
        out.pose.position.z = float(msg.z)
        out.pose.orientation.w = 1.0
        self.pub.publish(out)

        self.n += 1
        if self.n % 50 == 0:  # log toutes les ~50 poses
            self.get_logger().info(
                f'pose #{self.n}  NED x={msg.x:.2f} y={msg.y:.2f} z={msg.z:.2f} (alt={-msg.z:.2f} m)'
            )


def main():
    rclpy.init()
    node = PoseRelay()
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
