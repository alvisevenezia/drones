#!/usr/bin/env python3

import sys
import termios
import tty
import select
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
)


class PX4KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('px4_keyboard_teleop')

        # =========================================================
        # Publishers
        # =========================================================

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # =========================================================
        # Vehicle odometry
        # =========================================================

        # PX4 uORB topics utilisent généralement Best Effort
        px4_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            px4_qos
        )

        # Yaw actuel du drone, en radians
        self.current_yaw = 0.0

        # =========================================================
        # Parameters
        # =========================================================

        self.declare_parameter('speed', 1.0)
        self.declare_parameter('vertical_speed', 0.5)
        self.declare_parameter('yaw_speed', 0.8)
        self.declare_parameter('key_timeout', 0.15)

        self.speed = float(
            self.get_parameter('speed').value
        )

        self.vertical_speed = float(
            self.get_parameter('vertical_speed').value
        )

        self.yaw_speed = float(
            self.get_parameter('yaw_speed').value
        )

        self.key_timeout = float(
            self.get_parameter('key_timeout').value
        )

        # =========================================================
        # Keyboard state
        #
        # On mémorise la dernière fois où chaque touche a été reçue.
        #
        # Cela permet :
        #
        # Z + D
        #
        # de fonctionner simultanément.
        # =========================================================

        self.key_last_seen = {}

        # =========================================================
        # Offboard state
        # =========================================================

        self.offboard_started = False

        # =========================================================
        # Terminal
        # =========================================================

        self.old_terminal_settings = termios.tcgetattr(sys.stdin)

        tty.setcbreak(sys.stdin.fileno())

        # =========================================================
        # Timer
        #
        # 20 Hz
        # =========================================================

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        # =========================================================
        # Information
        # =========================================================

        self.get_logger().info('')
        self.get_logger().info('========================================')
        self.get_logger().info('         PX4 KEYBOARD TELEOP')
        self.get_logger().info('========================================')
        self.get_logger().info('')
        self.get_logger().info('      Z')
        self.get_logger().info('      ^')
        self.get_logger().info('      |')
        self.get_logger().info(' Q <--+--> D')
        self.get_logger().info('      |')
        self.get_logger().info('      v')
        self.get_logger().info('      S')
        self.get_logger().info('')
        self.get_logger().info('G : monter')
        self.get_logger().info('T : descendre')
        self.get_logger().info('')
        self.get_logger().info('A : yaw gauche')
        self.get_logger().info('E : yaw droite')
        self.get_logger().info('')
        self.get_logger().info('SPACE : OFFBOARD + ARM')
        self.get_logger().info('X     : DISARM')
        self.get_logger().info('ESC   : quitter')
        self.get_logger().info('')
        self.get_logger().info(
            f'Vitesse      : {self.speed:.2f} m/s'
        )
        self.get_logger().info(
            f'Vitesse Z    : {self.vertical_speed:.2f} m/s'
        )
        self.get_logger().info(
            f'Yaw          : {self.yaw_speed:.2f} rad/s'
        )
        self.get_logger().info(
            f'Timeout      : {self.key_timeout:.2f} s'
        )
        self.get_logger().info('')
        self.get_logger().info('========================================')

    # =============================================================
    # Timestamp PX4
    # =============================================================

    def timestamp(self):
        return self.get_clock().now().nanoseconds // 1000

    # =============================================================
    # Vehicle odometry callback
    # =============================================================

    def odometry_callback(self, msg):

        # PX4 fournit le quaternion d'attitude.
        #
        # q = [w, x, y, z]
        #
        # On récupère le yaw.
        #
        # PX4 utilise le repère NED :
        #   X = North
        #   Y = East
        #   Z = Down

        q = msg.q

        if len(q) < 4:
            return

        w = float(q[0])
        x = float(q[1])
        y = float(q[2])
        z = float(q[3])

        # Yaw autour de Z
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)

        self.current_yaw = math.atan2(
            sin_yaw,
            cos_yaw
        )

    # =============================================================
    # Vehicle command
    # =============================================================

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0
    ):

        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.command = command

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)

    # =============================================================
    # Arm
    # =============================================================

    def arm(self):

        self.get_logger().info('ARM')

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0
        )

    # =============================================================
    # Disarm
    # =============================================================

    def disarm(self):

        self.get_logger().warn('DISARM')

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0
        )

        self.offboard_started = False

    # =============================================================
    # Start Offboard
    # =============================================================

    def start_offboard(self):

        if self.offboard_started:
            return

        self.get_logger().info(
            'Switching to OFFBOARD + ARM...'
        )

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0
        )

        self.arm()

        self.offboard_started = True

    # =============================================================
    # Offboard heartbeat
    # =============================================================

    def publish_offboard_control_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.timestamp()

        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        self.offboard_pub.publish(msg)

    # =============================================================
    # Check if key is still pressed
    # =============================================================

    def key_is_active(self, key):

        if key not in self.key_last_seen:
            return False

        elapsed = (
            time.monotonic()
            - self.key_last_seen[key]
        )

        return elapsed < self.key_timeout

    # =============================================================
    # Read keyboard
    # =============================================================

    def read_keyboard(self):

        while True:

            ready, _, _ = select.select(
                [sys.stdin],
                [],
                [],
                0
            )

            if not ready:
                break

            key = sys.stdin.read(1)

            if not key:
                continue

            # =====================================================
            # Touches spéciales
            # =====================================================

            if key == ' ':
                self.start_offboard()
                continue

            if key == 'x' or key == 'X':
                self.disarm()
                continue

            if key == '\x1b':
                rclpy.shutdown()
                return

            # =====================================================
            # Touches de mouvement
            # =====================================================

            self.key_last_seen[key.lower()] = time.monotonic()

    # =============================================================
    # Convert body velocity -> NED velocity
    # =============================================================

    def body_to_ned(
        self,
        vx_body,
        vy_body
    ):

        yaw = self.current_yaw

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # ---------------------------------------------------------
        # Body frame:
        #
        # Xbody = devant
        # Ybody = droite
        #
        # NED:
        #
        # X = North
        # Y = East
        # ---------------------------------------------------------

        vx_ned = (
            cos_yaw * vx_body
            - sin_yaw * vy_body
        )

        vy_ned = (
            sin_yaw * vx_body
            + cos_yaw * vy_body
        )

        return vx_ned, vy_ned

    # =============================================================
    # Calculate desired velocity
    # =============================================================

    def calculate_velocity(self):

        # ---------------------------------------------------------
        # Body-frame velocity
        #
        # Z = forward
        # S = backward
        # Q = left
        # D = right
        # ---------------------------------------------------------

        vx_body = 0.0
        vy_body = 0.0

        # Forward / backward

        if self.key_is_active('z'):
            vx_body += self.speed

        if self.key_is_active('s'):
            vx_body -= self.speed

        # Left / right

        if self.key_is_active('q'):
            vy_body -= self.speed

        if self.key_is_active('d'):
            vy_body += self.speed

        # ---------------------------------------------------------
        # Convert body -> NED
        # ---------------------------------------------------------

        vx_ned, vy_ned = self.body_to_ned(
            vx_body,
            vy_body
        )

        # ---------------------------------------------------------
        # Vertical
        #
        # PX4 NED:
        #
        #   negative Z = UP
        #   positive Z = DOWN
        # ---------------------------------------------------------

        vz_ned = 0.0

        if self.key_is_active('g'):
            vz_ned -= self.vertical_speed

        if self.key_is_active('t'):
            vz_ned += self.vertical_speed

        return (
            vx_ned,
            vy_ned,
            vz_ned
        )

    # =============================================================
    # Calculate yaw speed
    # =============================================================

    def calculate_yaw_speed(self):

        yaw_speed = 0.0

        # A = gauche
        #
        # PX4/NED :
        # yaw positif = rotation horaire vue du dessus
        #
        # Donc gauche = négatif.

        if self.key_is_active('a'):
            yaw_speed -= self.yaw_speed

        if self.key_is_active('e'):
            yaw_speed += self.yaw_speed

        return yaw_speed

    # =============================================================
    # Publish trajectory setpoint
    # =============================================================

    def publish_trajectory_setpoint(self):

        vx, vy, vz = self.calculate_velocity()

        yaw_speed = self.calculate_yaw_speed()

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        # ---------------------------------------------------------
        # Position non utilisée
        # ---------------------------------------------------------

        msg.position = [
            float('nan'),
            float('nan'),
            float('nan')
        ]

        # ---------------------------------------------------------
        # Velocity utilisée
        # ---------------------------------------------------------

        msg.velocity = [
            float(vx),
            float(vy),
            float(vz)
        ]

        # ---------------------------------------------------------
        # Acceleration / jerk non utilisés
        # ---------------------------------------------------------

        msg.acceleration = [
            float('nan'),
            float('nan'),
            float('nan')
        ]

        msg.jerk = [
            float('nan'),
            float('nan'),
            float('nan')
        ]

        # ---------------------------------------------------------
        # Yaw absolu non utilisé
        # ---------------------------------------------------------

        msg.yaw = float('nan')

        # ---------------------------------------------------------
        # Yaw rate utilisé
        # ---------------------------------------------------------

        msg.yawspeed = float(yaw_speed)

        self.trajectory_pub.publish(msg)

    # =============================================================
    # Main control loop
    # =============================================================

    def control_loop(self):

        # ---------------------------------------------------------
        # 1. Lire les nouvelles touches
        # ---------------------------------------------------------

        self.read_keyboard()

        # ---------------------------------------------------------
        # 2. Toujours maintenir le heartbeat Offboard
        # ---------------------------------------------------------

        self.publish_offboard_control_mode()

        # ---------------------------------------------------------
        # 3. Envoyer le setpoint
        #
        # Même lorsqu'aucune touche n'est active :
        #
        # vx = 0
        # vy = 0
        # vz = 0
        #
        # => le drone s'arrête.
        # ---------------------------------------------------------

        self.publish_trajectory_setpoint()

    # =============================================================
    # Cleanup
    # =============================================================

    def destroy_node(self):

        # Avant de quitter, envoyer une commande de vitesse nulle.

        self.key_last_seen.clear()

        if rclpy.ok():
            self.publish_trajectory_setpoint()

        # Restaurer le terminal

        try:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                self.old_terminal_settings
            )
        except Exception:
            pass

        super().destroy_node()


# =================================================================
# Main
# =================================================================

def main(args=None):

    rclpy.init(args=args)

    node = PX4KeyboardTeleop()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()