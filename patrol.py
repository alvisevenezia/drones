#!/usr/bin/env python3
"""
Exploration autonome du depot (OFFBOARD) + EVITEMENT d'obstacles via VL53.

Strategie = balayage systematique (lawnmower) d'une zone bornee + scan yaw 360 deg
aux waypoints, MAIS avec evitement reactif : le VL53 (regarde devant) mesure la
distance a l'obstacle ; si un mur est trop proche, le drone s'arrete (ne fonce pas),
et s'il reste bloque, il abandonne le waypoint et passe au suivant.

Machine a etats : MOVE (vol doux vers waypoint, stoppe si mur proche) -> SCAN (360 deg) -> suivant.
Armement : 'commander arm -f' dans pxh>.
Lancer :  python3 ~/Documents/drones/patrol.py --ros-args -p use_sim_time:=true
Repere PX4 = NED : x=Nord, y=Est, z=Bas (z=-2.5 => 2.5 m).
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped

# ---- Zone d'exploration (repere local NED) ----
X_MIN, X_MAX = -1.0, 9.0
Y_MIN, Y_MAX = -3.0, 4.0
LINE_SPACING = 2.0
ALT = -2.5
SETTLE_TIME = 4.0        # s : stabilisation (hover) apres avoir atteint l'altitude, avant d'explorer

# ---- Vol ----
SPEED = 0.4      # doux : l'ICP (FoV avant etroit) a besoin de recouvrement entre scans
DT = 0.1
STEP = SPEED * DT
LEAD = 1.0
REACH = 0.6
YAW_RATE = 0.25   # lacet LENT : rotation rapide = ICP decroche (FoV avant etroit)

# ---- Cercle (demarrage doux, ferme une boucle -> correction de drift) ----
RADIUS = 3.0
OMEGA = 0.15      # rad/s -> tour complet ~42 s
RAMP = 5.0        # s : rampe douce du rayon 0 -> RADIUS

# ---- Evitement d'obstacles (VL53) ----
SAFE_DIST = 0.4          # m : arret d'URGENCE seul (l'A* de frontier_explore contourne deja avec marge 0.75 m)
BLOCKED_LIMIT = 30       # ticks (~3 s) bloque -> abandonne le waypoint
LOG_EVERY = 20

ARMED = 2
OFFBOARD = 14


def build_lawnmower():
    wps = []
    ys = np.arange(Y_MIN, Y_MAX + 1e-6, LINE_SPACING)
    for i, y in enumerate(ys):
        pair = [(X_MIN, float(y)), (X_MAX, float(y))]
        wps += pair if i % 2 == 0 else pair[::-1]
    return wps


class Explorer(Node):
    def __init__(self):
        super().__init__('patrol')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub_ocm = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.on_pos, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', self.on_status, qos)
        self.create_subscription(PointCloud2, '/tof/points', self.on_vl53, sensor_qos)
        self.create_subscription(Odometry, '/odom', self.on_vio, 10)
        self.create_subscription(PointStamped, '/explore/goal', self.on_goal, 10)

        self.pos = np.zeros(3)
        self.sp = np.array([0.0, 0.0, 0.0])
        self.armed = False
        self.offboard = False
        self.clearance = 999.0        # distance mini a un obstacle devant (m)
        self.blocked = 0
        self.tick = 0

        self.wps = build_lawnmower()
        self.wp = 0
        self.phase = 'MOVE'
        self.yaw = 0.0
        self.scan_accum = 0.0
        self.center = None       # centre du cercle (memorise au 1er tick arme)
        self.t0 = 0
        self.settle_t0 = None    # tick d'arrivee a l'altitude (debut stabilisation)
        self.vio_pos = None      # position VIO (odom) du drone
        self.goal_odom = None    # but d'exploration (odom, depuis frontier_explore)
        self.goal_count = 0
        self.vio_count = 0

        self.create_timer(DT, self.loop)
        self.get_logger().info(f'Exploration + evitement VL53 : {len(self.wps)} waypoints. '
                               f'Arme via "commander arm -f".')

    def on_pos(self, m: VehicleLocalPosition):
        self.pos = np.array([float(m.x), float(m.y), float(m.z)])

    def on_status(self, m: VehicleStatus):
        self.armed = (m.arming_state == ARMED)
        self.offboard = (m.nav_state == OFFBOARD)

    def on_vio(self, m: Odometry):
        p = m.pose.pose.position
        self.vio_pos = np.array([p.x, p.y])   # odom ~ ENU (x=Est, y=Nord)
        self.vio_count += 1
        if self.vio_count == 1:
            self.get_logger().info(f'odom recv x={p.x:.2f} y={p.y:.2f}')

    def on_goal(self, m: PointStamped):
        self.goal_odom = np.array([m.point.x, m.point.y])
        self.goal_count += 1
        if self.goal_count == 1:
            self.get_logger().info(f'goal recv x={m.point.x:.2f} y={m.point.y:.2f}')

    def on_vl53(self, cloud: PointCloud2):
        # distance mini vers l'avant (points x>0), = obstacle le plus proche devant
        best = 999.0
        for p in pc2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True):
            x = float(p[0])
            z = float(p[2])
            # x>0 = devant ; |z|<0.8 = bande ~horizontale -> ignore le sol/plafond
            if x <= 0.1 or abs(z) > 0.8:
                continue
            r = math.sqrt(x * x + float(p[1]) ** 2 + z * z)
            if r < best:
                best = r
        self.clearance = best

    def _stamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def send_cmd(self, command, p1=0.0, p2=0.0):
        c = VehicleCommand()
        c.timestamp = self._stamp()
        c.command = command
        c.param1 = p1
        c.param2 = p2
        c.target_system = c.target_component = 1
        c.source_system = c.source_component = 1
        c.from_external = True
        self.pub_cmd.publish(c)

    def publish_sp(self, target_xyz, yaw):
        ocm = OffboardControlMode()
        ocm.timestamp = self._stamp()
        ocm.position = True
        self.pub_ocm.publish(ocm)
        sp = TrajectorySetpoint()
        sp.timestamp = self._stamp()
        sp.position = [float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])]
        sp.yaw = float(yaw)
        self.pub_sp.publish(sp)

    def next_waypoint(self, reason):
        self.phase = 'MOVE'
        self.blocked = 0
        self.wp = (self.wp + 1) % len(self.wps)
        self.get_logger().info(f'{reason} -> waypoint {self.wp} : {self.wps[self.wp]}')

    def loop(self):
        self.tick += 1

        # heartbeat offboard + armement (tant qu'on n'est pas arme/offboard)
        if not (self.armed and self.offboard):
            self.publish_sp(self.pos, 0.0)
            if self.tick >= 10 and self.tick % 10 == 0:
                self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.get_logger().info('offboard demande -> tape "commander arm -f" dans pxh>')
            return

        if self.center is None:
            self.center = self.pos[:2].copy()

        # PHASE DECOLLAGE : monter DROIT jusqu'a ALT avant toute exploration.
        # (partir horizontalement pendant la montee fait decrocher l'ICP -> gros drift)
        if abs(self.pos[2] - ALT) > 0.4:
            self.publish_sp(np.array([self.center[0], self.center[1], ALT]), self.yaw)
            if self.tick % 20 == 0:
                self.get_logger().info(f'montee... alt={-self.pos[2]:.1f} m (cible {-ALT:.1f} m)')
            return

        # STABILISATION : hover quelques secondes a l'altitude avant de partir explorer
        if self.settle_t0 is None:
            self.settle_t0 = self.tick
            self.get_logger().info(f'altitude atteinte -> stabilisation {SETTLE_TIME:.0f} s')
        if (self.tick - self.settle_t0) * DT < SETTLE_TIME:
            self.publish_sp(np.array([self.center[0], self.center[1], ALT]), self.yaw)
            return

        # Pas encore de but (ou pas d'odom) -> maintien doux au-dessus du point de decollage
        if self.goal_odom is None or self.vio_pos is None:
            self.publish_sp(np.array([self.center[0], self.center[1], ALT]), self.yaw)
            if self.tick % 20 == 0:
                self.get_logger().info('attente d\'un but /explore/goal...')
            return

        # But odom(ENU) -> NED via vecteur relatif (annule le drift) :
        #   NED = pos_ned + [ (goal-vio).y=Nord , (goal-vio).x=Est ]
        rel = self.goal_odom - self.vio_pos
        target_full = np.array([self.pos[0] + rel[1], self.pos[1] + rel[0], ALT])

        # evitement : stop si obstacle devant
        if self.pos[2] < -1.5 and self.clearance < SAFE_DIST:
            self.publish_sp(self.pos.copy(), self.yaw)
            if self.tick % 10 == 0:
                self.get_logger().warn(f'obstacle devant ({self.clearance:.1f} m) -> stop')
            return

        # consigne lissee (carotte) vers le but + lacet lisse vers la direction
        to_goal = target_full - self.sp
        d = np.linalg.norm(to_goal)
        self.sp = target_full.copy() if d <= STEP else self.sp + to_goal / d * STEP
        lead = self.sp - self.pos
        nlead = np.linalg.norm(lead)
        if nlead > LEAD:
            self.sp = self.pos + lead / nlead * LEAD
        tyaw = math.atan2(target_full[1] - self.pos[1], target_full[0] - self.pos[0])
        dyaw = math.atan2(math.sin(tyaw - self.yaw), math.cos(tyaw - self.yaw))
        self.yaw += max(-YAW_RATE * DT, min(YAW_RATE * DT, dyaw))
        self.publish_sp(self.sp, self.yaw)
        if self.tick % LOG_EVERY == 0:
            self.get_logger().info(
            f'[nav] pos_ned=({self.pos[0]:.2f},{self.pos[1]:.2f},{self.pos[2]:.2f}) | '
            f'vio=({self.vio_pos[0]:.2f},{self.vio_pos[1]:.2f}) | '
            f'goal_odom=({self.goal_odom[0]:.2f},{self.goal_odom[1]:.2f}) | '
            f'target_ned=({target_full[0]:.2f},{target_full[1]:.2f},{target_full[2]:.2f}) | '
            f'yaw={self.yaw:.2f} tyaw={tyaw:.2f} dist={np.linalg.norm(rel):.2f}m clr={self.clearance:.2f}m')


def main():
    rclpy.init()
    node = Explorer()
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
