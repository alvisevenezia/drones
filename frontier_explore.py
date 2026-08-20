#!/usr/bin/env python3
"""
Exploration par FRONTIERES + grille d'occupation 2D (vue de dessus des obstacles).

- Grille 2D (repere 'odom') par ray-tracing du nuage d'obstacles ToF :
    -1 = inconnu, 0 = libre (rayon capteur traverse), 100 = occupe (obstacle).
- FRONTIERES = cellules libres adjacentes a l'inconnu = bords de la zone exploree.
- Choisit le meilleur amas de frontieres (gros + proche, > distance mini) = but d'exploration.
- Publie pour RViz + le controleur :
    /explore/grid        (OccupancyGrid)  -> display "Map"
    /explore/frontiers   (Marker POINTS)  -> cellules frontiere (jaune)
    /explore/goal_marker (Marker SPHERE)  -> point vise (magenta)
    /explore/goal        (PointStamped, odom) -> lu par patrol.py pour y voler
"""
import math
import heapq
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Point, PointStamped, Quaternion
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker

RES = 0.25             # m / cellule
HALF = 40.0            # demi-taille carte (m) -> 80x80 m
N = int(2 * HALF / RES)
ORIGIN = -HALF         # coin bas-gauche de la grille (x=y=ORIGIN) en odom
FREE_SAMPLES = 8       # echantillons libres le long de chaque rayon
FLOOR_Z = -1.8         # base_link : en-dessous = sol (drone a ~2.5 m) -> ignore
MIN_GOAL_DIST = 2.5    # m : ne pas viser une frontiere trop proche
CLUSTER = 1.0          # m : taille de regroupement des frontieres
INFLATE = 3            # cellules : marge de securite autour des obstacles (~0.75 m)
LOOKAHEAD = 2.0        # m : waypoint publie a cette distance le long du chemin A*
EXPLORE_TIME = 45      # s d'exploration frontieres avant un RETOUR force (active loop-closure)
REVISIT_TIME = 25      # s max pour rejoindre l'ancien point avant de reprendre l'exploration
REVISIT_REACH = 1.5    # m : considere l'ancien point atteint
LOG_EVERY = 20


def w2c(x, y):
    """monde odom (m) -> indices grille (i=ligne~y, j=col~x)."""
    j = np.floor((x - ORIGIN) / RES).astype(int)
    i = np.floor((y - ORIGIN) / RES).astype(int)
    return i, j


class Frontier(Node):
    def __init__(self):
        super().__init__('frontier_explore')
        sq = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(PointCloud2, '/tof/points', self.on_cloud, sq)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/explore/grid', 1)
        self.fr_pub = self.create_publisher(Marker, '/explore/frontiers', 1)
        self.goal_mk_pub = self.create_publisher(Marker, '/explore/goal_marker', 1)
        self.goal_pub = self.create_publisher(PointStamped, '/explore/goal', 1)

        self.grid = np.full((N, N), -1, dtype=np.int8)
        self.drone = None
        self.drone_yaw = 0.0
        self.trail = []          # trajectoire memorisee (anciens lieux) pour l'active loop-closure
        self.mode = 'explore'    # 'explore' (frontieres) ou 'revisit' (retour ancien point)
        self.mode_step = 0       # compteur de steps dans le mode courant
        self.revisit_target = None
        self.odom_count = 0
        self.create_timer(1.0, self.step)
        self.get_logger().info(f'Frontieres : grille {N}x{N} @ {RES} m -> /explore/grid + /explore/goal')

    def on_odom(self, m: Odometry):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.drone = np.array([p.x, p.y])
        self.drone_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                    1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom_count += 1
        if self.odom_count == 1:
            self.get_logger().info(
                f'odom={self.drone[0]:.2f},{self.drone[1]:.2f} yaw={self.drone_yaw:.2f} rad')

    def on_cloud(self, cloud: PointCloud2):
        if self.drone is None:
            return
        pts = pc2.read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            return
        keep = np.isfinite(pts).all(axis=1) & (pts[:, 2] > FLOOR_Z)
        pts = pts[keep]
        if pts.shape[0] == 0:
            return
        # base_link -> odom (yaw + translation)
        c, s = math.cos(self.drone_yaw), math.sin(self.drone_yaw)
        ox = self.drone[0] + c * pts[:, 0] - s * pts[:, 1]
        oy = self.drone[1] + s * pts[:, 0] + c * pts[:, 1]
        dx, dy = self.drone

        # espace LIBRE : echantillons le long des rayons drone->obstacle
        for t in np.linspace(0.0, 0.9, FREE_SAMPLES):
            i, j = w2c(dx + t * (ox - dx), dy + t * (oy - dy))
            ok = (i >= 0) & (i < N) & (j >= 0) & (j < N)
            ii, jj = i[ok], j[ok]
            notocc = self.grid[ii, jj] != 100
            self.grid[ii[notocc], jj[notocc]] = 0
        # OBSTACLES
        i, j = w2c(ox, oy)
        ok = (i >= 0) & (i < N) & (j >= 0) & (j < N)
        self.grid[i[ok], j[ok]] = 100

    def step(self):
        if self.drone is None:
            return
        self.publish_grid()
        self.mode_step += 1

        # memorise la trajectoire (un point tous les ~1.5 m) = les "lieux connus"
        if not self.trail or np.linalg.norm(self.drone - self.trail[-1]) > 1.5:
            self.trail.append(self.drone.copy())

        # --- machine explore <-> revisite (ACTIVE loop-closure) ---
        if self.mode == 'explore':
            if self.mode_step > EXPLORE_TIME and len(self.trail) > 12:
                self.revisit_target = self.trail[len(self.trail) // 4]   # ancien lieu (1er quart)
                self.mode, self.mode_step = 'revisit', 0
                self.get_logger().info(f'ACTIVE LOOP-CLOSURE : retour vers ancien lieu '
                                       f'({self.revisit_target[0]:.1f},{self.revisit_target[1]:.1f})')
        else:  # revisit
            if (np.linalg.norm(self.drone - self.revisit_target) < REVISIT_REACH
                    or self.mode_step > REVISIT_TIME):
                self.mode, self.mode_step = 'explore', 0
                self.get_logger().info('retour termine -> reprise de l\'exploration')

        # --- but selon le mode ---
        if self.mode == 'revisit' and self.revisit_target is not None:
            wp = self.plan_waypoint(self.revisit_target)
            self.publish_goal(wp if wp is not None else self.revisit_target, self.revisit_target)
        else:
            self.find_frontier()

    def find_frontier(self):
        free = self.grid == 0
        unk = self.grid == -1
        nb = np.zeros_like(unk)
        nb[1:, :] |= unk[:-1, :]; nb[:-1, :] |= unk[1:, :]
        nb[:, 1:] |= unk[:, :-1]; nb[:, :-1] |= unk[:, 1:]
        idx = np.argwhere(free & nb)
        if idx.shape[0] == 0:
            return
        fx = ORIGIN + idx[:, 1] * RES + RES / 2
        fy = ORIGIN + idx[:, 0] * RES + RES / 2
        fw = np.column_stack([fx, fy])
        self.publish_frontiers(fw)

        # regroupement grossier -> meilleur amas (gros + proche, au-dela de MIN_GOAL_DIST)
        keys = np.round(fw / CLUSTER).astype(int)
        groups = {}
        for k, p in zip(map(tuple, keys.tolist()), fw):
            groups.setdefault(k, []).append(p)
        # candidats tries par score (gros + proche), au-dela de MIN_GOAL_DIST
        cands = []
        for cells in groups.values():
            cen = np.mean(cells, axis=0)
            d = float(np.linalg.norm(cen - self.drone))
            if d < MIN_GOAL_DIST:
                continue
            cands.append((len(cells) / (1.0 + d), cen))
        cands.sort(key=lambda c: c[0], reverse=True)

        # on prend la meilleure frontiere ATTEIGNABLE (A* trouve un chemin) -> waypoint qui contourne.
        # Si la n°1 est derriere un mur (pas de chemin), on essaie la suivante -> plus de blocage.
        for _, cen in cands[:8]:
            wp = self.plan_waypoint(cen)
            if wp is not None and np.linalg.norm(wp - self.drone) > 0.8:
                self.publish_goal(wp, cen)
                return
        # aucune frontiere atteignable -> dernier recours : vise la meilleure en direct
        if cands:
            self.publish_goal(cands[0][1], cands[0][1])

    def plan_waypoint(self, goal_world):
        """A* sur la grille (obstacles gonflis bloquis) -> point a ~LOOKAHEAD m sur le chemin."""
        sj = int((self.drone[0] - ORIGIN) / RES); si = int((self.drone[1] - ORIGIN) / RES)
        gj = int((goal_world[0] - ORIGIN) / RES); gi = int((goal_world[1] - ORIGIN) / RES)
        if not (0 <= si < N and 0 <= sj < N and 0 <= gi < N and 0 <= gj < N):
            return None
        occ = self.grid == 100
        infl = occ.copy()
        for _ in range(INFLATE):
            infl[1:, :] |= occ[:-1, :]; infl[:-1, :] |= occ[1:, :]
            infl[:, 1:] |= occ[:, :-1]; infl[:, :-1] |= occ[:, 1:]
            occ = infl.copy()
        infl[si, sj] = False                       # ne pas bloquer la case du drone
        path = self.astar(infl, (si, sj), (gi, gj))
        if not path or len(path) < 2:
            return None
        dcum, prev = 0.0, path[0]
        for c in path[1:]:
            dcum += math.hypot((c[0] - prev[0]) * RES, (c[1] - prev[1]) * RES)
            prev = c
            if dcum >= LOOKAHEAD:
                return np.array([ORIGIN + c[1] * RES + RES / 2, ORIGIN + c[0] * RES + RES / 2])
        c = path[-1]
        return np.array([ORIGIN + c[1] * RES + RES / 2, ORIGIN + c[0] * RES + RES / 2])

    def astar(self, blocked, start, goal):
        def h(a):
            return math.hypot(a[0] - goal[0], a[1] - goal[1])
        nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        openh = [(h(start), 0.0, start)]
        came, gsc, seen, cnt = {}, {start: 0.0}, set(), 0
        while openh:
            cnt += 1
            if cnt > 30000:
                return None
            _, gc, cur = heapq.heappop(openh)
            if cur == goal:
                path = [cur]
                while cur in came:
                    cur = came[cur]; path.append(cur)
                return path[::-1]
            if cur in seen:
                continue
            seen.add(cur)
            for di, dj in nbrs:
                ni, nj = cur[0] + di, cur[1] + dj
                if ni < 0 or ni >= N or nj < 0 or nj >= N or blocked[ni, nj]:
                    continue
                ng = gc + (1.4142 if di and dj else 1.0)
                if (ni, nj) not in gsc or ng < gsc[(ni, nj)]:
                    gsc[(ni, nj)] = ng; came[(ni, nj)] = cur
                    heapq.heappush(openh, (ng + h((ni, nj)), ng, (ni, nj)))
        return None

    def publish_grid(self):
        g = OccupancyGrid()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'odom'
        g.info.resolution = RES
        g.info.width = N
        g.info.height = N
        g.info.origin.position.x = float(ORIGIN)
        g.info.origin.position.y = float(ORIGIN)
        g.info.origin.orientation.w = 1.0
        g.data = self.grid.reshape(-1).astype(np.int8).tolist()
        self.grid_pub.publish(g)

    def publish_frontiers(self, fw):
        mk = Marker()
        mk.header.frame_id = 'odom'
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = 'frontiers'; mk.id = 0
        mk.type = Marker.POINTS; mk.action = Marker.ADD
        mk.scale.x = mk.scale.y = 0.15
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 1.0, 0.0, 0.8
        mk.points = [Point(x=float(p[0]), y=float(p[1]), z=0.0) for p in fw]
        self.fr_pub.publish(mk)

    def publish_goal(self, cen, marker_cen=None):
        if marker_cen is None:
            marker_cen = cen
        ps = PointStamped()
        ps.header.frame_id = 'odom'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.point = Point(x=float(cen[0]), y=float(cen[1]), z=0.0)
        self.goal_pub.publish(ps)
        if self.odom_count == 1:
            self.get_logger().info(
                f'goal publish target=({cen[0]:.2f},{cen[1]:.2f}) marker=({marker_cen[0]:.2f},{marker_cen[1]:.2f})')

        mk = Marker()
        mk.header.frame_id = 'odom'
        mk.header.stamp = ps.header.stamp
        mk.ns = 'goal'; mk.id = 0
        mk.type = Marker.SPHERE; mk.action = Marker.ADD
        mk.pose.position = Point(x=float(marker_cen[0]), y=float(marker_cen[1]), z=0.5)
        mk.pose.orientation = Quaternion(w=1.0)
        mk.scale.x = mk.scale.y = mk.scale.z = 0.6
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.0, 1.0, 1.0
        self.goal_mk_pub.publish(mk)


def main():
    rclpy.init()
    node = Frontier()
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
