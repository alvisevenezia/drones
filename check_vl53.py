#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


class CheckVL53(Node):

    def __init__(self):
        super().__init__('check_vl53')

        self.sub = self.create_subscription(
            PointCloud2,
            '/tof/points',
            self.callback,
            10
        )

    def callback(self, msg):

        points = list(
            pc2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=False
            )
        )

        valid = 0
        invalid = 0

        xs = []
        ys = []
        zs = []

        for x, y, z in points:

            x = float(x)
            y = float(y)
            z = float(z)

            if (
                math.isfinite(x)
                and math.isfinite(y)
                and math.isfinite(z)
                and x > 0
            ):
                valid += 1

                xs.append(x)
                ys.append(y)
                zs.append(z)

            else:
                invalid += 1

        print(
            f"VL53: total={len(points):2d} "
            f"valid={valid:2d} "
            f"invalid={invalid:2d}"
        )

        if valid:
            print(
                f"  X: {min(xs):.2f} -> {max(xs):.2f} m"
            )
            print(
                f"  Y: {min(ys):.2f} -> {max(ys):.2f} m"
            )
            print(
                f"  Z: {min(zs):.2f} -> {max(zs):.2f} m"
            )


def main():

    rclpy.init()

    node = CheckVL53()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()