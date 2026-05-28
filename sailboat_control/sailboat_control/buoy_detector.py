#!/usr/bin/env python3

"""Detect nearby buoys from the 2D LiDAR scan."""

import math

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class BuoyDetector(Node):
    """Cluster valid LiDAR returns and publish the closest buoy candidate."""

    def __init__(self) -> None:
        super().__init__('buoy_detector')

        self.min_cluster_points = 3
        self.max_cluster_gap = 0.45
        self.min_detection_range = 0.5
        self.max_detection_range = 20.0

        self.create_subscription(
            LaserScan,
            '/boat/lidar/scan',
            self.scan_callback,
            10
        )
        self.buoy_pub = self.create_publisher(
            PointStamped,
            '/perception/buoy/relative_position',
            10
        )

        self.get_logger().info('Buoy detector started.')

    def is_valid_range(self, scan: LaserScan, value: float) -> bool:
        return (
            math.isfinite(value) and
            max(scan.range_min, self.min_detection_range) <= value and
            value <= min(scan.range_max, self.max_detection_range)
        )

    def point_from_scan(
        self,
        scan: LaserScan,
        index: int,
        distance: float
    ) -> tuple[float, float, float]:
        angle = scan.angle_min + index * scan.angle_increment
        return (
            distance * math.cos(angle),
            distance * math.sin(angle),
            distance,
        )

    def build_clusters(
        self,
        scan: LaserScan
    ) -> list[list[tuple[float, float, float]]]:
        clusters = []
        current = []
        previous = None

        for index, distance in enumerate(scan.ranges):
            if not self.is_valid_range(scan, distance):
                if current:
                    clusters.append(current)
                    current = []
                previous = None
                continue

            point = self.point_from_scan(scan, index, distance)
            if previous is not None:
                gap = math.hypot(
                    point[0] - previous[0],
                    point[1] - previous[1]
                )
                if gap > self.max_cluster_gap:
                    if current:
                        clusters.append(current)
                    current = []

            current.append(point)
            previous = point

        if current:
            clusters.append(current)

        return clusters

    def cluster_center(
        self,
        cluster: list[tuple[float, float, float]]
    ) -> tuple[float, float, float]:
        x = sum(point[0] for point in cluster) / len(cluster)
        y = sum(point[1] for point in cluster) / len(cluster)
        distance = math.hypot(x, y)
        return x, y, distance

    def scan_callback(self, scan: LaserScan) -> None:
        candidates = []
        for cluster in self.build_clusters(scan):
            if len(cluster) < self.min_cluster_points:
                continue

            x, y, distance = self.cluster_center(cluster)
            candidates.append((distance, x, y))

        if not candidates:
            return

        distance, x, y = min(candidates, key=lambda item: item[0])
        msg = PointStamped()
        msg.header = scan.header
        msg.header.frame_id = 'boat_lidar'
        msg.point.x = x
        msg.point.y = y
        msg.point.z = 0.0
        self.buoy_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BuoyDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
