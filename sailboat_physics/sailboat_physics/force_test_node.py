#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench


class ForceTestNode(Node):
    def __init__(self):
        super().__init__('force_test_node')

        # PUBLISHER
        self.wrench_pub = self.create_publisher(
            Wrench,
            '/world/lake_balaton/wrench',
            10
        )

        # TIMER
        self.timer = self.create_timer(
            0.02,
            self.loop
        )
        self.get_logger().info(
            'Force test node started.'
        )

    def loop(self):
        msg = Wrench()
        # CONSTANT FORCE
        msg.force.x = 50.0
        # Publish
        self.wrench_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ForceTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
