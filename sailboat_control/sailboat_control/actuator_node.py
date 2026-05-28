#!/usr/bin/env python3

"""Forward high-level actuator commands to Gazebo actuator topics."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SailboatActuatorNode(Node):
    """Pass baum and rudder commands through to the simulator interfaces."""

    def __init__(self) -> None:
        super().__init__('sailboat_actuator_node')
        self.baum_pub = self.create_publisher(
            Float64,
            '/baum_pos',
            10
        )
        self.rudder_pub = self.create_publisher(
            Float64,
            '/rudder_pos',
            10
        )
        self.create_subscription(
            Float64,
            '/cmd_baum_pos',
            self.baum_callback,
            10
        )
        self.create_subscription(
            Float64,
            '/cmd_rudder_pos',
            self.rudder_callback,
            10
        )
        self.get_logger().info('Actuator passthrough node started.')

    def baum_callback(self, msg: Float64) -> None:
        self.baum_pub.publish(msg)

    def rudder_callback(self, msg: Float64) -> None:
        self.rudder_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SailboatActuatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
