#!/usr/bin/env python3

"""Keyboard teleoperation for baum sheet length and rudder angle."""

import math
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SailboatTeleop(Node):
    """Publish manual baum and rudder commands from keyboard input."""

    def __init__(self) -> None:
        super().__init__('sailboat_teleop')
        self.sheet_length = math.radians(80)
        self.rudder_angle = 0.0
        self.sail_pub = self.create_publisher(
            Float64,
            '/baum_pos',
            10
        )
        self.rudder_pub = self.create_publisher(
            Float64,
            '/rudder_pos',
            10
        )
        self.running = True
        self.input_thread = threading.Thread(
            target=self.keyboard_loop
        )
        self.input_thread.daemon = True
        self.input_thread.start()
        self.timer = self.create_timer(
            0.05,
            self.publish_commands
        )
        self.print_help()

    def print_help(self) -> None:
        print('')
        print('==============================')
        print(' SAILBOAT TELEOP')
        print('==============================')
        print('A / D  : Rudder left/right')
        print('W / S  : Sail out/in')
        print('SPACE  : Reset controls')
        print('Q      : Quit')
        print('==============================')
        print('')

    def get_key(self) -> str:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(
                fd,
                termios.TCSADRAIN,
                old_settings
            )
        return key

    def keyboard_loop(self) -> None:
        while self.running:
            key = self.get_key()
            if key == 'a':
                self.rudder_angle += 0.05
            elif key == 'd':
                self.rudder_angle -= 0.05
            elif key == 'w':
                self.sheet_length += 0.05
            elif key == 's':
                self.sheet_length -= 0.05
            elif key == ' ':
                self.sheet_length = math.radians(10)
                self.rudder_angle = 0.0
            elif key == 'q':
                self.running = False
                rclpy.shutdown()
                break
            self.rudder_angle = max(
                min(self.rudder_angle, 0.6),
                -0.6
            )
            self.sheet_length = max(
                min(self.sheet_length, math.radians(85)),
                math.radians(5)
            )
            print(
                f'\rSheet={math.degrees(self.sheet_length):5.1f} deg   '
                f'Rudder={self.rudder_angle:+.2f} rad   ',
                end='',
                flush=True
            )

    def publish_commands(self) -> None:
        sail_msg = Float64()
        sail_msg.data = self.sheet_length
        rudder_msg = Float64()
        rudder_msg.data = self.rudder_angle
        self.sail_pub.publish(sail_msg)
        self.rudder_pub.publish(rudder_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SailboatTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
