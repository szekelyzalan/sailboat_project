#!/usr/bin/env python3

"""Publish the active course leg and track completed mark roundings."""

from dataclasses import dataclass
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray


def strip_verbose_flags(args: list[str]) -> tuple[bool, list[str]]:
    verbose = False
    clean_args = []
    for arg in args:
        if arg in ('--debug', '--verbose'):
            verbose = True
        else:
            clean_args.append(arg)
    return verbose, clean_args


@dataclass
class CourseMark:
    """A mark in the sailing course."""

    name: str
    x: float
    y: float
    rounding_side: str


class CourseManager(Node):
    """Manage mark order, lap counting, and active-leg publication."""

    def __init__(self, verbose_default: bool = False) -> None:
        super().__init__('course_manager')

        self.declare_parameter('verbose', verbose_default)
        self.verbose_enabled = self.get_parameter('verbose').value
        self.declare_parameter('repeat_course', True)
        self.declare_parameter('lap_limit', 0)

        self.marks = self.load_course()
        self.current_mark = 0
        self.completed_laps = 0
        self.repeat_course = self.get_parameter('repeat_course').value
        self.lap_limit = self.get_parameter('lap_limit').value
        self.finished = False

        self.leg_pub = self.create_publisher(
            Float64MultiArray,
            '/course/active_leg',
            10
        )
        self.marks_pub = self.create_publisher(
            Float64MultiArray,
            '/course/marks',
            10
        )
        self.create_subscription(
            Bool,
            '/course/mark_rounded',
            self.mark_rounded_callback,
            10
        )
        self.timer = self.create_timer(0.2, self.publish_active_leg)

        if self.verbose_enabled:
            self.log_course()

    def load_course(self) -> list[CourseMark]:
        defaults = [
            ('red', -510.0, 180.0, 'starboard'),
            ('black', -490.0, 180.0, 'starboard'),
            ('white', -490.0, 200.0, 'starboard'),
            ('green', -510.0, 200.0, 'starboard'),
        ]

        marks = []
        for index, (name, x, y, side) in enumerate(defaults):
            self.declare_parameter(f'mark_{index}_name', name)
            self.declare_parameter(f'mark_{index}_x', x)
            self.declare_parameter(f'mark_{index}_y', y)
            self.declare_parameter(f'mark_{index}_rounding_side', side)
            marks.append(
                CourseMark(
                    self.get_parameter(f'mark_{index}_name').value,
                    self.get_parameter(f'mark_{index}_x').value,
                    self.get_parameter(f'mark_{index}_y').value,
                    self.get_parameter(f'mark_{index}_rounding_side').value,
                )
            )
        return marks

    def rounding_side_code(self, side: str) -> float:
        return 1.0 if side == 'port' else -1.0

    def mark_rounded_callback(self, msg: Bool) -> None:
        if not msg.data or self.finished:
            return

        previous_mark = self.current_mark
        self.current_mark = (self.current_mark + 1) % len(self.marks)

        if self.current_mark == 0:
            self.completed_laps += 1
            if (
                not self.repeat_course or
                (self.lap_limit > 0 and self.completed_laps >= self.lap_limit)
            ):
                self.finished = True

        self.get_logger().info(
            'Rounded mark %d, next mark %d, laps=%d' %
            (previous_mark, self.current_mark, self.completed_laps)
        )

    def publish_active_leg(self) -> None:
        current = self.marks[self.current_mark]
        next_index = (self.current_mark + 1) % len(self.marks)
        next_mark = self.marks[next_index]

        msg = Float64MultiArray()
        msg.data = [
            float(self.current_mark),
            current.x,
            current.y,
            self.rounding_side_code(current.rounding_side),
            float(next_index),
            next_mark.x,
            next_mark.y,
            1.0 if self.finished else 0.0,
        ]
        self.leg_pub.publish(msg)
        self.publish_course_marks()

    def publish_course_marks(self) -> None:
        msg = Float64MultiArray()
        msg.data = [
            float(len(self.marks)),
            float(self.current_mark),
            float(self.completed_laps),
            1.0 if self.finished else 0.0,
        ]
        for mark in self.marks:
            msg.data.extend([
                mark.x,
                mark.y,
                self.rounding_side_code(mark.rounding_side),
            ])
        self.marks_pub.publish(msg)

    def log_course(self) -> None:
        for index, mark in enumerate(self.marks):
            self.get_logger().info(
                'Course mark %d: %s at (%.2f, %.2f), rounding=%s' %
                (index, mark.name, mark.x, mark.y, mark.rounding_side)
            )
        self.get_logger().info(
            'repeat_course=%s lap_limit=%d' %
            (self.repeat_course, self.lap_limit)
        )


def main(args=None) -> None:
    raw_args = sys.argv[1:] if args is None else args
    verbose_enabled, clean_args = strip_verbose_flags(raw_args)
    rclpy.init(args=clean_args)
    node = CourseManager(verbose_default=verbose_enabled)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
