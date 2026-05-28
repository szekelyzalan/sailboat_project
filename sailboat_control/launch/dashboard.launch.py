"""Launch the local sailboat dashboard node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='sailboat_control',
            executable='sailboat_dashboard',
            name='sailboat_dashboard',
            output='screen',
        ),
    ])
