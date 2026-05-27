from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_buoy_detector = LaunchConfiguration('use_buoy_detector')
    use_dashboard = LaunchConfiguration('use_dashboard')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_buoy_detector',
            default_value='true',
            description='Start the LiDAR buoy detector.'
        ),
        DeclareLaunchArgument(
            'use_dashboard',
            default_value='true',
            description='Start the local browser dashboard.'
        ),

        Node(
            package='sailboat_control',
            executable='course_manager',
            name='course_manager',
            output='screen',
        ),
        Node(
            package='sailboat_control',
            executable='apparent_wind_sensor',
            name='apparent_wind_sensor',
            output='screen',
        ),
        Node(
            package='sailboat_control',
            executable='buoy_detector',
            name='buoy_detector',
            condition=IfCondition(use_buoy_detector),
            output='screen',
        ),
        Node(
            package='sailboat_control',
            executable='sailboat_autonomy',
            name='sailboat_autonomy',
            output='screen',
        ),
        Node(
            package='sailboat_control',
            executable='sailboat_dashboard',
            name='sailboat_dashboard',
            condition=IfCondition(use_dashboard),
            output='screen',
        ),
    ])
