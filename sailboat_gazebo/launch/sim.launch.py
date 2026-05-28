from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    debug = LaunchConfiguration('debug')
    verbose = LaunchConfiguration('verbose')
    gz_verbosity = LaunchConfiguration('gz_verbosity')
    sailboat_gazebo_dir = get_package_share_directory('sailboat_gazebo')
    vrx_gz_dir = get_package_share_directory('vrx_gz')
    world = os.path.join(
        sailboat_gazebo_dir,
        'worlds',
        'minimal_ocean.sdf'
    )

    resource_paths = [
        # Own sailboat models
        os.path.join(sailboat_gazebo_dir, 'models'),

        # VRX maritime assets
        # (coast_waves, buoys, etc.)
        os.path.join(vrx_gz_dir, 'models'),

        # Default Gazebo / ROS assets
        '/opt/ros/jazzy/share',
    ]

    combined_resources = ':'.join(resource_paths)

    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=combined_resources
    )

    # GAZEBO SIMULATION
    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '-v', gz_verbosity,
            world
        ],
        output='screen'
    )

    # ROS <-> Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/baum_pos@std_msgs/msg/Float64@gz.msgs.Double',
            '/actual_baum_pos@std_msgs/msg/Float64@gz.msgs.Double',
            '/rudder_pos@std_msgs/msg/Float64@gz.msgs.Double',
            '/boat/gps/data@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/vrx/debug/wind/speed@std_msgs/msg/Float32[gz.msgs.Float',
            '/vrx/debug/wind/direction@std_msgs/msg/Float32[gz.msgs.Float',
            '/boat/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/boat/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
    )

    game_physics = Node(
        package='sailboat_physics',
        executable='game_physics_node',
        name='sailboat_game_physics',
        parameters=[{
            'debug': ParameterValue(debug, value_type=bool),
            'verbose': ParameterValue(verbose, value_type=bool),
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable per-tick debug output from sailboat nodes.'
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Enable startup/status logs from sailboat nodes.'
        ),
        DeclareLaunchArgument(
            'gz_verbosity',
            default_value='1',
            description='Gazebo console verbosity level.'
        ),
        set_resource_path,
        gazebo,
        bridge,
        game_physics
    ])
