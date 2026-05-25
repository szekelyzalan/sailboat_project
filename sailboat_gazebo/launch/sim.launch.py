from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
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
            '-v', '4',
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
        ],
        output='screen'
    )

    return LaunchDescription([
        set_resource_path,
        gazebo,
        bridge
    ])
