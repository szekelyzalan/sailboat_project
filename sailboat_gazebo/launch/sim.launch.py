from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_dir = get_package_share_directory('sailboat_gazebo')

    world = os.path.join(
        pkg_dir,
        'worlds',
        'minimal_ocean.sdf'
    )

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', world],
        output='screen'
    )

    return LaunchDescription([
        gz_sim
    ])