from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_path = get_package_share_directory("sailboat_description")

    worlds_path = os.path.join(pkg_path, "worlds")

    models_path = os.path.join(pkg_path, "models")

    world_file = os.path.join(worlds_path, "my_ocean.sdf")

    vrx_path = os.path.expanduser("~/sailboat_ws/src/vrx")

    resource_paths = ":".join([
        models_path,
        vrx_path
    ])

    return LaunchDescription([

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=resource_paths
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen'
        )
    ])