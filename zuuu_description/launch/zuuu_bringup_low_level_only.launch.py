import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, event_handlers
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rplidar_launch_dir = os.path.join(get_package_share_directory("rplidar_ros2"), "launch")

    zuuu_hal_launch_dir = get_package_share_directory("zuuu_hal")

    # Launch arguments
    arguments = [
        DeclareLaunchArgument(name="use_sim_time", default_value="False", description="Flag to enable use_sim_time"),
    ]

    nodes = []

    # Launch files to call
    launches = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(zuuu_hal_launch_dir, "hal.launch.py")),
        ),
    ]

    return LaunchDescription(arguments + launches + nodes)
