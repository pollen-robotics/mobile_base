import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("zuuu_hal"), "config", "params.yaml"
    )

    rplidar_launch_dir = os.path.join(
        get_package_share_directory("rplidar_ros2"), "launch"
    )

    # Launch arguments
    arguments = [
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="False",
            description="Flag to enable use_sim_time",
        ),
        DeclareLaunchArgument(
            name="fake_hardware",
            default_value="False",
            description="Flag to indicate simulation mode",
        ),
    ]

    # Retrieve the launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    fake_hardware = LaunchConfiguration("fake_hardware")

    # Conditionally include the LIDAR launch file only if not in simulation mode
    launches = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rplidar_launch_dir, "zuuu_rplidar_s2_launch.py")
            ),
            condition=UnlessCondition(fake_hardware),
        ),
    ]

    # Node configuration
    nodes = [
        Node(
            package="zuuu_hal",
            executable="hal",
            name="zuuu_hal",
            parameters=[
                config,
                {"fake_hardware": fake_hardware, "use_sim_time": use_sim_time},
            ],
        )
    ]

    return LaunchDescription(arguments + launches + nodes)