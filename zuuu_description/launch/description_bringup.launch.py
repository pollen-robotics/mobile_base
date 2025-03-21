import os

import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path finding (peak comedy)
    pkg_share = launch_ros.substitutions.FindPackageShare(package="zuuu_description").find("zuuu_description")
    default_model_path = os.path.join(pkg_share, "urdf/zuuu.urdf.xacro")

    # Paths to the robot description and its controllers
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("zuuu_description"), "urdf", "zuuu.urdf.xacro"]),
            " ",
            "use_gazebo:=false",
            " ",
            "use_fake_components:=true",
            " ",
            "use_fixed_wheels:=false",
            " ",
            "use_ros_control:=false",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    zuuu_controller = PathJoinSubstitution(
        [
            FindPackageShare("zuuu_description"),
            "config",
            "zuuu_controllers.yaml",
        ]
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    use_sim_time_param = {"use_sim_time": use_sim_time}

    # Launch arguments
    arguments = [
        DeclareLaunchArgument(
            "controllers_file",
            default_value=["zuuu_controllers.yaml"],
            description="YAML file with the controllers configuration.",
        ),
        DeclareLaunchArgument(
            name="gui",
            default_value="True",
            description="Flag to enable joint_state_publisher_gui",
        ),
        DeclareLaunchArgument(
            name="model",
            default_value=default_model_path,
            description="Absolute path to robot urdf file",
        ),
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="True",
            description="Flag to enable use_sim_time",
        ),
    ]

    # Nodes declaration
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, use_sim_time_param],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, zuuu_controller, use_sim_time_param],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    # Call this to have 'ros2 control list_controllers' give :
    # joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[use_sim_time_param],
        output="screen",
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[use_sim_time_param],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
    )

    # Nodes to call
    nodes = [
        joint_state_publisher_node,
        joint_state_broadcaster_spawner,
        robot_state_publisher_node,
        # controller_manager_node,
    ]
    # Launch files to call
    launches = []

    return LaunchDescription(arguments + launches + nodes)


""" 
Infamous bug where the TFs from map to anything other than odom was outdated (with the weird 0.2 timestamp) 
solved by calling libgazebo_ros_init.so and libgazebo_ros_factory.so

https://discourse.ros.org/t/spawning-a-robot-entity-using-a-node-with-gazebo-and-ros-2/9985
"""
