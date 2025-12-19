#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    description_pkg = FindPackageShare("arm_description")
    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution([description_pkg, "urdf", "arm.urdf.xacro"]),
        ]
    )

    use_gui_arg = DeclareLaunchArgument(
        "use_joint_state_gui", default_value="false", description="Launch joint_state_publisher_gui"
    )

    return LaunchDescription(
        [
            use_gui_arg,
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                parameters=[{"use_gui": LaunchConfiguration("use_joint_state_gui")}],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description_content}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", PathJoinSubstitution([description_pkg, "launch", "view.rviz"])],
            ),
        ]
    )
