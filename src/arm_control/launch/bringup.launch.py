#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([FindPackageShare("arm_control"), "config", "arm.yaml"])
    use_teleop_arg = DeclareLaunchArgument("use_teleop", default_value="true")

    return LaunchDescription(
        [
            use_teleop_arg,
            Node(
                package="arm_control",
                executable="arm_driver",
                name="arm_driver",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="arm_control",
                executable="arm_keyboard_teleop",
                name="arm_keyboard_teleop",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_teleop")),
            ),
        ]
    )
