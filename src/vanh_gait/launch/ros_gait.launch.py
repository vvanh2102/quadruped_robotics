#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    ros_gait_node = Node(
        package='vanh_gait',
        executable='ros_gait_node',
        output='both',
        parameters=[],
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    return LaunchDescription(
        [
            ros_gait_node,
        ]
    )
