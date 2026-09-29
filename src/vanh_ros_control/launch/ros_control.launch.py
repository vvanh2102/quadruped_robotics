#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    ros_control_node = Node(
        package='vanh_ros_control',
        executable='ros_control_node',
        output='both',
        parameters=[
            {
                'simulation': True,
                # 'host_ip': '10.1.56.45'
            }
        ],
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    return LaunchDescription(
        [
            ros_control_node,
        ]
    )
