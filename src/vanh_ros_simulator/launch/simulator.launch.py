#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    # Robot description (URDF via xacro)
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
                PathJoinSubstitution([FindPackageShare('vanh_description'), 'config', 'vanh.urdf.xacro']),
            ]),
            value_type=str,
        )
    }

    config = os.path.join(get_package_share_directory('vanh_ros_simulator'), 'param', 'config.yaml')
    rviz_config = os.path.join(get_package_share_directory('vanh_description'), 'rviz', 'rviz.rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    simulator_node = Node(
        package='vanh_ros_simulator',
        executable='simulator_node',
        output='both',
        parameters=[config],
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            simulator_node,
            rviz,
        ]
    )
