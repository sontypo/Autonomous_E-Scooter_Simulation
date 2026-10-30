#!/usr/bin/env python3
"""Run the e-scooter balancing dynamics model on its own.

The simulator bringup files include this launch file, but it can also be used
against a already-running simulation or against the real vehicle:

    ros2 launch escooter_dynamics escooter_dynamics.launch.py use_sim_time:=true
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare('escooter_dynamics'), 'params', 'escooter.yaml'])

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML file holding the escooter_dynamics parameters.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Follow /clock instead of the wall clock.'),
        DeclareLaunchArgument(
            'node_name',
            default_value='escooter_dynamics',
            description='Name given to the dynamics node.'),
    ]

    dynamics_node = Node(
        package='escooter_dynamics',
        executable='escooter_dynamics_node',
        name=LaunchConfiguration('node_name'),
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription(args + [dynamics_node])


# Keeps linters that import this module happy.
_ = get_package_share_directory
