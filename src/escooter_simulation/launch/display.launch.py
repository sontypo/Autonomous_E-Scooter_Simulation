#!/usr/bin/env python3
"""Show the e-scooter in RViz without a simulator.

ROS 2 replacement for the ROS 1 `escooter_simulation/launch/display.launch`,
which pointed at a package that no longer exists. The joints are driven by
joint_state_publisher_gui so the kinematics can be checked by hand.

    ros2 launch escooter_simulation display.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PKG = 'escooter_simulation'


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)

    args = [
        DeclareLaunchArgument('namespace', default_value='/e_scooter',
                              description='Namespace of robot_state_publisher '
                                          'and joint_state_publisher.'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Use joint_state_publisher_gui instead of the '
                                          'headless joint_state_publisher.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(pkg_share, 'rviz', 'escooter.rviz'),
                              description='RViz configuration file.'),
    ]

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot_description.launch.py')),
        launch_arguments={
            'sim': 'none',
            'use_sim_time': 'false',
            'namespace': LaunchConfiguration('namespace'),
        }.items(),
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=LaunchConfiguration('namespace'),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription(args + [
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz,
    ])


_ = (PathJoinSubstitution, FindPackageShare)
