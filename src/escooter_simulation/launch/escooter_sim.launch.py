#!/usr/bin/env python3
"""Single entry point that selects the simulator backend.

    ros2 launch escooter_simulation escooter_sim.launch.py                   # Gazebo Classic
    ros2 launch escooter_simulation escooter_sim.launch.py backend:=ignition # Ignition Fortress

Every other argument is forwarded verbatim to the backend launch file, so e.g.
`gui:=false`, `dynamics:=false`, `rviz:=true`, `x_pos:=1.0` all work here too.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


PKG = 'escooter_simulation'

# Arguments both backends understand; forwarded as-is.
FORWARDED = [
    ('controllers_file', None),
    ('dynamics_params_file', 'DYNAMICS_PARAMS'),
    ('namespace', '/e_scooter'),
    ('gui', 'true'),
    ('paused_model', 'false'),
    ('use_sim_time', 'true'),
    ('imu_topic', '/imu'),
    ('mesh_prefix', 'package://escooter_simulation'),
    ('plate_effort_limit', '500'),
    ('handlebar_effort_limit', '200'),
    ('plate_velocity_limit', '400'),
    ('x_pos', '0.0'),
    ('y_pos', '0.0'),
    ('z_pos', '0.197'),
    ('Roll', '0.0243175499999999985417'),
    ('Yaw', '0.0'),
    ('entity_name', 'e_scooter'),
    ('dynamics', 'true'),
    ('rviz', 'false'),
]


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)
    default_controllers = os.path.join(pkg_share, 'config', 'escooter_controllers.yaml')

    args = [
        DeclareLaunchArgument(
            'backend', default_value='classic', choices=['classic', 'ignition'],
            description='Simulator to run: Gazebo Classic 11 or Ignition Fortress.'),
    ]
    default_dynamics_params = os.path.join(
        get_package_share_directory('escooter_dynamics'), 'params', 'escooter.yaml')

    forwarded = {}
    for name, default in FORWARDED:
        if default is None:
            default = default_controllers
        elif default == 'DYNAMICS_PARAMS':
            default = default_dynamics_params
        args.append(DeclareLaunchArgument(name, default_value=default))
        forwarded[name] = LaunchConfiguration(name)

    backends = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', '%s.launch.py' % launch_name)),
            condition=LaunchConfigurationEquals('backend', backend),
            launch_arguments=forwarded.items(),
        )
        for backend, launch_name in (('classic', 'gazebo_classic'),
                                     ('ignition', 'gazebo_ignition'))
    ]

    return LaunchDescription(args + backends)
