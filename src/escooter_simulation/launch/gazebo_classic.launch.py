#!/usr/bin/env python3
"""Bring the e-scooter up in Gazebo Classic 11.

ROS 2 replacement for the ROS 1 `escooter_dynamics/launch/e-scooter_dynamics_model.launch`
and `escooter_simulation/launch/e-scooter_gazebo.launch`.

    ros2 launch escooter_simulation gazebo_classic.launch.py
    ros2 launch escooter_simulation gazebo_classic.launch.py gui:=false dynamics:=false
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PKG = 'escooter_simulation'

# Order matters: the broadcaster must be running before the command controllers
# so /e_scooter/joint_states is available as soon as a joint can move.
CONTROLLERS = [
    'joint_state_broadcaster',
    'rotational_plate_joint_effort_controller',
    'handle_bar_chasis_joint_position_controller',
    'front_wheel_joint_velocity_controller',
]


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)
    default_world = os.path.join(pkg_share, 'worlds', 'escooter_empty.world')
    default_controllers = os.path.join(pkg_share, 'config', 'escooter_controllers.yaml')
    default_dynamics_params = os.path.join(
        get_package_share_directory('escooter_dynamics'), 'params', 'escooter.yaml')

    args = [
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Gazebo Classic world file.'),
        DeclareLaunchArgument('controllers_file', default_value=default_controllers,
                              description='ros2_control YAML for the controller manager.'),
        DeclareLaunchArgument('namespace', default_value='/e_scooter',
                              description='ROS namespace of the controller manager.'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Run gzclient alongside gzserver.'),
        DeclareLaunchArgument('paused_model', default_value='false',
                              description='Leave the world paused and the controllers '
                                          'loaded but inactive, for inspecting the model.'),
        DeclareLaunchArgument('verbose', default_value='false',
                              description='Verbose gzserver output.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('mesh_prefix', default_value='package://escooter_simulation',
                              description='URI prefix used to resolve the mesh files.'),
        DeclareLaunchArgument('plate_effort_limit', default_value='500',
                              description='Effort limit of rotational_plate_joint [N.m].'),
        DeclareLaunchArgument('plate_velocity_limit', default_value='200',
                              description='Speed limit of rotational_plate_joint [rad/s]; '
                                          'bounds the momentum the balancing loop can use.'),
        # Spawn pose, same defaults as the ROS 1 launch file.
        DeclareLaunchArgument('x_pos', default_value='0.0'),
        DeclareLaunchArgument('y_pos', default_value='0.0'),
        DeclareLaunchArgument('z_pos', default_value='0.197'),
        # TODO: keep this "Roll" value: 0.024223550000000002
        DeclareLaunchArgument('Roll', default_value='0.0243175499999999985417'),
        DeclareLaunchArgument('Yaw', default_value='0.0'),
        DeclareLaunchArgument('entity_name', default_value='e_scooter'),
        DeclareLaunchArgument('dynamics', default_value='true',
                              description='Also run the escooter_dynamics balancing node.'),
        DeclareLaunchArgument('dynamics_params_file', default_value=default_dynamics_params,
                              description='Parameter file for the escooter_dynamics node.'),
        DeclareLaunchArgument('spawn_timeout', default_value='120.0',
                              description='Seconds spawn_entity.py waits for gzserver. '
                                          'Gazebo Classic can need well over the stock 30 s '
                                          'to come up on a cold cache.'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open RViz with the e-scooter configuration.'),
    ]

    # Make the package meshes and worlds reachable from gzserver.
    gazebo_resources = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.pathsep.join(
            [os.path.dirname(pkg_share), os.environ.get('GAZEBO_MODEL_PATH', '')]).rstrip(
                os.pathsep))

    # Both includes are scoped: gzserver.launch.py declares its own
    # `params_file` argument with an empty default, and launch configurations
    # set inside an unscoped include leak into the shared context -- which would
    # blank out the parameter file of every Node launched afterwards.
    # scoped=True keeps the parent's configurations visible (forwarding stays on)
    # while confining anything the include sets.
    gzserver = GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'),
                             'launch', 'gzserver.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                # Always paused: start_simulation.py releases the world once
                # the controllers are loaded, so the balancing loop is live
                # from the very first physics step.
                'pause': 'true',
                'verbose': LaunchConfiguration('verbose'),
            }.items(),
        ),
    ])

    gzclient = GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'),
                             'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
    ])

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot_description.launch.py')),
        launch_arguments={
            'sim': 'classic',
            'namespace': LaunchConfiguration('namespace'),
            'controllers_file': LaunchConfiguration('controllers_file'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'mesh_prefix': LaunchConfiguration('mesh_prefix'),
            'plate_effort_limit': LaunchConfiguration('plate_effort_limit'),
            'plate_velocity_limit': LaunchConfiguration('plate_velocity_limit'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_urdf',
        output='screen',
        arguments=[
            '-topic', [LaunchConfiguration('namespace'), '/robot_description'],
            '-entity', LaunchConfiguration('entity_name'),
            '-x', LaunchConfiguration('x_pos'),
            '-y', LaunchConfiguration('y_pos'),
            '-z', LaunchConfiguration('z_pos'),
            '-R', LaunchConfiguration('Roll'),
            '-Y', LaunchConfiguration('Yaw'),
            '-timeout', LaunchConfiguration('spawn_timeout'),
        ],
    )

    # ROS 1 used `controller_manager/spawner` with ns:=/e_scooter; the ROS 2
    # spawner takes the fully qualified controller manager node instead.
    #
    # All four are loaded and configured in a single call while the world is
    # still paused. `--inactive` is what makes that possible: the controller
    # manager performs activation from its update loop, and that loop does not
    # run until physics does.
    load_controllers = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner', *CONTROLLERS,
            '--controller-manager',
            [LaunchConfiguration('namespace'), '/controller_manager'],
            '--controller-manager-timeout', '120',
            '--inactive',
        ],
        output='screen',
    )

    # Activates the controllers and releases the world in one step, so the
    # balancing loop is live within a control cycle of the first physics step.
    start_simulation = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'escooter_simulation', 'start_simulation.py',
            '--backend', 'classic',
            '--controller-manager',
            [LaunchConfiguration('namespace'), '/controller_manager'],
            *CONTROLLERS,
        ],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('paused_model')),
    )

    controller_chain = [
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_entity, on_exit=[load_controllers])),
        RegisterEventHandler(
            OnProcessExit(target_action=load_controllers, on_exit=[start_simulation])),
    ]

    dynamics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('escooter_dynamics'),
                                  'launch', 'escooter_dynamics.launch.py'])),
        condition=IfCondition(LaunchConfiguration('dynamics')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('dynamics_params_file'),
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare(PKG), 'rviz', 'escooter.rviz'])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription(
        args + [
            gazebo_resources,
            gzserver,
            gzclient,
            robot_state_publisher,
            spawn_entity,
        ] + controller_chain + [
            dynamics,
            rviz,
        ])
