#!/usr/bin/env python3
"""Bring the e-scooter up in Ignition Gazebo Fortress.

The ROS 2 Humble `ros_gz_*` and `ign_ros2_control` binaries on this machine link
against ignition-gazebo6 (Fortress), so this is the Ignition version the ROS
integration supports even though Harmonic is installed alongside it.

    ros2 launch escooter_simulation gazebo_ignition.launch.py
    ros2 launch escooter_simulation gazebo_ignition.launch.py gui:=false
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PKG = 'escooter_simulation'

CONTROLLERS = [
    'joint_state_broadcaster',
    'rotational_plate_joint_effort_controller',
    'handle_bar_chasis_joint_position_controller',
    'front_wheel_joint_velocity_controller',
]

# Only the clock and the IMU have to cross the ROS <-> Ignition boundary:
# ign_ros2_control drives the controller manager in-process.
BRIDGE_TOPICS = [
    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
]


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)
    default_world = os.path.join(pkg_share, 'worlds', 'escooter_empty.sdf')
    default_controllers = os.path.join(pkg_share, 'config', 'escooter_controllers.yaml')
    default_dynamics_params = os.path.join(
        get_package_share_directory('escooter_dynamics'), 'params', 'escooter.yaml')

    args = [
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Ignition world file.'),
        DeclareLaunchArgument('controllers_file', default_value=default_controllers,
                              description='ros2_control YAML for the controller manager.'),
        DeclareLaunchArgument('namespace', default_value='/e_scooter',
                              description='ROS namespace of the controller manager.'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Run the Ignition GUI; false runs ign gazebo -s.'),
        DeclareLaunchArgument('paused_model', default_value='false',
                              description='Leave the world paused and the controllers '
                                          'loaded but inactive, for inspecting the model.'),
        DeclareLaunchArgument('verbose', default_value='1',
                              description='Ignition verbosity level, 0-4.'),
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
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open RViz with the e-scooter configuration.'),
    ]

    # Ignition resolves `package://escooter_simulation/...` by stripping the
    # scheme and searching its resource path, so the share directory's parent
    # has to be on it. Both spellings are exported for Fortress and Harmonic.
    share_root = os.path.dirname(pkg_share)
    resource_path = [
        SetEnvironmentVariable(
            var,
            os.pathsep.join(
                [share_root, os.environ.get(var, '')]).rstrip(os.pathsep))
        for var in ('IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH')
    ]

    # `-r` is deliberately omitted: Ignition starts paused and
    # start_simulation.py releases it once the controllers are loaded, so the
    # balancing loop is live from the very first physics step.
    gz_args = PythonExpression([
        "'", LaunchConfiguration('world'), "'",
        " + ('' if '", LaunchConfiguration('gui'), "' == 'true' else ' -s --headless-rendering')",
        " + ' -v ' + '", LaunchConfiguration('verbose'), "'",
    ])

    # Scoped for the same reason as in gazebo_classic.launch.py: an unscoped
    # include leaks its own launch arguments into the shared context.
    ign_gazebo = GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'),
                             'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'gz_version': '6'}.items(),
        ),
    ])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ign_bridge',
        output='screen',
        arguments=BRIDGE_TOPICS,
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot_description.launch.py')),
        launch_arguments={
            'sim': 'ignition',
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
        package='ros_gz_sim',
        executable='create',
        name='spawn_urdf',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-world', 'default',
            '-topic', [LaunchConfiguration('namespace'), '/robot_description'],
            '-name', LaunchConfiguration('entity_name'),
            '-allow_renaming', 'true',
            '-x', LaunchConfiguration('x_pos'),
            '-y', LaunchConfiguration('y_pos'),
            '-z', LaunchConfiguration('z_pos'),
            '-R', LaunchConfiguration('Roll'),
            '-Y', LaunchConfiguration('Yaw'),
        ],
    )

    # Loaded and configured in one call while the world is still paused;
    # activation needs the controller manager's update loop, which only runs
    # once physics does.
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

    start_simulation = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'escooter_simulation', 'start_simulation.py',
            '--backend', 'ignition',
            '--world', 'default',
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
        args + resource_path + [
            ign_gazebo,
            bridge,
            robot_state_publisher,
            spawn_entity,
        ] + controller_chain + [
            dynamics,
            rviz,
        ])
