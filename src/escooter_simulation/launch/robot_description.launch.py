#!/usr/bin/env python3
"""Publish the e-scooter robot_description and its TF tree.

Shared by every bringup file in this package. Keeping it separate means the
URDF is processed in exactly one place, so Gazebo Classic, Ignition and RViz
always see the same kinematics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare


ARGUMENTS = [
    DeclareLaunchArgument(
        'sim', default_value='classic',
        choices=['classic', 'ignition', 'none'],
        description='Which simulator the description is generated for.'),
    DeclareLaunchArgument(
        'namespace', default_value='/e_scooter',
        description='ROS namespace of robot_state_publisher, the controller '
                    'manager and the controllers.'),
    DeclareLaunchArgument(
        'controllers_file', default_value='',
        description='Absolute path to the ros2_control YAML handed to the simulator plugin.'),
    DeclareLaunchArgument(
        'imu_topic', default_value='/imu',
        description='Topic the IMU is published on.'),
    DeclareLaunchArgument(
        'mesh_prefix', default_value='package://escooter_simulation',
        description='URI prefix used to resolve the mesh files.'),
    DeclareLaunchArgument(
        'plate_effort_limit', default_value='500',
        description='Effort limit of rotational_plate_joint, in N.m.'),
    DeclareLaunchArgument(
        'handlebar_effort_limit', default_value='200',
        description='Effort limit of handle_bar_chasis_joint, in N.m.'),
    DeclareLaunchArgument(
        'plate_velocity_limit', default_value='400',
        description='Speed limit of rotational_plate_joint, in rad/s. This is '
                    'what bounds the reaction plate angular momentum, and so '
                    'how much lean the balancing controller can recover.'),
    DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Follow /clock instead of the wall clock.'),
]


def robot_description_command():
    """Build the command that turns the xacro into a URDF.

    `xacro_to_urdf.py` is a thin wrapper that emits the URDF on a single line
    with the comments stripped. Plain `xacro` output breaks the ros2_control
    system plugins on Humble, which pass the description to the controller
    manager as a `--param robot_description:=...` YAML rule; see the script's
    docstring for the details.
    """
    return Command([
        ExecutableInPackage(package='escooter_simulation',
                            executable='xacro_to_urdf.py'), ' ',
        PathJoinSubstitution(
            [FindPackageShare('escooter_simulation'), 'urdf', 'e-scooter.urdf.xacro']),
        ' sim:=', LaunchConfiguration('sim'),
        ' namespace:=', LaunchConfiguration('namespace'),
        ' controllers_file:=', LaunchConfiguration('controllers_file'),
        ' imu_topic:=', LaunchConfiguration('imu_topic'),
        ' mesh_prefix:=', LaunchConfiguration('mesh_prefix'),
        ' plate_effort_limit:=', LaunchConfiguration('plate_effort_limit'),
        ' handlebar_effort_limit:=', LaunchConfiguration('handlebar_effort_limit'),
        ' plate_velocity_limit:=', LaunchConfiguration('plate_velocity_limit'),
    ])


def robot_state_publisher_node():
    """robot_state_publisher, running inside the robot namespace.

    Both ros2_control system plugins look the robot description up on a node
    named by `robot_param_node`, resolved relative to their own namespace. The
    Ignition plugin additionally refuses an absolute name: it concatenates its
    namespace with the value and throws `InvalidServiceNameError` on the
    resulting `//`. Keeping robot_state_publisher inside the same namespace
    makes the plain, relative default work for Classic and Ignition alike.

    TF is unaffected: tf2_ros' broadcasters publish on the absolute topics /tf
    and /tf_static regardless of the node's namespace.
    """
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                robot_description_command(), value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 50.0,
        }],
        # `joint_states` now resolves to <namespace>/joint_states on its own,
        # which is where joint_state_broadcaster publishes.
    )


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [robot_state_publisher_node()])
