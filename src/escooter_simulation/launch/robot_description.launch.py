#!/usr/bin/env python3
"""Publish the e-scooter robot_description and its TF tree.

Shared by every bringup file in this package. Keeping it separate means the
URDF is processed in exactly one place, so Gazebo Classic, Ignition and RViz
always see the same kinematics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare


# Actuator plants the description can be generated against.
#
# "classic" is the plant the port shipped with and the one escooter_dynamics
# was tuned against; it balances to +-0.3 deg and rights itself from -87 deg.
#
# "isaac" reproduces, term for term, the Isaac Lab ImplicitActuatorCfg the RL
# policy was trained on (src/escooter_rl/escooter_rl/assets/escooter.py). An
# implicit actuator computes
#
#     tau = clamp(stiffness*(q_d - q) + damping*(w_d - w), +-effort_limit)
#
# with w_d = 0 for a position or effort action, so its damping is a plain
# -damping*w drag on the joint. gazebo_ros2_control instead calls
# control_toolbox::Pid::computeCommand(error, dt), which differentiates the
# ERROR: a pos_kd there fires a full-scale torque spike on every new setpoint,
# 50 times a second. So the damping is carried as URDF joint damping, which
# Gazebo applies at the 1 kHz physics rate, and the PID keeps only its P term.
#
#   joint        Isaac actuator                      this profile
#   -----------  ----------------------------------  ---------------------------
#   steering     k 300, d 25, effort 40              pos_kp 300, damping 25,
#                                                    effort limit 40
#   plate        effort action, d 0.2, |w| <= 200    pass-through effort,
#                                                    damping 0.2, velocity 200
#   front wheel  velocity action, d 5, effort 80     vel_kp 5, effort 80
#   back wheel   effort limit 0, so no drive torque  free, as it already was
#
# The integral terms are dropped on purpose: Isaac has none, so under a policy
# that re-commands at 50 Hz they are pure phase lag.
ACTUATOR_PROFILES = {
    'classic': {
        'plate_effort_limit': '500',
        'plate_velocity_limit': '400',
        'plate_damping': '0.0',
        'handlebar_effort_limit': '200',
        'handlebar_damping': '0.0',
        'steer_kp': '400.0',
        'steer_ki': '20.0',
        'steer_kd': '40.0',
        'front_wheel_effort_limit': '-1',
        'front_wheel_velocity_limit': '-1',
        'wheel_kp': '10.0',
        'wheel_ki': '2.0',
        'wheel_kd': '0.1',
    },
    'isaac': {
        'plate_effort_limit': '500',
        'plate_velocity_limit': '200',
        'plate_damping': '0.2',
        'handlebar_effort_limit': '40',
        'handlebar_damping': '25.0',
        'steer_kp': '300.0',
        'steer_ki': '0.0',
        'steer_kd': '0.0',
        'front_wheel_effort_limit': '80',
        'front_wheel_velocity_limit': '60',
        'wheel_kp': '5.0',
        'wheel_ki': '0.0',
        'wheel_kd': '0.0',
    },
}

# Profile entries a caller may still override one at a time. Their launch
# argument defaults to the empty string, which means "take the profile value".
OVERRIDABLE = ('plate_effort_limit', 'handlebar_effort_limit', 'plate_velocity_limit')


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
        'actuator_profile', default_value='classic',
        choices=sorted(ACTUATOR_PROFILES),
        description='Which actuator plant to generate the description against: '
                    '"classic" is the tuning escooter_dynamics was built for, '
                    '"isaac" matches the Isaac Lab actuators the RL policy was '
                    'trained on. See ACTUATOR_PROFILES in this file.'),
    DeclareLaunchArgument(
        'plate_effort_limit', default_value='',
        description='Effort limit of rotational_plate_joint, in N.m. '
                    'Empty means take it from actuator_profile.'),
    DeclareLaunchArgument(
        'handlebar_effort_limit', default_value='',
        description='Effort limit of handle_bar_chasis_joint, in N.m. '
                    'Empty means take it from actuator_profile.'),
    DeclareLaunchArgument(
        'plate_velocity_limit', default_value='',
        description='Speed limit of rotational_plate_joint, in rad/s. This is '
                    'what bounds the reaction plate angular momentum, and so '
                    'how much lean the balancing controller can recover. '
                    'Empty means take it from actuator_profile.'),
    DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Follow /clock instead of the wall clock.'),
]


def robot_description_command(actuators):
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
    ] + [
        ' %s:=%s' % (name, value) for name, value in sorted(actuators.items())
    ])


def robot_state_publisher_node(actuators):
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
                robot_description_command(actuators), value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 50.0,
        }],
        # `joint_states` now resolves to <namespace>/joint_states on its own,
        # which is where joint_state_broadcaster publishes.
    )


def resolve_actuators(context, *_args, **_kwargs):
    """Pick the actuator profile, then let explicit arguments override it.

    Done in an OpaqueFunction because the values have to be read as plain
    strings to index ACTUATOR_PROFILES; a LaunchConfiguration cannot be
    compared until the context exists.
    """
    profile = LaunchConfiguration('actuator_profile').perform(context)
    if profile not in ACTUATOR_PROFILES:
        raise RuntimeError(
            'unknown actuator_profile %r; expected one of %s'
            % (profile, ', '.join(sorted(ACTUATOR_PROFILES))))

    actuators = dict(ACTUATOR_PROFILES[profile])
    for name in OVERRIDABLE:
        override = LaunchConfiguration(name).perform(context).strip()
        if override:
            actuators[name] = override

    return [robot_state_publisher_node(actuators)]


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=resolve_actuators)])
