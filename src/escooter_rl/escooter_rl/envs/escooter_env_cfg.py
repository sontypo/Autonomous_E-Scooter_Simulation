# Copyright (c) 2026, Autonomous E-Scooter Simulation Project.
# All rights reserved.

import math
from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from escooter_rl.assets import ESCOOTER_CFG
from escooter_rl.obs_spec import (
    REACTION_VEL_CLIP,
    REACTION_VEL_SCALE,
    WHEEL_VEL_CLIP,
    WHEEL_VEL_SCALE,
)
import escooter_rl.envs.mdp as mdp


##
# Scene Definition
##

@configclass
class EScooterSceneCfg(InteractiveSceneCfg):
    """Configuration for the e-scooter interactive scene."""

    # Ground plane
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="multiply",
                restitution_combine_mode="multiply",
                static_friction=1.0,
                dynamic_friction=0.8,
                restitution=0.0,
            )
        ),
    )

    # Dome light
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.85, 0.85, 0.85)),
    )

    # E-Scooter robot
    robot: ArticulationCfg = ESCOOTER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


##
# MDP Actions
##

@configclass
class EScooterActionsCfg:
    """Action specifications for the E-Scooter MDP.
    
    Total action dimension: 3
    - steering: target position offset for handlebar chassis joint (radians)
    - reaction_plate: direct effort/torque command for momentum wheel (N*m)
    - drive_wheel: velocity target for front drive wheel (rad/s)
    """

    steering = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["handle_bar_chasis_joint"],
        scale=0.7,
        use_default_offset=True,
    )
    reaction_plate = mdp.JointEffortActionCfg(
        asset_name="robot",
        joint_names=["rotational_plate_joint"],
        scale=60.0,
    )
    drive_wheel = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=["front_wheel_joint"],
        scale=20.0,
    )


##
# MDP Observations
##

@configclass
class EScooterGoalNavObservationsCfg:
    """Observations for Goal Navigation Policy."""

    @configclass
    class PolicyCfg(ObsGroup):
        # Base state
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.05, n_max=0.05))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.1, n_max=0.1))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.02, n_max=0.02))

        # Joint states. reaction_vel and joint_vel are clipped and scaled
        # from escooter_rl.obs_spec: raw, the plate runs to a few hundred
        # rad/s and the wheels to ~20 against order-1 inputs everywhere
        # else, and the policy has to extrapolate the moment either leaves
        # the range it saw in training. Noise is in rad/s, applied before
        # the scale, so the plate's was widened to stay a real perturbation.
        steering_pos = ObsTerm(func=mdp.steering_angle, noise=Unoise(n_min=-0.01, n_max=0.01))
        reaction_vel = ObsTerm(
            func=mdp.reaction_plate_velocity,
            noise=Unoise(n_min=-2.0, n_max=2.0),
            clip=REACTION_VEL_CLIP,
            scale=REACTION_VEL_SCALE,
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["front_wheel_joint", "back_wheel_joint"])},
            noise=Unoise(n_min=-0.5, n_max=0.5),
            clip=WHEEL_VEL_CLIP,
            scale=WHEEL_VEL_SCALE,
        )

        # Goal command (relative goal pose in robot base frame: [x, y, heading_error])
        goal_command = ObsTerm(func=mdp.generated_commands, params={"command_name": "goal_pose"})

        # Action history
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EScooterVelocityObservationsCfg:
    """Observations for Velocity Tracking Policy."""

    @configclass
    class PolicyCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.05, n_max=0.05))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.1, n_max=0.1))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.02, n_max=0.02))

        steering_pos = ObsTerm(func=mdp.steering_angle, noise=Unoise(n_min=-0.01, n_max=0.01))
        # Clipped and scaled from escooter_rl.obs_spec; see the goal_nav
        # group above for why.
        reaction_vel = ObsTerm(
            func=mdp.reaction_plate_velocity,
            noise=Unoise(n_min=-2.0, n_max=2.0),
            clip=REACTION_VEL_CLIP,
            scale=REACTION_VEL_SCALE,
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["front_wheel_joint", "back_wheel_joint"])},
            noise=Unoise(n_min=-0.5, n_max=0.5),
            clip=WHEEL_VEL_CLIP,
            scale=WHEEL_VEL_SCALE,
        )

        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


##
# MDP Commands
##

@configclass
class EScooterGoalCommandsCfg:
    """Command generator for 2D Goal Navigation."""

    goal_pose = mdp.UniformPose2dCommandCfg(
        asset_name="robot",
        simple_heading=True,
        resampling_time_range=(15.0, 20.0),
        debug_vis=True,
        ranges=mdp.UniformPose2dCommandCfg.Ranges(
            pos_x=(-6.0, 6.0),
            pos_y=(-6.0, 6.0),
            heading=(-math.pi, math.pi),
        ),
    )


@configclass
class EScooterVelocityCommandsCfg:
    """Command generator for Velocity Tracking."""

    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(8.0, 12.0),
        rel_standing_envs=0.05,
        rel_heading_envs=0.0,
        heading_command=False,
        debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 2.5),
            lin_vel_y=(0.0, 0.0),
            ang_vel_z=(-1.0, 1.0),
            heading=(-math.pi, math.pi),
        ),
    )


##
# MDP Events (Domain Randomization)
##

@configclass
class EScooterEventCfg:
    """Domain Randomization and Reset Events."""

    # -- Physics Materials & Friction
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*wheel"),
            "static_friction_range": (0.6, 1.2),
            "dynamic_friction_range": (0.5, 1.0),
            "restitution_range": (0.0, 0.05),
            "num_buckets": 64,
        },
    )

    # -- Mass Randomization (Base chassis payload: from empty 0 kg up to human rider ~70 kg)
    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "mass_distribution_params": (0.0, 70.0),
            "operation": "add",
        },
    )

    # -- Mass Randomization (Handlebar column: arms resting weight 0 - 6 kg)
    add_handlebar_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="handle_bar_chasis"),
            "mass_distribution_params": (0.0, 6.0),
            "operation": "add",
        },
    )

    # -- Center of Mass (CoM) Shift (Rider Standing Height & Stance Variations)
    base_com = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "com_range": {
                "x": (-0.08, 0.08),    # Stance forward/backward shift
                "y": (-0.035, 0.035),  # Stance lateral tilt offset
                "z": (0.0, 0.45),      # Standing human raises CoM significantly
            },
        },
    )

    # -- Joint Friction Randomization
    joint_friction = EventTerm(
        func=mdp.randomize_joint_parameters,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "friction_distribution_params": (0.8, 1.3),
            "operation": "scale",
        },
    )

    # -- Actuator Damping & Stiffness Randomization (Arm biomechanical damping)
    actuator_gains = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "stiffness_distribution_params": (0.8, 1.3),
            "damping_distribution_params": (0.8, 2.0),
            "operation": "scale",
        },
    )

    # -- Periodic External Lateral Push (Disturbance Recovery)
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(8.0, 12.0),
        params={"velocity_range": {"x": (-0.2, 0.2), "y": (-0.35, 0.35)}},
    )

    # -- Episode Resets
    reset_root = EventTerm(
        func=mdp.reset_escooter_root_state,
        mode="reset",
        params={
            "pose_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "yaw": (-math.pi, math.pi),
                "roll": (-0.05, 0.05),
                "pitch": (-0.02, 0.02),
            },
            "velocity_range": {
                "x": (0.0, 0.5),
                "y": (-0.05, 0.05),
                "z": (0.0, 0.0),
                "roll": (-0.1, 0.1),
                "pitch": (-0.05, 0.05),
                "yaw": (-0.1, 0.1),
            },
        },
    )

    # -- Flywheel momentum, at reset and again mid-episode
    #
    # The plate used to start within +-5 rad/s and stay there, so a few hundred
    # rad/s was a state the policy had never seen. It answered one by spinning
    # the plate up further rather than bleeding it down, which turned any small
    # persistent bias into a guaranteed runaway to the speed limit. A reward
    # cannot teach a reflex for a state that never occurs, so the plate now
    # starts anywhere in its working range and gets thrown again during the
    # episode - the flywheel's equivalent of push_robot.
    spin_plate = EventTerm(
        func=mdp.spin_reaction_plate,
        mode="interval",
        interval_range_s=(4.0, 8.0),
        params={"speed_range": (-180.0, 180.0)},
    )

    reset_joints = EventTerm(
        func=mdp.reset_escooter_joints,
        mode="reset",
        params={
            "steer_range": (-0.05, 0.05),
            "plate_vel_range": (-150.0, 150.0),
        },
    )


##
# MDP Rewards
##

@configclass
class EScooterGoalNavRewardsCfg:
    """Reward terms for E-Scooter Goal Navigation & Self-Balancing."""

    # -- Self-Balancing Primary Rewards
    upright = RewTerm(func=mdp.upright_posture_exp, weight=3.0, params={"std": 0.14})
    roll_rate_penalty = RewTerm(func=mdp.roll_velocity_penalty, weight=-0.08)
    lateral_slip_penalty = RewTerm(func=mdp.lateral_velocity_penalty, weight=-0.1)

    # -- Goal Navigation Rewards
    goal_distance = RewTerm(func=mdp.goal_distance_exp, weight=2.0, params={"command_name": "goal_pose", "std": 1.5})
    goal_reached = RewTerm(func=mdp.goal_reached_bonus, weight=5.0, params={"command_name": "goal_pose", "threshold": 0.4})

    # -- Energy & Actuator Longevity Regularization
    # The -0.0001 quadratic that used to sit here is gone: it was worth 1.6% of
    # the upright reward at 22 rad/s and only bit above 100, so it never shaped
    # anything the clamped-linear term below does not now cover.
    #
    # -0.5 puts the plate at 17% of the upright reward by 60 rad/s and 50% at
    # the 180 cap, with the same gradient throughout, so there is always a
    # reason to bleed momentum out. Watch Episode_Reward/reaction_plate_centering
    # while training: it should settle near zero, and sitting near -1.5 means
    # the plate is parked at its limit and the weight needs raising.
    #
    # action_saturation stays. It is the only thing bounding the raw policy
    # output, and scripts/check_policy_range.py gates on exactly that.
    reaction_plate_centering = RewTerm(func=mdp.reaction_plate_centering, weight=-0.5,
                                       params={"ref_speed": 60.0, "cap": 3.0})
    action_saturation = RewTerm(func=mdp.action_saturation_penalty, weight=-0.5,
                                params={"threshold": 1.0})
    steering_limits = RewTerm(func=mdp.steering_angle_limit_penalty, weight=-0.15, params={"limit_threshold": 0.85})
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.01)


@configclass
class EScooterVelocityRewardsCfg:
    """Reward terms for E-Scooter Velocity Tracking & Self-Balancing."""

    # -- Self-Balancing Primary Rewards
    upright = RewTerm(func=mdp.upright_posture_exp, weight=3.0, params={"std": 0.14})
    roll_rate_penalty = RewTerm(func=mdp.roll_velocity_penalty, weight=-0.08)
    lateral_slip_penalty = RewTerm(func=mdp.lateral_velocity_penalty, weight=-0.1)

    # -- Velocity Tracking Rewards
    track_lin_vel = RewTerm(func=mdp.track_lin_vel_x_exp, weight=2.0, params={"command_name": "base_velocity", "std": 0.25})
    track_ang_vel = RewTerm(func=mdp.track_ang_vel_z_exp, weight=1.0, params={"command_name": "base_velocity", "std": 0.25})

    # -- Regularization. See EScooterGoalNavRewardsCfg for the reasoning.
    reaction_plate_centering = RewTerm(func=mdp.reaction_plate_centering, weight=-0.5,
                                       params={"ref_speed": 60.0, "cap": 3.0})
    action_saturation = RewTerm(func=mdp.action_saturation_penalty, weight=-0.5,
                                params={"threshold": 1.0})
    steering_limits = RewTerm(func=mdp.steering_angle_limit_penalty, weight=-0.15, params={"limit_threshold": 0.85})
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.01)


##
# MDP Terminations
##

@configclass
class EScooterTerminationsCfg:
    """Termination conditions for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    fall_orientation = DoneTerm(
        func=mdp.bad_orientation,
        params={"max_roll_rad": 0.61, "max_pitch_rad": 0.61},
    )
    chassis_crash = DoneTerm(
        func=mdp.base_height_too_low,
        params={"min_height": 0.18},
    )


##
# Environment Configurations
##

@configclass
class EScooterGoalNavEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for E-Scooter 2D Goal Navigation Environment."""

    # Scene
    scene: EScooterSceneCfg = EScooterSceneCfg(num_envs=2048, env_spacing=4.0)

    # MDP
    actions: EScooterActionsCfg = EScooterActionsCfg()
    observations: EScooterGoalNavObservationsCfg = EScooterGoalNavObservationsCfg()
    commands: EScooterGoalCommandsCfg = EScooterGoalCommandsCfg()
    events: EScooterEventCfg = EScooterEventCfg()
    rewards: EScooterGoalNavRewardsCfg = EScooterGoalNavRewardsCfg()
    terminations: EScooterTerminationsCfg = EScooterTerminationsCfg()

    def __post_init__(self):
        # Simulation settings (200 Hz physics, 50 Hz control decimation)
        self.decimation = 4
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.episode_length_s = 20.0


@configclass
class EScooterGoalNavEnvCfg_PLAY(EScooterGoalNavEnvCfg):
    """Configuration for interactive evaluation of Goal Navigation."""

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 32
        self.scene.env_spacing = 3.5
        self.observations.policy.enable_corruption = False
        # Remove random disturbances during visual play
        self.events.push_robot = None
        self.events.spin_plate = None


@configclass
class EScooterVelocityEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for E-Scooter Velocity Tracking Environment."""

    # Scene
    scene: EScooterSceneCfg = EScooterSceneCfg(num_envs=2048, env_spacing=4.0)

    # MDP
    actions: EScooterActionsCfg = EScooterActionsCfg()
    observations: EScooterVelocityObservationsCfg = EScooterVelocityObservationsCfg()
    commands: EScooterVelocityCommandsCfg = EScooterVelocityCommandsCfg()
    events: EScooterEventCfg = EScooterEventCfg()
    rewards: EScooterVelocityRewardsCfg = EScooterVelocityRewardsCfg()
    terminations: EScooterTerminationsCfg = EScooterTerminationsCfg()

    def __post_init__(self):
        self.decimation = 4
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.episode_length_s = 20.0


@configclass
class EScooterVelocityEnvCfg_PLAY(EScooterVelocityEnvCfg):
    """Configuration for interactive evaluation of Velocity Tracking."""

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 32
        self.scene.env_spacing = 3.5
        self.observations.policy.enable_corruption = False
        self.events.push_robot = None
        self.events.spin_plate = None
