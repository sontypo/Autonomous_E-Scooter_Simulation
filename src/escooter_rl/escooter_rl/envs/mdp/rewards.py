# Copyright (c) 2026, Autonomous E-Scooter Simulation Project.
# All rights reserved.

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def upright_posture_exp(
    env: ManagerBasedRLEnv,
    std: float = 0.15,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reward for keeping the e-scooter upright (roll and pitch near zero).
    
    Uses projected gravity vector: when upright, gz = -1.0, gx = gy = 0.0.
    """
    robot: Articulation = env.scene[asset_cfg.name]
    gravity_proj = robot.data.projected_gravity_b
    # gx = sin(pitch), gy = -sin(roll)*cos(pitch)
    error_sq = gravity_proj[:, 0] ** 2 + gravity_proj[:, 1] ** 2
    return torch.exp(-error_sq / (std ** 2))


def roll_velocity_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize high roll angular velocity to dampen lateral wobbling."""
    robot: Articulation = env.scene[asset_cfg.name]
    roll_rate = robot.data.root_ang_vel_b[:, 0]
    return torch.square(roll_rate)


def lateral_velocity_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize lateral linear velocity (slip/skid) since the e-scooter wheels roll forward."""
    robot: Articulation = env.scene[asset_cfg.name]
    vy = robot.data.root_lin_vel_b[:, 1]
    return torch.square(vy)


def track_lin_vel_x_exp(
    env: ManagerBasedRLEnv,
    command_name: str = "base_velocity",
    std: float = 0.25,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reward forward linear velocity tracking."""
    robot: Articulation = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    vel_error = torch.square(robot.data.root_lin_vel_b[:, 0] - command[:, 0])
    return torch.exp(-vel_error / (std ** 2))


def track_ang_vel_z_exp(
    env: ManagerBasedRLEnv,
    command_name: str = "base_velocity",
    std: float = 0.25,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reward yaw rate tracking (turning rate)."""
    robot: Articulation = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    vel_error = torch.square(robot.data.root_ang_vel_b[:, 2] - command[:, 2])
    return torch.exp(-vel_error / (std ** 2))


def reaction_plate_centering(
    env: ManagerBasedRLEnv,
    ref_speed: float = 60.0,
    cap: float = 3.0,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Drive the flywheel back towards zero from anywhere in its range.

    Clamped-linear, not the squared-exponential this started as. That shape,
    1 - exp(-(w/20)^2), is flat above about 50 rad/s: once the plate got there
    it told the policy nothing about coming back, which is exactly the regime
    the restoring reflex has to be learned in. The deployed policy ended up
    with the opposite reflex, pushing harder the faster the plate turned
    (-2.3 N.m at 0 rad/s, +40.8 at 200), so any persistent bias ran it to its
    limit and left the scooter with no balancing authority at all.

    A linear term has the same gradient everywhere, so there is always a reason
    to bleed momentum out whatever the speed; `cap` keeps it from growing large
    enough to outweigh staying upright.

    Learning this also needs the plate to actually visit those speeds, which is
    what `reset_escooter_joints` and `spin_reaction_plate` are for.
    """
    robot: Articulation = env.scene[asset_cfg.name]
    joint_ids, _ = robot.find_joints("rotational_plate_joint")
    plate_vel = robot.data.joint_vel[:, joint_ids[0]]
    return torch.clamp(torch.abs(plate_vel) / ref_speed, max=cap)


def action_saturation_penalty(
    env: ManagerBasedRLEnv,
    threshold: float = 1.0,
) -> torch.Tensor:
    """Penalise raw actions outside the range the action scales assume.

    Nothing else does. `ActionTermCfg.clip` bounds `_processed_actions`, not
    `_raw_actions`, and `mdp.last_action` feeds the raw value back into the
    observation, so a policy can sit at a raw action of 7 and pay nothing for
    it while the joint quietly saturates.

    Zero inside [-threshold, threshold], quadratic outside, so normal operation
    is untouched - which is why this is used instead of `mdp.action_l2`.
    """
    excess = torch.relu(torch.abs(env.action_manager.action) - threshold)
    return torch.sum(torch.square(excess), dim=1)


def steering_angle_limit_penalty(
    env: ManagerBasedRLEnv,
    limit_threshold: float = 0.85,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize steering angle when nearing physical joint limits (1.047 rad)."""
    robot: Articulation = env.scene[asset_cfg.name]
    joint_ids, _ = robot.find_joints("handle_bar_chasis_joint")
    steer_pos = robot.data.joint_pos[:, joint_ids[0]]
    excess = torch.relu(torch.abs(steer_pos) - limit_threshold)
    return torch.square(excess)


def goal_distance_exp(
    env: ManagerBasedRLEnv,
    command_name: str = "goal_pose",
    std: float = 1.0,
) -> torch.Tensor:
    """Reward for being close to the 2D target position."""
    command = env.command_manager.get_command(command_name)
    # command contains [target_x_b, target_y_b, ...]
    dist_sq = command[:, 0] ** 2 + command[:, 1] ** 2
    return torch.exp(-dist_sq / (std ** 2))


def goal_reached_bonus(
    env: ManagerBasedRLEnv,
    threshold: float = 0.4,
    command_name: str = "goal_pose",
) -> torch.Tensor:
    """Bonus reward when arriving within the target goal radius."""
    command = env.command_manager.get_command(command_name)
    dist = torch.sqrt(command[:, 0] ** 2 + command[:, 1] ** 2)
    return (dist < threshold).float()
