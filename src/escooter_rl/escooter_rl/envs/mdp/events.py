# Copyright (c) 2026, Autonomous E-Scooter Simulation Project.
# All rights reserved.

from __future__ import annotations

import math
import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_from_euler_xyz, sample_uniform

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def reset_escooter_root_state(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float]] | None = None,
    velocity_range: dict[str, tuple[float, float]] | None = None,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    """Reset root state with small random perturbation to orientation and velocity.
    
    Prevents the agent from overfitting to a perfectly upright, stationary start.
    """
    robot: Articulation = env.scene[asset_cfg.name]
    num_resets = len(env_ids)
    
    # Default ranges if not specified
    if pose_range is None:
        pose_range = {
            "x": (-0.5, 0.5),
            "y": (-0.5, 0.5),
            "yaw": (-math.pi, math.pi),
            "roll": (-0.05, 0.05),  # ~ +/- 3 degrees initial tilt
            "pitch": (-0.02, 0.02),
        }
    if velocity_range is None:
        velocity_range = {
            "x": (0.0, 0.5),        # Slight initial forward velocity
            "y": (-0.05, 0.05),
            "z": (0.0, 0.0),
            "roll": (-0.1, 0.1),
            "pitch": (-0.05, 0.05),
            "yaw": (-0.1, 0.1),
        }

    # Obtain default root state
    root_states = robot.data.default_root_state[env_ids].clone()

    # Positions
    root_states[:, 0] += sample_uniform(pose_range.get("x", (0.0, 0.0))[0], pose_range.get("x", (0.0, 0.0))[1], (num_resets,), device=robot.device)
    root_states[:, 1] += sample_uniform(pose_range.get("y", (0.0, 0.0))[0], pose_range.get("y", (0.0, 0.0))[1], (num_resets,), device=robot.device)
    root_states[:, :3] += env.scene.env_origins[env_ids]

    # Orientation (Roll, Pitch, Yaw)
    roll = sample_uniform(pose_range.get("roll", (0.0, 0.0))[0], pose_range.get("roll", (0.0, 0.0))[1], (num_resets,), device=robot.device)
    pitch = sample_uniform(pose_range.get("pitch", (0.0, 0.0))[0], pose_range.get("pitch", (0.0, 0.0))[1], (num_resets,), device=robot.device)
    yaw = sample_uniform(pose_range.get("yaw", (0.0, 0.0))[0], pose_range.get("yaw", (0.0, 0.0))[1], (num_resets,), device=robot.device)
    quats = quat_from_euler_xyz(roll, pitch, yaw)
    root_states[:, 3:7] = quats

    # Velocities
    root_states[:, 7] = sample_uniform(velocity_range.get("x", (0.0, 0.0))[0], velocity_range.get("x", (0.0, 0.0))[1], (num_resets,), device=robot.device)
    root_states[:, 8] = sample_uniform(velocity_range.get("y", (0.0, 0.0))[0], velocity_range.get("y", (0.0, 0.0))[1], (num_resets,), device=robot.device)
    root_states[:, 9] = sample_uniform(velocity_range.get("z", (0.0, 0.0))[0], velocity_range.get("z", (0.0, 0.0))[1], (num_resets,), device=robot.device)
    root_states[:, 10] = sample_uniform(velocity_range.get("roll", (0.0, 0.0))[0], velocity_range.get("roll", (0.0, 0.0))[1], (num_resets,), device=robot.device)
    root_states[:, 11] = sample_uniform(velocity_range.get("pitch", (0.0, 0.0))[0], velocity_range.get("pitch", (0.0, 0.0))[1], (num_resets,), device=robot.device)
    root_states[:, 12] = sample_uniform(velocity_range.get("yaw", (0.0, 0.0))[0], velocity_range.get("yaw", (0.0, 0.0))[1], (num_resets,), device=robot.device)

    # Write root state to simulation
    robot.write_root_pose_to_sim(root_states[:, :7], env_ids=env_ids)
    robot.write_root_velocity_to_sim(root_states[:, 7:], env_ids=env_ids)


def reset_escooter_joints(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    steer_range: tuple[float, float] = (-0.05, 0.05),
    plate_vel_range: tuple[float, float] = (-5.0, 5.0),
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    """Reset joint positions and velocities with small initial perturbation."""
    robot: Articulation = env.scene[asset_cfg.name]
    num_resets = len(env_ids)

    joint_pos = robot.data.default_joint_pos[env_ids].clone()
    joint_vel = robot.data.default_joint_vel[env_ids].clone()

    # Steering joint
    steer_ids, _ = robot.find_joints("handle_bar_chasis_joint")
    if len(steer_ids) > 0:
        joint_pos[:, steer_ids[0]] = sample_uniform(steer_range[0], steer_range[1], (num_resets,), device=robot.device)

    # Reaction plate joint velocity
    plate_ids, _ = robot.find_joints("rotational_plate_joint")
    if len(plate_ids) > 0:
        joint_vel[:, plate_ids[0]] = sample_uniform(plate_vel_range[0], plate_vel_range[1], (num_resets,), device=robot.device)

    robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)


def spin_reaction_plate(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    speed_range: tuple[float, float] = (-180.0, 180.0),
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    """Throw the flywheel to a random speed mid-episode.

    The policy has to practise unloading momentum, and it can only practise what
    it sees. With the plate only ever starting near zero and the episode holding
    it there, a run that did reach a few hundred rad/s was off-distribution, and
    the deployed policy responded by spinning it up further rather than bleeding
    it down. This is the plate's equivalent of `push_robot`.
    """
    robot: Articulation = env.scene[asset_cfg.name]
    plate_ids, _ = robot.find_joints("rotational_plate_joint")
    if not plate_ids:
        return
    joint_vel = robot.data.joint_vel[env_ids].clone()
    joint_vel[:, plate_ids[0]] = sample_uniform(
        speed_range[0], speed_range[1], (len(env_ids),), device=robot.device)
    robot.write_joint_state_to_sim(
        robot.data.joint_pos[env_ids].clone(), joint_vel, env_ids=env_ids)
