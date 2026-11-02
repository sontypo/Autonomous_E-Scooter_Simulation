# Copyright (c) 2026, Autonomous E-Scooter Simulation Project.
# All rights reserved.

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_rotate_inverse, wrap_to_pi

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def base_roll_pitch(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Extract Euler roll and pitch angles of the robot base link from projected gravity.
    
    Returns:
        Tensor of shape (num_envs, 2) containing [roll, pitch] in radians.
    """
    robot: Articulation = env.scene[asset_cfg.name]
    gravity_proj = robot.data.projected_gravity_b
    # gx = sin(pitch), gy = -sin(roll)*cos(pitch), gz = -cos(roll)*cos(pitch)
    pitch = torch.asin(torch.clamp(gravity_proj[:, 0], -1.0, 1.0))
    roll = torch.atan2(-gravity_proj[:, 1], -gravity_proj[:, 2])
    return torch.stack([roll, pitch], dim=-1)


def reaction_plate_velocity(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Extract angular velocity of the reaction plate joint (flywheel).
    
    Returns:
        Tensor of shape (num_envs, 1) containing reaction plate angular velocity in rad/s.
    """
    robot: Articulation = env.scene[asset_cfg.name]
    joint_ids, _ = robot.find_joints("rotational_plate_joint")
    return robot.data.joint_vel[:, joint_ids]


def steering_angle(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Extract steering angle of the handlebar chassis joint.
    
    Returns:
        Tensor of shape (num_envs, 1) containing handlebar steering position in radians.
    """
    robot: Articulation = env.scene[asset_cfg.name]
    joint_ids, _ = robot.find_joints("handle_bar_chasis_joint")
    return robot.data.joint_pos[:, joint_ids]


def relative_goal_pos_2d(env: ManagerBasedRLEnv, command_name: str = "goal_pose") -> torch.Tensor:
    """Extract 2D relative position and orientation to the goal in the robot's base frame.
    
    Returns:
        Tensor of shape (num_envs, 3) containing [distance, sin(heading_error), cos(heading_error)].
    """
    command = env.command_manager.get_command(command_name)
    # Target position in base frame: command[:, :2]
    rel_x = command[:, 0]
    rel_y = command[:, 1]
    dist = torch.sqrt(rel_x ** 2 + rel_y ** 2)
    heading_err = torch.atan2(rel_y, rel_x)
    return torch.stack([dist, torch.sin(heading_err), torch.cos(heading_err)], dim=-1)
