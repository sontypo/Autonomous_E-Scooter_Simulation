# Copyright (c) 2026, Autonomous E-Scooter Simulation Project.
# All rights reserved.

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def bad_orientation(
    env: ManagerBasedRLEnv,
    max_roll_rad: float = 0.61,  # ~35 degrees
    max_pitch_rad: float = 0.61, # ~35 degrees
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Terminate if the e-scooter leans beyond the unrecoverable roll or pitch limit."""
    robot: Articulation = env.scene[asset_cfg.name]
    gravity_proj = robot.data.projected_gravity_b
    # gx = sin(pitch), gy = -sin(roll)*cos(pitch), gz = -cos(roll)*cos(pitch)
    pitch = torch.asin(torch.clamp(gravity_proj[:, 0], -1.0, 1.0))
    roll = torch.atan2(-gravity_proj[:, 1], -gravity_proj[:, 2])
    
    roll_bad = torch.abs(roll) > max_roll_rad
    pitch_bad = torch.abs(pitch) > max_pitch_rad
    return roll_bad | pitch_bad


def base_height_too_low(
    env: ManagerBasedRLEnv,
    min_height: float = 0.18,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Terminate if the base link height falls too low (chassis crashed into the ground)."""
    robot: Articulation = env.scene[asset_cfg.name]
    base_pos_z = robot.data.root_pos_w[:, 2]
    return base_pos_z < min_height
