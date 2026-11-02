# Copyright (c) 2026, Autonomous E-Scooter Simulation Project.
# All rights reserved.

import gymnasium as gym
from . import agents, envs

##
# Register Gym Environments
##

# 1. Goal Navigation & Self-Balancing
gym.register(
    id="Isaac-EScooter-GoalNav-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{envs.__name__}:EScooterGoalNavEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}:EScooterGoalNavPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-EScooter-GoalNav-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{envs.__name__}:EScooterGoalNavEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}:EScooterGoalNavPPORunnerCfg",
    },
)

# 2. Velocity Tracking & Self-Balancing
gym.register(
    id="Isaac-EScooter-Velocity-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{envs.__name__}:EScooterVelocityEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}:EScooterVelocityPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-EScooter-Velocity-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{envs.__name__}:EScooterVelocityEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}:EScooterVelocityPPORunnerCfg",
    },
)
