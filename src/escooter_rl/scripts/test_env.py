# Copyright (c) 2026, Autonomous E-Scooter Simulation Project.
# All rights reserved.

"""Script to test and verify the E-Scooter RL environment setup."""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Test E-Scooter Isaac Lab Environment.")
parser.add_argument("--task", type=str, default="Isaac-EScooter-GoalNav-v0", help="Task name to test.")
parser.add_argument("--num_envs", type=int, default=4, help="Number of environments.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch simulation app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
import escooter_rl  # noqa: F401 - registers gym environments


def main():
    print(f"\n=======================================================")
    print(f"[INFO] Testing Environment: {args_cli.task} with {args_cli.num_envs} envs")
    print(f"=======================================================\n")

    from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

    # Load and parse environment configuration
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    # Create environment
    env = gym.make(args_cli.task, cfg=env_cfg)
    
    print(f"[INFO] Observation Space: {env.observation_space}")
    print(f"[INFO] Action Space:      {env.action_space}")

    # Reset environment
    obs, info = env.reset()
    policy_obs = obs["policy"] if isinstance(obs, dict) else obs
    print(f"[INFO] Reset successful! Policy Obs Shape: {policy_obs.shape}")

    # Step environment for 50 steps
    print(f"[INFO] Running 50 test steps with random actions...")
    for step in range(50):
        # Sample random actions
        actions = 2.0 * torch.rand(env.unwrapped.num_envs, env.unwrapped.action_manager.total_action_dim, device=env.unwrapped.device) - 1.0
        obs, reward, terminated, truncated, info = env.step(actions)
        
        if step % 10 == 0:
            avg_rew = reward.mean().item()
            print(f"  Step {step:2d} | Avg Reward: {avg_rew:.4f} | Done: {terminated.sum().item() + truncated.sum().item()}")

    print("\n[SUCCESS] Environment step, reset, and physics verified successfully!")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
