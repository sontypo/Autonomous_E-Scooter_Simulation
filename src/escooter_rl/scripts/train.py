# Copyright (c) 2026, Autonomous E-Scooter Simulation Project.
# All rights reserved.

"""Script to train E-Scooter RL agent with RSL-RL in Isaac Lab."""

import argparse
import os
import sys
from datetime import datetime

from isaaclab.app import AppLauncher

# Add argparse arguments
parser = argparse.ArgumentParser(description="Train E-Scooter RL agent with RSL-RL.")
parser.add_argument("--task", type=str, default="Isaac-EScooter-GoalNav-v0", help="Task name to train.")
parser.add_argument("--num_envs", type=int, default=2048, help="Number of parallel environments.")
parser.add_argument("--max_iterations", type=int, default=None, help="Maximum PPO training iterations.")
parser.add_argument("--seed", type=int, default=42, help="Random seed.")
parser.add_argument("--resume", action="store_true", default=False, help="Resume training from checkpoint.")
parser.add_argument("--load_run", type=str, default=None, help="Run directory to load checkpoint from.")
parser.add_argument("--checkpoint", type=str, default=None, help="Direct path to checkpoint file.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of recorded video (in steps).")
parser.add_argument("--video_interval", type=int, default=2000, help="Interval between video recordings (in steps).")

# Append AppLauncher CLI arguments (e.g. --headless, --device)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()

# Clear out sys.argv for Hydra if any
sys.argv = [sys.argv[0]] + hydra_args

# Launch Omniverse simulator
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Post-AppLaunch Imports"""

import gymnasium as gym
import torch

from rsl_rl.runners import OnPolicyRunner
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper, export_policy_as_jit, export_policy_as_onnx
from isaaclab.utils.dict import print_dict
from isaaclab.utils.io import dump_pickle, dump_yaml

import escooter_rl  # noqa: F401 - registers custom gym tasks


def main():
    print(f"\n=======================================================")
    print(f"[INFO] Initializing Massive Parallel RL for E-Scooter")
    print(f"Task:       {args_cli.task}")
    print(f"Num Envs:   {args_cli.num_envs}")
    print(f"Device:     {args_cli.device or 'cuda:0'}")
    print(f"=======================================================\n")

    # Get registered task entrypoints
    task_spec = gym.spec(args_cli.task)
    env_cfg_entry_point = task_spec.kwargs["env_cfg_entry_point"]
    rsl_rl_cfg_entry_point = task_spec.kwargs["rsl_rl_cfg_entry_point"]

    # Load environment and agent configurations
    mod_name, cls_name = env_cfg_entry_point.split(":")
    env_cfg_cls = getattr(__import__(mod_name, fromlist=[cls_name]), cls_name)
    env_cfg = env_cfg_cls()

    mod_name, cls_name = rsl_rl_cfg_entry_point.split(":")
    agent_cfg_cls = getattr(__import__(mod_name, fromlist=[cls_name]), cls_name)
    agent_cfg: RslRlOnPolicyRunnerCfg = agent_cfg_cls()

    # Apply CLI overrides
    if args_cli.num_envs is not None:
        env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.max_iterations is not None:
        agent_cfg.max_iterations = args_cli.max_iterations
    if args_cli.seed is not None:
        env_cfg.seed = args_cli.seed
        agent_cfg.seed = args_cli.seed
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
        agent_cfg.device = args_cli.device

    # Set up experiment logging directory
    log_root_path = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
    log_dir = os.path.join(log_root_path, datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
    os.makedirs(log_dir, exist_ok=True)
    print(f"[INFO] Experiment log directory: {log_dir}")

    # Create vectorized environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # Wrap environment for RSL-RL
    vec_env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    # Initialize RSL-RL OnPolicyRunner
    runner = OnPolicyRunner(vec_env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)

    # Resume from checkpoint if requested
    if args_cli.resume:
        resume_path = args_cli.checkpoint
        if resume_path is None and args_cli.load_run:
            resume_path = os.path.join(log_root_path, args_cli.load_run, "model_*.pt")
        if resume_path and os.path.exists(resume_path):
            print(f"[INFO] Resuming training from checkpoint: {resume_path}")
            runner.load(resume_path)
        else:
            print(f"[WARN] Checkpoint path not found: {resume_path}. Starting from scratch.")

    # Dump environment and agent configuration files
    params_dir = os.path.join(log_dir, "params")
    os.makedirs(params_dir, exist_ok=True)
    dump_yaml(os.path.join(params_dir, "env.yaml"), env_cfg)
    dump_yaml(os.path.join(params_dir, "agent.yaml"), agent_cfg)

    # Run PPO training loop
    print(f"[INFO] Starting training for {agent_cfg.max_iterations} iterations...")
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

    # Export trained policy to ONNX and JIT
    print(f"[INFO] Exporting trained policy to ONNX and TorchScript...")
    try:
        policy_actor = runner.get_inference_policy(device=env.unwrapped.device)
        onnx_path = os.path.join(log_dir, "exported", "policy.onnx")
        jit_path = os.path.join(log_dir, "exported", "policy.pt")
        os.makedirs(os.path.dirname(onnx_path), exist_ok=True)
        export_policy_as_onnx(policy_actor, onnx_path)
        export_policy_as_jit(policy_actor, jit_path)
        print(f"[SUCCESS] Exported models:\n  ONNX: {onnx_path}\n  JIT:  {jit_path}")
    except Exception as e:
        print(f"[WARN] Failed to export policy automatically: {e}")

    # Close environment
    env.close()
    print(f"\n[SUCCESS] Training finished! All logs and models saved to:\n  {log_dir}\n")


if __name__ == "__main__":
    main()
    simulation_app.close()
