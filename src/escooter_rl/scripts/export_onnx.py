# Copyright (c) 2026, Autonomous E-Scooter Simulation Project.
# All rights reserved.

"""Script to export a trained RSL-RL policy checkpoint to ONNX and TorchScript for ROS 2 deployment."""

import argparse
import importlib.util
import json
import os
import re
import torch
import torch.nn as nn


def load_obs_spec_module():
    """Load escooter_rl/obs_spec.py by path, without importing the package.

    `import escooter_rl.obs_spec` would run the package __init__, which pulls in
    Isaac Lab; exporting a checkpoint has to work on a machine with no GPU free.
    obs_spec.py has no imports of its own, so loading it directly is safe.
    """
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
                        "escooter_rl", "obs_spec.py")
    spec = importlib.util.spec_from_file_location("escooter_obs_spec", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def task_from(obs_dim: int, checkpoint_path: str) -> str:
    """Which task this checkpoint belongs to: goal_nav carries a 4-wide command."""
    if "goal_nav" in checkpoint_path:
        return "goal_nav"
    if "velocity" in checkpoint_path:
        return "velocity"
    return "goal_nav" if obs_dim == 20 else "velocity"


def find_latest_checkpoint(log_root_base: str = "logs/rsl_rl", experiment_name: str = None) -> str:
    """Find the latest checkpoint file from the latest experiment run directory."""
    log_root_base = os.path.abspath(log_root_base)
    if not os.path.exists(log_root_base):
        raise FileNotFoundError(
            f"Log directory not found at: '{log_root_base}'.\n"
            f"Please run training first using: python src/escooter_rl/scripts/train.py"
        )

    # Check if a specific experiment folder was given
    target_exp_dir = os.path.join(log_root_base, experiment_name) if experiment_name else None
    if target_exp_dir and os.path.isdir(target_exp_dir):
        exp_dirs = [target_exp_dir]
    else:
        direct_pt = [f for f in os.listdir(log_root_base) if f.endswith(".pt") and "model" in f]
        if direct_pt:
            exp_dirs = [log_root_base]
        else:
            subdirs = [
                os.path.join(log_root_base, d)
                for d in os.listdir(log_root_base)
                if os.path.isdir(os.path.join(log_root_base, d))
            ]
            if not subdirs:
                raise FileNotFoundError(f"No experiment folders found inside '{log_root_base}'.")
            subdirs.sort(key=os.path.getmtime, reverse=True)
            exp_dirs = subdirs

    all_runs = []
    for ed in exp_dirs:
        direct_pt = [f for f in os.listdir(ed) if f.endswith(".pt") and "model" in f]
        if direct_pt:
            all_runs.append(ed)
        else:
            runs = [
                os.path.join(ed, d)
                for d in os.listdir(ed)
                if os.path.isdir(os.path.join(ed, d))
            ]
            all_runs.extend(runs)

    if not all_runs:
        raise FileNotFoundError(f"No experiment run directories found in '{log_root_base}'.")

    # Sort runs by modification time (latest run is last)
    all_runs.sort(key=os.path.getmtime)
    latest_run_dir = all_runs[-1]

    pt_files = [f for f in os.listdir(latest_run_dir) if f.endswith(".pt") and "model" in f]
    if not pt_files:
        raise FileNotFoundError(f"No checkpoint files (.pt) found in directory '{latest_run_dir}'.")

    def get_iteration(filename):
        match = re.search(r"model_(\d+)\.pt", filename)
        return int(match.group(1)) if match else -1

    if any(get_iteration(f) != -1 for f in pt_files):
        pt_files.sort(key=get_iteration)
    else:
        pt_files.sort(key=lambda f: os.path.getmtime(os.path.join(latest_run_dir, f)))

    latest_file = os.path.join(latest_run_dir, pt_files[-1])
    return latest_file


def export_model(checkpoint_path: str = None, experiment_name: str = None, output_dir: str = None):
    # Auto-detect latest checkpoint if not specified
    if checkpoint_path is None or checkpoint_path.strip() == "":
        print(f"[INFO] No checkpoint specified. Auto-detecting latest checkpoint from logs/rsl_rl...")
        checkpoint_path = find_latest_checkpoint(log_root_base="logs/rsl_rl", experiment_name=experiment_name)
    elif os.path.isdir(checkpoint_path):
        print(f"[INFO] Directory provided: {checkpoint_path}. Searching for latest model checkpoint...")
        checkpoint_path = find_latest_checkpoint(checkpoint_path)

    if not os.path.exists(checkpoint_path):
        raise FileNotFoundError(f"Checkpoint not found at: {checkpoint_path}")

    if output_dir is None:
        output_dir = os.path.join(os.path.dirname(checkpoint_path), "exported")
    os.makedirs(output_dir, exist_ok=True)

    print(f"\n=======================================================")
    print(f"[INFO] Exporting E-Scooter Policy to ONNX & TorchScript")
    print(f"Checkpoint: {checkpoint_path}")
    print(f"Output Dir: {output_dir}")
    print(f"=======================================================\n")

    checkpoint = torch.load(checkpoint_path, map_location="cpu")

    # Extract model state dict
    if "model_state_dict" in checkpoint:
        state_dict = checkpoint["model_state_dict"]
    else:
        state_dict = checkpoint

    # Reconstruct Actor network layers
    # RSL-RL ActorCritic uses MLP: actor.0, actor.2, actor.4, ...
    actor_weights = {k.replace("actor.", ""): v for k, v in state_dict.items() if k.startswith("actor.")}
    
    # Infer input and output dimensions from weights
    first_layer_weight = actor_weights["0.weight"]
    obs_dim = first_layer_weight.shape[1]
    
    # Find last layer
    layer_indices = sorted([int(k.split(".")[0]) for k in actor_weights.keys() if "weight" in k])
    last_idx = layer_indices[-1]
    action_dim = actor_weights[f"{last_idx}.weight"].shape[0]

    print(f"[INFO] Detected Model Architecture:")
    print(f"  Observation Dimension: {obs_dim}")
    print(f"  Action Dimension:      {action_dim}")

    # Build pure feed-forward Actor module
    layers = []
    for i in range(0, last_idx, 2):
        in_f = actor_weights[f"{i}.weight"].shape[1]
        out_f = actor_weights[f"{i}.weight"].shape[0]
        linear = nn.Linear(in_f, out_f)
        linear.weight.data = actor_weights[f"{i}.weight"]
        linear.bias.data = actor_weights[f"{i}.bias"]
        layers.append(linear)
        layers.append(nn.ELU())

    # Final output layer
    out_linear = nn.Linear(actor_weights[f"{last_idx}.weight"].shape[1], action_dim)
    out_linear.weight.data = actor_weights[f"{last_idx}.weight"]
    out_linear.bias.data = actor_weights[f"{last_idx}.bias"]
    layers.append(out_linear)

    policy_net = nn.Sequential(*layers)
    policy_net.eval()

    # Dummy observation tensor for tracing
    dummy_input = torch.zeros(1, obs_dim, dtype=torch.float32)

    # 1. Export TorchScript JIT
    jit_path = os.path.join(output_dir, "escooter_policy.pt")
    traced_model = torch.jit.trace(policy_net, dummy_input)
    traced_model.save(jit_path)
    print(f"[SUCCESS] Exported TorchScript model: {jit_path}")

    # 2. Export ONNX
    onnx_path = os.path.join(output_dir, "escooter_policy.onnx")
    torch.onnx.export(
        policy_net,
        dummy_input,
        onnx_path,
        export_params=True,
        opset_version=14,
        do_constant_folding=True,
        input_names=["observations"],
        output_names=["actions"],
        dynamic_axes={"observations": {0: "batch_size"}, "actions": {0: "batch_size"}},
    )
    print(f"[SUCCESS] Exported ONNX model:        {onnx_path}")

    # 3. Export the observation spec beside the weights.
    #
    # The deployment node cannot infer the observation scaling from the ONNX
    # graph, and every time the two sides have been kept in sync by hand they
    # have drifted apart silently. Writing the spec next to the policy makes
    # the pair self-describing.
    obs_spec_module = load_obs_spec_module()
    task = task_from(obs_dim, checkpoint_path)
    obs_spec = obs_spec_module.build(task)
    if obs_spec["obs_dim"] != obs_dim:
        raise RuntimeError(
            "obs_spec for task %r describes %d observations but the checkpoint "
            "takes %d. escooter_rl/obs_spec.py and escooter_env_cfg.py have "
            "drifted apart; fix them before deploying this policy."
            % (task, obs_spec["obs_dim"], obs_dim))
    obs_spec["checkpoint"] = os.path.basename(checkpoint_path)
    spec_path = os.path.join(output_dir, "obs_spec.json")
    with open(spec_path, "w") as handle:
        json.dump(obs_spec, handle, indent=2)
    print(f"[SUCCESS] Exported observation spec:  {spec_path}  (task: {task})")

    print("\n[INFO] Action format: [steering_angle_offset, reaction_wheel_torque, front_wheel_speed]")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export RSL-RL checkpoint to ONNX and JIT.")
    parser.add_argument(
        "--checkpoint",
        type=str,
        default=None,
        help="Path to checkpoint .pt file or run directory. If omitted, auto-loads the latest model from logs/rsl_rl.",
    )
    parser.add_argument(
        "--experiment_name",
        type=str,
        default=None,
        help="Optional experiment name filter (e.g. escooter_goal_nav or escooter_velocity).",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default=None,
        help="Directory to save exported models. Defaults to <run_dir>/exported/.",
    )
    args = parser.parse_args()
    export_model(args.checkpoint, args.experiment_name, args.output_dir)
