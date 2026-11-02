# Copyright (c) 2026, Autonomous E-Scooter Simulation Project.
# All rights reserved.

"""Single source of truth for how observations are scaled.

The training environment and the ROS 2 deployment node have to agree on the
observation vector exactly: same term order, same joint order, same units. They
have disagreed twice already, each time silently - the joint_vel pair was
swapped, and the drive wheel sign was inverted - and in both cases the policy
kept running and produced plausible-looking nonsense.

So the numbers live here, `escooter_env_cfg` imports them, `export_onnx.py`
writes them next to the exported policy as `obs_spec.json`, and the ROS node
reads that file instead of carrying its own copy.

This module must stay free of imports. `export_onnx.py` loads it by file path,
without going through `escooter_rl/__init__.py`, so that exporting a checkpoint
never needs Isaac Sim.
"""

SPEC_VERSION = 1

# The reaction plate runs to a few hundred rad/s while every other observation
# is of order 1. Fed raw, it dominates the input layer and, worse, the policy
# has to extrapolate the moment the plate leaves the range it saw in training:
# the steering head of the 2026-10-30 checkpoint reaches a raw action of +7.3 at
# -150 rad/s, far outside the valid [-1, 1]. Clipping makes that structurally
# impossible whatever the deployed plant does; the scale puts the input on the
# same footing as the rest.
#
# Clip is in rad/s and is applied BEFORE scale, matching
# ObservationTermCfg's noise -> clip -> scale order.
REACTION_VEL_CLIP = (-250.0, 250.0)
REACTION_VEL_SCALE = 0.01

# Wheel speeds reach 2.5 m/s / 0.135 m = 18.5 rad/s, the second largest
# observation by an order of magnitude.
WHEEL_VEL_CLIP = (-40.0, 40.0)
WHEEL_VEL_SCALE = 0.05

# Action scales, from EScooterActionsCfg.
ACTION_SCALES = {
    "steering": 0.7,          # rad
    "reaction_plate": 60.0,   # N.m
    "drive_wheel": 20.0,      # rad/s
}

# SceneEntityCfg resolves joint names in articulation order unless
# preserve_order=True. This articulation orders them back_wheel(0),
# handle_bar(1), front_wheel(2), rotational_plate(3), so the joint_vel
# observation is [back_wheel, front_wheel] even though the config lists
# front_wheel first.
JOINT_VEL_ORDER = ["back_wheel_joint", "front_wheel_joint"]

_COMMON_TERMS = [
    {"name": "base_lin_vel", "dim": 3},
    {"name": "base_ang_vel", "dim": 3},
    {"name": "projected_gravity", "dim": 3},
    {"name": "steering_pos", "dim": 1},
    {
        "name": "reaction_vel",
        "dim": 1,
        "clip": list(REACTION_VEL_CLIP),
        "scale": REACTION_VEL_SCALE,
    },
    {
        "name": "joint_vel",
        "dim": 2,
        "clip": list(WHEEL_VEL_CLIP),
        "scale": WHEEL_VEL_SCALE,
        "joint_order": list(JOINT_VEL_ORDER),
    },
]

_COMMAND_DIM = {"velocity": 3, "goal_nav": 4}


def build(task: str) -> dict:
    """Serialisable description of the observation vector for `task`.

    `task` is "velocity" or "goal_nav".
    """
    if task not in _COMMAND_DIM:
        raise ValueError("unknown task %r; expected one of %s"
                         % (task, ", ".join(sorted(_COMMAND_DIM))))
    terms = [dict(t) for t in _COMMON_TERMS]
    terms.append({"name": "command", "dim": _COMMAND_DIM[task]})
    terms.append({"name": "actions", "dim": 3})
    return {
        "spec_version": SPEC_VERSION,
        "task": task,
        "obs_dim": sum(t["dim"] for t in terms),
        "terms": terms,
        "action_scales": dict(ACTION_SCALES),
    }
