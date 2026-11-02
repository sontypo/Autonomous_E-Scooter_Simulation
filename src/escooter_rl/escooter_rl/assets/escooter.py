# Copyright (c) 2026, Autonomous E-Scooter Simulation Project.
# All rights reserved.

import os
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

# Locate the USD file
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "../../../.."))
DEFAULT_USD_PATH = os.path.join(
    REPO_ROOT,
    "src/escooter_simulation/new_meshes/e-scooter/e-scooter_isaac.usd",
)

if not os.path.exists(DEFAULT_USD_PATH):
    # Fallback to direct absolute path if needed
    DEFAULT_USD_PATH = "/home/hong-son/Autonomous_E-Scooter_Simulation/src/escooter_simulation/new_meshes/e-scooter/e-scooter_isaac.usd"

ESCOOTER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=DEFAULT_USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=100.0,
            max_angular_velocity=100.0,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.34),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos={
            "handle_bar_chasis_joint": 0.0,
            "rotational_plate_joint": 0.0,
            "front_wheel_joint": 0.0,
            "back_wheel_joint": 0.0,
        },
        joint_vel={
            "handle_bar_chasis_joint": 0.0,
            "rotational_plate_joint": 0.0,
            "front_wheel_joint": 0.0,
            "back_wheel_joint": 0.0,
        },
    ),
    actuators={
        # Steering handlebar: PD position control
        "steering": ImplicitActuatorCfg(
            joint_names_expr=["handle_bar_chasis_joint"],
            effort_limit_sim=100.0,
            velocity_limit_sim=10.0,
            stiffness=350.0,
            damping=25.0,
        ),
        # Reaction momentum plate: Direct torque/effort control
        "reaction_plate": ImplicitActuatorCfg(
            joint_names_expr=["rotational_plate_joint"],
            effort_limit_sim=500.0,
            velocity_limit_sim=200.0,
            stiffness=0.0,
            damping=0.2,
        ),
        # Front drive wheel: Velocity tracking control
        "traction_wheel": ImplicitActuatorCfg(
            joint_names_expr=["front_wheel_joint"],
            effort_limit_sim=80.0,
            velocity_limit_sim=60.0,
            stiffness=0.0,
            damping=5.0,
        ),
        # Back wheel: Passive free-rolling wheel
        "passive_wheel": ImplicitActuatorCfg(
            joint_names_expr=["back_wheel_joint"],
            effort_limit_sim=0.0,
            velocity_limit_sim=60.0,
            stiffness=0.0,
            damping=0.2,
        ),
    },
)
