# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(
    description="This script demonstrates adding a custom robot to an Isaac Lab environment."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

ESCOOTER_CONFIG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(usd_path="/home/hong-son/Autonomous_E-Scooter_Simulation/src/escooter_simulation/new_meshes/e-scooter/e-scooter_isaac.usd", rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),),
    actuators={"wheel_acts": ImplicitActuatorCfg(joint_names_expr=[".*"], effort_limit_sim=100, velocity_limit_sim=500, damping=50, stiffness=400)},
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0)),
)


class NewRobotsSceneCfg(InteractiveSceneCfg):
    """Designs the scene."""

    # Ground-plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # robot
    Escooter = ESCOOTER_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Escooter")

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    while simulation_app.is_running():
        # reset
        if count % 500 == 0:
            # reset counters
            count = 0
            # reset the scene entities to their initial positions offset by the environment origins
            root_escooter_state = scene["Escooter"].data.default_root_state.clone()
            root_escooter_state[:, :3] += scene.env_origins

            # copy the default root state to the sim for the escooter's orientation and velocity

            scene["Escooter"].write_root_pose_to_sim(root_escooter_state[:, :7])
            scene["Escooter"].write_root_velocity_to_sim(root_escooter_state[:, 7:])

            # copy the default joint states to the sim
            joint_pos, joint_vel = (
                scene["Escooter"].data.default_joint_pos.clone(),
                scene["Escooter"].data.default_joint_vel.clone(),
            )
            scene["Escooter"].write_joint_state_to_sim(joint_pos, joint_vel)
            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting Escooter state...")

        # drive around
        if count % 100 < 75:
            # Drive straight by setting equal wheel velocities
            action = torch.Tensor([[10.0, 10.0, 10.0, 10.0]])
        else:
            # Turn by applying different velocities
            action = torch.Tensor([[5.0, -5.0, 5.0, -5.0]])

        # # drive around for Escooter
        # if count % 100 < 75:
        #     # Drive straight by setting equal wheel velocities
        #     scout_action = torch.Tensor([[2.0, 2.0, 2.0, 2.0]])
        # else:
        #     # Turn by applying different velocities
        #     scout_action = torch.Tensor([[1.0, -1.0, 1.0, -1.0]])
        
        # [back_left, back_right, front_left, front_right]
        scene["Escooter"].set_joint_velocity_target(action)

        scene.write_data_to_sim()
        sim.step()
        sim_time += sim_dt
        count += 1
        scene.update(sim_dt)


def main():
    """Main function."""
    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])
    # Design scene
    scene_cfg = NewRobotsSceneCfg(args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()