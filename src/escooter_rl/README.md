# Autonomous E-Scooter RL: Massive Parallel Self-Balancing & Goal Navigation

This package provides a production-ready, massive parallel Reinforcement Learning (RL) training environment for the autonomous self-balancing E-Scooter using **Isaac Lab** (formerly Orbit) and **RSL-RL** (PPO) in NVIDIA Isaac Sim.

---

## 1. System Overview & Physical Dynamics

The autonomous e-scooter is an underactuated, inherently unstable dynamic system (an inverted pendulum on two in-line wheels). Stabilizing and navigating it requires coordinating two complementary physical mechanisms:

1. **Reaction Momentum Wheel (`rotational_plate_joint`)**:
   - Generates direct active roll torque ($\tau$) on the chassis.
   - Highly effective at zero and low speeds where steering has negligible lateral acceleration authority.
   - Enforces **momentum unloading** via a velocity penalty to prevent flywheel RPM saturation.

2. **Steer-to-Balance Dynamics (`handle_bar_chasis_joint`)**:
   - Steers the contact patch under the falling center of mass ($\delta = \text{atan}(L \cdot \tau_{lat} / (M \cdot h \cdot v^2))$).
   - Generates roll torque scaling with forward speed squared ($v^2$).

3. **Front Traction Drive (`front_wheel_joint`)**:
   - Propels the e-scooter forward towards navigation goals or target speeds.

4. **Passive Rear Wheel (`back_wheel_joint`)**:
   - Free-rolling follower wheel with realistic bearing damping.

---

## 2. Action & Observation Spaces

### Action Space (Dimension: 3)
| Action Index | Joint Name | Control Mode | Action Range | Physical Scale |
| :--- | :--- | :--- | :--- | :--- |
| **0** | `handle_bar_chasis_joint` | Position Target | $[-1.0, 1.0]$ | $[-0.7, 0.7]\text{ rad}$ ($\pm 40^\circ$) |
| **1** | `rotational_plate_joint` | Direct Effort (Torque) | $[-1.0, 1.0]$ | $[-60.0, 60.0]\text{ N}\cdot\text{m}$ |
| **2** | `front_wheel_joint` | Velocity Target | $[-1.0, 1.0]$ | $[-20.0, 20.0]\text{ rad/s}$ ($\sim 2.8\text{ m/s}$) |

*Note: `back_wheel_joint` remains passive and is simulated naturally by PhysX.*

### Observation Space
#### Goal Navigation Task (`Isaac-EScooter-GoalNav-v0` - Dimension: 20)
1. **Base Linear Velocity ($v_x, v_y, v_z$)**: 3 dims (robot base frame)
2. **Base Angular Velocity ($\omega_x, \omega_y, \omega_z$)**: 3 dims (IMU gyroscope)
3. **Projected Gravity Vector ($g_x, g_y, g_z$)**: 3 dims (encodes roll and pitch orientations)
4. **Steering Joint Position ($\delta$)**: 1 dim
5. **Reaction Plate Velocity ($\dot{\theta}_{plate}$)**: 1 dim (momentum state)
6. **Wheel Angular Velocities ($\dot{\theta}_{front}, \dot{\theta}_{back}$)**: 2 dims
7. **2D Goal Command Vector**: 4 dims (target in robot frame: $[x, y, \sin(\theta_{err}), \cos(\theta_{err})]$)
8. **Previous Actions ($a_{t-1}$)**: 3 dims

#### Velocity Tracking Task (`Isaac-EScooter-Velocity-v0` - Dimension: 19)
- Same kinematic and dynamic feedback as above, with `[v_x_cmd, v_y_cmd, omega_z_cmd]` command terms.

---

## 3. Domain Randomization (Robust Sim-to-Real)

To ensure the learned policy is resilient against model uncertainties and external disturbances, the environment randomizes:

- **Mass Randomization (Empty Scooter to Human Rider: 0 - 75 kg)**:
  - `base_link`: payload added by $[0.0, +70.0]\text{ kg}$
  - `handle_bar_chasis`: arms resting weight added by $[0.0, +6.0]\text{ kg}$
- **Center of Mass (CoM) Shifts (Rider Standing Height & Stance)**:
  - $\Delta x \in [-0.08, +0.08]\text{ m}$ (forward/backward rider stance shift)
  - $\Delta y \in [-0.035, +0.035]\text{ m}$ (lateral rider tilt offset)
  - $\Delta z \in [0.0, +0.45]\text{ m}$ (standing human raises overall system CoM significantly)
- **Joint Dynamics & Rider Arm Biomechanical Damping**:
  - Joint friction scaled by $[0.8, 1.3]$
  - Actuator stiffness $[0.8, 1.3]$ and damping $[0.8, 2.0]$
- **Ground Surface Material**:
  - Static friction: $[0.6, 1.2]$
  - Dynamic friction: $[0.5, 1.0]$
- **External Push Disturbances**:
  - Periodic lateral impulse ($\Delta v_y \in [-0.35, +0.35]\text{ m/s}$) every 8 to 12 seconds to enforce dynamic balance recovery.
- **Initial State Perturbations**:
  - Random initial tilt ($\pm 3^\circ$), random heading, random forward velocity ($0 - 0.5\text{ m/s}$).

---

## 4. Usage Instructions

Make sure the conda environment `env_isaaclab` is activated:
```bash
conda activate env_isaaclab
```

### 1. Test Environment Setup (Headless)
Run a quick 50-step sanity check across 4 parallel environments:
```bash
python src/escooter_rl/scripts/test_env.py --headless --num_envs 4
```

### 2. Launch Massive Parallel Training (RSL-RL PPO)
Train 2048 parallel environments on GPU:

**Goal Navigation Task**:
```bash
python src/escooter_rl/scripts/train.py --task Isaac-EScooter-GoalNav-v0 --num_envs 2048 --headless
```

**Velocity Tracking Task**:
```bash
python src/escooter_rl/scripts/train.py --task Isaac-EScooter-Velocity-v0 --num_envs 2048 --headless
```

To monitor training curves with TensorBoard:
```bash
tensorboard --logdir logs/rsl_rl
```

### 3. Visual Playback & Interactive Evaluation
Visualize the trained policy in Isaac Sim GUI:

- **Auto-load latest checkpoint** (recommended):
```bash
python src/escooter_rl/scripts/play.py --task Isaac-EScooter-GoalNav-Play-v0 --num_envs 16
```

- **Or specify a specific checkpoint**:
```bash
python src/escooter_rl/scripts/play.py --task Isaac-EScooter-GoalNav-Play-v0 --checkpoint logs/rsl_rl/escooter_goal_nav/<TIMESTAMP>/model_<ITER>.pt --num_envs 16
```

### 4. Exporting Policy to ONNX / TorchScript for ROS 2 Deployment

- **Auto-export latest trained policy** (recommended):
```bash
python src/escooter_rl/scripts/export_onnx.py
```
*(Hoặc chọn bài toán cụ thể nếu cần: `python src/escooter_rl/scripts/export_onnx.py --experiment_name escooter_velocity`)*

- **Or specify a specific checkpoint**:
```bash
python src/escooter_rl/scripts/export_onnx.py --checkpoint logs/rsl_rl/escooter_goal_nav/<TIMESTAMP>/model_<ITER>.pt
```

This generates:
- `escooter_policy.onnx`: Ready for ONNXRuntime inference in ROS 2 C++ or Python nodes.
- `escooter_policy.pt`: TorchScript model for PyTorch C++ / LibTorch inference.

---

## 5. ROS 2 Deployment in Gazebo Classic & Ignition Fortress

The trained RL policy is integrated directly into the ROS 2 Humble simulation stack via `escooter_rl_controller.py`, replacing the original PID dynamics controller (`escooter_dynamics`).

### Build the ROS 2 Workspace
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select escooter_simulation escooter_dynamics
source install/setup.bash
```

### Launch Simulation with RL Controller

#### Gazebo Classic 11
```bash
# Auto-loads the latest exported ONNX policy and runs the RL balancing controller
ros2 launch escooter_simulation escooter_sim.launch.py backend:=classic

# Or specify a custom policy path and task:
ros2 launch escooter_simulation escooter_sim.launch.py backend:=classic \
    controller_type:=rl \
    policy_path:=src/escooter_rl/logs/rsl_rl/escooter_velocity/2026-10-30_14-54-24/exported/escooter_policy.onnx
```

#### Ignition Gazebo Fortress
```bash
ros2 launch escooter_simulation escooter_sim.launch.py backend:=ignition
```

### Commanding the Robot

#### 1. Velocity Tracking Task
Send speed and turn rate commands on `/cmd_vel` or `/e_scooter/cmd_vel`:
```bash
# Move forward at 1.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.5}, angular: {z: 0.0}}" -r 10

# Turn while driving
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.2}, angular: {z: 0.5}}" -r 10
```

#### 2. Goal Navigation Task
Send 2D waypoint goals:
```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'odom'}, pose: {position: {x: 5.0, y: 2.0, z: 0.0}}}" --once
```

*(To fall back to the legacy PID dynamics controller at any time, pass `controller_type:=dynamics`).*

