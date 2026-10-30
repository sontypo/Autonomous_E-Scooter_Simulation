# ROS 1 Noetic → ROS 2 Humble port

This branch (`ros2-humble`) is a port of the catkin workspace on `main`. The
goal was to keep the existing structure — the same three packages, the same
`Control` / `Dynamics` classes, the same topic names, the same equations of
motion — and change only what ROS 2 and the newer simulators require.

Target configuration, matching this machine:

* ROS 2 Humble
* Gazebo Classic 11.14.0
* Ignition Gazebo Fortress 6.18.0 (Harmonic 8.15.0 is installed too, but the
  Humble `ros_gz_*`, `gz_ros2_control` and `ign_ros2_control` binaries link
  against `ignition-gazebo6`, so Fortress is the supported Ignition version)

The ROS 1 sources remain available:

```bash
git show main:src/escooter_dynamics/src/control.cpp
git show main:src/escooter_simulation/urdf/e-scooter_gazebo_2.urdf
git checkout main -- src/escooter_simulation/urdf       # restore a file
```

---

## 1. What is unchanged

* **The equations of motion.** `Dynamics::updateDynamics`,
  `Dynamics::constraints`, `Dynamics::balancingController` and
  `Dynamics::computeCOMVelocity` are transcribed line for line.
* **The model.** The generated URDF was diffed against
  `urdf/e-scooter_gazebo_2.urdf` term by term: all 8 links and 7 joints have
  identical masses, inertia tensors, inertial origins, joint origins, axes,
  types, parents/children, limits and mesh references.
* **All topic names**, including `/e_scooter/cmd_vel`, `/e_scooter/joint_states`,
  `/e_scooter/odometry/calculated`, `/imu` and `/twist`.
* **All controller names** and **all parameter names** in `escooter.yaml`.
* **The spawn pose defaults**, down to the `Roll` value with its comment.

## 2. Build system

| ROS 1 | ROS 2 |
| --- | --- |
| `catkin_make`, `src/CMakeLists.txt` → noetic toplevel | `colcon build`, no toplevel file |
| `package.xml` format 1/2 | format 3, `<build_type>ament_cmake` |
| `message_generation` / `message_runtime` | `rosidl_default_generators` / `rosidl_default_runtime` |
| `build/`, `devel/`, `.catkin_workspace` committed | removed, `.gitignore` added |

The 934 committed catkin build artefacts were dropped: a stale `build/` with a
noetic `CMakeCache.txt` makes `colcon build` fail outright, because colcon uses
the same directory.

`escooter_msgs` needed no message changes: `RPY` is a valid ROS 2 message name
(`^[A-Z][A-Za-z0-9]*$`), and `escooter_msgs/RPY` resolves from inside the same
package.

## 3. Node and parameters

* `nodelet` was dropped together with `escooter_dynamics.xml`; nothing loaded it.
* `ros::param::get(name, var)` leaves `var` untouched when the key is missing,
  which left `BODY_MAX_ROLL`, `MAX_BODY_ROLL_COMP`, `COMPENSATE_INCREMENT` and
  friends **uninitialised** — they are read from a YAML that never defined them.
  ROS 2 requires a declaration, so they are now declared with explicit defaults
  and written into `escooter.yaml`. They remain unused by the control law.
* ROS 2 parameters cannot hold nested maps, so

  ```yaml
  JOINTS: {'0': {name: handle_bar_chasis_joint, sign: 1}, ...}
  ```

  became two index-aligned arrays with identical ordering:

  ```yaml
  JOINT_NAMES: ["handle_bar_chasis_joint", "rotational_plate_joint",
                "front_wheel_joint", "back_wheel_joint"]
  JOINT_SIGNS: [1, 1, -1, -1]
  ```

  The fixed-size `std::string[3]` / `int[3]` members became `std::vector`s, and
  `NUMBER_OF_JOINTS` is clamped to the number of entries actually supplied
  instead of indexing past the end.
* Topic names, odometry frame ids and an optional odometry TF broadcast are
  parameters now, all defaulting to the ROS 1 behaviour.

## 4. Threading and timing

ROS 1 ran the control loop on the main thread while an `AsyncSpinner(2)`
serviced the callbacks, so the loop and the subscriptions raced on every shared
member (`joints_`, `Vx_`, `delta_`, `body_frame_`, …).

In ROS 2 the same loop body is a **ROS timer on the node clock**, executed by a
single-threaded executor. The call order is identical, the races are gone, and
with `use_sim_time:=true` the loop follows simulation time — it also pauses when
the simulator is paused.

`rclcpp::Time` members are seeded from the node clock and every `dt` goes
through one helper that returns `0.0` on the first call and whenever the clock
has not advanced, so no integrator ever divides by zero. In ROS 1,
`current_time_js_` / `last_time_js_` were never initialised at all.

## 5. Defects fixed during the port

Each of these is marked with a `PORT:` comment at the call site.

1. **Odometry integrated the odometer instead of the increment.**
   `publishOdometry(distance_traveled_, delta_)` receives the *accumulated*
   distance, but the body used it as a per-cycle delta:

   ```cpp
   double delta_psi = (distance_traveled / WHEEL_SEPARATION_DIS) * tan(delta);
   psi_ += delta_psi;
   pose_x_ += distance_traveled * cos(psi_);
   ```

   At 500 Hz the pose grew quadratically and diverged within seconds. The port
   differences the odometer internally; the signature and the published message
   are unchanged.

2. **`jointStateCallback` only ever read one joint.** The loop `break`s inside
   the first `if`, so whichever of `front_wheel_joint` / `handle_bar_chasis_joint`
   came first in the message hid the other. Both are read now.

3. **`computeCOMVelocity` returned `V_x, V_y, V_z`** — a comma operator that
   yields `V_z`. The three components are members and are the real output; the
   function now returns `V_z` explicitly with a comment. (It has no callers.)

4. **Six anonymous `<material name="">` blocks** in the URDF cannot be
   registered by urdfdom. They are declared once as `escooter_orange`,
   `escooter_steel` and `escooter_grey` and referenced by name. The RGBA values
   are unchanged.

What was deliberately **not** "fixed": `Vx_dot_ = Vx_ / dt` in `cmd_velCallback`
is a rate, not an acceleration, and feeds `psi_2dot_`. Changing it would change
the closed-loop behaviour, so it is left exactly as it was.

## 5b. The balancing controller

The PD+ balancing loop did not work on `main` either; the port only made that
visible, because the ROS 1 launch file never ran `escooter_dynamics` (the node
is commented out in `e-scooter_dynamics_model.launch`). Four separate faults,
all fixed on this branch.

### The actuator command was not a torque

`updateDynamics()` ended with

```cpp
theta_dot_ += theta_2dot_ * dt;
joints->rotational_plate = theta_dot_;     // -> effort controller
```

`theta_dot_` is the running integral of the modelled roll acceleration, in
rad/s, and it was written straight into an effort controller that expects N.m.
Nothing reset it, so it ramped without bound: a 60 s run reached -348743 N.m
against a 500 N.m joint limit. The plate now receives `tau`, the torque that
`balancingController()` computes, and the forward model keeps integrating as a
prediction with its state clamped.

### The feedback had the wrong sign

Measured in Gazebo rather than argued from the model: a +40 N.m effort on
`rotational_plate_joint` turned a +1.70 rad/s free-fall roll rate into
-0.32 rad/s, so the plate torque reacts onto the body with the opposite sign.
Righting a positive lean therefore needs a *positive* plate torque, and the
stabilising feedback carries the same sign as the gravity feed-forward.

ROS 1 used `error_p = 0.0 - theta_`, which subtracted it. With
`G_ = M*g*h = 72.7` N.m/rad against `KP = 2.0`, the loop had an effective
stiffness of **+70.7 N.m/rad** - positive feedback. It drove the scooter over
roughly twice as fast as gravity alone.

### The D term was aliased

`theta_dot_imu` was obtained by differencing the roll angle at
`MASTER_LOOP_RATE` = 500 Hz, while the IMU publishes at 100 Hz. Four cycles out
of five the difference is exactly zero and the fifth carries 10 ms of motion
divided by 2 ms, so the derivative is a spike train five times too large. The
loop now uses the gyro reading (`angular_velocity.x`) that the IMU already
provides; the finite difference remains as a fallback.

### The gains were two orders of magnitude too soft

With gravity cancelled the closed loop is `I_b * theta_ddot = -KP*theta -
KD*theta_dot`, and `I_b ~ M * h^2 = 2.905` kg.m^2. ROS 1's `KP: 2.0, KD: 0.05`
give `wn = 0.83` rad/s and `zeta = 0.015`. The branch ships `KP: 186.0,
KD: 42.0`, i.e. `wn = 8` rad/s and `zeta = 0.9`, derived in the comments of
`params/escooter.yaml`.

Also fixed: `4 / 3 * WHOLE_BODY_WEIGHT * ...` is integer division in C++ and
evaluates to `1 *`, making `I_theta` 25 % too small.

### Bring-up order

A balancing vehicle cannot be spawned into a running world and then have its
controllers activated one by one - that left about five seconds in which
nothing held the scooter up, and it was flat on the ground long before the
first command arrived.

The world now starts paused; all four controllers are loaded and configured
with `spawner --inactive` (which works while paused, because only *activation*
needs the controller manager's update loop); and
`escooter_simulation/scripts/start_simulation.py` sends the activation request
first and unpauses second, so the pending switch is carried out on the very
first physics step. The loop is live within one control cycle.
`paused_model:=true` skips the release and leaves the model for inspection.

### The reaction plate was undersized

The SolidWorks export gave the plate `izz = 0.002376` kg.m^2 behind a 200 rad/s
speed limit, so it stored `0.475` N.m.s of angular momentum. A reaction wheel
can do no more than trade that momentum with the body, which bounded the whole
design:

| quantity | exported plate | this branch |
| --- | --- | --- |
| spin inertia `izz` | 0.002376 kg.m^2 | **0.06 kg.m^2** |
| stored momentum at 200 rad/s | 0.475 N.m.s | 12.0 N.m.s |
| body roll rate it can arrest | 0.164 rad/s (9.4 deg/s) | 4.13 rad/s (237 deg/s) |
| lean it can right from rest | 1.87 deg | 49 deg |

1.87 degrees is less than the scooter picks up while settling onto the ground,
so with the exported plate both simulators lost it inside half a second however
the controller was tuned.

The default keeps the plate's 1.09 kg and redistributes it as a 23 cm rim
(`izz = m*r^2 = 0.06`, transverse `= izz/2 = 0.03`). Vehicle mass is unchanged,
so `WHOLE_BODY_WEIGHT` and the PD gains are unaffected. The visual mesh is still
the small exported disc - it is geometry for display only, and redrawing it is a
CAD job.

Everything is a xacro argument, so the exported figures are one flag away:

```bash
ros2 launch escooter_simulation escooter_sim.launch.py \
    plate_spin_inertia:=0.002376 plate_transverse_inertia:=0.0014131
```

`plate_effort_limit` (500 N.m) and `plate_velocity_limit` (200 rad/s) are
arguments too.

### Momentum management

A bigger flywheel alone was not enough. The chassis centre of mass sits a few
millimetres off the centreline, so gravity applies a constant roll torque of
about 0.5 N.m. Holding `theta` at exactly zero makes the plate absorb that
torque forever: measured drift was 7.6 rad/s per second, the plate saturated at
about t = 26 s and the scooter fell.

`MOMENTUM_UNLOAD_GAIN` biases the upright setpoint by the plate speed,
`theta_ref = -K * omega_plate`, so the body holds a fraction of a degree of lean
that cancels the bias and the plate settles instead of winding up. No steering
is involved. The sign comes from the equilibrium of the two bodies - a lean
`theta_ref` accelerates the plate at `G*sin(theta_ref)/I_w`, so bleeding a
positive plate speed needs a negative lean - and getting it backwards is not
subtle: the scooter was over at 43 degrees within one second.

The plate settles at `omega = -(1 + G/KP) * theta_bias / K`, which is why the
gain matters: `1e-4` settles at -176 rad/s, right on the limit, while `5e-4`
settles near -35 rad/s. The unloading loop's time constant, `I_w/(G*K) ~ 1.7` s,
stays well clear of the 0.25 s balancing loop.

### Measured result

Both backends, headless, with everything above in place:

| | Gazebo Classic | Ignition Fortress |
| --- | --- | --- |
| steady roll | +0.62 deg | +0.78 deg |
| peak excursion | 1.46 deg | 0.79 deg |
| plate speed | settles at -55 rad/s | settles at -70 rad/s |
| run length | 70 s, still upright | 60 s, still upright |

For comparison, every configuration before these fixes was flat on the ground
at -55 degrees within about 1.5 s.


## 5c. Teleoperation: steering and driving

Three more faults showed up once the scooter was driven rather than just left to
stand.

### Steering did not respond

Not the URDF - the joint frames are right. `handle_bar_chasis_joint`'s rpy
(1.5481, 0, 0) takes its axis (0, 1, 0) to (0, 0.023, 0.9997), a near-vertical
steering axis with a little rake, and `front_wheel_joint` resolves to
(0, -0.9997, 0.023), a lateral spin axis. Both correct.

The fault was in `cmd_velCallback`, next to the author's own
`TODO: it should be delta`: `des_delta += angular.z * dt` integrates
`angular.z`. Teleop publishes a held value, so one key press ramped the bars to
the 45 degree stop over about 8 s and left them there; releasing to
`angular.z = 0` froze them hard over instead of centring, and reversing meant
waiting out the same ramp backwards.

`STEERING_MODE` now defaults to `yaw_rate`, the ROS convention for `cmd_vel` on
a bicycle: `delta = atan(L * omega / v)`, so `omega = 0` centres the bars at
once. `angle` takes `angular.z` as the steering angle directly, and `rate`
restores the ROS 1 integrator. `MAX_STEERING_RATE` slews the result so the
balancing loop can absorb the roll disturbance.

### A fictitious acceleration

`Vx_dot_ = Vx_ / dt` is a speed divided by a sample interval. At a steady 2 m/s
with cmd_vel at 20 Hz it reports **40 m/s^2**, and `constraints()` multiplies it
into `psi_2dot_`, so the balancing feed-forward picked up a phantom 114 N.m at
28 degrees of steering. That is what made the scooter lurch through +-87 degrees
the moment it was driven and steered together. It is now
`(Vx_ - prev_Vx_) / dt`.

This was deliberately left alone during the port, as a modelling choice rather
than a defect. That call was wrong: it is the dominant disturbance under teleop.

### Commanded speed was never converted

`joints_.front_wheel = Vx_` sent a m/s figure to a joint that takes rad/s - the
author's `TODO: need to convert to joint command later`. The scooter moved at
`v * WHEEL_RADIUS`, 7.4 times slower than asked, while the dynamics model read
the same number as a speed and overestimated the yaw rate by that factor.
`WHEEL_RATE_FROM_SPEED` (default true) divides by the wheel radius.

### The steering joint was too weak to hold its command

The export gave `handle_bar_chasis_joint` an effort limit of 40 N.m. Against the
position PID's p = 100 that saturates at 23 degrees of error, so once the
scooter leaned, the bicycle's own trail back-drove the bars past the 60 degree
joint stop - measured at +58.5 degrees while the command was 0 - which turned
the scooter and pulled it over. `handlebar_effort_limit` defaults to 200 N.m and
the PID to p = 400; the bars now hold the commanded angle to 0.002 degrees.

### Where it stands

| | result |
| --- | --- |
| standing still | +-0.3 deg indefinitely, steering 0.002 deg |
| righting itself after a fall | -87.7 deg to +0.74 deg in 2 s |
| steering response | immediate, holds the commanded angle |
| driving | **stable for about 1.6 s, then diverges** |

Driving is still open. It fails the same way at every speed tried between 0.25
and 2 m/s: the scooter holds to a few tenths of a degree, then goes from 0.5 to
19 degrees in 400 ms. A reaction wheel fights the lean-steer coupling of a
rolling bicycle with a fixed momentum budget, and it loses. The standard answer
is steer-to-balance - steering into the fall, which is how bicycles actually
stay up - using the handle bar the balancing loop currently leaves to the
operator. That is a new controller, not a tuning change.

Gains are deliberately gentle for the same budget reason: wn = 5 rad/s spent the
whole flywheel on the settling impulse at spawn and saturated it, after which
the loop is open. wn = 4 rides the same impulse out inside +-60 rad/s.


## 6. Controllers

ros_control → ros2_control, keeping the controller names:

| Joint | ROS 1 | ROS 2 |
| --- | --- | --- |
| `handle_bar_chasis_joint` | `effort_controllers/JointPositionController`, pid `{p: 100, i: 10, d: 20}` | `position_controllers/JointGroupPositionController` over a **`position_pid`** command interface with `pos_kp/pos_ki/pos_kd = 100/10/20` |
| `rotational_plate_joint` | `effort_controllers/JointEffortController` | `effort_controllers/JointGroupEffortController` over an `effort` interface |
| `front_wheel_joint` | `velocity_controllers/JointVelocityController` + `gazebo_ros_control/pid_gains {p: 10, i: 2, d: 0.1}` | `velocity_controllers/JointGroupVelocityController` over a **`velocity_pid`** interface with `vel_kp/vel_ki/vel_kd = 10/2/0.1` |
| `back_wheel_joint` | passive, absent from `/joint_states` | passive, but declared state-only so it appears in `/joint_states` and TF |
| all | `joint_state_controller/JointStateController` | `joint_state_broadcaster/JointStateBroadcaster` |

`gazebo_ros2_control`'s `position_pid` / `velocity_pid` interfaces are the exact
analogue of the ROS 1 arrangement: the simulator closes a PID loop with those
gains and writes an effort, and the gains are the ones from
`e-scooter_control.yaml`. Start-up logs `Found position PIDs for joint:
handle_bar_chasis_joint!` and `Found velocity PIDs for joint: front_wheel_joint!`
when they are picked up.

The ROS 1 `JointEffortController` ignored its `pid` key — it is a pass-through
forward controller — so the rotational plate uses a plain `effort` interface and
the gains are intentionally not carried over.

Ignition's hardware component has no PID-backed interfaces. There the handle bar
uses a plain `position` interface with `position_proportional_gain`, and the
front wheel a plain `velocity` interface.

The command message changed because ros2_control's forward controllers use it:
`std_msgs/Float64` on `<controller>/command` became
`std_msgs/Float64MultiArray` on `<controller>/commands`.

## 7. Description

`e-scooter_gazebo_2.urdf` became `urdf/e-scooter.urdf.xacro`, split into

* `e-scooter.urdf.xacro` — links, joints, materials (shared by every target)
* `e-scooter.ros2_control.xacro` — the `<ros2_control>` block per backend
* `e-scooter.gazebo.xacro` — materials, IMU sensor and system plugin per backend

selected with `sim:=classic|ignition|none`. `none` emits a plain URDF with no
simulator tags, which is what `display.launch.py` uses.

`e-scooter_gazebo.urdf` and `test.urdf` were removed: both referenced
`new_meshes/`, which is not tracked, so they were already broken on a fresh
clone.

The IMU noise block was rewritten from the sdformat 1.4 `<noise><rate>/<accel>`
form into the modern `<angular_velocity><x><noise>` schema that both Gazebo 11
and Fortress parse. The numbers are unchanged.

## 8. Two gotchas worth knowing

**The description must be single-line and comment-free.** Both system plugins
hand the URDF to the controller manager as `--param robot_description:=<urdf>`,
and rcl parses everything after `:=` as a standalone YAML document. A
pretty-printed URDF is not a valid YAML scalar — every line after the first
starts at column 0, and xacro's comments contain `: ` and ` #`. The plugin then
fails with

```
parser error Couldn't parse parameter override rule: '--param robot_description:=...
```

and the controller manager never starts. `escooter_simulation/scripts/xacro_to_urdf.py`
runs xacro, strips the comments and emits one line.

**`robot_state_publisher` runs inside `/e_scooter`.** Both plugins resolve
`robot_param_node` relative to their own namespace, and the Ignition one
concatenates the two by hand, so an absolute `/robot_state_publisher` becomes
`/e_scooter//robot_state_publisher` and throws `InvalidServiceNameError`.
Putting robot_state_publisher in the robot namespace makes the plain relative
default work for both. TF is unaffected — `tf2_ros`' broadcasters publish on the
absolute `/tf` and `/tf_static` whatever the node's namespace — but the
description topic is now `/e_scooter/robot_description`.

Two smaller ones: `gzserver.launch.py` declares a `params_file` argument with an
empty default, and launch configurations set by an unscoped include leak into
the shared context, blanking the parameter file of every node launched
afterwards — hence the `GroupAction(scoped=True)` wrappers. And `spawn_entity.py`
waits only 30 s for `/spawn_entity` by default, which Gazebo Classic can exceed
on a cold start, so `spawn_timeout` defaults to 120 s here.

## 9. Scripts

| ROS 1 (repository root) | ROS 2 |
| --- | --- |
| `teleop.py` | `ros2 run escooter_dynamics teleop_escooter.py` |
| `cmd_vel-keyboard.py` | `ros2 run escooter_dynamics keyboard_cmd_vel.py` (needs `pynput`) |
| `euler_angle.py` | `ros2 run escooter_dynamics imu_to_euler.py` |

`tf.transformations` does not exist in ROS 2, so `imu_to_euler.py` uses
`tf_transformations` when available and falls back to a local conversion in the
same ZYX convention as `tf2::Matrix3x3::getRPY`.

## 10. Packages left out of the port

`colcon` skips these through a `COLCON_IGNORE` file that explains why; delete
the file once a package is ported.

* `src/pid_controller` — an empty catkin skeleton with no sources.
* `src/realsense_gazebo_plugin` — a vendored third-party ROS 1 Gazebo plugin.
  The e-scooter description references no RealSense sensor.
* `src/escooter_simulation/urdf_escooter_new` — a raw SolidWorks export kept for
  reference.

`src/autonomous-scooter`, `src/gazebo-ros-actor-plugin` and
`src/thormang3-rides-gogoro` are recorded in git as submodule gitlinks but the
repository has no `.gitmodules`, so they are empty directories. They are left
untouched.

## 11. How the port was verified

Both backends were brought up headless and exercised end to end:

* all four controllers reach `active`, and Classic logs that it picked up both
  ported PID gain sets;
* `/clock`, `/imu`, `/e_scooter/joint_states`, `/e_scooter/odometry/calculated`,
  `/twist` and `/tf` all publish;
* `cmd_vel {linear.x: 2.0, angular.z: 0.2}` produces a front-wheel command of
  `-2.0` (the `JOINT_SIGNS` entry applied, as in ROS 1) and a measured wheel
  velocity of `-2.0` rad/s;
* `angular.z` integrates into the steering angle and clamps at
  `MAX_STEERING_ANGLE = 0.785398163`, which both simulators then track;
* the generated URDF was diffed term by term against the ROS 1 original.
