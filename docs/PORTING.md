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

### Bring-up could deadlock

`start_simulation.py` originally sent the activation request first and unpaused
second, to close the gap to a single control cycle. That deadlocks: the
controller manager blocks inside its own `switch_controller` callback waiting
for an update that cannot happen until the world runs, and `/unpause_physics` is
served from the same gzserver process, so neither call is ever answered and the
launch dies with `controller activation timed out`. It unpauses first now. The
gap is one service round trip, tens of milliseconds, in which the scooter
picks up well under 0.1 rad/s from its 1.4 degree spawn lean - far inside what
the plate can arrest.

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
| standing still | +-0.3 deg indefinitely, steering 0.004 deg |
| righting itself after a fall | -87.7 deg to +0.74 deg in 2 s; -33 deg to -1.5 deg in 2 s |
| steering response | immediate, holds the commanded angle |
| driving | **stable for about 1.6 s, then diverges** |

Driving is still open. It fails the same way at every speed tried between 0.25
and 2 m/s: the scooter holds to a few tenths of a degree, then goes from 0.5 to
19 degrees in 400 ms.

Two things were tried against it and are in the tree.

**Ramping the speed command** (`MAX_LINEAR_ACCEL`). `front_wheel_joint` is
velocity controlled, so a step command makes its PID slam the drive torque and
the reaction tips a vehicle balanced on two in-line wheels: stepping straight to
2 m/s took the wheel from 0 to -11.5 rad/s in 200 ms and had the scooter at -67
degrees 200 ms later. Worth having regardless, but it did not fix the
divergence.

**Steer-to-balance** (`STEER_BALANCE_KP` / `KD`, `updateSteering()`), the
textbook answer: steer into the fall so the contact patch is thrown back under
the centre of mass. Steering makes a roll torque of `M*h*v^2*tan(delta)/L`, so
the law asks for `delta = atan(L*tau/(M*h*v^2))`, which scales with speed on its
own, and it is gated below `STEER_BALANCE_MIN_SPEED` because the torque goes
with `v^2` - ungated at 0.25 m/s it pinned the bars at the 20 degree stop for a
one degree lean and made things actively worse.

**It is shipped off (`KP`/`KD` = 0.0) because it did not work.** Gated and
tuned it still does not rescue the scooter at 2 m/s. One PD on the lean angle
mapped through the steering is not enough: lean and steer are coupled states and
want a full-state controller designed on the linearised bicycle, and the
exported trail and head angle should be checked for realism first. The
implementation and its parameters are left in place for that work.

### How much torque the steering actually uses

Worth knowing, because 200 N.m is not a handlebar figure - a rider applies more
like 10 to 20.

Measured on `handle_bar_chasis_joint` while the scooter is genuinely balanced,
standing still and through the first second of driving: **0.0 to 0.6 N.m**. The
large numbers only appear once it has already started going over - 34 N.m at 8
degrees of lean, saturating at 200 N.m while it is on its way down. That is the
position PID chasing a joint the fall is throwing around, not a driving load. So
the limit is headroom rather than something the controller spends.

The stiffness is nevertheless load-bearing, which is the part worth deciding on.
Two softer settings were tried with everything else unchanged:

| steering PID / effort limit | standing still |
| --- | --- |
| p = 400, 200 N.m (shipped) | holds -0.05 deg |
| p = 100, 200 N.m (exported gains) | falls |
| p = 60, 40 N.m (close to the export) | falls |

So the balance that does work depends on a steering column far more rigid than a
human-scale handlebar - in effect it assumes a geared steering servo holding the
column. Whether that matches the real vehicle is a design question; if it does
not, the balance needs to come from somewhere other than a rigid column.

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

## 12. Actuator profiles: matching Gazebo to Isaac Lab

The RL path deploys a policy trained in Isaac Lab against a Gazebo plant that
was tuned for something else entirely. A policy is a frozen function of the
plant it was trained on, so the two have to agree. `actuator_profile` selects
which plant the description is generated against; `launch/robot_description.
launch.py` holds the table.

`auto` follows `controller_type`: `rl` gets `isaac`, everything else gets
`classic`. Override it directly with `actuator_profile:=classic|isaac`.

### What differed

An Isaac `ImplicitActuatorCfg` computes

    tau = clamp(stiffness*(q_d - q) + damping*(w_d - w), +-effort_limit)

with `w_d = 0` for a position or effort action, so its `damping` is a plain
`-damping*w` drag on the joint.

| | Isaac | Gazebo (`classic`) | who was right |
|---|---|---|---|
| steering PD | k 300, d 25 | kp 400, kd 40 | a draw: zeta 0.83 vs 1.14, both near 3.3 Hz on the 0.765 kg.m2 steering inertia |
| steering effort | 40 N.m | 200 N.m | Isaac. 200 N.m is not a handlebar that exists, and it turns the large-signal slew from 92 deg/s into 286 deg/s |
| steering integral | none | ki 20, clamp +-10 | Isaac. A real servo wants one, but the policy re-commands at 50 Hz into an accumulator it never saw |
| plate damping | 0.2 | none | Isaac. `b = kt^2/R` with kt 0.1 N.m/A and R 0.05 ohm is exactly 0.2; a frictionless flywheel is the unphysical one |
| plate speed | 200 rad/s | 400 rad/s | Isaac |
| front wheel | P servo, d 5, effort 80 | kp 10, ki 2, effort unbounded | Isaac |
| back wheel | effort limit 0 | free | same thing either way |

### Where the damping had to go

`gazebo_ros2_control` calls `control_toolbox::Pid::computeCommand(error, dt)`,
which differentiates the **error**. A `pos_kd` there fires a full-scale torque
spike on every new setpoint, 50 times a second. Isaac's `damping` differentiates
the **measurement** and has no setpoint kick. So the `isaac` profile carries the
damping as URDF joint damping, which Gazebo applies at the 1 kHz physics rate,
and leaves the PID with its P term alone.

This also corrects an earlier wrong call. The same damping was once emulated
inside `escooter_rl_controller.py`, subtracting `0.2*w` from the command, and it
made things worse (peak lean 117 deg against 91 deg). That failed because of
where it was applied, not what it was: a velocity-proportional term delayed by
the 20 ms control period is positive feedback. The `plate_damping` parameter is
still there, still 0.0, and must stay that way now the plant provides the term.

### Measured result: it got worse, and that was the useful part

25 s at 1.5 m/s, counting time inside the 0.61 rad envelope that
`fall_orientation` terminates training at:

| | `classic` | `isaac` |
|---|---|---|
| time within 35 deg | 81.7 % | 10.4 % |
| peak \|roll\| | 96.6 deg | 84.3 deg |
| lost the envelope at | - | 2.6 s |

The classical controller is unaffected: `controller_type:=dynamics` still holds
+-0.67 deg, and the `classic` profile's URDF differs from the pre-change one
only by `damping="0.0"` on two joints and an explicit `effort="-1"` on the
front wheel, which is what SDF already defaulted to.

### What the regression exposed

The matched plant is correct; `classic` was masking a policy-side fault with an
over-powered reaction plate. Feeding the exported ONNX a clean synthetic
observation, with no simulator in the loop at all:

All other observations held at the nominal upright, stationary, zero-command
state, `last_action` zero:

| `reaction_plate_vel` | raw steering action |
|---|---|
| 0 rad/s | +0.02 |
| -16 rad/s | +0.98 |
| -22.4 rad/s | +1.40 |
| -75 rad/s | +4.03 |
| -150 rad/s | +7.32 |

The steering head moves about 0.06 action units per rad/s of plate speed, so it
leaves the valid `[-1, 1]` range at roughly 16 rad/s and is clipped to the
0.7 rad action scale from then on. The live `[OBS]` log shows exactly that: the
scooter upright, stationary, zero command, every other observation clean, and
`raw=[+2.08, -0.08, +0.00]` with the steering pinned at its 40 deg limit.

The plate settles at -22 rad/s on its own, because the policy's residual torque
of about -4.7 N.m against the 0.2 damping has its equilibrium at -23.5 rad/s.
That alone is enough to rail the steering.

`debug_obs:=true` turns the observation log on. The remaining gap is on the
training side and is not something the deployment plumbing can close.

## 13. The observation contract between training and deployment

The policy is a frozen function of the observation vector, and nothing in an
ONNX graph records what units that vector is in. This interface has drifted
silently twice: the `joint_vel` pair was swapped, and `drive_wheel_sign` was
inverted. Both times the node kept running and produced plausible nonsense.

So the scaling now travels with the weights:

    escooter_rl/obs_spec.py        the numbers, imported by escooter_env_cfg
        |
        v
    scripts/export_onnx.py         writes obs_spec.json into exported/
        |
        v
    escooter_rl_controller.py      reads it beside the policy file

`obs_spec.py` deliberately has no imports, so `export_onnx.py` can load it by
path without pulling in Isaac Lab; exporting a checkpoint never needs a GPU.

The node falls back to unscaled observations when no `obs_spec.json` is
present, which is how every policy exported before this existed was trained,
and says so in a warning. A re-export of an **old** checkpoint will therefore
pick up a spec describing the **current** `obs_spec.py` - only re-export
checkpoints trained against the config they shipped with.

### What is scaled, and why

| observation | raw range | clip | scale |
|---|---|---|---|
| `reaction_vel` | +-300 rad/s | +-250 | 0.01 |
| `joint_vel` | +-20 rad/s | +-40 | 0.05 |
| everything else | order 1 | - | - |

Clip is in rad/s and applied before scale, matching `ObservationTermCfg`'s
noise -> clip -> scale order. The clip matters more than the scale: it makes
the extrapolation blow-up of section 12 structurally impossible, whatever the
deployed plant does.

### Gate before deploying

`scripts/check_policy_range.py` sweeps an exported policy over the physically
reachable envelope and fails if any raw action leaves `[-1, 1]`, which is the
range the action scales assume. It needs only onnxruntime on the CPU. On the
2026-10-30 checkpoint it fails 8 of 12 probes, worst steering 15.11:

    plate -300..300 rad/s         13.83     3.14     5.30   OUT OF RANGE
    random, 20000 samples         15.11     4.73     7.92   OUT OF RANGE

### Do not enable empirical normalization

`empirical_normalization = True` would handle the scaling automatically, but
`export_onnx.py` rebuilds the network from the `actor.*` keys alone:

```python
actor_weights = {k.replace("actor.", ""): v for k, v in state_dict.items()
                 if k.startswith("actor.")}
```

The normaliser would be dropped without a word, and the exported policy would
be fed raw observations while expecting normalised ones. Either leave it off
and scale explicitly, as above, or fix the exporter first.

## 14. /clock at 10 Hz, and why the RL policy could not balance

`gazebo_ros_init` publishes `/clock` at **10 Hz** by default. Every node running
with `use_sim_time` therefore sees simulated time advance in 100 ms jumps, and a
timer asking for 50 Hz does not get 50 Hz - it fires twice back to back and then
waits 100 ms:

    dt = 0.0000, 0.1000, 0.0000, 0.1000, ...   (expected 0.0200)

The RL policy is trained with `decimation = 4` on `sim.dt = 0.005`, so it
assumes a 20 ms control period. Holding each action for 100 ms delivers five
times the intended impulse per decision. The reaction plate reached its speed
limit inside half a second, the scooter lost its only balancing authority and
went over, every time.

`config/gazebo_params.yaml` sets `publish_rate: 1000.0`, matching the world's
`max_step_size` of 0.001, and `gazebo_classic.launch.py` passes it through
gzserver's `params_file`. Ignition is unaffected: it gets `/clock` from the
ros_gz bridge at the sim rate.

### How it was found

The plate went from 0 to 20 rad/s between two consecutive 50 Hz debug samples.
With `I = 0.12` and a commanded 25 N.m that takes 96 ms, not 20 ms - so the
samples were not 20 ms apart. Logging the control period directly showed the
100 ms quantisation. `debug_obs_rate:=50.0` turns that trace on.

`escooter_dynamics` was quantised exactly the same way. It survived because its
control law is pure state feedback and carries no `dt`; raising the clock rate
leaves it at +-0.65 deg, unchanged.

### Measured, 25 s at 1.5 m/s

| | before | after |
|---|---|---|
| `isaac` profile, time within 35 deg | 3.6 % | **100 %** |
| `isaac` peak \|roll\| | 84.3 deg | 16.2 deg, 21.4 deg over two runs |
| `classic` profile, time within 35 deg | 57.4 % | **100 %** |
| `classic` peak \|roll\| | 94.3 deg | **7.1 deg** |

Forward speed now tracks the command, 1.38 to 1.63 m/s against 1.5.

### What is still wrong

Under the `isaac` profile the reaction plate drifts up over about 20 s and
parks on its 200 rad/s limit, after which it produces no torque at all and the
scooter holds a steady 10 deg lean on steering alone. The plate observation is
then +2.0 after scaling, against the roughly +-0.03 the policy saw in training
(`Episode_Reward/reaction_plate_centering` = -0.0152 means the plate sat near
3 rad/s), so the policy is extrapolating again. `classic` does not hit this: its
400 rad/s ceiling leaves headroom, peak plate speed was 180.7 rad/s and the
torque never saturated.

Raising only `plate_velocity_limit` to 400 inside the `isaac` profile did not
reproduce the `classic` result - that run fell - so the ceiling is not the whole
story and the difference is not yet isolated. That is one run, and not enough to
conclude from.
