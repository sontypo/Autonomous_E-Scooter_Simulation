#!/usr/bin/env python3
# Copyright (c) 2026, Autonomous E-Scooter Simulation Project.
# All rights reserved.

"""ROS 2 Node: Real-time RL inference controller for Autonomous E-Scooter.

Deploys trained Isaac Lab / RSL-RL policies (ONNX or TorchScript) to control
the e-scooter in Gazebo Classic, Ignition Fortress, or on real hardware.
Replaces the classical PID dynamics controller (escooter_dynamics).
"""

import json
import math
import os
import re
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


def find_workspace_root(start: str) -> str:
    """Walk up from `start` looking for the colcon workspace that holds escooter_rl."""
    here = os.path.abspath(start)
    while True:
        if os.path.isdir(os.path.join(here, "src", "escooter_rl")):
            return here
        parent = os.path.dirname(here)
        if parent == here:
            return os.path.abspath(start)
        here = parent


def find_latest_exported_policy(repo_root: str, task_preference: str = "velocity") -> str:
    """Find the latest exported policy file (.onnx or .pt) from logs/rsl_rl.

    Prefers files under an `exported/` directory: those are the TorchScript and
    ONNX artefacts that export_onnx.py writes. The `model_*.pt` files sitting
    next to them are RSL-RL training checkpoints - plain state dictionaries that
    torch.jit.load cannot open.
    """
    search_paths = [
        os.path.join(repo_root, "src", "escooter_rl", "logs", "rsl_rl", f"escooter_{task_preference}"),
        os.path.join(repo_root, "src", "escooter_rl", "logs", "rsl_rl"),
        os.path.join(repo_root, "logs", "rsl_rl", f"escooter_{task_preference}"),
        os.path.join(repo_root, "logs", "rsl_rl"),
    ]
    candidates = []
    for log_base in search_paths:
        if not os.path.exists(log_base):
            continue
        for root, dirs, files in os.walk(log_base):
            for f in files:
                if f.endswith(".onnx") or f.endswith(".pt"):
                    full_path = os.path.join(root, f)
                    if not os.path.isfile(full_path):
                        continue
                    exported = os.path.basename(root) == "exported"
                    onnx = f.endswith(".onnx")
                    candidates.append(
                        (exported, onnx, os.path.getmtime(full_path), full_path)
                    )

    if not candidates:
        return None

    candidates.sort()
    return candidates[-1][-1]


class EScooterRLController(Node):
    def __init__(self):
        super().__init__("escooter_rl_controller")

        # Declare parameters
        self.declare_parameter("policy_path", "")
        self.declare_parameter("task_type", "auto")      # "auto", "velocity", or "goal_nav"
        self.declare_parameter("namespace", "/e_scooter")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("control_rate", 50.0)      # 50 Hz matching Isaac Lab 4-step decimation
        self.declare_parameter("steering_scale", 0.7)     # radians (action[0] scale)
        self.declare_parameter("reaction_plate_scale", 60.0) # N*m (action[1] scale)
        self.declare_parameter("drive_wheel_scale", 20.0)    # rad/s (action[2] scale)
        # +1.0, not -1.0. Isaac and Gazebo are driven from the same URDF, so
        # front_wheel_joint turns the same way in both and the action needs no
        # flip. The policy already outputs negative to go forward, because the
        # joint axis resolves to about (0, -0.9997, 0.023) in the base frame:
        # positive joint rotation drives the scooter backwards. Flipping it here
        # made the scooter reverse, which pulls the steered wheel the wrong way
        # and loses balance immediately. (escooter_dynamics does need -1, but
        # that is its JOINT_SIGNS convention and this node bypasses it.)
        self.declare_parameter("drive_wheel_sign", 1.0)
        self.declare_parameter("wheel_radius", 0.135)        # meters
        self.declare_parameter("wheelbase", 1.0)             # meters
        # Geometry for the lateral velocity estimate, from the URDF:
        # back_wheel_joint sits 0.50339 m behind the base_link origin, which is
        # 0.197 m above the ground (OG_POS_Z in escooter_dynamics).
        self.declare_parameter("base_to_rear_axle", 0.50339)  # meters
        self.declare_parameter("base_height", 0.197)          # meters
        # Isaac Lab resolves SceneEntityCfg joint names in articulation order
        # unless preserve_order=True. This articulation orders them
        # back_wheel(0), handle_bar(1), front_wheel(2), rotational_plate(3), so
        # the joint_vel observation is [back_wheel, front_wheel]. Set this to
        # "declared" only if the training config sets preserve_order=True.
        self.declare_parameter("joint_vel_order", "articulation")
        # Topic to take base velocity from, when something else provides it.
        # Empty means estimate it here. It must never be the topic this node
        # publishes on, or it listens to its own output.
        self.declare_parameter("odom_topic", "")
        # Isaac Lab's ImplicitActuator for the reaction plate applies
        #     tau = stiffness*(q_des - q) + damping*(qd_des - qd) + effort_target
        # with stiffness 0 and damping 0.2, so the plate always felt an extra
        # -0.2*omega on top of the policy's torque - 30 N.m at the 150 rad/s the
        # flywheel actually runs at, half the 60 N.m action scale.
        #
        # LEAVE THIS AT 0.0. Subtracting the term here was measured to make
        # things worse, not better: peak lean went from 91 to 117 degrees. A
        # velocity-proportional term delayed by the 20 ms control period is
        # positive feedback, which is not what the plant does. The damping now
        # lives where it belongs, as <dynamics damping> on rotational_plate_joint
        # under the "isaac" actuator profile, so Gazebo applies it at the 1 kHz
        # physics rate. Setting this non-zero on top of that double counts it.
        self.declare_parameter("plate_damping", 0.0)   # N.m.s/rad
        # Log the full observation vector and the UNCLIPPED policy output at
        # debug_obs_rate Hz. The policy's steering head is close to unbounded in
        # the reaction_plate_vel input, so a raw action far outside [-1, 1] is
        # the signature of an observation the policy never saw in training.
        self.declare_parameter("debug_obs", False)
        self.declare_parameter("debug_obs_rate", 2.0)        # Hz
        self.declare_parameter("publish_odom", True)         # Publish dead-reckoning odometry
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")

        self.policy_path = self.get_parameter("policy_path").get_parameter_value().string_value
        self.task_type = self.get_parameter("task_type").get_parameter_value().string_value
        self.namespace = self.get_parameter("namespace").get_parameter_value().string_value
        self.imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        self.control_rate = self.get_parameter("control_rate").get_parameter_value().double_value

        self.steering_scale = self.get_parameter("steering_scale").get_parameter_value().double_value
        self.reaction_plate_scale = self.get_parameter("reaction_plate_scale").get_parameter_value().double_value
        self.drive_wheel_scale = self.get_parameter("drive_wheel_scale").get_parameter_value().double_value
        self.drive_wheel_sign = self.get_parameter("drive_wheel_sign").get_parameter_value().double_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheelbase = self.get_parameter("wheelbase").get_parameter_value().double_value
        self.base_to_rear_axle = self.get_parameter(
            "base_to_rear_axle").get_parameter_value().double_value
        self.base_height = self.get_parameter("base_height").get_parameter_value().double_value
        self.joint_vel_order = self.get_parameter(
            "joint_vel_order").get_parameter_value().string_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.plate_damping = self.get_parameter("plate_damping").get_parameter_value().double_value
        self.debug_obs = self.get_parameter("debug_obs").get_parameter_value().bool_value
        self.debug_obs_rate = self.get_parameter("debug_obs_rate").get_parameter_value().double_value
        self._debug_every = max(1, int(round(
            self.control_rate / max(self.debug_obs_rate, 1e-3))))
        self._debug_tick = 0
        self.publish_odom = self.get_parameter("publish_odom").get_parameter_value().bool_value
        self.odom_frame_id = self.get_parameter("odom_frame_id").get_parameter_value().string_value
        self.base_frame_id = self.get_parameter("base_frame_id").get_parameter_value().string_value

        # Auto-detect policy if not specified
        if not self.policy_path or not os.path.exists(self.policy_path):
            # realpath, not abspath: with `colcon build --symlink-install` this
            # script is reached through a symlink in install/, and abspath keeps
            # that path, so the search started at <ws>/install and never saw
            # src/escooter_rl. The node then silently ran with no policy at all
            # and published nothing.
            script_dir = os.path.dirname(os.path.realpath(__file__))
            repo_root = find_workspace_root(script_dir)
            pref = "velocity" if self.task_type in ["auto", "velocity"] else "goal_nav"
            detected_policy = find_latest_exported_policy(repo_root, pref)
            if detected_policy:
                self.get_logger().info(f"[RL] Auto-detected policy: {detected_policy}")
                self.policy_path = detected_policy
            else:
                self.get_logger().error(
                    f"[RL] No exported policy found under {repo_root}. Run "
                    "scripts/export_onnx.py, or pass policy_path:=<file>. "
                    "Retrying every 5 s; the scooter is uncontrolled until then.")
                self._retry_root = repo_root
                self._retry_pref = pref
                self.create_timer(5.0, self._retry_load_policy)

        # Observation scaling, read from obs_spec.json next to the policy.
        # Defaults are "raw", which is how every policy exported before the
        # spec existed was trained, so an old checkpoint keeps working.
        self.plate_vel_clip = None
        self.plate_vel_scale = 1.0
        self.wheel_vel_clip = None
        self.wheel_vel_scale = 1.0
        self.obs_spec = None
        if self.policy_path:
            self._load_obs_spec(self.policy_path)

        # Load Policy Inference Engine
        self.inference_engine = None
        self.is_onnx = False
        self.expected_obs_dim = 19
        if self.policy_path and os.path.exists(self.policy_path):
            self._load_policy(self.policy_path)

        # Internal Robot State
        self.base_lin_vel = np.zeros(3, dtype=np.float32)
        self.base_ang_vel = np.zeros(3, dtype=np.float32)
        self.projected_gravity = np.array([0.0, 0.0, -1.0], dtype=np.float32)

        self.steering_pos = 0.0
        self.steering_vel = 0.0
        self.reaction_plate_vel = 0.0
        self.front_wheel_pos = 0.0
        self.front_wheel_vel = 0.0
        self.back_wheel_pos = 0.0
        self.back_wheel_vel = 0.0

        # Odometry tracking
        self.prev_front_wheel_pos = None
        self.prev_odom_time = None
        self.has_external_odom = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Goal & Commands
        self.goal_x = 2.0
        self.goal_y = 0.0
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0

        # Action history (a_{t-1}). Isaac Lab's last_action observation is the
        # raw policy output, so this is stored before clipping.
        self.last_action = np.zeros(3, dtype=np.float32)

        # The control loop must not run on an all-zero observation.
        self.got_imu = False
        self.got_joints = False

        # Setup ROS 2 Subscribers
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self._imu_callback, sensor_qos
        )
        self.joint_sub = self.create_subscription(
            JointState, f"{self.namespace}/joint_states", self._joint_callback, sensor_qos
        )
        # Only subscribe when another source is explicitly configured. The node
        # publishes odometry on {namespace}/odometry/calculated, and subscribing
        # to that made it consume its own output: the first message set
        # has_external_odom, which permanently stopped the dead-reckoning
        # integration and let base_lin_vel be overwritten by its own estimate.
        self.odom_pub_topic = f"{self.namespace}/odometry/calculated"
        self.odom_sub = None
        if self.odom_topic and self.odom_topic != self.odom_pub_topic:
            self.odom_sub = self.create_subscription(
                Odometry, self.odom_topic, self._odom_callback, 10
            )
            self.get_logger().info(f"[RL] Taking base velocity from '{self.odom_topic}'")
        self.cmd_vel_sub = self.create_subscription(
            Twist, f"{self.namespace}/cmd_vel", self._cmd_vel_callback, 10
        )
        self.cmd_vel_global_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self._goal_callback, 10
        )

        # Setup ROS 2 Publishers (Commands to ros2_control hardware)
        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            f"{self.namespace}/handle_bar_chasis_joint_position_controller/commands",
            10,
        )
        self.reaction_plate_pub = self.create_publisher(
            Float64MultiArray,
            f"{self.namespace}/rotational_plate_joint_effort_controller/commands",
            10,
        )
        self.front_wheel_pub = self.create_publisher(
            Float64MultiArray,
            f"{self.namespace}/front_wheel_joint_velocity_controller/commands",
            10,
        )
        self.odom_pub = self.create_publisher(Odometry, self.odom_pub_topic, 10)

        # Main Control Loop Timer
        timer_period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(timer_period, self._control_loop)
        self.get_logger().info(
            f"[RL] EScooter RL Controller initialized at {self.control_rate} Hz (Task: {self.task_type})"
        )

    def _retry_load_policy(self):
        if self.inference_engine is not None:
            return
        found = find_latest_exported_policy(self._retry_root, self._retry_pref)
        if found:
            self.get_logger().info(f"[RL] Policy appeared: {found}")
            self.policy_path = found
            self._load_obs_spec(found)
            self._load_policy(found)

    def _load_policy(self, path: str):
        try:
            if path.endswith(".onnx"):
                import onnxruntime as ort
                self.session = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
                input_meta = self.session.get_inputs()[0]
                self.input_name = input_meta.name
                obs_dim = input_meta.shape[-1]
                self.expected_obs_dim = obs_dim
                self.is_onnx = True
                self.get_logger().info(
                    f"[RL] Loaded ONNX policy: {path} (Input: {self.input_name}, Obs Dim: {obs_dim})"
                )
            else:
                import torch
                self.model = torch.jit.load(path, map_location="cpu")
                self.model.eval()
                self.is_onnx = False
                self.get_logger().info(f"[RL] Loaded TorchScript policy: {path}")

            # Auto-configure task_type based on observation dimension
            if self.task_type == "auto":
                if self.expected_obs_dim == 20:
                    self.task_type = "goal_nav"
                else:
                    self.task_type = "velocity"
                self.get_logger().info(f"[RL] Auto-configured task_type -> '{self.task_type}'")

            self.inference_engine = True
        except Exception as e:
            self.get_logger().error(f"[RL] Failed to load policy: {e}")
            self.inference_engine = None

    def _imu_callback(self, msg: Imu):
        self.got_imu = True
        # Angular velocity in body frame
        self.base_ang_vel[0] = msg.angular_velocity.x
        self.base_ang_vel[1] = msg.angular_velocity.y
        self.base_ang_vel[2] = msg.angular_velocity.z

        # Orientation quaternion: x, y, z, w
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Compute projected gravity in base frame: g_base = R(q)^T * [0, 0, -1]
        gx = -2.0 * (qx * qz - qw * qy)
        gy = -2.0 * (qy * qz + qw * qx)
        gz = -(1.0 - 2.0 * (qx * qx + qy * qy))
        self.projected_gravity = np.array([gx, gy, gz], dtype=np.float32)

        # Update robot yaw for dead-reckoning navigation
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _joint_callback(self, msg: JointState):
        self.got_joints = True
        for i, name in enumerate(msg.name):
            if name == "handle_bar_chasis_joint":
                self.steering_pos = msg.position[i]
                if len(msg.velocity) > i:
                    self.steering_vel = msg.velocity[i]
            elif name == "rotational_plate_joint":
                if len(msg.velocity) > i:
                    self.reaction_plate_vel = msg.velocity[i]
            elif name == "front_wheel_joint":
                self.front_wheel_pos = msg.position[i]
                if len(msg.velocity) > i:
                    self.front_wheel_vel = msg.velocity[i]
            elif name == "back_wheel_joint":
                self.back_wheel_pos = msg.position[i]
                if len(msg.velocity) > i:
                    self.back_wheel_vel = msg.velocity[i]

        if not self.has_external_odom:
            self._update_base_velocity()

        now = self.get_clock().now()
        if self.prev_odom_time is not None and not self.has_external_odom:
            dt = (now - self.prev_odom_time).nanoseconds * 1e-9
            if dt > 0.0:
                delta_s = float(self.base_lin_vel[0]) * dt
                self.robot_x += delta_s * math.cos(self.robot_yaw)
                self.robot_y += delta_s * math.sin(self.robot_yaw)
                if self.publish_odom:
                    self._publish_odometry(now)
        self.prev_odom_time = now

    def _update_base_velocity(self):
        """Estimate root_lin_vel_b, the quantity Isaac Lab feeds the policy.

        Isaac hands the policy the full three-component base-frame linear
        velocity straight from the physics engine. Only the forward component
        was being filled in here and the other two were left at zero, which
        matters: sweeping the lateral component from -0.6 to +0.6 m/s moves the
        reaction-plate action by 1.28, about 77 N.m on the 60 N.m action scale.
        It is the strongest balancing input after gravity, and the policy was
        being told it was always exactly zero.

        Forward comes from the driven wheel. Negative joint rotation is forward
        (the axis resolves to about (0, -0.9997, 0.023) in the base frame), and
        the wheel is steered, so only its along-heading component is forward
        motion of the body.

        Lateral comes from the rigid-body kinematics of the base origin, using
        the gyro directly rather than any integration:

            v_y = a * omega_z - h * omega_x

        where `a` is how far the origin sits ahead of the rear contact and `h`
        how far above the ground. The first term is the yaw of a point ahead of
        the rear axle, the second the swing of a point at height h about the
        contact line as the scooter rolls.
        """
        v_wheel = -self.front_wheel_vel * self.wheel_radius
        self.base_lin_vel[0] = float(v_wheel * math.cos(self.steering_pos))
        self.base_lin_vel[1] = float(
            self.base_to_rear_axle * self.base_ang_vel[2]
            - self.base_height * self.base_ang_vel[0]
        )
        self.base_lin_vel[2] = 0.0

    def _publish_odometry(self, now):
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = float(self.robot_x)
        odom.pose.pose.position.y = float(self.robot_y)
        odom.pose.pose.position.z = 0.0
        half_yaw = float(self.robot_yaw * 0.5)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = float(math.sin(half_yaw))
        odom.pose.pose.orientation.w = float(math.cos(half_yaw))
        odom.twist.twist.linear.x = float(self.base_lin_vel[0])
        odom.twist.twist.linear.y = float(self.base_lin_vel[1])
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = float(self.base_ang_vel[0])
        odom.twist.twist.angular.y = float(self.base_ang_vel[1])
        odom.twist.twist.angular.z = float(self.base_ang_vel[2])
        self.odom_pub.publish(odom)

    def _odom_callback(self, msg: Odometry):
        # Mark that external odometry is active
        self.has_external_odom = True
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Base linear velocity (forward)
        self.base_lin_vel[0] = msg.twist.twist.linear.x
        self.base_lin_vel[1] = msg.twist.twist.linear.y
        self.base_lin_vel[2] = msg.twist.twist.linear.z

    def _cmd_vel_callback(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z

    def _goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info(f"[RL] New target goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def _load_obs_spec(self, policy_path: str):
        """Read the observation spec the exporter wrote beside the weights.

        The reaction plate and the wheels are scaled and clipped in training
        (escooter_rl/obs_spec.py), and the policy is only valid on observations
        prepared the same way. Nothing in the ONNX graph records that, and the
        two sides of this interface have silently drifted apart twice already,
        so the scaling travels with the weights rather than being retyped here.
        """
        spec_path = os.path.join(os.path.dirname(policy_path), "obs_spec.json")
        if not os.path.exists(spec_path):
            self.get_logger().warn(
                "[RL] No obs_spec.json beside %s. Falling back to unscaled "
                "observations, which is correct only for a policy trained "
                "before the spec existed. Re-export with scripts/export_onnx.py "
                "to remove the guesswork." % os.path.basename(policy_path))
            return

        try:
            with open(spec_path) as handle:
                spec = json.load(handle)
        except Exception as exc:
            self.get_logger().error(
                "[RL] Could not read %s: %s. Using unscaled observations."
                % (spec_path, exc))
            return

        terms = {t["name"]: t for t in spec.get("terms", [])}

        reaction = terms.get("reaction_vel", {})
        if "clip" in reaction:
            self.plate_vel_clip = tuple(reaction["clip"])
        self.plate_vel_scale = float(reaction.get("scale", 1.0))

        wheels = terms.get("joint_vel", {})
        if "clip" in wheels:
            self.wheel_vel_clip = tuple(wheels["clip"])
        self.wheel_vel_scale = float(wheels.get("scale", 1.0))
        order = wheels.get("joint_order")
        if order:
            # The spec is authoritative over the joint_vel_order parameter.
            self.joint_vel_order = (
                "declared" if order[0] == "front_wheel_joint" else "articulation")

        task = spec.get("task")
        if task and self.task_type == "auto":
            self.task_type = task

        self.obs_spec = spec
        self.get_logger().info(
            "[RL] obs_spec.json: task=%s obs_dim=%s plate(clip=%s scale=%g) "
            "wheels(clip=%s scale=%g) order=%s"
            % (task, spec.get("obs_dim"), self.plate_vel_clip,
               self.plate_vel_scale, self.wheel_vel_clip, self.wheel_vel_scale,
               order))

    def _scaled(self, value, clip, scale):
        """Clip then scale, matching ObservationTermCfg's noise -> clip -> scale."""
        if clip is not None:
            value = min(max(value, clip[0]), clip[1])
        return value * scale

    def _joint_vel_obs(self) -> np.ndarray:
        """The two wheel velocities, in the order Isaac Lab actually produced.

        The training config asks for ["front_wheel_joint", "back_wheel_joint"],
        but SceneEntityCfg defaults to preserve_order=False, so Isaac Lab
        resolves them in articulation order instead. This articulation is
        ordered back_wheel(0), handle_bar(1), front_wheel(2),
        rotational_plate(3), so the policy was trained on
        [back_wheel_vel, front_wheel_vel]. Feeding the declared order swapped
        them, which moved the steering action by 0.34 - about 14 degrees.
        """
        if self.joint_vel_order == "declared":
            pair = (self.front_wheel_vel, self.back_wheel_vel)
        else:
            pair = (self.back_wheel_vel, self.front_wheel_vel)
        return np.array(
            [self._scaled(v, self.wheel_vel_clip, self.wheel_vel_scale) for v in pair],
            dtype=np.float32)

    def _assemble_observations(self) -> np.ndarray:
        """Assembles the observation vector matching Isaac Lab MDP order."""
        if self.task_type == "goal_nav":
            # 2D relative goal vector in robot frame
            dx_world = self.goal_x - self.robot_x
            dy_world = self.goal_y - self.robot_y

            cos_yaw = math.cos(self.robot_yaw)
            sin_yaw = math.sin(self.robot_yaw)
            rel_x = cos_yaw * dx_world + sin_yaw * dy_world
            rel_y = -sin_yaw * dx_world + cos_yaw * dy_world

            heading_err = math.atan2(rel_y, rel_x)
            sin_heading = math.sin(heading_err)
            cos_heading = math.cos(heading_err)

            goal_cmd = np.array([rel_x, rel_y, sin_heading, cos_heading], dtype=np.float32)

            # Observation vector (20 dims)
            obs = np.concatenate([
                self.base_lin_vel,                              # (3,)
                self.base_ang_vel,                              # (3,)
                self.projected_gravity,                         # (3,)
                np.array([self.steering_pos], dtype=np.float32),# (1,)
                np.array([self._scaled(self.reaction_plate_vel,
                                       self.plate_vel_clip,
                                       self.plate_vel_scale)],
                         dtype=np.float32),                     # (1,)
                self._joint_vel_obs(),                          # (2,)
                goal_cmd,                                       # (4,)
                self.last_action,                               # (3,)
            ])
        else:
            # Velocity commands: [v_x, v_y, omega_z]
            vel_cmd = np.array([self.cmd_vx, 0.0, self.cmd_wz], dtype=np.float32)

            # Observation vector (19 dims)
            obs = np.concatenate([
                self.base_lin_vel,                              # (3,)
                self.base_ang_vel,                              # (3,)
                self.projected_gravity,                         # (3,)
                np.array([self.steering_pos], dtype=np.float32),# (1,)
                np.array([self._scaled(self.reaction_plate_vel,
                                       self.plate_vel_clip,
                                       self.plate_vel_scale)],
                         dtype=np.float32),                     # (1,)
                self._joint_vel_obs(),                          # (2,)
                vel_cmd,                                        # (3,)
                self.last_action,                               # (3,)
            ])

        return obs.reshape(1, -1).astype(np.float32)

    def _control_loop(self):
        if self.inference_engine is None:
            return
        # Without this the first cycles run on an all-zero observation and push
        # commands out before a single IMU or joint_states message has arrived.
        if not (self.got_imu and self.got_joints):
            return

        # 1. Assemble observations
        obs = self._assemble_observations()

        # 2. Run inference
        try:
            if self.is_onnx:
                raw_actions = self.session.run(None, {self.input_name: obs})[0][0]
            else:
                import torch
                with torch.no_grad():
                    raw_actions = self.model(torch.from_numpy(obs)).numpy()[0]
        except Exception as e:
            self.get_logger().error(f"[RL] Inference failed: {e}")
            return

        # Isaac Lab's last_action observation is the raw policy output, so feed
        # the unclipped action back and clip only what reaches the actuators.
        self.last_action = np.asarray(raw_actions, dtype=np.float32).copy()
        actions = np.clip(raw_actions, -1.0, 1.0)

        if self.debug_obs:
            self._debug_tick += 1
            if self._debug_tick % self._debug_every == 0:
                o = obs[0]
                self.get_logger().info(
                    "[OBS] v=[%+.2f %+.2f %+.2f] w=[%+.2f %+.2f %+.2f] "
                    "g=[%+.2f %+.2f %+.2f] steer=%+.3f plate=%+7.1f "
                    "jv=[%+.1f %+.1f] cmd=[%+.2f %+.2f %+.2f] | raw=[%+.3f %+.3f %+.3f]%s"
                    % (o[0], o[1], o[2], o[3], o[4], o[5], o[6], o[7], o[8],
                       o[9], o[10], o[11], o[12], o[13], o[14], o[15],
                       raw_actions[0], raw_actions[1], raw_actions[2],
                       "  <== RAW OUT OF RANGE"
                       if np.max(np.abs(raw_actions)) > 1.0 else ""))

        # 3. Scale actions to physical actuator units
        # Steering angle target (radians); matches JointPositionActionCfg with
        # use_default_offset=True and a default steering position of 0.
        steer_target = float(actions[0] * self.steering_scale)
        # Reaction wheel effort (torque, N*m), including the implicit-actuator
        # damping the policy was trained against.
        plate_torque = float(
            actions[1] * self.reaction_plate_scale
            - self.plate_damping * self.reaction_plate_vel
        )
        # Front drive wheel speed (rad/s)
        drive_speed = float(actions[2] * self.drive_wheel_scale * self.drive_wheel_sign)

        # 4. Publish commands to ros2_control
        steer_msg = Float64MultiArray()
        steer_msg.data = [steer_target]
        self.steering_pub.publish(steer_msg)

        plate_msg = Float64MultiArray()
        plate_msg.data = [plate_torque]
        self.reaction_plate_pub.publish(plate_msg)

        drive_msg = Float64MultiArray()
        drive_msg.data = [drive_speed]
        self.front_wheel_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = EScooterRLController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
