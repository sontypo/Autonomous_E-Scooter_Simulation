"""
Data structures module for E-Scooter Dynamics
Replaces ROS message types with Python dataclasses
"""
from dataclasses import dataclass, field
from typing import List


@dataclass
class RPY:
    """Roll, Pitch, Yaw orientation"""
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class Pose:
    """Pose containing position and orientation"""
    position: 'Vector3' = field(default_factory=lambda: Vector3())
    orientation: RPY = field(default_factory=RPY)


@dataclass
class Vector3:
    """3D Vector"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Joints:
    """Joint states for the e-scooter"""
    handle_bar: float = 0.0
    rotational_plate: float = 0.0
    front_wheel: float = 0.0
    back_wheel: float = 0.0


@dataclass
class Twist:
    """Velocity in free space"""
    linear: Vector3 = field(default_factory=Vector3)
    angular: Vector3 = field(default_factory=Vector3)


@dataclass
class JointState:
    """Joint state information"""
    name: List[str] = field(default_factory=list)
    position: List[float] = field(default_factory=list)
    velocity: List[float] = field(default_factory=list)
    effort: List[float] = field(default_factory=list)


@dataclass
class Odometry:
    """Odometry information"""
    pose_x: float = 0.0
    pose_y: float = 0.0
    pose_z: float = 0.0
    psi: float = 0.0  # yaw angle
    v_x: float = 0.0
    v_y: float = 0.0
    v_psi: float = 0.0
    timestamp: float = 0.0
