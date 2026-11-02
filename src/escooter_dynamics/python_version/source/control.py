"""
Control module for E-Scooter
Converted from C++ control.cpp and control.h
"""
import math
import time
from typing import Dict, List, Tuple
from data_structures import Joints, Pose, Twist, Vector3, JointState, Odometry, RPY


PI = math.atan(1.0) * 4.0


class Control:
    """
    Control class for E-Scooter control system
    Handles odometry, joint commands, and velocity commands
    """
    
    def __init__(self, config: dict):
        """
        Initialize control variables from configuration
        
        Args:
            config: Dictionary containing configuration parameters
        """
        # Load parameters from config
        self.NAME_SPACE = config.get('NAME_SPACE', '/e_scooter/')
        self.BODY_MAX_ROLL = config.get('BODY_MAX_ROLL', 0.209)  # ~12 degrees
        self.BODY_MAX_PITCH = config.get('BODY_MAX_PITCH', 0.209)
        self.BODY_MAX_YAW = config.get('BODY_MAX_YAW', 3.14)
        self.MAX_BODY_ROLL_COMP = config.get('MAX_BODY_ROLL_COMP', 0.1)
        self.MAX_BODY_PITCH_COMP = config.get('MAX_BODY_PITCH_COMP', 0.1)
        self.COMPENSATE_INCREMENT = config.get('COMPENSATE_INCREMENT', 0.01)
        self.COMPENSATE_TO_WITHIN = config.get('COMPENSATE_TO_WITHIN', 0.01)
        self.MASTER_LOOP_RATE = config.get('MASTER_LOOP_RATE', 500)
        self.VELOCITY_DIVISION = config.get('VELOCITY_DIVISION', 0.033)
        self.NUMBER_OF_JOINTS = config.get('NUMBER_OF_JOINTS', 3)
        self.WHEEL_RADIUS = config.get('WHEEL_RADIUS', 0.135)
        self.WHEEL_SEPARATION_DIS = config.get('WHEEL_SEPARATION_DIS', 1.0)
        self.MAX_STEERING_ANGLE = config.get('MAX_STEERING_ANGLE', 0.785398163)
        
        # Joint configuration
        self.joint_names_array_ = []
        self.joint_orientation_array_ = []
        joints_config = config.get('JOINTS', {})
        
        for i in range(self.NUMBER_OF_JOINTS):
            joint_info = joints_config.get(str(i), {})
            self.joint_names_array_.append(joint_info.get('name', f'joint_{i}'))
            self.joint_orientation_array_.append(joint_info.get('sign', 1))
        
        # Time tracking
        self.current_time_odometry_ = time.time()
        self.last_time_odometry_ = time.time()
        self.current_time_cmd_vel_ = time.time()
        self.last_time_cmd_vel_ = time.time()
        self.current_time_js_ = time.time()
        self.last_time_js_ = time.time()
        
        # State variables
        self.prev_scooter_state_ = False
        self.scooter_state_ = False
        self.imu_init_stored_ = False
        self.imu_roll_lowpass_ = 0.0
        self.imu_pitch_lowpass_ = 0.0
        self.imu_yaw_lowpass_ = 0.0
        self.imu_roll_init_ = 0.0
        self.imu_pitch_init_ = 0.0
        
        # Odometry variables
        self.pose_x_ = 0.0
        self.pose_y_ = 0.0
        self.psi_ = 0.0
        self.prev_front_wheel_angle = 0.0
        self.prev_steering_angle = 0.0
        self.steering_angle = 0.0
        self.des_theta = 0.0
        
        # Public variables (accessed by main)
        self.joint_state_ = JointState()
        self.body_frame_ = Pose()
        self.joints_ = Joints()
        self.com_vel_ = Twist()
        self.cmd_vel_ = Twist()
        self.distance_traveled_ = 0.0
        self.delta_ = 0.0
        self.prev_delta_ = 0.0
        self.delta_dot_ = 0.0
        self.Vx_ = 0.0
        self.des_delta = 0.0
        self.Vx_dot_ = 0.0
    
    def set_scooter_state(self, state: bool) -> None:
        """Set scooter state"""
        self.scooter_state_ = state
    
    def get_scooter_state(self) -> bool:
        """Get scooter state"""
        return self.scooter_state_
    
    def set_prev_scooter_state(self, state: bool) -> None:
        """Set previous scooter state"""
        self.prev_scooter_state_ = state
    
    def get_prev_scooter_state(self) -> bool:
        """Get previous scooter state"""
        return self.prev_scooter_state_
    
    def publish_odometry(self, distance_traveled: float, delta: float) -> Odometry:
        """
        Calculate and return odometry information
        
        Args:
            distance_traveled: Distance traveled since last update
            delta: Steering angle
            
        Returns:
            Odometry object with calculated values
        """
        # Calculate time elapsed
        self.current_time_odometry_ = time.time()
        dt = self.current_time_odometry_ - self.last_time_odometry_
        
        # Update heading
        delta_psi = (distance_traveled / self.WHEEL_SEPARATION_DIS) * math.tan(delta)
        self.psi_ += delta_psi
        
        # Update position
        delta_x = distance_traveled * math.cos(self.psi_)
        delta_y = distance_traveled * math.sin(self.psi_)
        self.pose_x_ += delta_x
        self.pose_y_ += delta_y
        
        # Calculate velocities
        v_x = delta_x / dt if dt > 0 else 0.0
        v_y = delta_y / dt if dt > 0 else 0.0
        v_psi = delta_psi / dt if dt > 0 else 0.0
        
        self.last_time_odometry_ = self.current_time_odometry_
        
        # Create odometry object
        odom = Odometry()
        odom.pose_x = self.pose_x_
        odom.pose_y = self.pose_y_
        odom.pose_z = self.body_frame_.orientation.yaw
        odom.psi = self.psi_
        odom.v_x = v_x
        odom.v_y = v_y
        odom.v_psi = v_psi
        odom.timestamp = self.current_time_odometry_
        
        return odom
    
    def publish_twist(self, com_vel: Twist) -> Dict:
        """
        Create twist message with covariance
        
        Args:
            com_vel: Command velocity
            
        Returns:
            Dictionary containing twist data
        """
        twist_stamped = {
            'timestamp': time.time(),
            'frame_id': 'odom',
            'linear': {
                'x': com_vel.linear.x,
                'y': com_vel.linear.y,
                'z': 0.0
            },
            'angular': {
                'x': 0.0,
                'y': 0.0,
                'z': com_vel.angular.z
            },
            'covariance': [
                0.00001, 0, 0, 0, 0, 0,  # x
                0, 0.00001, 0, 0, 0, 0,  # y
                0, 0, 0.00001, 0, 0, 0,  # z
                0, 0, 0, 1000000000000.0, 0, 0,  # rot x
                0, 0, 0, 0, 1000000000000.0, 0,  # rot y
                0, 0, 0, 0, 0, 0.001  # rot z
            ]
        }
        return twist_stamped
    
    def publish_joint_command(self, joints: Joints) -> Dict[str, float]:
        """
        Create joint command dictionary
        
        Args:
            joints: Joint states
            
        Returns:
            Dictionary of joint commands
        """
        commands = {
            'handle_bar_position': self.joint_orientation_array_[0] * joints.handle_bar,
            'rotational_plate_effort': self.joint_orientation_array_[1] * joints.rotational_plate,
            'front_wheel_velocity': self.joint_orientation_array_[2] * joints.front_wheel
        }
        return commands
    
    def joint_state_callback(self, joint_state_msg: JointState) -> None:
        """
        Process joint state information (simulates ROS callback)
        
        Args:
            joint_state_msg: Joint state message
        """
        self.current_time_js_ = time.time()
        dt = self.current_time_js_ - self.last_time_js_
        
        front_wheel_joint_name = "front_wheel_joint"
        handle_bar_chasis_joint_name = "handle_bar_chasis_joint"
        front_wheel_angle = 0.0
        
        # Extract joint information
        for i, name in enumerate(joint_state_msg.name):
            if name == front_wheel_joint_name:
                front_wheel_angle = joint_state_msg.position[i]
            elif name == handle_bar_chasis_joint_name:
                self.steering_angle = joint_state_msg.position[i]
        
        # Calculate incremental distance traveled from wheel rotation
        delta_angle = front_wheel_angle - self.prev_front_wheel_angle
        delta_distance = self.WHEEL_RADIUS * delta_angle
        
        self.distance_traveled_ = delta_distance
        self.delta_ = self.steering_angle
        
        # Calculate steering rate
        delta_steering = self.steering_angle - self.prev_steering_angle
        self.delta_dot_ = delta_steering / dt if dt > 0 else 0.0
        
        # Update previous values
        self.prev_front_wheel_angle = front_wheel_angle
        self.prev_steering_angle = self.steering_angle
        self.last_time_js_ = self.current_time_js_
    
    def cmd_vel_callback(self, cmd_vel_msg: Twist) -> None:
        """
        Process velocity command (simulates ROS callback)
        
        Args:
            cmd_vel_msg: Velocity command message
        """
        self.current_time_cmd_vel_ = time.time()
        dt = self.current_time_cmd_vel_ - self.last_time_cmd_vel_
        
        # Store command velocity
        self.cmd_vel_ = cmd_vel_msg
        
        # Calculate linear velocity and acceleration
        prev_Vx = self.Vx_
        self.Vx_ = cmd_vel_msg.linear.x
        self.Vx_dot_ = (self.Vx_ - prev_Vx) / dt if dt > 0 else 0.0
        
        # Calculate desired steering angle from angular velocity
        # omega = v * tan(delta) / L
        # delta = atan(omega * L / v)
        if abs(self.Vx_) > 0.001:
            self.des_delta = math.atan(
                cmd_vel_msg.angular.z * self.WHEEL_SEPARATION_DIS / self.Vx_
            )
        else:
            self.des_delta = 0.0
        
        # Clamp steering angle
        self.des_delta = max(
            -self.MAX_STEERING_ANGLE, 
            min(self.MAX_STEERING_ANGLE, self.des_delta)
        )
        
        # Update handle bar joint
        self.joints_.handle_bar = self.des_delta
        
        # Calculate front wheel velocity
        if abs(self.WHEEL_RADIUS) > 0.001:
            self.joints_.front_wheel = self.Vx_ / self.WHEEL_RADIUS
        else:
            self.joints_.front_wheel = 0.0
        
        self.last_time_cmd_vel_ = self.current_time_cmd_vel_
    
    def imu_callback(self, imu_msg: Dict) -> None:
        """
        Process IMU data for auto-leveling (simulates ROS callback)
        
        Args:
            imu_msg: IMU message dictionary with orientation quaternion
        """
        # Extract quaternion from IMU message
        quat = imu_msg.get('orientation', {})
        x = quat.get('x', 0.0)
        y = quat.get('y', 0.0)
        z = quat.get('z', 0.0)
        w = quat.get('w', 1.0)
        
        # Convert quaternion to roll, pitch, yaw
        roll, pitch, yaw = self.quaternion_to_euler(x, y, z, w)
        
        # Store initial IMU values for calibration
        if not self.imu_init_stored_:
            self.imu_roll_init_ = roll
            self.imu_pitch_init_ = pitch
            self.imu_init_stored_ = True
        
        # Apply lowpass filter
        alpha = 0.1  # Filter coefficient
        self.imu_roll_lowpass_ = alpha * roll + (1 - alpha) * self.imu_roll_lowpass_
        self.imu_pitch_lowpass_ = alpha * pitch + (1 - alpha) * self.imu_pitch_lowpass_
        self.imu_yaw_lowpass_ = alpha * yaw + (1 - alpha) * self.imu_yaw_lowpass_
        
        # Update body frame orientation
        self.body_frame_.orientation.roll = self.imu_roll_lowpass_ - self.imu_roll_init_
        self.body_frame_.orientation.pitch = self.imu_pitch_lowpass_ - self.imu_pitch_init_
        self.body_frame_.orientation.yaw = self.imu_yaw_lowpass_
    
    @staticmethod
    def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
        """
        Convert quaternion to Euler angles (roll, pitch, yaw)
        
        Args:
            x, y, z, w: Quaternion components
            
        Returns:
            Tuple of (roll, pitch, yaw) in radians
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(PI / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
