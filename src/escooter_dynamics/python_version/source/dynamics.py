"""
Dynamics module for E-Scooter
Converted from C++ dynamics.cpp and dynamics.h
"""
import math
import time
from typing import Tuple
from data_structures import Joints, Pose


PI = math.atan(1.0) * 4.0
g = 9.81  # gravity


class Trig:
    """Trigonometric values container"""
    def __init__(self, sine: float = 0.0, cosine: float = 0.0):
        self.sine = sine
        self.cosine = cosine


class Dynamics:
    """
    Dynamics class for E-Scooter dynamics model
    Handles balance control and dynamics updates
    """
    
    def __init__(self, config: dict):
        """
        Initialize dynamics variables from configuration
        
        Args:
            config: Dictionary containing configuration parameters
        """
        # Load parameters from config
        self.COM_POS_X = config.get('COM_POS_X', 0.203)
        self.COM_POS_Y = config.get('COM_POS_Y', 0.0)
        self.COM_POS_Z = config.get('COM_POS_Z', 0.392)
        self.BODY_WEIGHT = config.get('BODY_WEIGHT', 12.07606)
        self.R_PLATE_WEIGHT = config.get('R_PLATE_WEIGHT', 2.18948)
        self.F_WHEEL_WEIGHT = config.get('F_WHEEL_WEIGHT', 2.0732)
        self.HANDLE_BAR_WEIGHT = config.get('HANDLE_BAR_WEIGHT', 3.5678)
        self.WHOLE_BODY_WEIGHT = config.get('WHOLE_BODY_WEIGHT', 18.90654)
        self.WHOLE_BODY_WEIGHT_EXC = config.get('WHOLE_BODY_WEIGHT_EXC', 17.71706)
        self.WHEEL_RADIUS = config.get('WHEEL_RADIUS', 0.135)
        self.WHEEL_SEPARATION_DIS = config.get('WHEEL_SEPARATION_DIS', 1.0)
        self.UNCERTAINTIES = config.get('UNCERTAINTIES', 0.01)
        self.B_WHEEL_TO_COM = config.get('B_WHEEL_TO_COM', 0.703)
        self.PD_PDplus = config.get('PD_PDplus', 1.0)
        self.KP = config.get('KP', 2.0)
        self.KD = config.get('KD', 0.05)
        
        # State variables
        self.psi_dot_ = 0.0
        self.psi_2dot_ = 0.0
        self.V_x = 0.0
        self.V_y = 0.0
        self.V_z = 0.0
        self.theta_ = 0.0
        self.tau_ = 0.0
        
        # Private variables
        self.current_time_dynamics_ = time.time()
        self.last_time_dynamics_ = time.time()
        self.current_time_blc_ = time.time()
        self.last_time_blc_ = time.time()
        
        self.C_ = 0.0
        self.G_ = 0.0
        self.CpG_ = 0.0
        self.theta_dot_ = 0.0
        self.pre_theta_ = 0.0
        self.theta_2dot_ = 0.0
        self.I_theta = 0.0
        self.I_rp_x = 0.0
        self.I_rp_y = 0.0
        self.I_rp_z = 0.0
    
    def get_sin_cos(self, angle_rad: float) -> Trig:
        """
        Get the sine and cosine from the angle
        
        Args:
            angle_rad: Angle in radians
            
        Returns:
            Trig object with sine and cosine values
        """
        body_trig = Trig()
        body_trig.sine = math.sin(angle_rad)
        body_trig.cosine = math.cos(angle_rad)
        return body_trig
    
    def update_dynamics(self, theta: float, joints: Joints, tau: float, Vx: float) -> None:
        """
        Update the dynamics model
        
        Args:
            theta: Body pitch angle
            joints: Joint states (modified in place)
            tau: Control torque
            Vx: Linear velocity
        """
        self.current_time_dynamics_ = time.time()
        dt = self.current_time_dynamics_ - self.last_time_dynamics_
        
        # Calculate moments of inertia
        self.I_theta = 4/3 * self.WHOLE_BODY_WEIGHT * pow(self.COM_POS_Z, 2) * self.UNCERTAINTIES
        self.I_rp_x = pow(0.036, 2) * self.R_PLATE_WEIGHT * self.UNCERTAINTIES
        self.I_rp_y = 0.0
        self.I_rp_z = pow(0.076, 2) * self.R_PLATE_WEIGHT * self.UNCERTAINTIES
        
        T = self.get_sin_cos(theta)
        cos_theta = T.cosine
        sin_theta = T.sine
        
        # Calculate angular acceleration
        self.theta_2dot_ = (
            tau + 
            (self.WHOLE_BODY_WEIGHT * self.COM_POS_Z * self.B_WHEEL_TO_COM * self.psi_2dot_ * cos_theta) + 
            (self.WHOLE_BODY_WEIGHT * self.COM_POS_Z * self.psi_dot_ * cos_theta * 
             (Vx - self.COM_POS_Z * self.psi_dot_ * sin_theta)) + 
            self.WHOLE_BODY_WEIGHT * g * self.COM_POS_Z * sin_theta
        ) / (self.I_theta + self.I_rp_x + self.I_rp_z)
        
        # Update angular velocity
        self.theta_dot_ += self.theta_2dot_ * dt
        self.last_time_dynamics_ = self.current_time_dynamics_
        
        # Update joint state
        joints.rotational_plate = self.theta_dot_
    
    def constraints(self, Vx: float, Vx_dot: float, delta: float, delta_dot: float) -> None:
        """
        Calculate constraint equations for the system
        
        Args:
            Vx: Linear velocity
            Vx_dot: Linear acceleration
            delta: Steering angle
            delta_dot: Steering angular velocity
        """
        self.psi_dot_ = math.tan(delta) * Vx / self.WHEEL_SEPARATION_DIS
        self.psi_2dot_ = (
            delta_dot * (1 + math.tan(delta) * math.tan(delta)) / self.WHEEL_SEPARATION_DIS + 
            Vx_dot * math.tan(delta) / self.WHEEL_SEPARATION_DIS
        )
    
    def balancing_controller(self, body_frame: Pose, Vx: float) -> None:
        """
        PD+ balancing controller
        
        Args:
            body_frame: Current body pose/orientation
            Vx: Linear velocity
        """
        self.current_time_blc_ = time.time()
        dt = self.current_time_blc_ - self.last_time_blc_
        
        self.theta_ = body_frame.orientation.roll
        delta_theta = self.theta_ - self.pre_theta_
        theta_dot_imu = delta_theta / dt if dt > 0 else 0.0
        
        self.last_time_blc_ = self.current_time_blc_
        self.pre_theta_ = self.theta_
        
        T = self.get_sin_cos(self.theta_)
        cos_theta = T.cosine
        sin_theta = T.sine
        
        # Calculate Coriolis and centrifugal terms
        self.C_ = (
            (self.WHOLE_BODY_WEIGHT * self.COM_POS_Z * self.B_WHEEL_TO_COM * self.psi_2dot_) + 
            (self.WHOLE_BODY_WEIGHT * self.COM_POS_Z * self.psi_dot_ * 
             (Vx - self.COM_POS_Z * self.psi_dot_ * sin_theta))
        )
        
        # Calculate gravity term
        self.G_ = self.WHOLE_BODY_WEIGHT * g * self.COM_POS_Z
        
        # Combined C+G term
        self.CpG_ = self.C_ * cos_theta + self.G_ * sin_theta
        
        # PD+ controller
        error_p = 0.0 - self.theta_  # Target angle is 0 (upright)
        error_d = 0.0 - theta_dot_imu
        
        self.tau_ = (self.PD_PDplus * self.CpG_) + (self.KP * error_p) + (self.KD * error_d)
    
    def compute_com_velocity(self, Vx: float, theta: float, theta_dot: float, 
                            psi: float, psi_dot: float) -> Tuple[float, float, float]:
        """
        Compute center of mass velocity
        
        Args:
            Vx: Linear velocity
            theta: Body pitch angle
            theta_dot: Angular velocity of body pitch
            psi: Yaw angle
            psi_dot: Yaw angular velocity
            
        Returns:
            Tuple of (V_x, V_y, V_z) velocities
        """
        T = self.get_sin_cos(theta)
        cos_theta = T.cosine
        sin_theta = T.sine
        
        P = self.get_sin_cos(psi)
        cos_psi = P.cosine
        sin_psi = P.sine
        
        self.V_x = (
            (Vx * cos_theta) - 
            (self.B_WHEEL_TO_COM * psi_dot * sin_psi) + 
            (self.COM_POS_Z * theta_dot * cos_theta * sin_psi) + 
            (self.COM_POS_Z * psi_dot * sin_theta * cos_psi)
        )
        
        self.V_y = (
            (Vx * sin_theta) + 
            (self.B_WHEEL_TO_COM * psi_dot * cos_psi) - 
            (self.COM_POS_Z * theta_dot * cos_theta * cos_psi) + 
            (self.COM_POS_Z * psi_dot * sin_theta * sin_psi)
        )
        
        self.V_z = -self.COM_POS_Z * theta_dot * sin_theta
        
        return self.V_x, self.V_y, self.V_z
