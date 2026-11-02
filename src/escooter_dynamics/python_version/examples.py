"""
Example script demonstrating how to use the E-Scooter Dynamics system
with custom input data
"""
import os
import sys
import time
import yaml
import math

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from dynamics import Dynamics
from control import Control
from data_structures import Twist, JointState, Vector3


def load_config(config_path: str) -> dict:
    """Load configuration from YAML file"""
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def example_straight_line():
    """
    Example: Simulate straight line motion
    """
    print("\n" + "=" * 60)
    print("Example 1: Straight Line Motion")
    print("=" * 60)
    
    # Load configuration
    config_path = os.path.join(os.path.dirname(__file__), 'config', 'escooter.yaml')
    config = load_config(config_path)
    
    # Initialize systems
    dynamics = Dynamics(config)
    control = Control(config)
    
    # Set desired velocity (straight forward)
    cmd_vel = Twist()
    cmd_vel.linear = Vector3(x=0.5, y=0.0, z=0.0)  # 0.5 m/s forward
    cmd_vel.angular = Vector3(x=0.0, y=0.0, z=0.0)  # No turning
    
    control.cmd_vel_callback(cmd_vel)
    
    # Simulate for 5 seconds
    loop_rate = 100  # Hz
    loop_period = 1.0 / loop_rate
    duration = 5.0
    iterations = int(duration * loop_rate)
    
    print(f"Simulating {duration}s at {loop_rate}Hz ({iterations} iterations)")
    print()
    
    for i in range(iterations):
        # Update dynamics
        dynamics.constraints(control.Vx_, control.Vx_dot_, control.delta_, control.delta_dot_)
        odom = control.publish_odometry(control.distance_traveled_, control.delta_)
        dynamics.balancing_controller(control.body_frame_, control.Vx_)
        dynamics.update_dynamics(dynamics.theta_, control.joints_, dynamics.tau_, control.Vx_)
        
        # Simulate sensor feedback (in real scenario, this comes from sensors)
        joint_state = JointState()
        joint_state.name = ['front_wheel_joint', 'handle_bar_chasis_joint']
        # Assume wheel rotates based on velocity
        wheel_angle = control.Vx_ * (i * loop_period) / config['WHEEL_RADIUS']
        joint_state.position = [wheel_angle, control.joints_.handle_bar]
        control.joint_state_callback(joint_state)
        
        # Print status every second
        if i % loop_rate == 0:
            print(f"t={i/loop_rate:.1f}s: Pos=({odom.pose_x:.3f}, {odom.pose_y:.3f})m, "
                  f"Vel={control.Vx_:.3f}m/s, Theta={dynamics.theta_:.4f}rad")
        
        time.sleep(loop_period)
    
    print(f"\nFinal position: ({odom.pose_x:.3f}, {odom.pose_y:.3f})m")
    print(f"Distance traveled: {math.sqrt(odom.pose_x**2 + odom.pose_y**2):.3f}m")


def example_circular_motion():
    """
    Example: Simulate circular motion
    """
    print("\n" + "=" * 60)
    print("Example 2: Circular Motion")
    print("=" * 60)
    
    # Load configuration
    config_path = os.path.join(os.path.dirname(__file__), 'config', 'escooter.yaml')
    config = load_config(config_path)
    
    # Initialize systems
    dynamics = Dynamics(config)
    control = Control(config)
    
    # Set desired velocity (forward with turning)
    cmd_vel = Twist()
    cmd_vel.linear = Vector3(x=0.3, y=0.0, z=0.0)  # 0.3 m/s forward
    cmd_vel.angular = Vector3(x=0.0, y=0.0, z=0.2)  # 0.2 rad/s turning
    
    control.cmd_vel_callback(cmd_vel)
    
    # Simulate for 10 seconds
    loop_rate = 100  # Hz
    loop_period = 1.0 / loop_rate
    duration = 10.0
    iterations = int(duration * loop_rate)
    
    print(f"Simulating {duration}s at {loop_rate}Hz ({iterations} iterations)")
    print(f"Linear velocity: {cmd_vel.linear.x}m/s, Angular velocity: {cmd_vel.angular.z}rad/s")
    print()
    
    for i in range(iterations):
        # Update dynamics
        dynamics.constraints(control.Vx_, control.Vx_dot_, control.delta_, control.delta_dot_)
        odom = control.publish_odometry(control.distance_traveled_, control.delta_)
        dynamics.balancing_controller(control.body_frame_, control.Vx_)
        dynamics.update_dynamics(dynamics.theta_, control.joints_, dynamics.tau_, control.Vx_)
        
        # Simulate sensor feedback
        joint_state = JointState()
        joint_state.name = ['front_wheel_joint', 'handle_bar_chasis_joint']
        wheel_angle = control.Vx_ * (i * loop_period) / config['WHEEL_RADIUS']
        joint_state.position = [wheel_angle, control.joints_.handle_bar]
        control.joint_state_callback(joint_state)
        
        # Print status every 2 seconds
        if i % (loop_rate * 2) == 0:
            print(f"t={i/loop_rate:.1f}s: Pos=({odom.pose_x:.3f}, {odom.pose_y:.3f})m, "
                  f"Heading={odom.psi*180/math.pi:.1f}°, Steering={control.delta_*180/math.pi:.1f}°")
        
        time.sleep(loop_period)
    
    print(f"\nFinal position: ({odom.pose_x:.3f}, {odom.pose_y:.3f})m")
    print(f"Final heading: {odom.psi*180/math.pi:.1f}°")


def example_with_imu():
    """
    Example: Demonstrate IMU integration and balance control
    """
    print("\n" + "=" * 60)
    print("Example 3: Balance Control with IMU")
    print("=" * 60)
    
    # Load configuration
    config_path = os.path.join(os.path.dirname(__file__), 'config', 'escooter.yaml')
    config = load_config(config_path)
    
    # Initialize systems
    dynamics = Dynamics(config)
    control = Control(config)
    
    # Set velocity
    cmd_vel = Twist()
    cmd_vel.linear = Vector3(x=0.4, y=0.0, z=0.0)
    cmd_vel.angular = Vector3(x=0.0, y=0.0, z=0.0)
    control.cmd_vel_callback(cmd_vel)
    
    # Simulate for 3 seconds
    loop_rate = 200  # Hz
    loop_period = 1.0 / loop_rate
    duration = 3.0
    iterations = int(duration * loop_rate)
    
    print(f"Simulating {duration}s at {loop_rate}Hz")
    print("Demonstrating balance controller response to disturbances")
    print()
    
    for i in range(iterations):
        # Simulate a disturbance (tilt) at t=1s
        if i == int(loop_rate * 1.0):
            print(">> Applying disturbance: 5° tilt")
            control.body_frame_.orientation.roll = 5.0 * math.pi / 180.0
        
        # Update dynamics
        dynamics.constraints(control.Vx_, control.Vx_dot_, control.delta_, control.delta_dot_)
        odom = control.publish_odometry(control.distance_traveled_, control.delta_)
        dynamics.balancing_controller(control.body_frame_, control.Vx_)
        dynamics.update_dynamics(dynamics.theta_, control.joints_, dynamics.tau_, control.Vx_)
        
        # Update body orientation based on dynamics (simplified)
        control.body_frame_.orientation.roll = dynamics.theta_
        
        # Simulate sensor feedback
        joint_state = JointState()
        joint_state.name = ['front_wheel_joint', 'handle_bar_chasis_joint']
        wheel_angle = control.Vx_ * (i * loop_period) / config['WHEEL_RADIUS']
        joint_state.position = [wheel_angle, control.joints_.handle_bar]
        control.joint_state_callback(joint_state)
        
        # Print status every 0.5 seconds
        if i % (loop_rate // 2) == 0:
            print(f"t={i/loop_rate:.1f}s: Theta={dynamics.theta_*180/math.pi:.2f}°, "
                  f"Tau={dynamics.tau_:.3f}Nm, "
                  f"Theta_dot={dynamics.theta_dot_:.3f}rad/s")
        
        time.sleep(loop_period)
    
    print("\nBalance controller demonstration complete")


def main():
    """Run all examples"""
    print("\n" + "=" * 60)
    print("E-Scooter Dynamics - Example Scripts")
    print("=" * 60)
    print("\nThis script demonstrates how to integrate external data")
    print("with the E-Scooter Dynamics system.")
    
    try:
        # Run examples
        example_straight_line()
        input("\nPress Enter to continue to next example...")
        
        example_circular_motion()
        input("\nPress Enter to continue to next example...")
        
        example_with_imu()
        
        print("\n" + "=" * 60)
        print("All examples completed successfully!")
        print("=" * 60)
        
    except KeyboardInterrupt:
        print("\n\nExamples interrupted by user")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
