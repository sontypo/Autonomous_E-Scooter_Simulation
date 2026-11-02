"""
Main execution script for E-Scooter Dynamics
Converted from C++ escooter_dynamics.cpp

This script initializes the dynamics and control systems, and runs the main control loop.
"""
import os
import sys
import time
import yaml
from pathlib import Path

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'source'))

from dynamics import Dynamics
from control import Control
from data_structures import Twist, JointState, Vector3


def load_config(config_path: str) -> dict:
    """
    Load configuration from YAML file
    
    Args:
        config_path: Path to configuration file
        
    Returns:
        Configuration dictionary
    """
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def main():
    """
    Main execution function
    """
    print("E-Scooter Dynamics - Python Version")
    print("=" * 50)
    
    # Load configuration
    config_path = os.path.join(os.path.dirname(__file__), 'config', 'escooter.yaml')
    if not os.path.exists(config_path):
        print(f"Error: Configuration file not found at {config_path}")
        sys.exit(1)
    
    config = load_config(config_path)
    print(f"Configuration loaded from: {config_path}")
    
    # Initialize Dynamics and Control instances
    dynamics = Dynamics(config)
    control = Control(config)
    
    print("Dynamics and Control systems initialized")
    print(f"Master loop rate: {control.MASTER_LOOP_RATE} Hz")
    print("=" * 50)
    
    # Calculate loop period
    loop_rate = control.MASTER_LOOP_RATE
    loop_period = 1.0 / loop_rate
    
    # Initialize system (run once before loop)
    dynamics.constraints(control.Vx_, control.Vx_dot_, control.delta_, control.delta_dot_)
    odom = control.publish_odometry(control.distance_traveled_, control.delta_)
    dynamics.balancing_controller(control.body_frame_, control.Vx_)
    dynamics.update_dynamics(dynamics.theta_, control.joints_, dynamics.tau_, control.Vx_)
    joint_commands = control.publish_joint_command(control.joints_)
    
    print("Starting main control loop... (Press Ctrl+C to stop)")
    print()
    
    iteration = 0
    start_time = time.time()
    
    try:
        while True:
            loop_start = time.time()
            
            # Main control loop - same sequence as C++ version
            dynamics.constraints(control.Vx_, control.Vx_dot_, control.delta_, control.delta_dot_)
            odom = control.publish_odometry(control.distance_traveled_, control.delta_)
            dynamics.balancing_controller(control.body_frame_, control.Vx_)
            dynamics.update_dynamics(dynamics.theta_, control.joints_, dynamics.tau_, control.Vx_)
            joint_commands = control.publish_joint_command(control.joints_)
            
            # Print status every 100 iterations
            iteration += 1
            if iteration % 100 == 0:
                elapsed = time.time() - start_time
                print(f"Iteration: {iteration:6d} | "
                      f"Time: {elapsed:6.2f}s | "
                      f"Theta: {dynamics.theta_:7.4f} rad | "
                      f"Tau: {dynamics.tau_:8.4f} | "
                      f"Vx: {control.Vx_:6.3f} m/s | "
                      f"Pos: ({odom.pose_x:.2f}, {odom.pose_y:.2f})")
            
            # Sleep to maintain loop rate
            loop_elapsed = time.time() - loop_start
            sleep_time = loop_period - loop_elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif iteration % 100 == 0:
                print(f"Warning: Loop running slower than target rate "
                      f"(took {loop_elapsed*1000:.2f}ms, target {loop_period*1000:.2f}ms)")
    
    except KeyboardInterrupt:
        print("\n" + "=" * 50)
        print("Shutting down...")
        elapsed = time.time() - start_time
        print(f"Total iterations: {iteration}")
        print(f"Total time: {elapsed:.2f}s")
        print(f"Average rate: {iteration/elapsed:.2f} Hz")
        print("=" * 50)


if __name__ == "__main__":
    main()
