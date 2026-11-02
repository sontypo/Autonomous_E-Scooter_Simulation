#!/usr/bin/env python3
"""
Quick test to verify the Python version is working correctly
"""
import os
import sys
import yaml

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def test_imports():
    """Test that all modules can be imported"""
    print("Testing imports...")
    try:
        from dynamics import Dynamics
        from control import Control
        from data_structures import Joints, Pose, Twist, Vector3, Odometry
        print("  ✓ All imports successful")
        return True
    except ImportError as e:
        print(f"  ✗ Import failed: {e}")
        return False

def test_config_loading():
    """Test that configuration file can be loaded"""
    print("\nTesting configuration loading...")
    try:
        config_path = os.path.join(os.path.dirname(__file__), 'config', 'escooter.yaml')
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        print(f"  ✓ Configuration loaded successfully")
        print(f"    - MASTER_LOOP_RATE: {config.get('MASTER_LOOP_RATE')} Hz")
        print(f"    - WHEEL_RADIUS: {config.get('WHEEL_RADIUS')} m")
        print(f"    - WHOLE_BODY_WEIGHT: {config.get('WHOLE_BODY_WEIGHT')} kg")
        return True
    except Exception as e:
        print(f"  ✗ Config loading failed: {e}")
        return False

def test_initialization():
    """Test that Dynamics and Control can be initialized"""
    print("\nTesting initialization...")
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
        from dynamics import Dynamics
        from control import Control
        
        config_path = os.path.join(os.path.dirname(__file__), 'config', 'escooter.yaml')
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        dynamics = Dynamics(config)
        control = Control(config)
        print("  ✓ Dynamics initialized successfully")
        print("  ✓ Control initialized successfully")
        return True
    except Exception as e:
        print(f"  ✗ Initialization failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_basic_calculation():
    """Test basic calculations"""
    print("\nTesting basic calculations...")
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
        from dynamics import Dynamics
        from control import Control
        from data_structures import Twist, Vector3
        
        config_path = os.path.join(os.path.dirname(__file__), 'config', 'escooter.yaml')
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        dynamics = Dynamics(config)
        control = Control(config)
        
        # Set velocity
        cmd_vel = Twist()
        cmd_vel.linear = Vector3(x=0.5, y=0.0, z=0.0)
        control.cmd_vel_callback(cmd_vel)
        
        # Run one iteration
        dynamics.constraints(control.Vx_, control.Vx_dot_, control.delta_, control.delta_dot_)
        odom = control.publish_odometry(control.distance_traveled_, control.delta_)
        dynamics.balancing_controller(control.body_frame_, control.Vx_)
        dynamics.update_dynamics(dynamics.theta_, control.joints_, dynamics.tau_, control.Vx_)
        
        print("  ✓ Basic calculations successful")
        print(f"    - Vx: {control.Vx_} m/s")
        print(f"    - Theta: {dynamics.theta_} rad")
        print(f"    - Tau: {dynamics.tau_} Nm")
        return True
    except Exception as e:
        print(f"  ✗ Calculations failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Run all tests"""
    print("=" * 60)
    print("E-Scooter Dynamics - Python Version Test")
    print("=" * 60)
    
    tests = [
        test_imports,
        test_config_loading,
        test_initialization,
        test_basic_calculation
    ]
    
    results = []
    for test in tests:
        results.append(test())
    
    print("\n" + "=" * 60)
    print("Test Results")
    print("=" * 60)
    passed = sum(results)
    total = len(results)
    print(f"Passed: {passed}/{total}")
    
    if passed == total:
        print("\n✓ All tests passed! System is ready to use.")
        print("\nNext steps:")
        print("  - Run 'python3 main.py' to start the main loop")
        print("  - Run 'python3 examples.py' to see usage examples")
        return 0
    else:
        print("\n✗ Some tests failed. Please check the errors above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
