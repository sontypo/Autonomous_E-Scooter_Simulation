# E-Scooter Dynamics - Python Version

This is a Python conversion of the ROS-based e-scooter dynamics simulation. The original C++ code has been translated to pure Python, removing ROS dependencies while maintaining the exact same dynamics and control algorithms.

## Overview

The e-scooter dynamics system simulates the behavior of an autonomous e-scooter, including:
- **Dynamics Model**: Physics-based simulation of the e-scooter's motion
- **Balance Controller**: PD+ controller to maintain upright position
- **Odometry Calculation**: Position and velocity estimation
- **Joint Control**: Handle bar steering and wheel velocity control

## Project Structure

```
python_version/
├── config/
│   └── escooter.yaml          # Configuration parameters
├── src/
│   ├── __init__.py
│   ├── data_structures.py     # Data classes (replaces ROS messages)
│   ├── dynamics.py            # Dynamics model and balance controller
│   └── control.py             # Control system and odometry
├── main.py                    # Main execution script
├── requirements.txt           # Python dependencies
└── README.md                  # This file
```

## Components

### 1. Dynamics (`dynamics.py`)
Implements the physics model of the e-scooter:
- `update_dynamics()`: Updates the dynamics equations of motion
- `constraints()`: Calculates kinematic constraints
- `balancing_controller()`: PD+ controller for balance control
- `compute_com_velocity()`: Calculates center of mass velocity

### 2. Control (`control.py`)
Handles control and odometry:
- `publish_odometry()`: Calculates odometry from wheel encoders
- `publish_joint_command()`: Generates joint control commands
- `cmd_vel_callback()`: Processes velocity commands
- `joint_state_callback()`: Processes joint state feedback
- `imu_callback()`: Processes IMU data for orientation

### 3. Data Structures (`data_structures.py`)
Defines data classes that replace ROS message types:
- `Joints`: Joint positions/velocities
- `Pose`: Position and orientation
- `Twist`: Linear and angular velocity
- `Odometry`: Odometry information
- And more...

## Installation

1. **Install Python dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Verify installation:**
   ```bash
   python3 main.py --help
   ```

## Configuration

Edit `config/escooter.yaml` to modify parameters:
- Physical properties (mass, dimensions, etc.)
- Control gains (KP, KD, PD_PDplus)
- Loop rates and velocities
- Joint configurations

### Key Parameters:
- `MASTER_LOOP_RATE`: Control loop frequency (Hz)
- `WHOLE_BODY_WEIGHT`: Total mass (kg)
- `COM_POS_X/Y/Z`: Center of mass position
- `WHEEL_RADIUS`: Wheel radius (m)
- `WHEEL_SEPARATION_DIS`: Distance between wheels (m)
- `KP`, `KD`: PD controller gains
- `MAX_STEERING_ANGLE`: Maximum steering angle (rad)

## Usage

### Basic Execution

Run the simulation with default parameters:
```bash
cd python_version
python3 main.py
```

### Expected Output

```
E-Scooter Dynamics - Python Version
==================================================
Configuration loaded from: config/escooter.yaml
Dynamics and Control systems initialized
Master loop rate: 500 Hz
==================================================
Starting main control loop... (Press Ctrl+C to stop)

Iteration:    100 | Time:   0.20s | Theta:  0.0000 rad | Tau:   0.0000 | Vx: 0.000 m/s | Pos: (0.00, 0.00)
Iteration:    200 | Time:   0.40s | Theta:  0.0000 rad | Tau:   0.0000 | Vx: 0.000 m/s | Pos: (0.00, 0.00)
...
```

Press `Ctrl+C` to stop the simulation.

## Integration with External Data Sources

The current implementation assumes input data will be provided from external sources. To integrate with your data:

### 1. **Velocity Commands**
Update `control.cmd_vel_` to provide velocity commands:
```python
from data_structures import Twist, Vector3

# In your data collection code:
cmd_vel = Twist()
cmd_vel.linear.x = 0.5  # m/s forward
cmd_vel.angular.z = 0.1  # rad/s turning

control.cmd_vel_callback(cmd_vel)
```

### 2. **Joint States**
Update joint positions from encoders:
```python
from data_structures import JointState

joint_state = JointState()
joint_state.name = ['front_wheel_joint', 'handle_bar_chasis_joint']
joint_state.position = [wheel_angle, steering_angle]

control.joint_state_callback(joint_state)
```

### 3. **IMU Data**
Provide IMU orientation data:
```python
imu_data = {
    'orientation': {
        'x': quat_x,
        'y': quat_y,
        'z': quat_z,
        'w': quat_w
    }
}

control.imu_callback(imu_data)
```

## Differences from C++ Version

### Removed:
- All ROS dependencies (ros, tf, sensor_msgs, etc.)
- ROS publishers and subscribers
- ROS parameter server

### Modified:
- Timing uses Python's `time` module instead of `ros::Time`
- Data structures use Python dataclasses instead of ROS messages
- Callbacks are regular methods (not true callbacks)

### Maintained:
- **Exact same algorithms** for dynamics and control
- **Same parameter names** and values
- **Same calculation methods**
- **Same control loop structure**

## Technical Details

### Dynamics Model
The system uses a simplified inverted pendulum model with:
- Body pitch angle (theta) as the state
- PD+ controller for balance
- Kinematic constraints for bicycle model

### Balance Controller
PD+ controller with feedforward compensation:
```
tau = (PD_PDplus * (C*cos(theta) + G*sin(theta))) + (KP * error_p) + (KD * error_d)
```

Where:
- `C`: Coriolis and centrifugal forces
- `G`: Gravitational force
- `error_p`: Position error (target angle - current angle)
- `error_d`: Velocity error

### Odometry Calculation
Uses bicycle model kinematics:
- Position updated based on wheel encoders
- Heading updated based on steering angle
- Velocities calculated from position derivatives

## Troubleshooting

### Loop Running Slowly
If you see warnings about loop rate:
- Reduce `MASTER_LOOP_RATE` in config
- Check system load
- Use a faster computer

### Import Errors
Make sure you're running from the `python_version` directory:
```bash
cd python_version
python3 main.py
```

### Configuration Not Found
Verify the config file exists:
```bash
ls config/escooter.yaml
```

## Future Enhancements

To fully integrate this system, you may want to add:
1. **Data input module**: Read from sensors/simulators
2. **Data output module**: Log or visualize results
3. **Real-time visualization**: Plot trajectories and states
4. **Parameter tuning interface**: Adjust gains dynamically
5. **Multiple scenarios**: Test different conditions

## License

This is a conversion of the original ROS-based e-scooter dynamics code.

## Contact

For questions about the Python conversion or integration, please refer to the original C++ code for algorithm details.
