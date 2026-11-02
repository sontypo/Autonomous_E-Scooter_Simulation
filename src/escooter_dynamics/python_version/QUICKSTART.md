# Quick Start Guide

## Installation (1 minute)

```bash
cd python_version
pip install -r requirements.txt
```

## Verify Installation (30 seconds)

```bash
python3 test_system.py
```

Expected output:
```
E-Scooter Dynamics - Python Version Test
============================================================
Testing imports...
  ✓ All imports successful
...
✓ All tests passed! System is ready to use.
```

## Run the System (Immediate)

### Option 1: Run the main loop
```bash
python3 main.py
```

### Option 2: Run examples
```bash
python3 examples.py
```

## Basic Usage in Your Code

```python
import yaml
from dynamics import Dynamics
from control import Control
from data_structures import Twist, Vector3

# Load config
with open('config/escooter.yaml', 'r') as f:
    config = yaml.safe_load(f)

# Initialize
dynamics = Dynamics(config)
control = Control(config)

# Set velocity command
cmd_vel = Twist()
cmd_vel.linear.x = 0.5  # 0.5 m/s forward
control.cmd_vel_callback(cmd_vel)

# Run control loop
while True:
    dynamics.constraints(control.Vx_, control.Vx_dot_, 
                        control.delta_, control.delta_dot_)
    odom = control.publish_odometry(control.distance_traveled_, 
                                    control.delta_)
    dynamics.balancing_controller(control.body_frame_, control.Vx_)
    dynamics.update_dynamics(dynamics.theta_, control.joints_, 
                           dynamics.tau_, control.Vx_)
```

## File Overview

| File | Purpose |
|------|---------|
| `main.py` | Main execution script with control loop |
| `examples.py` | Three example scenarios |
| `test_system.py` | Verify installation |
| `src/dynamics.py` | Dynamics model & balance controller |
| `src/control.py` | Control system & odometry |
| `src/data_structures.py` | Data classes |
| `config/escooter.yaml` | Configuration parameters |

## Key Variables

### Control
- `control.Vx_` - Linear velocity (m/s)
- `control.delta_` - Steering angle (rad)
- `control.joints_` - Joint states
- `control.body_frame_` - Body orientation

### Dynamics
- `dynamics.theta_` - Body pitch angle (rad)
- `dynamics.tau_` - Control torque (Nm)
- `dynamics.theta_dot_` - Angular velocity (rad/s)

## Common Tasks

### Change velocity
```python
control.cmd_vel_.linear.x = 0.3  # m/s
control.cmd_vel_callback(control.cmd_vel_)
```

### Read odometry
```python
odom = control.publish_odometry(control.distance_traveled_, control.delta_)
print(f"Position: ({odom.pose_x}, {odom.pose_y})")
```

### Tune controller
Edit `config/escooter.yaml`:
```yaml
KP: 2.0    # Proportional gain
KD: 0.05   # Derivative gain
```

## Troubleshooting

**Problem:** Import errors when running  
**Solution:** Run from `python_version/` directory

**Problem:** Loop too slow  
**Solution:** Reduce `MASTER_LOOP_RATE` in config

**Problem:** No pyyaml  
**Solution:** `pip install pyyaml`

## More Information

- Full documentation: `README.md`
- Project summary: `SUMMARY.md`
- Example code: `examples.py`
