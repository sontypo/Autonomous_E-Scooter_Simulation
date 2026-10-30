#!/usr/bin/env python3
"""Step-wise keyboard teleoperation using `pynput`.

ROS 2 port of the ROS 1 `cmd_vel-keyboard.py` that shipped at the repository
root. Unlike `teleop_escooter.py` this one does not need the terminal to stay
focused in raw mode, but it requires the optional `pynput` package:

    pip install pynput        # or: sudo apt install python3-pynput
"""

import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    from pynput import keyboard
except ImportError:  # pragma: no cover - optional dependency
    keyboard = None


# Define the initial velocities, step sizes, and maximum limits
R_w = 0.135
LINEAR_STEP = 0.4            # Linear velocity increment per key press
ANGULAR_STEP = 0.1           # Angular velocity increment per key press
MAX_LINEAR_VELOCITY = 8.333  # Maximum linear velocity
MAX_ANGULAR_VELOCITY = 1.0   # Maximum angular velocity


class KeyboardCmdVel(Node):

    def __init__(self):
        super().__init__('keyboard_cmd_vel_publisher')

        topic = self.declare_parameter('cmd_vel_topic', '/e_scooter/cmd_vel').value
        self.pub = self.create_publisher(Twist, topic, 1)

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.get_logger().info(
            "Use W/A/S/D keys to control the robot in steps, Esc to quit "
            "(publishing to '%s')." % topic)

    def on_press(self, key):
        twist = Twist()

        try:
            if key.char == 'w':
                self.linear_velocity += LINEAR_STEP    # Increase forward velocity
            elif key.char == 's':
                self.linear_velocity -= LINEAR_STEP    # Decrease forward velocity
            elif key.char == 'a':
                self.angular_velocity += ANGULAR_STEP  # Turn left more
            elif key.char == 'd':
                self.angular_velocity -= ANGULAR_STEP  # Turn right more

            # Enforce maximum limits
            self.linear_velocity = max(
                -MAX_LINEAR_VELOCITY, min(self.linear_velocity, MAX_LINEAR_VELOCITY))
            self.angular_velocity = max(
                -MAX_ANGULAR_VELOCITY, min(self.angular_velocity, MAX_ANGULAR_VELOCITY))

        except AttributeError:
            # Ignore non-character keys (like shift, ctrl)
            pass

        twist.linear.x = self.linear_velocity
        twist.angular.z = self.angular_velocity
        self.pub.publish(twist)

    @staticmethod
    def on_release(key):
        # Exit on 'esc' key press
        if key == keyboard.Key.esc:
            return False
        return True


def main():
    if keyboard is None:
        print(
            'keyboard_cmd_vel.py needs the `pynput` package '
            '(pip install pynput). Use teleop_escooter.py for a dependency-free '
            'alternative.',
            file=sys.stderr)
        return 1

    rclpy.init()
    node = KeyboardCmdVel()
    try:
        with keyboard.Listener(
                on_press=node.on_press, on_release=node.on_release) as listener:
            listener.join()
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
