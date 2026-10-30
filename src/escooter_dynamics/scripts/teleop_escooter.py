#!/usr/bin/env python3

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# ROS 2 port of the ROS 1 `teleop.py` that shipped at the repository root.

import os
import select
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import time

    import msvcrt
else:
    import termios
    import tty

DIFFER_MAX_LIN_VEL = 10.0
DIFFER_MAX_ANG_VEL = 3.2
LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your Scooter!
---------------------------
Moving around:
        w
   a    s    d
        x


space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

settings = None


def get_key():
    if os.name == 'nt':
        timeout = 0.1
        start_time = time.time()
        while True:
            if msvcrt.kbhit():
                return msvcrt.getch().decode()
            elif time.time() - start_time > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(target_linear_vel, target_angular_vel):
    return 'currently:\tlinear vel %s\t angular vel %s ' % (
        target_linear_vel, target_angular_vel)


def make_simple_profile(output, target, slop):
    if target > output:
        output = min(target, output + slop)
    elif target < output:
        output = max(target, output - slop)
    else:
        output = target

    return output


def constrain(value, low, high):
    return max(low, min(value, high))


def check_linear_limit_velocity(vel):
    return constrain(vel, -DIFFER_MAX_LIN_VEL, DIFFER_MAX_LIN_VEL)


def check_angular_limit_velocity(vel):
    return constrain(vel, -DIFFER_MAX_ANG_VEL, DIFFER_MAX_ANG_VEL)


def main():
    global settings
    if os.name != 'nt':
        if not sys.stdin.isatty():
            print(
                'teleop_escooter.py reads the keyboard directly and needs an '
                'interactive terminal. Run it in its own terminal, or publish '
                'to the topic yourself:\n'
                '  ros2 topic pub -r 20 /e_scooter/cmd_vel geometry_msgs/msg/Twist '
                "'{linear: {x: 1.0}, angular: {z: 0.1}}'",
                file=sys.stderr)
            return 1
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    node = Node('teleop_node')
    topic = node.declare_parameter('cmd_vel_topic', '/e_scooter/cmd_vel').value
    pub = node.create_publisher(Twist, topic, QoSProfile(depth=10))

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        print("publishing to '%s'" % topic)
        while rclpy.ok():
            key = get_key()
            if key == 'w':
                target_linear_vel = check_linear_limit_velocity(
                    target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'x':
                target_linear_vel = check_linear_limit_velocity(
                    target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'a':
                target_angular_vel = check_angular_limit_velocity(
                    target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'd':
                target_angular_vel = check_angular_limit_velocity(
                    target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == ' ' or key == 's':
                target_linear_vel = 0.0
                control_linear_vel = 0.0
                target_angular_vel = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            else:
                if key == '\x03':
                    break

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()

            control_linear_vel = make_simple_profile(
                control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0))
            twist.linear.x = control_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            control_angular_vel = make_simple_profile(
                control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0))
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_vel

            pub.publish(twist)

    except Exception:  # noqa: BLE001 - mirrors the ROS 1 script's catch-all
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
