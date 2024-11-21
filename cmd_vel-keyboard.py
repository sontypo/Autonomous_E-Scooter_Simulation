#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard

# Initialize the ROS node
rospy.init_node('keyboard_cmd_vel_publisher')

# Create a publisher for the cmd_vel topic
pub = rospy.Publisher('/e_scooter/cmd_vel', Twist, queue_size=1)

# Define the initial velocities, step sizes, and maximum limits
R_w = 0.135
linear_velocity = 0.0
angular_velocity = 0.0
linear_step = 0.4  # Linear velocity increment per key press
angular_step = 0.1  # Angular velocity increment per key press
max_linear_velocity = 8.333   # Maximum linear velocity
max_angular_velocity = 1.0  # Maximum angular velocity

# Function to handle key press events
def on_press(key):
    global linear_velocity, angular_velocity  # Access global variables for velocities
    twist = Twist()  # Create a new Twist message
    
    try:
        # Adjust velocities based on the key pressed
        if key.char == 'w':
            linear_velocity += linear_step  # Increase forward velocity
        elif key.char == 's':
            linear_velocity -= linear_step  # Decrease forward (or increase backward) velocity
        elif key.char == 'a':
            angular_velocity += angular_step  # Turn left more
        elif key.char == 'd':
            angular_velocity -= angular_step  # Turn right more

        # Enforce maximum limits
        linear_velocity = max(-max_linear_velocity, min(linear_velocity, max_linear_velocity))
        angular_velocity = max(-max_angular_velocity, min(angular_velocity, max_angular_velocity))

    except AttributeError:
        # Ignore non-character keys (like shift, ctrl)
        pass

    # Update the Twist message with the capped velocities
    twist.linear.x = linear_velocity
    twist.angular.z = angular_velocity

    # Publish the updated Twist message to the cmd_vel topic
    pub.publish(twist)

# Function to handle key release events
def on_release(key):
    # Exit on 'esc' key press
    if key == keyboard.Key.esc:
        return False

# Main function
def main():
    # Set up keyboard listener
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        rospy.loginfo("Use W/A/S/D keys to control the robot in steps, Esc to quit.")
        listener.join()  # Join listener thread to the main thread

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass