#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import tf.transformations as tf

class IMUToEuler:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('imu_to_euler', anonymous=True)
        
        # Subscriber for the IMU topic
        self.imu_subscriber = rospy.Subscriber('/imu', Imu, self.imu_callback)
        
        # Publisher for the Euler angles topic
        self.euler_publisher = rospy.Publisher('/euler_angles', Float64MultiArray, queue_size=10)
        
        rospy.loginfo("IMU to Euler converter node started.")

    def imu_callback(self, msg):
        # Extract quaternion from the IMU message
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        
        # Convert quaternion to Euler angles
        roll, pitch, yaw = tf.euler_from_quaternion(quaternion)
        
        # Create a message with the Euler angles
        euler_msg = Float64MultiArray()
        euler_msg.data = [roll, pitch, yaw]
        
        # Publish the Euler angles
        self.euler_publisher.publish(euler_msg)
        
        rospy.loginfo(f"Published Euler angles: Roll={roll}, Pitch={pitch}, Yaw={yaw}")

if __name__ == '__main__':
    try:
        IMUToEuler()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("IMU to Euler converter node terminated.")
