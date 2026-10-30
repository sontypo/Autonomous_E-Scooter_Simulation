#!/usr/bin/env python3
"""Republish the IMU orientation as roll/pitch/yaw.

ROS 2 port of the ROS 1 `euler_angle.py` that shipped at the repository root.
`tf.transformations` no longer exists in ROS 2, so the quaternion conversion is
done with `tf_transformations` when available and with a small local fallback
otherwise (identical ZYX convention, matching tf2's `Matrix3x3::getRPY`).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

try:
    from tf_transformations import euler_from_quaternion
except ImportError:  # pragma: no cover - optional dependency
    def euler_from_quaternion(quaternion):
        """Return (roll, pitch, yaw) for an (x, y, z, w) quaternion."""
        x, y, z, w = quaternion

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


class IMUToEuler(Node):

    def __init__(self):
        super().__init__('imu_to_euler')

        imu_topic = self.declare_parameter('imu_topic', '/imu').value
        euler_topic = self.declare_parameter('euler_topic', '/euler_angles').value
        self._verbose = self.declare_parameter('verbose', False).value

        self.imu_subscriber = self.create_subscription(
            Imu, imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.euler_publisher = self.create_publisher(Float64MultiArray, euler_topic, 10)

        self.get_logger().info(
            "IMU to Euler converter node started ('%s' -> '%s')." % (imu_topic, euler_topic))

    def imu_callback(self, msg):
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

        roll, pitch, yaw = euler_from_quaternion(quaternion)

        euler_msg = Float64MultiArray()
        euler_msg.data = [roll, pitch, yaw]

        self.euler_publisher.publish(euler_msg)

        if self._verbose:
            self.get_logger().info(
                'Published Euler angles: Roll=%f, Pitch=%f, Yaw=%f' % (roll, pitch, yaw))


def main():
    rclpy.init()
    node = IMUToEuler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('IMU to Euler converter node terminated.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
