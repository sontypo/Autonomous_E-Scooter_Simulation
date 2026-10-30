// Copyright (c) Saun Nguyen (Hong-Son).
//
// ROS interface of the e-scooter controller: reads joint states / IMU / cmd_vel
// and drives the three ros2_control command controllers.
// ROS 2 port of the original ROS 1 `escooter_dynamics/control.h`.

#ifndef ESCOOTER_DYNAMICS__CONTROL_HPP_
#define ESCOOTER_DYNAMICS__CONTROL_HPP_

#include <cmath>
#include <memory>
#include <string>
#include <cstdint>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>

#include <escooter_msgs/msg/joints.hpp>
#include <escooter_msgs/msg/pose.hpp>
#include <escooter_msgs/msg/rpy.hpp>

namespace escooter_dynamics
{

class Control
{
public:
  explicit Control(const rclcpp::Node::SharedPtr & node);

  void setScooterState(bool state);
  bool getScooterState() const;
  void setPrevScooterState(bool state);
  bool getPrevScooterState() const;

  void publishJointCommand(const escooter_msgs::msg::Joints & joints);
  void publishOdometry(double distance_traveled, double delta);
  void publishTwist(const geometry_msgs::msg::Twist & com_vel);

  int MASTER_LOOP_RATE{500};   // Master loop rate [Hz]
  sensor_msgs::msg::JointState joint_state_;
  escooter_msgs::msg::Pose body_frame_;    // Body link rotation
  // Roll rate as the IMU measures it, in rad/s. Differentiating
  // body_frame_.orientation.roll instead aliases badly: the sensor runs at
  // 100 Hz while the control loop runs at MASTER_LOOP_RATE = 500 Hz, so the
  // quotient is zero four cycles out of five and five times too large on the
  // fifth.
  double body_roll_rate_ = 0.0;
  /// Reaction plate speed in rad/s, from /joint_states.
  double plate_velocity_ = 0.0;
  escooter_msgs::msg::Joints joints_;
  geometry_msgs::msg::Twist com_vel_;
  geometry_msgs::msg::Twist cmd_vel_;
  double distance_traveled_ = 0.0;
  double delta_ = 0.0;
  double prev_delta_ = 0.0;
  double delta_dot_ = 0.0;
  double Vx_ = 0.0, des_delta = 0.0;
  double Vx_dot_ = 0.0;

private:
  /// Elapsed time since `last` in seconds, latching `last` to `now`.
  /// Returns 0.0 on the first call and whenever the clock has not advanced.
  double tick(rclcpp::Time & last) const;

  // Topics we are subscribing
  void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void cmd_velCallback(const geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel_msg);
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);

  bool imu_init_stored_;   // Auto-levelling
  double imu_roll_lowpass_, imu_pitch_lowpass_, imu_yaw_lowpass_;
  double imu_roll_init_, imu_pitch_init_;
  // Auto-levelling yaml config
  double MAX_BODY_ROLL_COMP{0.0}, MAX_BODY_PITCH_COMP{0.0};
  double COMPENSATE_INCREMENT{0.0}, COMPENSATE_TO_WITHIN{0.0}, MAX_STEERING_ANGLE{0.0};
  // Mechanical limits
  double BODY_MAX_ROLL{0.0}, BODY_MAX_PITCH{0.0}, BODY_MAX_YAW{0.0};
  double VELOCITY_DIVISION{0.0}, WHEEL_RADIUS{0.0}, WHEEL_SEPARATION_DIS{1.0};
  int NUMBER_OF_JOINTS{3};
  double pose_x_ = 0.0;   // pose odometry
  double pose_y_ = 0.0;   // pose odometry
  double psi_ = 0.0;      // pose odometry
  double prev_distance_traveled_ = 0.0;
  double prev_front_wheel_angle = 0.0;
  double prev_steering_angle = 0.0;
  double steering_angle = 0.0;
  double des_theta = 0.0;
  std::vector<std::string> joint_names_array_;
  std::vector<int64_t> joint_orientation_array_;  // ROS 2 integer arrays are int64
  std::string NAME_SPACE;
  std::string odom_frame_id_, base_frame_id_;
  bool publish_odom_tf_{false};

  bool scooter_state_;        // Current loop state
  bool prev_scooter_state_;   // Previous loop state

  // Declared before the timestamps below: the constructor initialiser list
  // seeds the clock type from clock_, and C++ initialises in declaration order.
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Time last_time_odometry_, last_time_cmd_vel_, last_time_js_;

  // Topics we are subscribing
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Topics we are publishing
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr handle_bar_jpc;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rotational_plate_jec;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr front_wheel_jvc;

  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
};

}  // namespace escooter_dynamics

#endif  // ESCOOTER_DYNAMICS__CONTROL_HPP_
