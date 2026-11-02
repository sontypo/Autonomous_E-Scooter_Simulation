// Copyright (c) Saun Nguyen (Hong-Son).
//
// ROS 2 port of the original ROS 1 `escooter_dynamics/src/control.cpp`.
//
// Behavioural differences versus the ROS 1 source are limited to defects that
// would otherwise break the port; each one is marked with a `PORT:` comment and
// listed in docs/PORTING.md.

#include "escooter_dynamics/control.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace escooter_dynamics
{

namespace
{
const double PI = atan(1.0) * 4.0;

template<typename T>
void getParam(const rclcpp::Node::SharedPtr & node, const std::string & name, T & value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, value);
  }
  node->get_parameter(name, value);
}
}  // namespace

//==============================================================================
// Constructor
//==============================================================================

Control::Control(const rclcpp::Node::SharedPtr & node)
: node_(node),
  logger_(node->get_logger().get_child("control")),
  clock_(node->get_clock()),
  last_time_odometry_(0, 0, node->get_clock()->get_clock_type()),
  last_time_cmd_vel_(0, 0, node->get_clock()->get_clock_type()),
  last_time_js_(0, 0, node->get_clock()->get_clock_type()),
  last_time_steer_(0, 0, node->get_clock()->get_clock_type())
{
  NAME_SPACE = "/e_scooter/";

  getParam(node_, "NAME_SPACE", NAME_SPACE);
  getParam(node_, "BODY_MAX_ROLL", BODY_MAX_ROLL);
  getParam(node_, "BODY_MAX_PITCH", BODY_MAX_PITCH);
  getParam(node_, "BODY_MAX_YAW", BODY_MAX_YAW);
  getParam(node_, "MAX_BODY_ROLL_COMP", MAX_BODY_ROLL_COMP);
  getParam(node_, "MAX_BODY_PITCH_COMP", MAX_BODY_PITCH_COMP);
  getParam(node_, "COMPENSATE_INCREMENT", COMPENSATE_INCREMENT);
  getParam(node_, "COMPENSATE_TO_WITHIN", COMPENSATE_TO_WITHIN);
  getParam(node_, "MASTER_LOOP_RATE", MASTER_LOOP_RATE);
  getParam(node_, "VELOCITY_DIVISION", VELOCITY_DIVISION);
  getParam(node_, "NUMBER_OF_JOINTS", NUMBER_OF_JOINTS);
  getParam(node_, "WHEEL_RADIUS", WHEEL_RADIUS);
  getParam(node_, "WHEEL_SEPARATION_DIS", WHEEL_SEPARATION_DIS);
  getParam(node_, "MAX_STEERING_ANGLE", MAX_STEERING_ANGLE);
  getParam(node_, "STEERING_MODE", STEERING_MODE);
  getParam(node_, "STEERING_MIN_SPEED", STEERING_MIN_SPEED);
  getParam(node_, "MAX_STEERING_RATE", MAX_STEERING_RATE);
  getParam(node_, "WHEEL_RATE_FROM_SPEED", WHEEL_RATE_FROM_SPEED);
  getParam(node_, "MAX_LINEAR_ACCEL", MAX_LINEAR_ACCEL);
  getParam(node_, "WHOLE_BODY_WEIGHT", WHOLE_BODY_WEIGHT);
  getParam(node_, "COM_POS_Z", COM_POS_Z);

  // Frames and optional odometry TF (the ROS 1 broadcaster was commented out,
  // so the default keeps it off).
  odom_frame_id_ = "odom";
  base_frame_id_ = "base_link";
  getParam(node_, "ODOM_FRAME_ID", odom_frame_id_);
  getParam(node_, "BASE_FRAME_ID", base_frame_id_);
  getParam(node_, "PUBLISH_ODOM_TF", publish_odom_tf_);

  // Topic names, defaulting to the ROS 1 names so existing tooling keeps working.
  std::string joint_states_topic = NAME_SPACE + "joint_states";
  std::string cmd_vel_topic = NAME_SPACE + "cmd_vel";
  std::string imu_topic = "/imu";
  std::string odom_topic = NAME_SPACE + "odometry/calculated";
  std::string twist_topic = "/twist";
  getParam(node_, "JOINT_STATES_TOPIC", joint_states_topic);
  getParam(node_, "CMD_VEL_TOPIC", cmd_vel_topic);
  getParam(node_, "IMU_TOPIC", imu_topic);
  getParam(node_, "ODOM_TOPIC", odom_topic);
  getParam(node_, "TWIST_TOPIC", twist_topic);

  // PORT: ROS 2 has no nested-dictionary parameters, so the ROS 1
  // `JOINTS: {'0': {name:..., sign:...}}` map became two index-aligned arrays.
  // The index semantics (0 = handle bar, 1 = rotational plate, 2 = front wheel)
  // are unchanged.
  joint_names_array_ = {
    "handle_bar_chasis_joint", "rotational_plate_joint",
    "front_wheel_joint", "back_wheel_joint"};
  joint_orientation_array_ = {1, 1, -1, -1};
  getParam(node_, "JOINT_NAMES", joint_names_array_);
  getParam(node_, "JOINT_SIGNS", joint_orientation_array_);

  // Find out how many servos/joints we have
  const int available = static_cast<int>(
    std::min(joint_names_array_.size(), joint_orientation_array_.size()));
  if (NUMBER_OF_JOINTS > available) {
    RCLCPP_WARN(
      logger_,
      "NUMBER_OF_JOINTS is %d but only %d JOINT_NAMES/JOINT_SIGNS entries were given; "
      "clamping to %d.", NUMBER_OF_JOINTS, available, available);
    NUMBER_OF_JOINTS = available;
  }
  if (NUMBER_OF_JOINTS < 3) {
    RCLCPP_FATAL(
      logger_, "The e-scooter needs at least 3 actuated joints, got %d.", NUMBER_OF_JOINTS);
    throw std::runtime_error("escooter_dynamics: NUMBER_OF_JOINTS < 3");
  }

  for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
    RCLCPP_INFO_STREAM(
      logger_, "Joint_name[" << i << "]: " << joint_names_array_[i] <<
        " (sign " << joint_orientation_array_[i] << ")");
  }

  prev_scooter_state_ = false;
  scooter_state_ = false;
  imu_init_stored_ = false;
  imu_roll_lowpass_ = 0.0;
  imu_pitch_lowpass_ = 0.0;
  imu_yaw_lowpass_ = 0.0;
  imu_roll_init_ = 0.0;
  imu_pitch_init_ = 0.0;

  // Topics we are subscribing
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic, rclcpp::SensorDataQoS(),
    std::bind(&Control::jointStateCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, rclcpp::QoS(1),
    std::bind(&Control::cmd_velCallback, this, std::placeholders::_1));
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(),
    std::bind(&Control::imuCallback, this, std::placeholders::_1));

  // Topics we are publishing.
  // PORT: ros2_control's forward command controllers take a Float64MultiArray on
  // `<controller>/commands` instead of ROS 1's Float64 on `<controller>/command`.
  handle_bar_jpc = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    NAME_SPACE + joint_names_array_[0] + "_position_controller/commands", 10);
  rotational_plate_jec = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    NAME_SPACE + joint_names_array_[1] + "_effort_controller/commands", 10);
  front_wheel_jvc = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    NAME_SPACE + joint_names_array_[2] + "_velocity_controller/commands", 10);

  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 50);
  twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    twist_topic, 50);

  if (publish_odom_tf_) {
    odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  }

  RCLCPP_INFO(
    logger_,
    "escooter_dynamics control interface ready "
    "(loop %d Hz, joint_states '%s', cmd_vel '%s', imu '%s').",
    MASTER_LOOP_RATE, joint_states_topic.c_str(), cmd_vel_topic.c_str(), imu_topic.c_str());
}

//==============================================================================
// tick: seconds elapsed since `last`, latching `last` to the current time
//==============================================================================

double Control::tick(rclcpp::Time & last) const
{
  const rclcpp::Time now = clock_->now();
  if (last.nanoseconds() == 0 || now <= last) {
    last = now;
    return 0.0;
  }
  const double dt = (now - last).seconds();
  last = now;
  return dt;
}

//==============================================================================
// Getter and Setters
//==============================================================================

void Control::setScooterState(bool state)
{
  scooter_state_ = state;
}

bool Control::getScooterState() const
{
  return scooter_state_;
}

void Control::setPrevScooterState(bool state)
{
  prev_scooter_state_ = state;
}

bool Control::getPrevScooterState() const
{
  return prev_scooter_state_;
}

//==============================================================================
// Odometry Publisher
//==============================================================================

void Control::publishOdometry(double distance_traveled, double delta)
{
  // calculate time elapsed
  const rclcpp::Time current_time_odometry = clock_->now();
  const double dt = tick(last_time_odometry_);
  if (dt <= 0.0) {
    // Clock has not advanced yet (first iteration, or simulation paused).
    prev_distance_traveled_ = distance_traveled;
    return;
  }

  // PORT: `distance_traveled` is the odometer reading accumulated by
  // jointStateCallback, not a per-cycle increment. The ROS 1 code integrated the
  // full odometer every loop iteration, so the pose grew quadratically and
  // diverged within seconds. Differencing it restores dead reckoning while
  // keeping the published API identical.
  const double delta_s = distance_traveled - prev_distance_traveled_;
  prev_distance_traveled_ = distance_traveled;

  double delta_psi = (delta_s / WHEEL_SEPARATION_DIS) * tan(delta);
  psi_ += delta_psi;

  double delta_x = delta_s * cos(psi_);
  double delta_y = delta_s * sin(psi_);
  pose_x_ += delta_x;
  pose_y_ += delta_y;

  double v_x = delta_x / dt;
  double v_y = delta_y / dt;
  double v_psi = delta_psi / dt;

  // Quaternion created from yaw
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, psi_);
  const geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

  // first, we'll publish the transform over tf
  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time_odometry;
  odom_trans.header.frame_id = odom_frame_id_;
  odom_trans.child_frame_id = base_frame_id_;

  odom_trans.transform.translation.x = pose_x_;
  odom_trans.transform.translation.y = pose_y_;
  odom_trans.transform.translation.z = body_frame_.orientation.yaw;
  odom_trans.transform.rotation = odom_quat;

  // Only used if debugging calculated odometry; enable with PUBLISH_ODOM_TF.
  if (odom_broadcaster_) {
    odom_broadcaster_->sendTransform(odom_trans);
  }

  // next, we'll publish the odometry message over ROS
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = current_time_odometry;
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id = base_frame_id_;

  // set the position
  odom.pose.pose.position.x = pose_x_;
  odom.pose.pose.position.y = pose_y_;
  odom.pose.pose.position.z = body_frame_.orientation.yaw;
  odom.pose.pose.orientation = odom_quat;

  odom.pose.covariance[0] = 0.00001;   // x
  odom.pose.covariance[7] = 0.00001;   // y
  odom.pose.covariance[14] = 0.00001;  // z
  odom.pose.covariance[21] = 0.001;    // rot x
  odom.pose.covariance[28] = 0.001;    // rot y
  odom.pose.covariance[35] = 0.001;    // rot z

  // set the velocity
  odom.twist.twist.linear.x = v_x;
  odom.twist.twist.linear.y = v_y;
  odom.twist.twist.angular.z = v_psi;
  odom.twist.covariance = odom.pose.covariance;  // needed?

  odom_pub_->publish(odom);
}

//==============================================================================
// Twist Publisher
//==============================================================================

void Control::publishTwist(const geometry_msgs::msg::Twist & com_vel)
{
  geometry_msgs::msg::TwistWithCovarianceStamped twistStamped;
  twistStamped.header.stamp = clock_->now();
  twistStamped.header.frame_id = odom_frame_id_;

  twistStamped.twist.twist.linear.x = com_vel.linear.x;
  twistStamped.twist.twist.linear.y = com_vel.linear.y;
  twistStamped.twist.twist.angular.z = com_vel.angular.z;

  twistStamped.twist.covariance[0] = 0.00001;             // x
  twistStamped.twist.covariance[7] = 0.00001;             // y
  twistStamped.twist.covariance[14] = 0.00001;            // z
  twistStamped.twist.covariance[21] = 1000000000000.0;    // rot x
  twistStamped.twist.covariance[28] = 1000000000000.0;    // rot y
  twistStamped.twist.covariance[35] = 0.001;              // rot z

  twist_pub_->publish(twistStamped);
}

//==============================================================================
// Steering: operator request plus steer-to-balance
//==============================================================================

void Control::updateSteering()
{
  const double dt = tick(last_time_steer_);

  // Ramp the speed command.
  //
  // front_wheel_joint is velocity controlled, so a step in the command makes its
  // PID slam the drive torque, and the reaction of that torque tips a vehicle
  // balanced on two in-line wheels. Measured: stepping straight to 2 m/s drove
  // the wheel from 0 to -11.5 rad/s in 200 ms and had the scooter at -67 degrees
  // 200 ms after that. Teleop already ramps its own output; this bounds whatever
  // else publishes cmd_vel.
  if (MAX_LINEAR_ACCEL > 0.0 && dt > 0.0) {
    const double max_step = MAX_LINEAR_ACCEL * dt;
    Vx_ += std::clamp(rider_speed_ - Vx_, -max_step, max_step);
  } else {
    Vx_ = rider_speed_;
  }
  Vx_dot_ = (dt > 0.0) ? ((Vx_ - prev_Vx_) / dt) : 0.0;
  prev_Vx_ = Vx_;

  // cmd_vel.linear.x is a ground speed in m/s but front_wheel_joint takes a
  // wheel rate in rad/s. ROS 1 passed the number through unconverted - the
  // author's `TODO: need to convert to joint command later` - so the scooter
  // moved at v*WHEEL_RADIUS, 7.4 times slower than asked, while
  // Dynamics::constraints() read the same number as a speed and overestimated
  // psi_dot by that factor.
  joints_.front_wheel = (WHEEL_RATE_FROM_SPEED && WHEEL_RADIUS > 0.0) ?
    (Vx_ / WHEEL_RADIUS) : Vx_;

  const double target =
    std::clamp(rider_delta_, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);

  // Slew limit so the bars move quickly but not instantly.
  if (MAX_STEERING_RATE > 0.0 && dt > 0.0) {
    const double max_step = MAX_STEERING_RATE * dt;
    des_delta += std::clamp(target - des_delta, -max_step, max_step);
  } else {
    des_delta = target;
  }

  joints_.handle_bar = des_delta;
}

void Control::publishJointCommand(const escooter_msgs::msg::Joints & joints)
{
  std_msgs::msg::Float64MultiArray jpc_command;
  jpc_command.data = {joint_orientation_array_[0] * joints.handle_bar};
  handle_bar_jpc->publish(jpc_command);

  std_msgs::msg::Float64MultiArray jec_command;
  jec_command.data = {joint_orientation_array_[1] * joints.rotational_plate};
  rotational_plate_jec->publish(jec_command);

  std_msgs::msg::Float64MultiArray jvc_command;
  jvc_command.data = {joint_orientation_array_[2] * joints.front_wheel};
  front_wheel_jvc->publish(jvc_command);
}

//==============================================================================
// Topics we subscribe to
//==============================================================================

//==============================================================================
// JointState callback
//==============================================================================
void Control::jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  const double dt = tick(last_time_js_);

  joint_state_ = *msg;

  const std::string & front_wheel_joint_name = joint_names_array_[2];
  const std::string & handle_bar_chasis_joint_name = joint_names_array_[0];
  const std::string & rotational_plate_joint_name = joint_names_array_[1];
  double front_wheel_angle = prev_front_wheel_angle;
  double steering_velocity = std::numeric_limits<double>::quiet_NaN();

  // PORT: the ROS 1 loop `break`s after either match, so whichever joint came
  // first in the message hid the other one. Both are read now.
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); i++) {
    if (msg->name[i] == front_wheel_joint_name) {
      front_wheel_angle = msg->position[i];
    } else if (msg->name[i] == handle_bar_chasis_joint_name) {
      steering_angle = msg->position[i];
      if (i < msg->velocity.size()) {
        steering_velocity = msg->velocity[i];
      }
    } else if (msg->name[i] == rotational_plate_joint_name &&
      i < msg->velocity.size())
    {
      plate_velocity_ = msg->velocity[i];
    }
  }

  // Calculate incremental distance traveled from wheel rotation
  double delta_angle = front_wheel_angle - prev_front_wheel_angle;
  double delta_distance = WHEEL_RADIUS * delta_angle;
  distance_traveled_ += delta_distance;    // Calculate the real feedback distance

  double delta_steering = steering_angle - prev_steering_angle;
  delta_ += delta_steering;
  // Prefer the joint's reported velocity over differencing its position, for the
  // same reason the roll rate comes from the gyro: the quotient is noisy and
  // feeds the psi_2dot_ term of the balancing feed-forward.
  if (std::isfinite(steering_velocity)) {
    delta_dot_ = steering_velocity;
  } else {
    delta_dot_ = (dt > 0.0) ? ((delta_ - prev_delta_) / dt) : 0.0;
  }

  prev_front_wheel_angle = front_wheel_angle;
  prev_steering_angle = steering_angle;
  prev_delta_ = delta_;
}

//==============================================================================
// cmd_vel callback
//==============================================================================

void Control::cmd_velCallback(const geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel_msg)
{
  const double dt = tick(last_time_cmd_vel_);
  if (dt <= 0.0) {
    return;
  }

  cmd_vel_ = *cmd_vel_msg;

  rider_speed_ = cmd_vel_msg->linear.x;
  const double omega = cmd_vel_msg->angular.z;

  // PORT: longitudinal acceleration.
  //
  // ROS 1 wrote `Vx_dot_ = Vx_ / dt`, which is a speed divided by a sample
  // interval, not an acceleration. At a steady 2 m/s with cmd_vel at 20 Hz it
  // reports 40 m/s^2, and Dynamics::constraints() multiplies that straight into
  // psi_2dot_, so the balancing feed-forward picked up a fictitious 114 N.m the
  // moment the scooter was driven and steered at the same time. That is what
  // made it lurch through +-87 degrees under teleop.


  // PORT: how angular.z becomes a steering angle.
  //
  // ROS 1 integrated it - `des_delta += angular.z * dt` - with the author's own
  // `TODO: it should be delta` next to it. Teleop publishes a held value, so one
  // key press ramped the steering to the 45 degree stop over about 8 s and left
  // it there; releasing to angular.z = 0 froze the bars hard over instead of
  // centring them, and reversing meant waiting out the same ramp the other way.
  // That is the "steering does not respond" behaviour.
  //
  // The default now follows the ROS convention for cmd_vel on a bicycle:
  // angular.z is the desired yaw rate, and the bicycle model inverts it into a
  // steering angle, delta = atan(L * omega / v). Centring is immediate because
  // omega = 0 gives delta = 0.
  double target_delta = 0.0;
  if (STEERING_MODE == "rate") {
    // Legacy ROS 1 behaviour: angular.z is a steering rate.
    target_delta = des_delta + omega * dt;
  } else if (STEERING_MODE == "angle") {
    // angular.z is the steering angle itself, in rad.
    target_delta = omega;
  } else {
    // "yaw_rate": the bicycle model. Below STEERING_MIN_SPEED the inversion is
    // singular - a bicycle cannot change heading while stationary - so the speed
    // is floored, which keeps the bars responsive while parked.
    double v_eff = Vx_;
    if (std::abs(v_eff) < STEERING_MIN_SPEED) {
      v_eff = (v_eff < 0.0) ? -STEERING_MIN_SPEED : STEERING_MIN_SPEED;
    }
    target_delta = std::atan(WHEEL_SEPARATION_DIS * omega / v_eff);
  }

  // The operator's request. updateSteering() adds the steer-to-balance term and
  // does the clamping and slew limiting, because it runs every control cycle
  // while this callback only fires when a cmd_vel arrives.
  rider_delta_ = std::clamp(target_delta, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);

}

//==============================================================================
// Override IMU and manipulate body orientation callback
//==============================================================================

//==============================================================================
// IMU callback to auto-level body if on non level ground
//==============================================================================

void Control::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  const geometry_msgs::msg::Vector3 & lin_acc = imu_msg->linear_acceleration;

  if (imu_init_stored_ == false) {
    imu_roll_init_ = atan2(lin_acc.x, sqrt(lin_acc.y * lin_acc.y + lin_acc.z * lin_acc.z));
    imu_pitch_init_ = -atan2(lin_acc.y, lin_acc.z);
    imu_pitch_init_ = (imu_pitch_init_ >= 0.0) ?
      (PI - imu_pitch_init_) : (-imu_pitch_init_ - PI);
    imu_init_stored_ = true;
  }

  const tf2::Quaternion quat(
    imu_msg->orientation.x,
    imu_msg->orientation.y,
    imu_msg->orientation.z,
    imu_msg->orientation.w);

  double imu_roll, imu_pitch, imu_yaw;
  tf2::Matrix3x3(quat).getRPY(imu_roll, imu_pitch, imu_yaw);

  body_roll_rate_ = imu_msg->angular_velocity.x;

  body_frame_.orientation.roll = imu_roll;
  body_frame_.orientation.pitch = imu_pitch;
  body_frame_.orientation.yaw = imu_yaw;
}

}  // namespace escooter_dynamics
