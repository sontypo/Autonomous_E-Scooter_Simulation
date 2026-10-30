// Copyright (c) Saun Nguyen (Hong-Son).
//
// Rigid-body / balancing dynamics of the autonomous e-scooter.
// ROS 2 port of the original ROS 1 `escooter_dynamics/dynamics.h`.
// The equations of motion are carried over unchanged; only the ROS plumbing
// (node handle, parameters, clock) has been migrated to rclcpp.

#ifndef ESCOOTER_DYNAMICS__DYNAMICS_HPP_
#define ESCOOTER_DYNAMICS__DYNAMICS_HPP_

#include <cmath>
#include <limits>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <escooter_msgs/msg/joints.hpp>
#include <escooter_msgs/msg/pose.hpp>
#include <escooter_msgs/msg/rpy.hpp>

namespace escooter_dynamics
{

//=============================================================================
// Define structs and classes for Dynamics system
//=============================================================================

struct Trig
{
  double sine;
  double cosine;
};

class Dynamics
{
public:
  explicit Dynamics(const rclcpp::Node::SharedPtr & node);

  void updateDynamics(
    double theta, escooter_msgs::msg::Joints * joints, double tau, double Vx);
  void constraints(double Vx, double Vx_dot, double delta, double delta_dot);
  /// \param theta_dot_measured  body roll rate in rad/s, straight from the IMU.
  ///        Pass NaN to fall back to differentiating the roll angle.
  /// \param plate_velocity  reaction plate speed in rad/s, for momentum
  ///        management; see MOMENTUM_UNLOAD_GAIN.
  void balancingController(
    const escooter_msgs::msg::Pose & body_frame, double Vx,
    double theta_dot_measured = std::numeric_limits<double>::quiet_NaN(),
    double plate_velocity = 0.0);
  double computeCOMVelocity(
    double Vx, double theta, double theta_dot, double psi, double psi_dot);

  double psi_dot_{0.0}, psi_2dot_{0.0};
  double V_x{0.0}, V_y{0.0}, V_z{0.0};
  double theta_ = 0.0;
  double tau_ = 0.0;

private:
  static Trig getSinCos(double rad_angle);

  /// Elapsed time since `last` in seconds, latching `last` to `now`.
  /// Returns 0.0 on the very first call (or whenever the clock has not moved),
  /// which lets every integrator skip a step instead of dividing by zero.
  /// The ROS 1 version left these stamps default-constructed, which produced a
  /// first `dt` of several decades under system time and of 0 under sim time.
  double tick(rclcpp::Time & last) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Time last_time_dynamics_, last_time_blc_;

  double COM_POS_X{0.0}, COM_POS_Y{0.0}, COM_POS_Z{0.0};  // initial position of the COM
  double BODY_WEIGHT{0.0}, R_PLATE_WEIGHT{0.0}, F_WHEEL_WEIGHT{0.0}, HANDLE_BAR_WEIGHT{0.0};
  double WHOLE_BODY_WEIGHT{0.0}, WHOLE_BODY_WEIGHT_EXC{0.0};
  double WHEEL_RADIUS{0.0}, WHEEL_SEPARATION_DIS{1.0}, B_WHEEL_TO_COM{0.0};
  double PD_PDplus{0.0}, KP{0.0}, KD{0.0};
  double MAX_BALANCING_TORQUE{500.0};   // matches the joint's effort limit
  double MAX_MODEL_ROLL_RATE{50.0};     // bound on the open-loop model state
  double MOMENTUM_UNLOAD_GAIN{0.0};     // rad of lean per rad/s of plate speed
  double MAX_LEAN_SETPOINT{0.035};      // 2 degrees
  double theta_ref_{0.0};               // biased upright setpoint, for logging
  double UNCERTAINTIES{0.0};
  double C_{0.0}, G_{0.0}, CpG_{0.0};
  double theta_dot_{0.0}, pre_theta_{0.0};
  double theta_2dot_{0.0}, I_theta{0.0}, I_rp_x{0.0}, I_rp_y{0.0}, I_rp_z{0.0};
};

}  // namespace escooter_dynamics

#endif  // ESCOOTER_DYNAMICS__DYNAMICS_HPP_
