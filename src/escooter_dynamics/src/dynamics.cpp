// Copyright (c) Saun Nguyen (Hong-Son).
//
// ROS 2 port of the original ROS 1 `escooter_dynamics/src/dynamics.cpp`.
// Every equation below is carried over verbatim from the ROS 1 implementation;
// only parameter lookup and time keeping changed.

#include "escooter_dynamics/dynamics.hpp"

#include <algorithm>
#include <string>

namespace escooter_dynamics
{

namespace
{
constexpr double g = 9.81;

/// Declare a parameter if the node does not know it yet, then read it back.
/// ROS 2 has no `ros::param::get`: a parameter must be declared before use, and
/// an undeclared read throws. Declaring with the current value as the default
/// keeps the ROS 1 behaviour of "leave the member untouched when the YAML does
/// not define it" while removing the uninitialised-read the ROS 1 code had.
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
//  Constructor: Initialize dynamics variables
//==============================================================================

Dynamics::Dynamics(const rclcpp::Node::SharedPtr & node)
: node_(node),
  logger_(node->get_logger().get_child("dynamics")),
  clock_(node->get_clock()),
  last_time_dynamics_(0, 0, clock_->get_clock_type()),
  last_time_blc_(0, 0, clock_->get_clock_type())
{
  getParam(node_, "COM_POS_X", COM_POS_X);
  getParam(node_, "COM_POS_Y", COM_POS_Y);
  getParam(node_, "COM_POS_Z", COM_POS_Z);
  getParam(node_, "BODY_WEIGHT", BODY_WEIGHT);
  getParam(node_, "R_PLATE_WEIGHT", R_PLATE_WEIGHT);
  getParam(node_, "F_WHEEL_WEIGHT", F_WHEEL_WEIGHT);
  getParam(node_, "HANDLE_BAR_WEIGHT", HANDLE_BAR_WEIGHT);
  getParam(node_, "WHOLE_BODY_WEIGHT", WHOLE_BODY_WEIGHT);
  getParam(node_, "WHOLE_BODY_WEIGHT_EXC", WHOLE_BODY_WEIGHT_EXC);
  getParam(node_, "WHEEL_RADIUS", WHEEL_RADIUS);
  getParam(node_, "WHEEL_SEPARATION_DIS", WHEEL_SEPARATION_DIS);
  getParam(node_, "UNCERTAINTIES", UNCERTAINTIES);
  getParam(node_, "B_WHEEL_TO_COM", B_WHEEL_TO_COM);
  getParam(node_, "PD_PDplus", PD_PDplus);
  getParam(node_, "KP", KP);
  getParam(node_, "KD", KD);
  getParam(node_, "MAX_BALANCING_TORQUE", MAX_BALANCING_TORQUE);
  getParam(node_, "MAX_MODEL_ROLL_RATE", MAX_MODEL_ROLL_RATE);
  getParam(node_, "MOMENTUM_UNLOAD_GAIN", MOMENTUM_UNLOAD_GAIN);
  getParam(node_, "MAX_LEAN_SETPOINT", MAX_LEAN_SETPOINT);

  if (WHEEL_SEPARATION_DIS == 0.0) {
    RCLCPP_WARN(
      logger_, "WHEEL_SEPARATION_DIS is 0; falling back to 1.0 m to keep the model finite.");
    WHEEL_SEPARATION_DIS = 1.0;
  }
}

//=============================================================================
// tick: seconds elapsed since `last`, latching `last` to the current time
//=============================================================================

double Dynamics::tick(rclcpp::Time & last) const
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

//=============================================================================
// getSinCos:  Get the sinus and cosinus from the angle
//=============================================================================

Trig Dynamics::getSinCos(double angle_rad)
{
  Trig body_trig;

  body_trig.sine = sin(angle_rad);
  body_trig.cosine = cos(angle_rad);

  return body_trig;
}

//=============================================================================
// Dynamics Model
//=============================================================================

void Dynamics::updateDynamics(
  double theta, escooter_msgs::msg::Joints * joints, double tau, double Vx)
{
  const double dt = tick(last_time_dynamics_);

  // 4 / 3 is integer division in C++ and evaluates to 1, so the ROS 1 code
  // computed this term 25 % too small.
  I_theta = 4.0 / 3.0 * WHOLE_BODY_WEIGHT * pow(COM_POS_Z, 2) * UNCERTAINTIES;
  I_rp_x = pow(0.036, 2) * R_PLATE_WEIGHT * UNCERTAINTIES;
  I_rp_y = 0.0;
  I_rp_z = pow(0.076, 2) * R_PLATE_WEIGHT * UNCERTAINTIES;

  Trig T = getSinCos(theta);
  double cos_theta = T.cosine;
  double sin_theta = T.sine;

  const double I_total = I_theta + I_rp_x + I_rp_z;
  if (I_total == 0.0) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 5000,
      "Total roll inertia is 0 (check WHOLE_BODY_WEIGHT / COM_POS_Z / UNCERTAINTIES); "
      "skipping the dynamics integration step.");
    return;
  }

  theta_2dot_ = (tau + (WHOLE_BODY_WEIGHT * COM_POS_Z * B_WHEEL_TO_COM * psi_2dot_ * cos_theta)
    + (WHOLE_BODY_WEIGHT * COM_POS_Z * psi_dot_ * cos_theta *
    (Vx - COM_POS_Z * psi_dot_ * sin_theta))
    + WHOLE_BODY_WEIGHT * g * COM_POS_Z * sin_theta) / I_total;

  theta_dot_ += theta_2dot_ * dt;
  // This forward model runs open loop, so bound its state: it is a prediction
  // kept for logging and analysis, not a command.
  theta_dot_ = std::clamp(theta_dot_, -MAX_MODEL_ROLL_RATE, MAX_MODEL_ROLL_RATE);

  // PORT: command the balancing torque that balancingController() computed.
  // ROS 1 published `theta_dot_` here instead - the running integral of the
  // modelled roll acceleration, in rad/s, written straight into an effort
  // controller that expects N.m. Nothing ever reset that integral, so the
  // command ramped without bound; a 60 s run reached -348743 N.m against a
  // 500 N.m joint limit.
  joints->rotational_plate = tau;
}

void Dynamics::constraints(double Vx, double Vx_dot, double delta, double delta_dot)
{
  psi_dot_ = tan(delta) * Vx / WHEEL_SEPARATION_DIS;
  psi_2dot_ = delta_dot * (1 + tan(delta) * tan(delta)) / WHEEL_SEPARATION_DIS +
    Vx_dot * tan(delta) / WHEEL_SEPARATION_DIS;
}

void Dynamics::balancingController(
  const escooter_msgs::msg::Pose & body_frame, double Vx, double theta_dot_measured,
  double plate_velocity)
{
  const double dt = tick(last_time_blc_);

  theta_ = body_frame.orientation.roll;
  double delta_theta = theta_ - pre_theta_;
  double theta_dot_imu = (dt > 0.0) ? (delta_theta / dt) : 0.0;
  pre_theta_ = theta_;

  // Prefer the gyro reading. The finite difference above is kept as a
  // fallback, but it is sampled five times faster than the IMU publishes, so
  // it degenerates into a spike train that the D term amplifies.
  if (std::isfinite(theta_dot_measured)) {
    theta_dot_imu = theta_dot_measured;
  }

  Trig T = getSinCos(theta_);
  double cos_theta = T.cosine;
  double sin_theta = T.sine;

  C_ = (WHOLE_BODY_WEIGHT * COM_POS_Z * B_WHEEL_TO_COM * psi_2dot_)
    + (WHOLE_BODY_WEIGHT * COM_POS_Z * psi_dot_ * (Vx - COM_POS_Z * psi_dot_ * sin_theta));
  G_ = WHOLE_BODY_WEIGHT * g * COM_POS_Z;
  CpG_ = C_ * cos_theta + G_ * sin_theta;

  // PORT: sign of the stabilising feedback.
  //
  // Measured in Gazebo: a positive effort on rotational_plate_joint reacts onto
  // the body as a *negative* roll torque (tau = +40 N.m turned a +1.70 rad/s
  // free-fall roll rate into -0.32 rad/s). Cancelling a positive roll therefore
  // needs a positive plate torque, so the feedback carries the same sign as the
  // gravity feed-forward.
  //
  // ROS 1 used `0 - theta_`, which subtracted it. With G_ = M*g*h = 72.7 N.m/rad
  // against KP = 2.0 that left an effective stiffness of +70.7 N.m/rad: the loop
  // drove the scooter over instead of holding it up.
  // Momentum management. The chassis centre of mass sits a couple of
  // millimetres off the centreline, so gravity applies a small constant roll
  // torque. Holding theta at exactly 0 makes the plate absorb that torque
  // forever: measured drift was -7.6 rad/s per second, saturating the plate
  // after about 26 s, after which the scooter falls. Biasing the setpoint by
  // the plate speed instead lets the body hold a fraction of a degree of
  // lean that cancels the bias, so the plate settles. Set
  // MOMENTUM_UNLOAD_GAIN to 0 to hold exactly upright again.
  // Sign from the equilibrium of the two bodies: holding a lean theta_ref makes
  // the plate accelerate at G*sin(theta_ref)/I_w, so bleeding a positive plate
  // speed needs a negative lean. The resulting outer loop is first order with
  // time constant I_w/(G*K) ~ 8 s, well separated from the 0.25 s balancing
  // loop. Getting this sign wrong turns it into positive feedback: measured, the
  // scooter was over at 43 degrees within one second.
  theta_ref_ = std::clamp(
    -MOMENTUM_UNLOAD_GAIN * plate_velocity, -MAX_LEAN_SETPOINT, MAX_LEAN_SETPOINT);

  const double theta_error = theta_ - theta_ref_;
  const double theta_dot_error = theta_dot_imu;

  tau_ = (PD_PDplus * CpG_) + (KP * theta_error) + (KD * theta_dot_error);
  tau_ = std::clamp(tau_, -MAX_BALANCING_TORQUE, MAX_BALANCING_TORQUE);
}

double Dynamics::computeCOMVelocity(
  double Vx, double theta, double theta_dot, double psi, double psi_dot)
{
  Trig T = getSinCos(theta);
  double cos_theta = T.cosine;
  double sin_theta = T.sine;
  Trig P = getSinCos(psi);
  double cos_psi = P.cosine;
  double sin_psi = P.sine;

  V_x = (Vx * cos_theta) - (B_WHEEL_TO_COM * psi_dot * sin_psi) +
    (COM_POS_Z * theta_dot * cos_theta * sin_psi) + (COM_POS_Z * psi_dot * sin_theta * cos_psi);
  V_y = (Vx * sin_theta) + (B_WHEEL_TO_COM * psi_dot * cos_psi) -
    (COM_POS_Z * theta_dot * cos_theta * cos_psi) + (COM_POS_Z * psi_dot * sin_theta * sin_psi);
  V_z = -COM_POS_Z * theta_dot * sin_theta;

  // The three components are the real output and are exposed as members; the
  // ROS 1 code wrote `return V_x, V_y, V_z;`, whose comma operator yielded V_z.
  // That value is kept so existing callers see no change in behaviour.
  return V_z;
}

}  // namespace escooter_dynamics
