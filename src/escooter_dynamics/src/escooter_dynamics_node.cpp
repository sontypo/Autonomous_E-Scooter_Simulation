// Copyright (c) Saun Nguyen (Hong-Son).
//
// ROS 2 port of the original ROS 1 `escooter_dynamics/src/escooter_dynamics.cpp`.
//
// PORT: the ROS 1 node ran the control loop on the main thread while an
// `AsyncSpinner(2)` serviced callbacks, so the loop and the subscriptions raced
// on every shared member. Here the same loop body is a ROS timer driven by the
// node clock, executed by a single-threaded executor: the ordering is identical,
// the races are gone, and the loop follows simulation time when
// `use_sim_time` is set.

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "escooter_dynamics/control.hpp"
#include "escooter_dynamics/dynamics.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("escooter_dynamics");

  // Initialize instances of Dynamics and Control
  auto dynamics = std::make_shared<escooter_dynamics::Dynamics>(node);
  auto control = std::make_shared<escooter_dynamics::Control>(node);

  const auto step = [dynamics, control]() {
      dynamics->constraints(control->Vx_, control->Vx_dot_, control->delta_, control->delta_dot_);
      control->publishOdometry(control->distance_traveled_, control->delta_);
      dynamics->balancingController(
        control->body_frame_, control->Vx_, control->body_roll_rate_,
        control->plate_velocity_);
      dynamics->updateDynamics(
        dynamics->theta_, &control->joints_, dynamics->tau_, control->Vx_);
      control->publishJointCommand(control->joints_);
    };

  step();

  const int rate_hz = (control->MASTER_LOOP_RATE > 0) ? control->MASTER_LOOP_RATE : 500;
  const auto period = rclcpp::Duration::from_seconds(1.0 / static_cast<double>(rate_hz));
  auto loop_timer = rclcpp::create_timer(node, node->get_clock(), period, step);  // NOLINT

  RCLCPP_INFO(node->get_logger(), "escooter_dynamics model running at %d Hz.", rate_hz);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
