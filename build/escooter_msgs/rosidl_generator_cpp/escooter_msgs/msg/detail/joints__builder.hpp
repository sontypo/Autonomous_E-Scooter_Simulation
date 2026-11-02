// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from escooter_msgs:msg/Joints.idl
// generated code does not contain a copyright notice

#ifndef ESCOOTER_MSGS__MSG__DETAIL__JOINTS__BUILDER_HPP_
#define ESCOOTER_MSGS__MSG__DETAIL__JOINTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "escooter_msgs/msg/detail/joints__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace escooter_msgs
{

namespace msg
{

namespace builder
{

class Init_Joints_front_wheel
{
public:
  explicit Init_Joints_front_wheel(::escooter_msgs::msg::Joints & msg)
  : msg_(msg)
  {}
  ::escooter_msgs::msg::Joints front_wheel(::escooter_msgs::msg::Joints::_front_wheel_type arg)
  {
    msg_.front_wheel = std::move(arg);
    return std::move(msg_);
  }

private:
  ::escooter_msgs::msg::Joints msg_;
};

class Init_Joints_rotational_plate
{
public:
  explicit Init_Joints_rotational_plate(::escooter_msgs::msg::Joints & msg)
  : msg_(msg)
  {}
  Init_Joints_front_wheel rotational_plate(::escooter_msgs::msg::Joints::_rotational_plate_type arg)
  {
    msg_.rotational_plate = std::move(arg);
    return Init_Joints_front_wheel(msg_);
  }

private:
  ::escooter_msgs::msg::Joints msg_;
};

class Init_Joints_handle_bar
{
public:
  Init_Joints_handle_bar()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Joints_rotational_plate handle_bar(::escooter_msgs::msg::Joints::_handle_bar_type arg)
  {
    msg_.handle_bar = std::move(arg);
    return Init_Joints_rotational_plate(msg_);
  }

private:
  ::escooter_msgs::msg::Joints msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::escooter_msgs::msg::Joints>()
{
  return escooter_msgs::msg::builder::Init_Joints_handle_bar();
}

}  // namespace escooter_msgs

#endif  // ESCOOTER_MSGS__MSG__DETAIL__JOINTS__BUILDER_HPP_
