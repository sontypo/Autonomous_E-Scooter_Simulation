// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from escooter_msgs:msg/RPY.idl
// generated code does not contain a copyright notice

#ifndef ESCOOTER_MSGS__MSG__DETAIL__RPY__BUILDER_HPP_
#define ESCOOTER_MSGS__MSG__DETAIL__RPY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "escooter_msgs/msg/detail/rpy__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace escooter_msgs
{

namespace msg
{

namespace builder
{

class Init_RPY_yaw
{
public:
  explicit Init_RPY_yaw(::escooter_msgs::msg::RPY & msg)
  : msg_(msg)
  {}
  ::escooter_msgs::msg::RPY yaw(::escooter_msgs::msg::RPY::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::escooter_msgs::msg::RPY msg_;
};

class Init_RPY_pitch
{
public:
  explicit Init_RPY_pitch(::escooter_msgs::msg::RPY & msg)
  : msg_(msg)
  {}
  Init_RPY_yaw pitch(::escooter_msgs::msg::RPY::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_RPY_yaw(msg_);
  }

private:
  ::escooter_msgs::msg::RPY msg_;
};

class Init_RPY_roll
{
public:
  Init_RPY_roll()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RPY_pitch roll(::escooter_msgs::msg::RPY::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_RPY_pitch(msg_);
  }

private:
  ::escooter_msgs::msg::RPY msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::escooter_msgs::msg::RPY>()
{
  return escooter_msgs::msg::builder::Init_RPY_roll();
}

}  // namespace escooter_msgs

#endif  // ESCOOTER_MSGS__MSG__DETAIL__RPY__BUILDER_HPP_
