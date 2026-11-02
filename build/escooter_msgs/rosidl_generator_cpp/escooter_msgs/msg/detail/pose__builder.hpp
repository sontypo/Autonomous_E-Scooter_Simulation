// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from escooter_msgs:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef ESCOOTER_MSGS__MSG__DETAIL__POSE__BUILDER_HPP_
#define ESCOOTER_MSGS__MSG__DETAIL__POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "escooter_msgs/msg/detail/pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace escooter_msgs
{

namespace msg
{

namespace builder
{

class Init_Pose_orientation
{
public:
  explicit Init_Pose_orientation(::escooter_msgs::msg::Pose & msg)
  : msg_(msg)
  {}
  ::escooter_msgs::msg::Pose orientation(::escooter_msgs::msg::Pose::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return std::move(msg_);
  }

private:
  ::escooter_msgs::msg::Pose msg_;
};

class Init_Pose_position
{
public:
  Init_Pose_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pose_orientation position(::escooter_msgs::msg::Pose::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Pose_orientation(msg_);
  }

private:
  ::escooter_msgs::msg::Pose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::escooter_msgs::msg::Pose>()
{
  return escooter_msgs::msg::builder::Init_Pose_position();
}

}  // namespace escooter_msgs

#endif  // ESCOOTER_MSGS__MSG__DETAIL__POSE__BUILDER_HPP_
