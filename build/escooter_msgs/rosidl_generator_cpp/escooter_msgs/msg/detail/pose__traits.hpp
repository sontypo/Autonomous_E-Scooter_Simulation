// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from escooter_msgs:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef ESCOOTER_MSGS__MSG__DETAIL__POSE__TRAITS_HPP_
#define ESCOOTER_MSGS__MSG__DETAIL__POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "escooter_msgs/msg/detail/pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'orientation'
#include "escooter_msgs/msg/detail/rpy__traits.hpp"

namespace escooter_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Pose & msg,
  std::ostream & out)
{
  out << "{";
  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: orientation
  {
    out << "orientation: ";
    to_flow_style_yaml(msg.orientation, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Pose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orientation:\n";
    to_block_style_yaml(msg.orientation, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Pose & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace escooter_msgs

namespace rosidl_generator_traits
{

[[deprecated("use escooter_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const escooter_msgs::msg::Pose & msg,
  std::ostream & out, size_t indentation = 0)
{
  escooter_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use escooter_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const escooter_msgs::msg::Pose & msg)
{
  return escooter_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<escooter_msgs::msg::Pose>()
{
  return "escooter_msgs::msg::Pose";
}

template<>
inline const char * name<escooter_msgs::msg::Pose>()
{
  return "escooter_msgs/msg/Pose";
}

template<>
struct has_fixed_size<escooter_msgs::msg::Pose>
  : std::integral_constant<bool, has_fixed_size<escooter_msgs::msg::RPY>::value && has_fixed_size<geometry_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<escooter_msgs::msg::Pose>
  : std::integral_constant<bool, has_bounded_size<escooter_msgs::msg::RPY>::value && has_bounded_size<geometry_msgs::msg::Point>::value> {};

template<>
struct is_message<escooter_msgs::msg::Pose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ESCOOTER_MSGS__MSG__DETAIL__POSE__TRAITS_HPP_
