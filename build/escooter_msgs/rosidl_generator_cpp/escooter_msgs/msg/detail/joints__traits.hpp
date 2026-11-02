// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from escooter_msgs:msg/Joints.idl
// generated code does not contain a copyright notice

#ifndef ESCOOTER_MSGS__MSG__DETAIL__JOINTS__TRAITS_HPP_
#define ESCOOTER_MSGS__MSG__DETAIL__JOINTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "escooter_msgs/msg/detail/joints__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace escooter_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Joints & msg,
  std::ostream & out)
{
  out << "{";
  // member: handle_bar
  {
    out << "handle_bar: ";
    rosidl_generator_traits::value_to_yaml(msg.handle_bar, out);
    out << ", ";
  }

  // member: rotational_plate
  {
    out << "rotational_plate: ";
    rosidl_generator_traits::value_to_yaml(msg.rotational_plate, out);
    out << ", ";
  }

  // member: front_wheel
  {
    out << "front_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.front_wheel, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Joints & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: handle_bar
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "handle_bar: ";
    rosidl_generator_traits::value_to_yaml(msg.handle_bar, out);
    out << "\n";
  }

  // member: rotational_plate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rotational_plate: ";
    rosidl_generator_traits::value_to_yaml(msg.rotational_plate, out);
    out << "\n";
  }

  // member: front_wheel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.front_wheel, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Joints & msg, bool use_flow_style = false)
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
  const escooter_msgs::msg::Joints & msg,
  std::ostream & out, size_t indentation = 0)
{
  escooter_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use escooter_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const escooter_msgs::msg::Joints & msg)
{
  return escooter_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<escooter_msgs::msg::Joints>()
{
  return "escooter_msgs::msg::Joints";
}

template<>
inline const char * name<escooter_msgs::msg::Joints>()
{
  return "escooter_msgs/msg/Joints";
}

template<>
struct has_fixed_size<escooter_msgs::msg::Joints>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<escooter_msgs::msg::Joints>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<escooter_msgs::msg::Joints>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ESCOOTER_MSGS__MSG__DETAIL__JOINTS__TRAITS_HPP_
