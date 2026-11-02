// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from escooter_msgs:msg/Joints.idl
// generated code does not contain a copyright notice

#ifndef ESCOOTER_MSGS__MSG__DETAIL__JOINTS__STRUCT_HPP_
#define ESCOOTER_MSGS__MSG__DETAIL__JOINTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__escooter_msgs__msg__Joints __attribute__((deprecated))
#else
# define DEPRECATED__escooter_msgs__msg__Joints __declspec(deprecated)
#endif

namespace escooter_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Joints_
{
  using Type = Joints_<ContainerAllocator>;

  explicit Joints_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->handle_bar = 0.0;
      this->rotational_plate = 0.0;
      this->front_wheel = 0.0;
    }
  }

  explicit Joints_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->handle_bar = 0.0;
      this->rotational_plate = 0.0;
      this->front_wheel = 0.0;
    }
  }

  // field types and members
  using _handle_bar_type =
    double;
  _handle_bar_type handle_bar;
  using _rotational_plate_type =
    double;
  _rotational_plate_type rotational_plate;
  using _front_wheel_type =
    double;
  _front_wheel_type front_wheel;

  // setters for named parameter idiom
  Type & set__handle_bar(
    const double & _arg)
  {
    this->handle_bar = _arg;
    return *this;
  }
  Type & set__rotational_plate(
    const double & _arg)
  {
    this->rotational_plate = _arg;
    return *this;
  }
  Type & set__front_wheel(
    const double & _arg)
  {
    this->front_wheel = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    escooter_msgs::msg::Joints_<ContainerAllocator> *;
  using ConstRawPtr =
    const escooter_msgs::msg::Joints_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<escooter_msgs::msg::Joints_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<escooter_msgs::msg::Joints_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      escooter_msgs::msg::Joints_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<escooter_msgs::msg::Joints_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      escooter_msgs::msg::Joints_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<escooter_msgs::msg::Joints_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<escooter_msgs::msg::Joints_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<escooter_msgs::msg::Joints_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__escooter_msgs__msg__Joints
    std::shared_ptr<escooter_msgs::msg::Joints_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__escooter_msgs__msg__Joints
    std::shared_ptr<escooter_msgs::msg::Joints_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Joints_ & other) const
  {
    if (this->handle_bar != other.handle_bar) {
      return false;
    }
    if (this->rotational_plate != other.rotational_plate) {
      return false;
    }
    if (this->front_wheel != other.front_wheel) {
      return false;
    }
    return true;
  }
  bool operator!=(const Joints_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Joints_

// alias to use template instance with default allocator
using Joints =
  escooter_msgs::msg::Joints_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace escooter_msgs

#endif  // ESCOOTER_MSGS__MSG__DETAIL__JOINTS__STRUCT_HPP_
