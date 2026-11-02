// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from escooter_msgs:msg/Joints.idl
// generated code does not contain a copyright notice

#ifndef ESCOOTER_MSGS__MSG__DETAIL__JOINTS__STRUCT_H_
#define ESCOOTER_MSGS__MSG__DETAIL__JOINTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Joints in the package escooter_msgs.
typedef struct escooter_msgs__msg__Joints
{
  double handle_bar;
  double rotational_plate;
  double front_wheel;
} escooter_msgs__msg__Joints;

// Struct for a sequence of escooter_msgs__msg__Joints.
typedef struct escooter_msgs__msg__Joints__Sequence
{
  escooter_msgs__msg__Joints * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} escooter_msgs__msg__Joints__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ESCOOTER_MSGS__MSG__DETAIL__JOINTS__STRUCT_H_
