// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from escooter_msgs:msg/RPY.idl
// generated code does not contain a copyright notice

#ifndef ESCOOTER_MSGS__MSG__DETAIL__RPY__STRUCT_H_
#define ESCOOTER_MSGS__MSG__DETAIL__RPY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RPY in the package escooter_msgs.
typedef struct escooter_msgs__msg__RPY
{
  double roll;
  double pitch;
  double yaw;
} escooter_msgs__msg__RPY;

// Struct for a sequence of escooter_msgs__msg__RPY.
typedef struct escooter_msgs__msg__RPY__Sequence
{
  escooter_msgs__msg__RPY * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} escooter_msgs__msg__RPY__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ESCOOTER_MSGS__MSG__DETAIL__RPY__STRUCT_H_
