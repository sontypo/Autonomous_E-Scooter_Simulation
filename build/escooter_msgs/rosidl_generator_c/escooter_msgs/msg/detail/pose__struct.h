// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from escooter_msgs:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef ESCOOTER_MSGS__MSG__DETAIL__POSE__STRUCT_H_
#define ESCOOTER_MSGS__MSG__DETAIL__POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'orientation'
#include "escooter_msgs/msg/detail/rpy__struct.h"

/// Struct defined in msg/Pose in the package escooter_msgs.
typedef struct escooter_msgs__msg__Pose
{
  geometry_msgs__msg__Point position;
  escooter_msgs__msg__RPY orientation;
} escooter_msgs__msg__Pose;

// Struct for a sequence of escooter_msgs__msg__Pose.
typedef struct escooter_msgs__msg__Pose__Sequence
{
  escooter_msgs__msg__Pose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} escooter_msgs__msg__Pose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ESCOOTER_MSGS__MSG__DETAIL__POSE__STRUCT_H_
