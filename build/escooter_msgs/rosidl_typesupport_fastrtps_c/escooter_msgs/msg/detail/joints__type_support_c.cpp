// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from escooter_msgs:msg/Joints.idl
// generated code does not contain a copyright notice
#include "escooter_msgs/msg/detail/joints__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "escooter_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "escooter_msgs/msg/detail/joints__struct.h"
#include "escooter_msgs/msg/detail/joints__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _Joints__ros_msg_type = escooter_msgs__msg__Joints;

static bool _Joints__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Joints__ros_msg_type * ros_message = static_cast<const _Joints__ros_msg_type *>(untyped_ros_message);
  // Field name: handle_bar
  {
    cdr << ros_message->handle_bar;
  }

  // Field name: rotational_plate
  {
    cdr << ros_message->rotational_plate;
  }

  // Field name: front_wheel
  {
    cdr << ros_message->front_wheel;
  }

  return true;
}

static bool _Joints__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Joints__ros_msg_type * ros_message = static_cast<_Joints__ros_msg_type *>(untyped_ros_message);
  // Field name: handle_bar
  {
    cdr >> ros_message->handle_bar;
  }

  // Field name: rotational_plate
  {
    cdr >> ros_message->rotational_plate;
  }

  // Field name: front_wheel
  {
    cdr >> ros_message->front_wheel;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_escooter_msgs
size_t get_serialized_size_escooter_msgs__msg__Joints(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Joints__ros_msg_type * ros_message = static_cast<const _Joints__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name handle_bar
  {
    size_t item_size = sizeof(ros_message->handle_bar);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rotational_plate
  {
    size_t item_size = sizeof(ros_message->rotational_plate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name front_wheel
  {
    size_t item_size = sizeof(ros_message->front_wheel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Joints__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_escooter_msgs__msg__Joints(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_escooter_msgs
size_t max_serialized_size_escooter_msgs__msg__Joints(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: handle_bar
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: rotational_plate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: front_wheel
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = escooter_msgs__msg__Joints;
    is_plain =
      (
      offsetof(DataType, front_wheel) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Joints__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_escooter_msgs__msg__Joints(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Joints = {
  "escooter_msgs::msg",
  "Joints",
  _Joints__cdr_serialize,
  _Joints__cdr_deserialize,
  _Joints__get_serialized_size,
  _Joints__max_serialized_size
};

static rosidl_message_type_support_t _Joints__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Joints,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, escooter_msgs, msg, Joints)() {
  return &_Joints__type_support;
}

#if defined(__cplusplus)
}
#endif
