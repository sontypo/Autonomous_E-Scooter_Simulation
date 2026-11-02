// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from escooter_msgs:msg/RPY.idl
// generated code does not contain a copyright notice
#include "escooter_msgs/msg/detail/rpy__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
escooter_msgs__msg__RPY__init(escooter_msgs__msg__RPY * msg)
{
  if (!msg) {
    return false;
  }
  // roll
  // pitch
  // yaw
  return true;
}

void
escooter_msgs__msg__RPY__fini(escooter_msgs__msg__RPY * msg)
{
  if (!msg) {
    return;
  }
  // roll
  // pitch
  // yaw
}

bool
escooter_msgs__msg__RPY__are_equal(const escooter_msgs__msg__RPY * lhs, const escooter_msgs__msg__RPY * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  return true;
}

bool
escooter_msgs__msg__RPY__copy(
  const escooter_msgs__msg__RPY * input,
  escooter_msgs__msg__RPY * output)
{
  if (!input || !output) {
    return false;
  }
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  return true;
}

escooter_msgs__msg__RPY *
escooter_msgs__msg__RPY__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  escooter_msgs__msg__RPY * msg = (escooter_msgs__msg__RPY *)allocator.allocate(sizeof(escooter_msgs__msg__RPY), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(escooter_msgs__msg__RPY));
  bool success = escooter_msgs__msg__RPY__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
escooter_msgs__msg__RPY__destroy(escooter_msgs__msg__RPY * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    escooter_msgs__msg__RPY__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
escooter_msgs__msg__RPY__Sequence__init(escooter_msgs__msg__RPY__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  escooter_msgs__msg__RPY * data = NULL;

  if (size) {
    data = (escooter_msgs__msg__RPY *)allocator.zero_allocate(size, sizeof(escooter_msgs__msg__RPY), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = escooter_msgs__msg__RPY__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        escooter_msgs__msg__RPY__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
escooter_msgs__msg__RPY__Sequence__fini(escooter_msgs__msg__RPY__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      escooter_msgs__msg__RPY__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

escooter_msgs__msg__RPY__Sequence *
escooter_msgs__msg__RPY__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  escooter_msgs__msg__RPY__Sequence * array = (escooter_msgs__msg__RPY__Sequence *)allocator.allocate(sizeof(escooter_msgs__msg__RPY__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = escooter_msgs__msg__RPY__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
escooter_msgs__msg__RPY__Sequence__destroy(escooter_msgs__msg__RPY__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    escooter_msgs__msg__RPY__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
escooter_msgs__msg__RPY__Sequence__are_equal(const escooter_msgs__msg__RPY__Sequence * lhs, const escooter_msgs__msg__RPY__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!escooter_msgs__msg__RPY__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
escooter_msgs__msg__RPY__Sequence__copy(
  const escooter_msgs__msg__RPY__Sequence * input,
  escooter_msgs__msg__RPY__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(escooter_msgs__msg__RPY);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    escooter_msgs__msg__RPY * data =
      (escooter_msgs__msg__RPY *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!escooter_msgs__msg__RPY__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          escooter_msgs__msg__RPY__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!escooter_msgs__msg__RPY__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
