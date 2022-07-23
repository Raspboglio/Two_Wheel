// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from two_wheel_control_msgs:msg/Command.idl
// generated code does not contain a copyright notice
#include "two_wheel_control_msgs/msg/detail/command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
two_wheel_control_msgs__msg__Command__init(two_wheel_control_msgs__msg__Command * msg)
{
  if (!msg) {
    return false;
  }
  // p
  // thetay
  // thetaz
  // p_dot
  // thetay_dot
  // thetaz_dot
  // p_ddot
  // thetay_ddot
  // thetaz_ddot
  return true;
}

void
two_wheel_control_msgs__msg__Command__fini(two_wheel_control_msgs__msg__Command * msg)
{
  if (!msg) {
    return;
  }
  // p
  // thetay
  // thetaz
  // p_dot
  // thetay_dot
  // thetaz_dot
  // p_ddot
  // thetay_ddot
  // thetaz_ddot
}

two_wheel_control_msgs__msg__Command *
two_wheel_control_msgs__msg__Command__create()
{
  two_wheel_control_msgs__msg__Command * msg = (two_wheel_control_msgs__msg__Command *)malloc(sizeof(two_wheel_control_msgs__msg__Command));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(two_wheel_control_msgs__msg__Command));
  bool success = two_wheel_control_msgs__msg__Command__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
two_wheel_control_msgs__msg__Command__destroy(two_wheel_control_msgs__msg__Command * msg)
{
  if (msg) {
    two_wheel_control_msgs__msg__Command__fini(msg);
  }
  free(msg);
}


bool
two_wheel_control_msgs__msg__Command__Sequence__init(two_wheel_control_msgs__msg__Command__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  two_wheel_control_msgs__msg__Command * data = NULL;
  if (size) {
    data = (two_wheel_control_msgs__msg__Command *)calloc(size, sizeof(two_wheel_control_msgs__msg__Command));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = two_wheel_control_msgs__msg__Command__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        two_wheel_control_msgs__msg__Command__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
two_wheel_control_msgs__msg__Command__Sequence__fini(two_wheel_control_msgs__msg__Command__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      two_wheel_control_msgs__msg__Command__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

two_wheel_control_msgs__msg__Command__Sequence *
two_wheel_control_msgs__msg__Command__Sequence__create(size_t size)
{
  two_wheel_control_msgs__msg__Command__Sequence * array = (two_wheel_control_msgs__msg__Command__Sequence *)malloc(sizeof(two_wheel_control_msgs__msg__Command__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = two_wheel_control_msgs__msg__Command__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
two_wheel_control_msgs__msg__Command__Sequence__destroy(two_wheel_control_msgs__msg__Command__Sequence * array)
{
  if (array) {
    two_wheel_control_msgs__msg__Command__Sequence__fini(array);
  }
  free(array);
}
