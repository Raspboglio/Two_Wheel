// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from two_wheel_control_msgs:msg/Command.idl
// generated code does not contain a copyright notice

#ifndef TWO_WHEEL_CONTROL_MSGS__MSG__DETAIL__COMMAND__STRUCT_H_
#define TWO_WHEEL_CONTROL_MSGS__MSG__DETAIL__COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Command in the package two_wheel_control_msgs.
typedef struct two_wheel_control_msgs__msg__Command
{
  double p;
  double thetay;
  double thetaz;
  double p_dot;
  double thetay_dot;
  double thetaz_dot;
  double p_ddot;
  double thetay_ddot;
  double thetaz_ddot;
} two_wheel_control_msgs__msg__Command;

// Struct for a sequence of two_wheel_control_msgs__msg__Command.
typedef struct two_wheel_control_msgs__msg__Command__Sequence
{
  two_wheel_control_msgs__msg__Command * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} two_wheel_control_msgs__msg__Command__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TWO_WHEEL_CONTROL_MSGS__MSG__DETAIL__COMMAND__STRUCT_H_
