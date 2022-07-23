// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from two_wheel_control_msgs:msg/Command.idl
// generated code does not contain a copyright notice

#ifndef TWO_WHEEL_CONTROL_MSGS__MSG__DETAIL__COMMAND__TRAITS_HPP_
#define TWO_WHEEL_CONTROL_MSGS__MSG__DETAIL__COMMAND__TRAITS_HPP_

#include "two_wheel_control_msgs/msg/detail/command__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<two_wheel_control_msgs::msg::Command>()
{
  return "two_wheel_control_msgs::msg::Command";
}

template<>
inline const char * name<two_wheel_control_msgs::msg::Command>()
{
  return "two_wheel_control_msgs/msg/Command";
}

template<>
struct has_fixed_size<two_wheel_control_msgs::msg::Command>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<two_wheel_control_msgs::msg::Command>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<two_wheel_control_msgs::msg::Command>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TWO_WHEEL_CONTROL_MSGS__MSG__DETAIL__COMMAND__TRAITS_HPP_
