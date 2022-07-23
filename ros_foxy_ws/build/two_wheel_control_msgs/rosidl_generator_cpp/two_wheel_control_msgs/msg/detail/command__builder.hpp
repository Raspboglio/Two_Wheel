// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from two_wheel_control_msgs:msg/Command.idl
// generated code does not contain a copyright notice

#ifndef TWO_WHEEL_CONTROL_MSGS__MSG__DETAIL__COMMAND__BUILDER_HPP_
#define TWO_WHEEL_CONTROL_MSGS__MSG__DETAIL__COMMAND__BUILDER_HPP_

#include "two_wheel_control_msgs/msg/detail/command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace two_wheel_control_msgs
{

namespace msg
{

namespace builder
{

class Init_Command_thetaz_ddot
{
public:
  explicit Init_Command_thetaz_ddot(::two_wheel_control_msgs::msg::Command & msg)
  : msg_(msg)
  {}
  ::two_wheel_control_msgs::msg::Command thetaz_ddot(::two_wheel_control_msgs::msg::Command::_thetaz_ddot_type arg)
  {
    msg_.thetaz_ddot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::two_wheel_control_msgs::msg::Command msg_;
};

class Init_Command_thetay_ddot
{
public:
  explicit Init_Command_thetay_ddot(::two_wheel_control_msgs::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_thetaz_ddot thetay_ddot(::two_wheel_control_msgs::msg::Command::_thetay_ddot_type arg)
  {
    msg_.thetay_ddot = std::move(arg);
    return Init_Command_thetaz_ddot(msg_);
  }

private:
  ::two_wheel_control_msgs::msg::Command msg_;
};

class Init_Command_p_ddot
{
public:
  explicit Init_Command_p_ddot(::two_wheel_control_msgs::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_thetay_ddot p_ddot(::two_wheel_control_msgs::msg::Command::_p_ddot_type arg)
  {
    msg_.p_ddot = std::move(arg);
    return Init_Command_thetay_ddot(msg_);
  }

private:
  ::two_wheel_control_msgs::msg::Command msg_;
};

class Init_Command_thetaz_dot
{
public:
  explicit Init_Command_thetaz_dot(::two_wheel_control_msgs::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_p_ddot thetaz_dot(::two_wheel_control_msgs::msg::Command::_thetaz_dot_type arg)
  {
    msg_.thetaz_dot = std::move(arg);
    return Init_Command_p_ddot(msg_);
  }

private:
  ::two_wheel_control_msgs::msg::Command msg_;
};

class Init_Command_thetay_dot
{
public:
  explicit Init_Command_thetay_dot(::two_wheel_control_msgs::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_thetaz_dot thetay_dot(::two_wheel_control_msgs::msg::Command::_thetay_dot_type arg)
  {
    msg_.thetay_dot = std::move(arg);
    return Init_Command_thetaz_dot(msg_);
  }

private:
  ::two_wheel_control_msgs::msg::Command msg_;
};

class Init_Command_p_dot
{
public:
  explicit Init_Command_p_dot(::two_wheel_control_msgs::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_thetay_dot p_dot(::two_wheel_control_msgs::msg::Command::_p_dot_type arg)
  {
    msg_.p_dot = std::move(arg);
    return Init_Command_thetay_dot(msg_);
  }

private:
  ::two_wheel_control_msgs::msg::Command msg_;
};

class Init_Command_thetaz
{
public:
  explicit Init_Command_thetaz(::two_wheel_control_msgs::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_p_dot thetaz(::two_wheel_control_msgs::msg::Command::_thetaz_type arg)
  {
    msg_.thetaz = std::move(arg);
    return Init_Command_p_dot(msg_);
  }

private:
  ::two_wheel_control_msgs::msg::Command msg_;
};

class Init_Command_thetay
{
public:
  explicit Init_Command_thetay(::two_wheel_control_msgs::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_thetaz thetay(::two_wheel_control_msgs::msg::Command::_thetay_type arg)
  {
    msg_.thetay = std::move(arg);
    return Init_Command_thetaz(msg_);
  }

private:
  ::two_wheel_control_msgs::msg::Command msg_;
};

class Init_Command_p
{
public:
  Init_Command_p()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Command_thetay p(::two_wheel_control_msgs::msg::Command::_p_type arg)
  {
    msg_.p = std::move(arg);
    return Init_Command_thetay(msg_);
  }

private:
  ::two_wheel_control_msgs::msg::Command msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::two_wheel_control_msgs::msg::Command>()
{
  return two_wheel_control_msgs::msg::builder::Init_Command_p();
}

}  // namespace two_wheel_control_msgs

#endif  // TWO_WHEEL_CONTROL_MSGS__MSG__DETAIL__COMMAND__BUILDER_HPP_
