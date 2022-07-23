// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from two_wheel_control_msgs:msg/Command.idl
// generated code does not contain a copyright notice

#ifndef TWO_WHEEL_CONTROL_MSGS__MSG__DETAIL__COMMAND__STRUCT_HPP_
#define TWO_WHEEL_CONTROL_MSGS__MSG__DETAIL__COMMAND__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__two_wheel_control_msgs__msg__Command __attribute__((deprecated))
#else
# define DEPRECATED__two_wheel_control_msgs__msg__Command __declspec(deprecated)
#endif

namespace two_wheel_control_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Command_
{
  using Type = Command_<ContainerAllocator>;

  explicit Command_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->p = 0.0;
      this->thetay = 0.0;
      this->thetaz = 0.0;
      this->p_dot = 0.0;
      this->thetay_dot = 0.0;
      this->thetaz_dot = 0.0;
      this->p_ddot = 0.0;
      this->thetay_ddot = 0.0;
      this->thetaz_ddot = 0.0;
    }
  }

  explicit Command_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->p = 0.0;
      this->thetay = 0.0;
      this->thetaz = 0.0;
      this->p_dot = 0.0;
      this->thetay_dot = 0.0;
      this->thetaz_dot = 0.0;
      this->p_ddot = 0.0;
      this->thetay_ddot = 0.0;
      this->thetaz_ddot = 0.0;
    }
  }

  // field types and members
  using _p_type =
    double;
  _p_type p;
  using _thetay_type =
    double;
  _thetay_type thetay;
  using _thetaz_type =
    double;
  _thetaz_type thetaz;
  using _p_dot_type =
    double;
  _p_dot_type p_dot;
  using _thetay_dot_type =
    double;
  _thetay_dot_type thetay_dot;
  using _thetaz_dot_type =
    double;
  _thetaz_dot_type thetaz_dot;
  using _p_ddot_type =
    double;
  _p_ddot_type p_ddot;
  using _thetay_ddot_type =
    double;
  _thetay_ddot_type thetay_ddot;
  using _thetaz_ddot_type =
    double;
  _thetaz_ddot_type thetaz_ddot;

  // setters for named parameter idiom
  Type & set__p(
    const double & _arg)
  {
    this->p = _arg;
    return *this;
  }
  Type & set__thetay(
    const double & _arg)
  {
    this->thetay = _arg;
    return *this;
  }
  Type & set__thetaz(
    const double & _arg)
  {
    this->thetaz = _arg;
    return *this;
  }
  Type & set__p_dot(
    const double & _arg)
  {
    this->p_dot = _arg;
    return *this;
  }
  Type & set__thetay_dot(
    const double & _arg)
  {
    this->thetay_dot = _arg;
    return *this;
  }
  Type & set__thetaz_dot(
    const double & _arg)
  {
    this->thetaz_dot = _arg;
    return *this;
  }
  Type & set__p_ddot(
    const double & _arg)
  {
    this->p_ddot = _arg;
    return *this;
  }
  Type & set__thetay_ddot(
    const double & _arg)
  {
    this->thetay_ddot = _arg;
    return *this;
  }
  Type & set__thetaz_ddot(
    const double & _arg)
  {
    this->thetaz_ddot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    two_wheel_control_msgs::msg::Command_<ContainerAllocator> *;
  using ConstRawPtr =
    const two_wheel_control_msgs::msg::Command_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<two_wheel_control_msgs::msg::Command_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<two_wheel_control_msgs::msg::Command_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      two_wheel_control_msgs::msg::Command_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<two_wheel_control_msgs::msg::Command_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      two_wheel_control_msgs::msg::Command_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<two_wheel_control_msgs::msg::Command_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<two_wheel_control_msgs::msg::Command_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<two_wheel_control_msgs::msg::Command_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__two_wheel_control_msgs__msg__Command
    std::shared_ptr<two_wheel_control_msgs::msg::Command_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__two_wheel_control_msgs__msg__Command
    std::shared_ptr<two_wheel_control_msgs::msg::Command_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_ & other) const
  {
    if (this->p != other.p) {
      return false;
    }
    if (this->thetay != other.thetay) {
      return false;
    }
    if (this->thetaz != other.thetaz) {
      return false;
    }
    if (this->p_dot != other.p_dot) {
      return false;
    }
    if (this->thetay_dot != other.thetay_dot) {
      return false;
    }
    if (this->thetaz_dot != other.thetaz_dot) {
      return false;
    }
    if (this->p_ddot != other.p_ddot) {
      return false;
    }
    if (this->thetay_ddot != other.thetay_ddot) {
      return false;
    }
    if (this->thetaz_ddot != other.thetaz_ddot) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_

// alias to use template instance with default allocator
using Command =
  two_wheel_control_msgs::msg::Command_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace two_wheel_control_msgs

#endif  // TWO_WHEEL_CONTROL_MSGS__MSG__DETAIL__COMMAND__STRUCT_HPP_
