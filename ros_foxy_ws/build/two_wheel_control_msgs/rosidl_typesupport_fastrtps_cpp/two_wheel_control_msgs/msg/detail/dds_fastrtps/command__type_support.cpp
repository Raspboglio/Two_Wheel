// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from two_wheel_control_msgs:msg/Command.idl
// generated code does not contain a copyright notice
#include "two_wheel_control_msgs/msg/detail/command__rosidl_typesupport_fastrtps_cpp.hpp"
#include "two_wheel_control_msgs/msg/detail/command__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace two_wheel_control_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_two_wheel_control_msgs
cdr_serialize(
  const two_wheel_control_msgs::msg::Command & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: p
  cdr << ros_message.p;
  // Member: thetay
  cdr << ros_message.thetay;
  // Member: thetaz
  cdr << ros_message.thetaz;
  // Member: p_dot
  cdr << ros_message.p_dot;
  // Member: thetay_dot
  cdr << ros_message.thetay_dot;
  // Member: thetaz_dot
  cdr << ros_message.thetaz_dot;
  // Member: p_ddot
  cdr << ros_message.p_ddot;
  // Member: thetay_ddot
  cdr << ros_message.thetay_ddot;
  // Member: thetaz_ddot
  cdr << ros_message.thetaz_ddot;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_two_wheel_control_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  two_wheel_control_msgs::msg::Command & ros_message)
{
  // Member: p
  cdr >> ros_message.p;

  // Member: thetay
  cdr >> ros_message.thetay;

  // Member: thetaz
  cdr >> ros_message.thetaz;

  // Member: p_dot
  cdr >> ros_message.p_dot;

  // Member: thetay_dot
  cdr >> ros_message.thetay_dot;

  // Member: thetaz_dot
  cdr >> ros_message.thetaz_dot;

  // Member: p_ddot
  cdr >> ros_message.p_ddot;

  // Member: thetay_ddot
  cdr >> ros_message.thetay_ddot;

  // Member: thetaz_ddot
  cdr >> ros_message.thetaz_ddot;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_two_wheel_control_msgs
get_serialized_size(
  const two_wheel_control_msgs::msg::Command & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: p
  {
    size_t item_size = sizeof(ros_message.p);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: thetay
  {
    size_t item_size = sizeof(ros_message.thetay);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: thetaz
  {
    size_t item_size = sizeof(ros_message.thetaz);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: p_dot
  {
    size_t item_size = sizeof(ros_message.p_dot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: thetay_dot
  {
    size_t item_size = sizeof(ros_message.thetay_dot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: thetaz_dot
  {
    size_t item_size = sizeof(ros_message.thetaz_dot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: p_ddot
  {
    size_t item_size = sizeof(ros_message.p_ddot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: thetay_ddot
  {
    size_t item_size = sizeof(ros_message.thetay_ddot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: thetaz_ddot
  {
    size_t item_size = sizeof(ros_message.thetaz_ddot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_two_wheel_control_msgs
max_serialized_size_Command(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: p
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: thetay
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: thetaz
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: p_dot
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: thetay_dot
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: thetaz_dot
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: p_ddot
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: thetay_ddot
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: thetaz_ddot
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static bool _Command__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const two_wheel_control_msgs::msg::Command *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Command__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<two_wheel_control_msgs::msg::Command *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Command__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const two_wheel_control_msgs::msg::Command *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Command__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_Command(full_bounded, 0);
}

static message_type_support_callbacks_t _Command__callbacks = {
  "two_wheel_control_msgs::msg",
  "Command",
  _Command__cdr_serialize,
  _Command__cdr_deserialize,
  _Command__get_serialized_size,
  _Command__max_serialized_size
};

static rosidl_message_type_support_t _Command__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Command__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace two_wheel_control_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_two_wheel_control_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<two_wheel_control_msgs::msg::Command>()
{
  return &two_wheel_control_msgs::msg::typesupport_fastrtps_cpp::_Command__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, two_wheel_control_msgs, msg, Command)() {
  return &two_wheel_control_msgs::msg::typesupport_fastrtps_cpp::_Command__handle;
}

#ifdef __cplusplus
}
#endif
