// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from motor_control_msgs:msg/DmInterface.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_MSGS__MSG__DETAIL__DM_INTERFACE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define MOTOR_CONTROL_MSGS__MSG__DETAIL__DM_INTERFACE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "motor_control_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "motor_control_msgs/msg/detail/dm_interface__struct.hpp"

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

#include "fastcdr/Cdr.h"

namespace motor_control_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_motor_control_msgs
cdr_serialize(
  const motor_control_msgs::msg::DmInterface & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_motor_control_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  motor_control_msgs::msg::DmInterface & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_motor_control_msgs
get_serialized_size(
  const motor_control_msgs::msg::DmInterface & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_motor_control_msgs
max_serialized_size_DmInterface(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace motor_control_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_motor_control_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, motor_control_msgs, msg, DmInterface)();

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_MSGS__MSG__DETAIL__DM_INTERFACE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
