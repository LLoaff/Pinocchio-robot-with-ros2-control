// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_control_msgs:msg/DmInterface.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_MSGS__MSG__DETAIL__DM_INTERFACE__STRUCT_H_
#define MOTOR_CONTROL_MSGS__MSG__DETAIL__DM_INTERFACE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'id'
// Member 'q'
// Member 'dq'
// Member 'kp'
// Member 'kd'
// Member 'tau'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/DmInterface in the package motor_control_msgs.
typedef struct motor_control_msgs__msg__DmInterface
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__int64__Sequence id;
  rosidl_runtime_c__float__Sequence q;
  rosidl_runtime_c__float__Sequence dq;
  rosidl_runtime_c__float__Sequence kp;
  rosidl_runtime_c__float__Sequence kd;
  rosidl_runtime_c__float__Sequence tau;
} motor_control_msgs__msg__DmInterface;

// Struct for a sequence of motor_control_msgs__msg__DmInterface.
typedef struct motor_control_msgs__msg__DmInterface__Sequence
{
  motor_control_msgs__msg__DmInterface * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_control_msgs__msg__DmInterface__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_MSGS__MSG__DETAIL__DM_INTERFACE__STRUCT_H_
