// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from motor_control_msgs:msg/DmInterface.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "motor_control_msgs/msg/detail/dm_interface__rosidl_typesupport_introspection_c.h"
#include "motor_control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "motor_control_msgs/msg/detail/dm_interface__functions.h"
#include "motor_control_msgs/msg/detail/dm_interface__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `id`
// Member `q`
// Member `dq`
// Member `kp`
// Member `kd`
// Member `tau`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__DmInterface_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  motor_control_msgs__msg__DmInterface__init(message_memory);
}

void motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__DmInterface_fini_function(void * message_memory)
{
  motor_control_msgs__msg__DmInterface__fini(message_memory);
}

size_t motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__size_function__DmInterface__id(
  const void * untyped_member)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return member->size;
}

const void * motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__id(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void * motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__id(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__fetch_function__DmInterface__id(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int64_t * item =
    ((const int64_t *)
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__id(untyped_member, index));
  int64_t * value =
    (int64_t *)(untyped_value);
  *value = *item;
}

void motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__assign_function__DmInterface__id(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int64_t * item =
    ((int64_t *)
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__id(untyped_member, index));
  const int64_t * value =
    (const int64_t *)(untyped_value);
  *item = *value;
}

bool motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__resize_function__DmInterface__id(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  rosidl_runtime_c__int64__Sequence__fini(member);
  return rosidl_runtime_c__int64__Sequence__init(member, size);
}

size_t motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__size_function__DmInterface__q(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__q(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__q(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__fetch_function__DmInterface__q(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__q(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__assign_function__DmInterface__q(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__q(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__resize_function__DmInterface__q(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__size_function__DmInterface__dq(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__dq(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__dq(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__fetch_function__DmInterface__dq(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__dq(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__assign_function__DmInterface__dq(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__dq(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__resize_function__DmInterface__dq(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__size_function__DmInterface__kp(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__kp(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__kp(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__fetch_function__DmInterface__kp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__kp(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__assign_function__DmInterface__kp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__kp(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__resize_function__DmInterface__kp(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__size_function__DmInterface__kd(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__kd(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__kd(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__fetch_function__DmInterface__kd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__kd(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__assign_function__DmInterface__kd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__kd(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__resize_function__DmInterface__kd(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__size_function__DmInterface__tau(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__tau(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__tau(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__fetch_function__DmInterface__tau(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__tau(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__assign_function__DmInterface__tau(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__tau(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__resize_function__DmInterface__tau(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__DmInterface_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_control_msgs__msg__DmInterface, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_control_msgs__msg__DmInterface, id),  // bytes offset in struct
    NULL,  // default value
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__size_function__DmInterface__id,  // size() function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__id,  // get_const(index) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__id,  // get(index) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__fetch_function__DmInterface__id,  // fetch(index, &value) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__assign_function__DmInterface__id,  // assign(index, value) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__resize_function__DmInterface__id  // resize(index) function pointer
  },
  {
    "q",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_control_msgs__msg__DmInterface, q),  // bytes offset in struct
    NULL,  // default value
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__size_function__DmInterface__q,  // size() function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__q,  // get_const(index) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__q,  // get(index) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__fetch_function__DmInterface__q,  // fetch(index, &value) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__assign_function__DmInterface__q,  // assign(index, value) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__resize_function__DmInterface__q  // resize(index) function pointer
  },
  {
    "dq",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_control_msgs__msg__DmInterface, dq),  // bytes offset in struct
    NULL,  // default value
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__size_function__DmInterface__dq,  // size() function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__dq,  // get_const(index) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__dq,  // get(index) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__fetch_function__DmInterface__dq,  // fetch(index, &value) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__assign_function__DmInterface__dq,  // assign(index, value) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__resize_function__DmInterface__dq  // resize(index) function pointer
  },
  {
    "kp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_control_msgs__msg__DmInterface, kp),  // bytes offset in struct
    NULL,  // default value
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__size_function__DmInterface__kp,  // size() function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__kp,  // get_const(index) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__kp,  // get(index) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__fetch_function__DmInterface__kp,  // fetch(index, &value) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__assign_function__DmInterface__kp,  // assign(index, value) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__resize_function__DmInterface__kp  // resize(index) function pointer
  },
  {
    "kd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_control_msgs__msg__DmInterface, kd),  // bytes offset in struct
    NULL,  // default value
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__size_function__DmInterface__kd,  // size() function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__kd,  // get_const(index) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__kd,  // get(index) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__fetch_function__DmInterface__kd,  // fetch(index, &value) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__assign_function__DmInterface__kd,  // assign(index, value) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__resize_function__DmInterface__kd  // resize(index) function pointer
  },
  {
    "tau",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_control_msgs__msg__DmInterface, tau),  // bytes offset in struct
    NULL,  // default value
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__size_function__DmInterface__tau,  // size() function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_const_function__DmInterface__tau,  // get_const(index) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__get_function__DmInterface__tau,  // get(index) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__fetch_function__DmInterface__tau,  // fetch(index, &value) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__assign_function__DmInterface__tau,  // assign(index, value) function pointer
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__resize_function__DmInterface__tau  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__DmInterface_message_members = {
  "motor_control_msgs__msg",  // message namespace
  "DmInterface",  // message name
  7,  // number of fields
  sizeof(motor_control_msgs__msg__DmInterface),
  motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__DmInterface_message_member_array,  // message members
  motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__DmInterface_init_function,  // function to initialize message memory (memory has to be allocated)
  motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__DmInterface_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__DmInterface_message_type_support_handle = {
  0,
  &motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__DmInterface_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_motor_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motor_control_msgs, msg, DmInterface)() {
  motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__DmInterface_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__DmInterface_message_type_support_handle.typesupport_identifier) {
    motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__DmInterface_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &motor_control_msgs__msg__DmInterface__rosidl_typesupport_introspection_c__DmInterface_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
