// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motor_control_msgs:msg/DmInterface.idl
// generated code does not contain a copyright notice
#include "motor_control_msgs/msg/detail/dm_interface__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `id`
// Member `q`
// Member `dq`
// Member `kp`
// Member `kd`
// Member `tau`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
motor_control_msgs__msg__DmInterface__init(motor_control_msgs__msg__DmInterface * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    motor_control_msgs__msg__DmInterface__fini(msg);
    return false;
  }
  // id
  if (!rosidl_runtime_c__int64__Sequence__init(&msg->id, 0)) {
    motor_control_msgs__msg__DmInterface__fini(msg);
    return false;
  }
  // q
  if (!rosidl_runtime_c__float__Sequence__init(&msg->q, 0)) {
    motor_control_msgs__msg__DmInterface__fini(msg);
    return false;
  }
  // dq
  if (!rosidl_runtime_c__float__Sequence__init(&msg->dq, 0)) {
    motor_control_msgs__msg__DmInterface__fini(msg);
    return false;
  }
  // kp
  if (!rosidl_runtime_c__float__Sequence__init(&msg->kp, 0)) {
    motor_control_msgs__msg__DmInterface__fini(msg);
    return false;
  }
  // kd
  if (!rosidl_runtime_c__float__Sequence__init(&msg->kd, 0)) {
    motor_control_msgs__msg__DmInterface__fini(msg);
    return false;
  }
  // tau
  if (!rosidl_runtime_c__float__Sequence__init(&msg->tau, 0)) {
    motor_control_msgs__msg__DmInterface__fini(msg);
    return false;
  }
  return true;
}

void
motor_control_msgs__msg__DmInterface__fini(motor_control_msgs__msg__DmInterface * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // id
  rosidl_runtime_c__int64__Sequence__fini(&msg->id);
  // q
  rosidl_runtime_c__float__Sequence__fini(&msg->q);
  // dq
  rosidl_runtime_c__float__Sequence__fini(&msg->dq);
  // kp
  rosidl_runtime_c__float__Sequence__fini(&msg->kp);
  // kd
  rosidl_runtime_c__float__Sequence__fini(&msg->kd);
  // tau
  rosidl_runtime_c__float__Sequence__fini(&msg->tau);
}

bool
motor_control_msgs__msg__DmInterface__are_equal(const motor_control_msgs__msg__DmInterface * lhs, const motor_control_msgs__msg__DmInterface * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // id
  if (!rosidl_runtime_c__int64__Sequence__are_equal(
      &(lhs->id), &(rhs->id)))
  {
    return false;
  }
  // q
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->q), &(rhs->q)))
  {
    return false;
  }
  // dq
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->dq), &(rhs->dq)))
  {
    return false;
  }
  // kp
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->kp), &(rhs->kp)))
  {
    return false;
  }
  // kd
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->kd), &(rhs->kd)))
  {
    return false;
  }
  // tau
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->tau), &(rhs->tau)))
  {
    return false;
  }
  return true;
}

bool
motor_control_msgs__msg__DmInterface__copy(
  const motor_control_msgs__msg__DmInterface * input,
  motor_control_msgs__msg__DmInterface * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // id
  if (!rosidl_runtime_c__int64__Sequence__copy(
      &(input->id), &(output->id)))
  {
    return false;
  }
  // q
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->q), &(output->q)))
  {
    return false;
  }
  // dq
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->dq), &(output->dq)))
  {
    return false;
  }
  // kp
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->kp), &(output->kp)))
  {
    return false;
  }
  // kd
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->kd), &(output->kd)))
  {
    return false;
  }
  // tau
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->tau), &(output->tau)))
  {
    return false;
  }
  return true;
}

motor_control_msgs__msg__DmInterface *
motor_control_msgs__msg__DmInterface__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_control_msgs__msg__DmInterface * msg = (motor_control_msgs__msg__DmInterface *)allocator.allocate(sizeof(motor_control_msgs__msg__DmInterface), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motor_control_msgs__msg__DmInterface));
  bool success = motor_control_msgs__msg__DmInterface__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motor_control_msgs__msg__DmInterface__destroy(motor_control_msgs__msg__DmInterface * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motor_control_msgs__msg__DmInterface__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motor_control_msgs__msg__DmInterface__Sequence__init(motor_control_msgs__msg__DmInterface__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_control_msgs__msg__DmInterface * data = NULL;

  if (size) {
    data = (motor_control_msgs__msg__DmInterface *)allocator.zero_allocate(size, sizeof(motor_control_msgs__msg__DmInterface), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motor_control_msgs__msg__DmInterface__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motor_control_msgs__msg__DmInterface__fini(&data[i - 1]);
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
motor_control_msgs__msg__DmInterface__Sequence__fini(motor_control_msgs__msg__DmInterface__Sequence * array)
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
      motor_control_msgs__msg__DmInterface__fini(&array->data[i]);
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

motor_control_msgs__msg__DmInterface__Sequence *
motor_control_msgs__msg__DmInterface__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_control_msgs__msg__DmInterface__Sequence * array = (motor_control_msgs__msg__DmInterface__Sequence *)allocator.allocate(sizeof(motor_control_msgs__msg__DmInterface__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motor_control_msgs__msg__DmInterface__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motor_control_msgs__msg__DmInterface__Sequence__destroy(motor_control_msgs__msg__DmInterface__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motor_control_msgs__msg__DmInterface__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motor_control_msgs__msg__DmInterface__Sequence__are_equal(const motor_control_msgs__msg__DmInterface__Sequence * lhs, const motor_control_msgs__msg__DmInterface__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motor_control_msgs__msg__DmInterface__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motor_control_msgs__msg__DmInterface__Sequence__copy(
  const motor_control_msgs__msg__DmInterface__Sequence * input,
  motor_control_msgs__msg__DmInterface__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motor_control_msgs__msg__DmInterface);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motor_control_msgs__msg__DmInterface * data =
      (motor_control_msgs__msg__DmInterface *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motor_control_msgs__msg__DmInterface__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motor_control_msgs__msg__DmInterface__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motor_control_msgs__msg__DmInterface__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
