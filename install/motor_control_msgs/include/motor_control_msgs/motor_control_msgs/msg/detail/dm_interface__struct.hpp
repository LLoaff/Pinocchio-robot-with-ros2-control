// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_control_msgs:msg/DmInterface.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_MSGS__MSG__DETAIL__DM_INTERFACE__STRUCT_HPP_
#define MOTOR_CONTROL_MSGS__MSG__DETAIL__DM_INTERFACE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__motor_control_msgs__msg__DmInterface __attribute__((deprecated))
#else
# define DEPRECATED__motor_control_msgs__msg__DmInterface __declspec(deprecated)
#endif

namespace motor_control_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DmInterface_
{
  using Type = DmInterface_<ContainerAllocator>;

  explicit DmInterface_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit DmInterface_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _id_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _id_type id;
  using _q_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _q_type q;
  using _dq_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _dq_type dq;
  using _kp_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _kp_type kp;
  using _kd_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _kd_type kd;
  using _tau_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _tau_type tau;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__id(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__q(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->q = _arg;
    return *this;
  }
  Type & set__dq(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->dq = _arg;
    return *this;
  }
  Type & set__kp(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->kp = _arg;
    return *this;
  }
  Type & set__kd(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->kd = _arg;
    return *this;
  }
  Type & set__tau(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->tau = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_control_msgs::msg::DmInterface_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_control_msgs::msg::DmInterface_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_control_msgs::msg::DmInterface_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_control_msgs::msg::DmInterface_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_control_msgs::msg::DmInterface_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_control_msgs::msg::DmInterface_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_control_msgs::msg::DmInterface_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_control_msgs::msg::DmInterface_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_control_msgs::msg::DmInterface_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_control_msgs::msg::DmInterface_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_control_msgs__msg__DmInterface
    std::shared_ptr<motor_control_msgs::msg::DmInterface_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_control_msgs__msg__DmInterface
    std::shared_ptr<motor_control_msgs::msg::DmInterface_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DmInterface_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->q != other.q) {
      return false;
    }
    if (this->dq != other.dq) {
      return false;
    }
    if (this->kp != other.kp) {
      return false;
    }
    if (this->kd != other.kd) {
      return false;
    }
    if (this->tau != other.tau) {
      return false;
    }
    return true;
  }
  bool operator!=(const DmInterface_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DmInterface_

// alias to use template instance with default allocator
using DmInterface =
  motor_control_msgs::msg::DmInterface_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_control_msgs

#endif  // MOTOR_CONTROL_MSGS__MSG__DETAIL__DM_INTERFACE__STRUCT_HPP_
