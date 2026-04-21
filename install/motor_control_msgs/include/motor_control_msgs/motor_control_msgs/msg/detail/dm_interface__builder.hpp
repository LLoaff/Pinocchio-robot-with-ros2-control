// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_control_msgs:msg/DmInterface.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_MSGS__MSG__DETAIL__DM_INTERFACE__BUILDER_HPP_
#define MOTOR_CONTROL_MSGS__MSG__DETAIL__DM_INTERFACE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_control_msgs/msg/detail/dm_interface__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_control_msgs
{

namespace msg
{

namespace builder
{

class Init_DmInterface_tau
{
public:
  explicit Init_DmInterface_tau(::motor_control_msgs::msg::DmInterface & msg)
  : msg_(msg)
  {}
  ::motor_control_msgs::msg::DmInterface tau(::motor_control_msgs::msg::DmInterface::_tau_type arg)
  {
    msg_.tau = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_control_msgs::msg::DmInterface msg_;
};

class Init_DmInterface_kd
{
public:
  explicit Init_DmInterface_kd(::motor_control_msgs::msg::DmInterface & msg)
  : msg_(msg)
  {}
  Init_DmInterface_tau kd(::motor_control_msgs::msg::DmInterface::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return Init_DmInterface_tau(msg_);
  }

private:
  ::motor_control_msgs::msg::DmInterface msg_;
};

class Init_DmInterface_kp
{
public:
  explicit Init_DmInterface_kp(::motor_control_msgs::msg::DmInterface & msg)
  : msg_(msg)
  {}
  Init_DmInterface_kd kp(::motor_control_msgs::msg::DmInterface::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_DmInterface_kd(msg_);
  }

private:
  ::motor_control_msgs::msg::DmInterface msg_;
};

class Init_DmInterface_dq
{
public:
  explicit Init_DmInterface_dq(::motor_control_msgs::msg::DmInterface & msg)
  : msg_(msg)
  {}
  Init_DmInterface_kp dq(::motor_control_msgs::msg::DmInterface::_dq_type arg)
  {
    msg_.dq = std::move(arg);
    return Init_DmInterface_kp(msg_);
  }

private:
  ::motor_control_msgs::msg::DmInterface msg_;
};

class Init_DmInterface_q
{
public:
  explicit Init_DmInterface_q(::motor_control_msgs::msg::DmInterface & msg)
  : msg_(msg)
  {}
  Init_DmInterface_dq q(::motor_control_msgs::msg::DmInterface::_q_type arg)
  {
    msg_.q = std::move(arg);
    return Init_DmInterface_dq(msg_);
  }

private:
  ::motor_control_msgs::msg::DmInterface msg_;
};

class Init_DmInterface_id
{
public:
  explicit Init_DmInterface_id(::motor_control_msgs::msg::DmInterface & msg)
  : msg_(msg)
  {}
  Init_DmInterface_q id(::motor_control_msgs::msg::DmInterface::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_DmInterface_q(msg_);
  }

private:
  ::motor_control_msgs::msg::DmInterface msg_;
};

class Init_DmInterface_header
{
public:
  Init_DmInterface_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DmInterface_id header(::motor_control_msgs::msg::DmInterface::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DmInterface_id(msg_);
  }

private:
  ::motor_control_msgs::msg::DmInterface msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_control_msgs::msg::DmInterface>()
{
  return motor_control_msgs::msg::builder::Init_DmInterface_header();
}

}  // namespace motor_control_msgs

#endif  // MOTOR_CONTROL_MSGS__MSG__DETAIL__DM_INTERFACE__BUILDER_HPP_
