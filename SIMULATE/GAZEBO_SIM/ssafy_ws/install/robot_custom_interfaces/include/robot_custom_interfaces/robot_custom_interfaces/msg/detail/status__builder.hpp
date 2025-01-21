// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_custom_interfaces:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_CUSTOM_INTERFACES__MSG__DETAIL__STATUS__BUILDER_HPP_
#define ROBOT_CUSTOM_INTERFACES__MSG__DETAIL__STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_custom_interfaces/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_Status_is_active
{
public:
  explicit Init_Status_is_active(::robot_custom_interfaces::msg::Status & msg)
  : msg_(msg)
  {}
  ::robot_custom_interfaces::msg::Status is_active(::robot_custom_interfaces::msg::Status::_is_active_type arg)
  {
    msg_.is_active = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_custom_interfaces::msg::Status msg_;
};

class Init_Status_temperature
{
public:
  explicit Init_Status_temperature(::robot_custom_interfaces::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_is_active temperature(::robot_custom_interfaces::msg::Status::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_Status_is_active(msg_);
  }

private:
  ::robot_custom_interfaces::msg::Status msg_;
};

class Init_Status_status
{
public:
  explicit Init_Status_status(::robot_custom_interfaces::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_temperature status(::robot_custom_interfaces::msg::Status::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Status_temperature(msg_);
  }

private:
  ::robot_custom_interfaces::msg::Status msg_;
};

class Init_Status_id
{
public:
  Init_Status_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Status_status id(::robot_custom_interfaces::msg::Status::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Status_status(msg_);
  }

private:
  ::robot_custom_interfaces::msg::Status msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_custom_interfaces::msg::Status>()
{
  return robot_custom_interfaces::msg::builder::Init_Status_id();
}

}  // namespace robot_custom_interfaces

#endif  // ROBOT_CUSTOM_INTERFACES__MSG__DETAIL__STATUS__BUILDER_HPP_
