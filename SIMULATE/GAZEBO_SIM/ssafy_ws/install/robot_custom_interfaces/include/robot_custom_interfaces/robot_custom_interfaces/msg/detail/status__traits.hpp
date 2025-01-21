// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_custom_interfaces:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_CUSTOM_INTERFACES__MSG__DETAIL__STATUS__TRAITS_HPP_
#define ROBOT_CUSTOM_INTERFACES__MSG__DETAIL__STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_custom_interfaces/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_custom_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Status & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << ", ";
  }

  // member: is_active
  {
    out << "is_active: ";
    rosidl_generator_traits::value_to_yaml(msg.is_active, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Status & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
  }

  // member: is_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_active: ";
    rosidl_generator_traits::value_to_yaml(msg.is_active, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Status & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robot_custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_custom_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_custom_interfaces::msg::Status & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_custom_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_custom_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_custom_interfaces::msg::Status & msg)
{
  return robot_custom_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_custom_interfaces::msg::Status>()
{
  return "robot_custom_interfaces::msg::Status";
}

template<>
inline const char * name<robot_custom_interfaces::msg::Status>()
{
  return "robot_custom_interfaces/msg/Status";
}

template<>
struct has_fixed_size<robot_custom_interfaces::msg::Status>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_custom_interfaces::msg::Status>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_custom_interfaces::msg::Status>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_CUSTOM_INTERFACES__MSG__DETAIL__STATUS__TRAITS_HPP_
