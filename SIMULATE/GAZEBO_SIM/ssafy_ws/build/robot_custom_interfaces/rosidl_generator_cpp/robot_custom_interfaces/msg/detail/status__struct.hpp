// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_custom_interfaces:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_CUSTOM_INTERFACES__MSG__DETAIL__STATUS__STRUCT_HPP_
#define ROBOT_CUSTOM_INTERFACES__MSG__DETAIL__STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_custom_interfaces__msg__Status __attribute__((deprecated))
#else
# define DEPRECATED__robot_custom_interfaces__msg__Status __declspec(deprecated)
#endif

namespace robot_custom_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Status_
{
  using Type = Status_<ContainerAllocator>;

  explicit Status_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->status = "";
      this->temperature = 0.0f;
      this->is_active = false;
    }
  }

  explicit Status_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->status = "";
      this->temperature = 0.0f;
      this->is_active = false;
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;
  using _temperature_type =
    float;
  _temperature_type temperature;
  using _is_active_type =
    bool;
  _is_active_type is_active;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__temperature(
    const float & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__is_active(
    const bool & _arg)
  {
    this->is_active = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_custom_interfaces::msg::Status_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_custom_interfaces::msg::Status_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_custom_interfaces::msg::Status_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_custom_interfaces::msg::Status_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_custom_interfaces::msg::Status_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_custom_interfaces::msg::Status_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_custom_interfaces::msg::Status_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_custom_interfaces::msg::Status_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_custom_interfaces::msg::Status_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_custom_interfaces::msg::Status_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_custom_interfaces__msg__Status
    std::shared_ptr<robot_custom_interfaces::msg::Status_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_custom_interfaces__msg__Status
    std::shared_ptr<robot_custom_interfaces::msg::Status_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Status_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->is_active != other.is_active) {
      return false;
    }
    return true;
  }
  bool operator!=(const Status_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Status_

// alias to use template instance with default allocator
using Status =
  robot_custom_interfaces::msg::Status_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_custom_interfaces

#endif  // ROBOT_CUSTOM_INTERFACES__MSG__DETAIL__STATUS__STRUCT_HPP_
