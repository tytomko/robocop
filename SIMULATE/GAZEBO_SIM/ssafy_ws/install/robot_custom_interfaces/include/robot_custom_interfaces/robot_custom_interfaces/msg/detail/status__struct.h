// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_custom_interfaces:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_CUSTOM_INTERFACES__MSG__DETAIL__STATUS__STRUCT_H_
#define ROBOT_CUSTOM_INTERFACES__MSG__DETAIL__STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'status'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Status in the package robot_custom_interfaces.
/**
  * 로봇의 고유 ID
 */
typedef struct robot_custom_interfaces__msg__Status
{
  int32_t id;
  /// 현재 상태를 나타내는 문자열
  rosidl_runtime_c__String status;
  /// 로봇의 온도(섭씨)
  float temperature;
  /// true면 동작 중, false면 정지
  bool is_active;
} robot_custom_interfaces__msg__Status;

// Struct for a sequence of robot_custom_interfaces__msg__Status.
typedef struct robot_custom_interfaces__msg__Status__Sequence
{
  robot_custom_interfaces__msg__Status * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_custom_interfaces__msg__Status__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_CUSTOM_INTERFACES__MSG__DETAIL__STATUS__STRUCT_H_
