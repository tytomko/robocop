// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_custom_interfaces:msg/Status.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_custom_interfaces/msg/detail/status__rosidl_typesupport_introspection_c.h"
#include "robot_custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_custom_interfaces/msg/detail/status__functions.h"
#include "robot_custom_interfaces/msg/detail/status__struct.h"


// Include directives for member types
// Member `status`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_custom_interfaces__msg__Status__rosidl_typesupport_introspection_c__Status_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_custom_interfaces__msg__Status__init(message_memory);
}

void robot_custom_interfaces__msg__Status__rosidl_typesupport_introspection_c__Status_fini_function(void * message_memory)
{
  robot_custom_interfaces__msg__Status__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_custom_interfaces__msg__Status__rosidl_typesupport_introspection_c__Status_message_member_array[4] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_custom_interfaces__msg__Status, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_custom_interfaces__msg__Status, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_custom_interfaces__msg__Status, temperature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_active",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_custom_interfaces__msg__Status, is_active),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_custom_interfaces__msg__Status__rosidl_typesupport_introspection_c__Status_message_members = {
  "robot_custom_interfaces__msg",  // message namespace
  "Status",  // message name
  4,  // number of fields
  sizeof(robot_custom_interfaces__msg__Status),
  robot_custom_interfaces__msg__Status__rosidl_typesupport_introspection_c__Status_message_member_array,  // message members
  robot_custom_interfaces__msg__Status__rosidl_typesupport_introspection_c__Status_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_custom_interfaces__msg__Status__rosidl_typesupport_introspection_c__Status_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_custom_interfaces__msg__Status__rosidl_typesupport_introspection_c__Status_message_type_support_handle = {
  0,
  &robot_custom_interfaces__msg__Status__rosidl_typesupport_introspection_c__Status_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_custom_interfaces, msg, Status)() {
  if (!robot_custom_interfaces__msg__Status__rosidl_typesupport_introspection_c__Status_message_type_support_handle.typesupport_identifier) {
    robot_custom_interfaces__msg__Status__rosidl_typesupport_introspection_c__Status_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_custom_interfaces__msg__Status__rosidl_typesupport_introspection_c__Status_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
