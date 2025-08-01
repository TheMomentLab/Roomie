// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from roomie_msgs:srv/ButtonStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "roomie_msgs/srv/button_status.h"


#ifndef ROOMIE_MSGS__SRV__DETAIL__BUTTON_STATUS__STRUCT_H_
#define ROOMIE_MSGS__SRV__DETAIL__BUTTON_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ButtonStatus in the package roomie_msgs.
typedef struct roomie_msgs__srv__ButtonStatus_Request
{
  int32_t robot_id;
  int32_t button_id;
} roomie_msgs__srv__ButtonStatus_Request;

// Struct for a sequence of roomie_msgs__srv__ButtonStatus_Request.
typedef struct roomie_msgs__srv__ButtonStatus_Request__Sequence
{
  roomie_msgs__srv__ButtonStatus_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roomie_msgs__srv__ButtonStatus_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in srv/ButtonStatus in the package roomie_msgs.
typedef struct roomie_msgs__srv__ButtonStatus_Response
{
  int32_t robot_id;
  int32_t button_id;
  bool success;
  float x;
  float y;
  float size;
  bool is_pressed;
  builtin_interfaces__msg__Time timestamp;
} roomie_msgs__srv__ButtonStatus_Response;

// Struct for a sequence of roomie_msgs__srv__ButtonStatus_Response.
typedef struct roomie_msgs__srv__ButtonStatus_Response__Sequence
{
  roomie_msgs__srv__ButtonStatus_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roomie_msgs__srv__ButtonStatus_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  roomie_msgs__srv__ButtonStatus_Event__request__MAX_SIZE = 1
};
// response
enum
{
  roomie_msgs__srv__ButtonStatus_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/ButtonStatus in the package roomie_msgs.
typedef struct roomie_msgs__srv__ButtonStatus_Event
{
  service_msgs__msg__ServiceEventInfo info;
  roomie_msgs__srv__ButtonStatus_Request__Sequence request;
  roomie_msgs__srv__ButtonStatus_Response__Sequence response;
} roomie_msgs__srv__ButtonStatus_Event;

// Struct for a sequence of roomie_msgs__srv__ButtonStatus_Event.
typedef struct roomie_msgs__srv__ButtonStatus_Event__Sequence
{
  roomie_msgs__srv__ButtonStatus_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roomie_msgs__srv__ButtonStatus_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROOMIE_MSGS__SRV__DETAIL__BUTTON_STATUS__STRUCT_H_
