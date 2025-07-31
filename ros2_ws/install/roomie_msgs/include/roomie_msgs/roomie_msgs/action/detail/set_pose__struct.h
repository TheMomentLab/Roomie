// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from roomie_msgs:action/SetPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "roomie_msgs/action/set_pose.h"


#ifndef ROOMIE_MSGS__ACTION__DETAIL__SET_POSE__STRUCT_H_
#define ROOMIE_MSGS__ACTION__DETAIL__SET_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/SetPose in the package roomie_msgs.
typedef struct roomie_msgs__action__SetPose_Goal
{
  int32_t robot_id;
  int32_t pose_id;
} roomie_msgs__action__SetPose_Goal;

// Struct for a sequence of roomie_msgs__action__SetPose_Goal.
typedef struct roomie_msgs__action__SetPose_Goal__Sequence
{
  roomie_msgs__action__SetPose_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roomie_msgs__action__SetPose_Goal__Sequence;

// Constants defined in the message

/// Struct defined in action/SetPose in the package roomie_msgs.
typedef struct roomie_msgs__action__SetPose_Result
{
  int32_t robot_id;
  bool success;
} roomie_msgs__action__SetPose_Result;

// Struct for a sequence of roomie_msgs__action__SetPose_Result.
typedef struct roomie_msgs__action__SetPose_Result__Sequence
{
  roomie_msgs__action__SetPose_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roomie_msgs__action__SetPose_Result__Sequence;

// Constants defined in the message

/// Struct defined in action/SetPose in the package roomie_msgs.
typedef struct roomie_msgs__action__SetPose_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} roomie_msgs__action__SetPose_Feedback;

// Struct for a sequence of roomie_msgs__action__SetPose_Feedback.
typedef struct roomie_msgs__action__SetPose_Feedback__Sequence
{
  roomie_msgs__action__SetPose_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roomie_msgs__action__SetPose_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "roomie_msgs/action/detail/set_pose__struct.h"

/// Struct defined in action/SetPose in the package roomie_msgs.
typedef struct roomie_msgs__action__SetPose_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  roomie_msgs__action__SetPose_Goal goal;
} roomie_msgs__action__SetPose_SendGoal_Request;

// Struct for a sequence of roomie_msgs__action__SetPose_SendGoal_Request.
typedef struct roomie_msgs__action__SetPose_SendGoal_Request__Sequence
{
  roomie_msgs__action__SetPose_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roomie_msgs__action__SetPose_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/SetPose in the package roomie_msgs.
typedef struct roomie_msgs__action__SetPose_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} roomie_msgs__action__SetPose_SendGoal_Response;

// Struct for a sequence of roomie_msgs__action__SetPose_SendGoal_Response.
typedef struct roomie_msgs__action__SetPose_SendGoal_Response__Sequence
{
  roomie_msgs__action__SetPose_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roomie_msgs__action__SetPose_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  roomie_msgs__action__SetPose_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  roomie_msgs__action__SetPose_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/SetPose in the package roomie_msgs.
typedef struct roomie_msgs__action__SetPose_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  roomie_msgs__action__SetPose_SendGoal_Request__Sequence request;
  roomie_msgs__action__SetPose_SendGoal_Response__Sequence response;
} roomie_msgs__action__SetPose_SendGoal_Event;

// Struct for a sequence of roomie_msgs__action__SetPose_SendGoal_Event.
typedef struct roomie_msgs__action__SetPose_SendGoal_Event__Sequence
{
  roomie_msgs__action__SetPose_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roomie_msgs__action__SetPose_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/SetPose in the package roomie_msgs.
typedef struct roomie_msgs__action__SetPose_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} roomie_msgs__action__SetPose_GetResult_Request;

// Struct for a sequence of roomie_msgs__action__SetPose_GetResult_Request.
typedef struct roomie_msgs__action__SetPose_GetResult_Request__Sequence
{
  roomie_msgs__action__SetPose_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roomie_msgs__action__SetPose_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "roomie_msgs/action/detail/set_pose__struct.h"

/// Struct defined in action/SetPose in the package roomie_msgs.
typedef struct roomie_msgs__action__SetPose_GetResult_Response
{
  int8_t status;
  roomie_msgs__action__SetPose_Result result;
} roomie_msgs__action__SetPose_GetResult_Response;

// Struct for a sequence of roomie_msgs__action__SetPose_GetResult_Response.
typedef struct roomie_msgs__action__SetPose_GetResult_Response__Sequence
{
  roomie_msgs__action__SetPose_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roomie_msgs__action__SetPose_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  roomie_msgs__action__SetPose_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  roomie_msgs__action__SetPose_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/SetPose in the package roomie_msgs.
typedef struct roomie_msgs__action__SetPose_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  roomie_msgs__action__SetPose_GetResult_Request__Sequence request;
  roomie_msgs__action__SetPose_GetResult_Response__Sequence response;
} roomie_msgs__action__SetPose_GetResult_Event;

// Struct for a sequence of roomie_msgs__action__SetPose_GetResult_Event.
typedef struct roomie_msgs__action__SetPose_GetResult_Event__Sequence
{
  roomie_msgs__action__SetPose_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roomie_msgs__action__SetPose_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "roomie_msgs/action/detail/set_pose__struct.h"

/// Struct defined in action/SetPose in the package roomie_msgs.
typedef struct roomie_msgs__action__SetPose_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  roomie_msgs__action__SetPose_Feedback feedback;
} roomie_msgs__action__SetPose_FeedbackMessage;

// Struct for a sequence of roomie_msgs__action__SetPose_FeedbackMessage.
typedef struct roomie_msgs__action__SetPose_FeedbackMessage__Sequence
{
  roomie_msgs__action__SetPose_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roomie_msgs__action__SetPose_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROOMIE_MSGS__ACTION__DETAIL__SET_POSE__STRUCT_H_
