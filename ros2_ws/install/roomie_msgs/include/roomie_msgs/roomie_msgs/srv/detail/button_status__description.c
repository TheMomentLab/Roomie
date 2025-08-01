// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from roomie_msgs:srv/ButtonStatus.idl
// generated code does not contain a copyright notice

#include "roomie_msgs/srv/detail/button_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_roomie_msgs
const rosidl_type_hash_t *
roomie_msgs__srv__ButtonStatus__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6e, 0xde, 0x6b, 0xaa, 0xfd, 0xd9, 0x7b, 0x6a,
      0x3c, 0x55, 0xf7, 0xaa, 0xe6, 0x2c, 0x73, 0xc0,
      0x78, 0x92, 0x27, 0x99, 0xad, 0x42, 0x1c, 0x6b,
      0x87, 0x3d, 0x4e, 0x8a, 0xe6, 0x0b, 0x8b, 0x8f,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_roomie_msgs
const rosidl_type_hash_t *
roomie_msgs__srv__ButtonStatus_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x07, 0x39, 0x59, 0xfd, 0x7d, 0x5a, 0x67, 0x41,
      0x1a, 0x21, 0x0a, 0xa0, 0x3c, 0xe7, 0x5b, 0x17,
      0x5f, 0x0c, 0xce, 0x4d, 0x15, 0x4d, 0xfd, 0x9f,
      0x01, 0x28, 0x00, 0x6e, 0xa1, 0xb8, 0x6e, 0xe2,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_roomie_msgs
const rosidl_type_hash_t *
roomie_msgs__srv__ButtonStatus_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfc, 0xc6, 0xaf, 0x09, 0x7c, 0xbd, 0xea, 0x1e,
      0x33, 0xaf, 0x02, 0xf8, 0x77, 0xee, 0x4a, 0xb4,
      0xcd, 0x0a, 0x6e, 0x39, 0xec, 0xbc, 0x19, 0x54,
      0x18, 0xf0, 0xb6, 0xe8, 0x13, 0x9c, 0x83, 0x72,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_roomie_msgs
const rosidl_type_hash_t *
roomie_msgs__srv__ButtonStatus_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd1, 0xcd, 0xcf, 0xc0, 0x4b, 0xb8, 0x73, 0xb4,
      0x05, 0x28, 0xca, 0xd8, 0x27, 0xb2, 0x6c, 0x38,
      0x49, 0x80, 0xbf, 0x03, 0x82, 0x10, 0xae, 0x69,
      0x22, 0xe6, 0x16, 0x93, 0x1b, 0x87, 0x50, 0xc1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char roomie_msgs__srv__ButtonStatus__TYPE_NAME[] = "roomie_msgs/srv/ButtonStatus";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char roomie_msgs__srv__ButtonStatus_Event__TYPE_NAME[] = "roomie_msgs/srv/ButtonStatus_Event";
static char roomie_msgs__srv__ButtonStatus_Request__TYPE_NAME[] = "roomie_msgs/srv/ButtonStatus_Request";
static char roomie_msgs__srv__ButtonStatus_Response__TYPE_NAME[] = "roomie_msgs/srv/ButtonStatus_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char roomie_msgs__srv__ButtonStatus__FIELD_NAME__request_message[] = "request_message";
static char roomie_msgs__srv__ButtonStatus__FIELD_NAME__response_message[] = "response_message";
static char roomie_msgs__srv__ButtonStatus__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field roomie_msgs__srv__ButtonStatus__FIELDS[] = {
  {
    {roomie_msgs__srv__ButtonStatus__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {roomie_msgs__srv__ButtonStatus_Request__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {roomie_msgs__srv__ButtonStatus__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {roomie_msgs__srv__ButtonStatus_Response__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
  {
    {roomie_msgs__srv__ButtonStatus__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {roomie_msgs__srv__ButtonStatus_Event__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription roomie_msgs__srv__ButtonStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {roomie_msgs__srv__ButtonStatus_Event__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {roomie_msgs__srv__ButtonStatus_Request__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {roomie_msgs__srv__ButtonStatus_Response__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
roomie_msgs__srv__ButtonStatus__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {roomie_msgs__srv__ButtonStatus__TYPE_NAME, 28, 28},
      {roomie_msgs__srv__ButtonStatus__FIELDS, 3, 3},
    },
    {roomie_msgs__srv__ButtonStatus__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = roomie_msgs__srv__ButtonStatus_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = roomie_msgs__srv__ButtonStatus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = roomie_msgs__srv__ButtonStatus_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char roomie_msgs__srv__ButtonStatus_Request__FIELD_NAME__robot_id[] = "robot_id";
static char roomie_msgs__srv__ButtonStatus_Request__FIELD_NAME__button_id[] = "button_id";

static rosidl_runtime_c__type_description__Field roomie_msgs__srv__ButtonStatus_Request__FIELDS[] = {
  {
    {roomie_msgs__srv__ButtonStatus_Request__FIELD_NAME__robot_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {roomie_msgs__srv__ButtonStatus_Request__FIELD_NAME__button_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
roomie_msgs__srv__ButtonStatus_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {roomie_msgs__srv__ButtonStatus_Request__TYPE_NAME, 36, 36},
      {roomie_msgs__srv__ButtonStatus_Request__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char roomie_msgs__srv__ButtonStatus_Response__FIELD_NAME__robot_id[] = "robot_id";
static char roomie_msgs__srv__ButtonStatus_Response__FIELD_NAME__button_id[] = "button_id";
static char roomie_msgs__srv__ButtonStatus_Response__FIELD_NAME__success[] = "success";
static char roomie_msgs__srv__ButtonStatus_Response__FIELD_NAME__x[] = "x";
static char roomie_msgs__srv__ButtonStatus_Response__FIELD_NAME__y[] = "y";
static char roomie_msgs__srv__ButtonStatus_Response__FIELD_NAME__size[] = "size";
static char roomie_msgs__srv__ButtonStatus_Response__FIELD_NAME__is_pressed[] = "is_pressed";
static char roomie_msgs__srv__ButtonStatus_Response__FIELD_NAME__timestamp[] = "timestamp";

static rosidl_runtime_c__type_description__Field roomie_msgs__srv__ButtonStatus_Response__FIELDS[] = {
  {
    {roomie_msgs__srv__ButtonStatus_Response__FIELD_NAME__robot_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {roomie_msgs__srv__ButtonStatus_Response__FIELD_NAME__button_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {roomie_msgs__srv__ButtonStatus_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {roomie_msgs__srv__ButtonStatus_Response__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {roomie_msgs__srv__ButtonStatus_Response__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {roomie_msgs__srv__ButtonStatus_Response__FIELD_NAME__size, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {roomie_msgs__srv__ButtonStatus_Response__FIELD_NAME__is_pressed, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {roomie_msgs__srv__ButtonStatus_Response__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription roomie_msgs__srv__ButtonStatus_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
roomie_msgs__srv__ButtonStatus_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {roomie_msgs__srv__ButtonStatus_Response__TYPE_NAME, 37, 37},
      {roomie_msgs__srv__ButtonStatus_Response__FIELDS, 8, 8},
    },
    {roomie_msgs__srv__ButtonStatus_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char roomie_msgs__srv__ButtonStatus_Event__FIELD_NAME__info[] = "info";
static char roomie_msgs__srv__ButtonStatus_Event__FIELD_NAME__request[] = "request";
static char roomie_msgs__srv__ButtonStatus_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field roomie_msgs__srv__ButtonStatus_Event__FIELDS[] = {
  {
    {roomie_msgs__srv__ButtonStatus_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {roomie_msgs__srv__ButtonStatus_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {roomie_msgs__srv__ButtonStatus_Request__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {roomie_msgs__srv__ButtonStatus_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {roomie_msgs__srv__ButtonStatus_Response__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription roomie_msgs__srv__ButtonStatus_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {roomie_msgs__srv__ButtonStatus_Request__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {roomie_msgs__srv__ButtonStatus_Response__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
roomie_msgs__srv__ButtonStatus_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {roomie_msgs__srv__ButtonStatus_Event__TYPE_NAME, 34, 34},
      {roomie_msgs__srv__ButtonStatus_Event__FIELDS, 3, 3},
    },
    {roomie_msgs__srv__ButtonStatus_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = roomie_msgs__srv__ButtonStatus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = roomie_msgs__srv__ButtonStatus_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# ButtonStatus.srv\n"
  "# Request\n"
  "int32 robot_id\n"
  "int32 button_id\n"
  "---\n"
  "# Response\n"
  "int32 robot_id\n"
  "int32 button_id\n"
  "bool success\n"
  "float32 x\n"
  "float32 y\n"
  "float32 size\n"
  "bool is_pressed\n"
  "builtin_interfaces/Time timestamp\n"
  "\n"
  "# x: 0~1\n"
  "# y: 0~1\n"
  "# size: 0~1\n"
  "\n"
  "# button_id values:\n"
  "# 0: (\\xed\\x98\\x84\\xec\\x9e\\xac \\xec\\x9c\\xa0\\xec\\x9d\\xbc\\xed\\x95\\x98\\xea\\xb2\\x8c \\xea\\xb0\\x90\\xec\\xa7\\x80\\xeb\\x90\\x98\\xeb\\x8a\\x94 \\xeb\\xb2\\x84\\xed\\x8a\\xbc)\n"
  "# \\xeb\\xb2\\x84\\xed\\x8a\\xbc\\xec\\x9d\\xb4 2\\xea\\xb0\\x9c \\xec\\x9d\\xb4\\xec\\x83\\x81 \\xea\\xb0\\x90\\xec\\xa7\\x80\\xeb\\x90\\xa0 \\xea\\xb2\\xbd\\xec\\x9a\\xb0 success=false\n"
  "# 1: 1\\xec\\xb8\\xb5\n"
  "# 2: 2\\xec\\xb8\\xb5\n"
  "# 3: 3\\xec\\xb8\\xb5\n"
  "# 4: 4\\xec\\xb8\\xb5\n"
  "# 5: 5\\xec\\xb8\\xb5\n"
  "# 6: 6\\xec\\xb8\\xb5\n"
  "# 7: 7\\xec\\xb8\\xb5\n"
  "# 8: 8\\xec\\xb8\\xb5\n"
  "# 9: 9\\xec\\xb8\\xb5\n"
  "# 10: 10\\xec\\xb8\\xb5\n"
  "# 11: 11\\xec\\xb8\\xb5\n"
  "# 12: 12\\xec\\xb8\\xb5\n"
  "# 13: B1\\xec\\xb8\\xb5\n"
  "# 14: B2\\xec\\xb8\\xb5\n"
  "# 100: \\xed\\x95\\x98\\xed\\x96\\x89\\xeb\\xb2\\x84\\xed\\x8a\\xbc\n"
  "# 101: \\xec\\x83\\x81\\xed\\x96\\x89\\xeb\\xb2\\x84\\xed\\x8a\\xbc\n"
  "# 102: \\xec\\x97\\xb4\\xea\\xb8\\xb0\\xeb\\xb2\\x84\\xed\\x8a\\xbc\n"
  "# 103: \\xeb\\x8b\\xab\\xea\\xb8\\xb0\\xeb\\xb2\\x84\\xed\\x8a\\xbc ";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
roomie_msgs__srv__ButtonStatus__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {roomie_msgs__srv__ButtonStatus__TYPE_NAME, 28, 28},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 480, 480},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
roomie_msgs__srv__ButtonStatus_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {roomie_msgs__srv__ButtonStatus_Request__TYPE_NAME, 36, 36},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
roomie_msgs__srv__ButtonStatus_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {roomie_msgs__srv__ButtonStatus_Response__TYPE_NAME, 37, 37},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
roomie_msgs__srv__ButtonStatus_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {roomie_msgs__srv__ButtonStatus_Event__TYPE_NAME, 34, 34},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
roomie_msgs__srv__ButtonStatus__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *roomie_msgs__srv__ButtonStatus__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *roomie_msgs__srv__ButtonStatus_Event__get_individual_type_description_source(NULL);
    sources[3] = *roomie_msgs__srv__ButtonStatus_Request__get_individual_type_description_source(NULL);
    sources[4] = *roomie_msgs__srv__ButtonStatus_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
roomie_msgs__srv__ButtonStatus_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *roomie_msgs__srv__ButtonStatus_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
roomie_msgs__srv__ButtonStatus_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *roomie_msgs__srv__ButtonStatus_Response__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
roomie_msgs__srv__ButtonStatus_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *roomie_msgs__srv__ButtonStatus_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *roomie_msgs__srv__ButtonStatus_Request__get_individual_type_description_source(NULL);
    sources[3] = *roomie_msgs__srv__ButtonStatus_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
