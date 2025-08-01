// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from roomie_msgs:srv/ButtonStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "roomie_msgs/srv/button_status.hpp"


#ifndef ROOMIE_MSGS__SRV__DETAIL__BUTTON_STATUS__STRUCT_HPP_
#define ROOMIE_MSGS__SRV__DETAIL__BUTTON_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__roomie_msgs__srv__ButtonStatus_Request __attribute__((deprecated))
#else
# define DEPRECATED__roomie_msgs__srv__ButtonStatus_Request __declspec(deprecated)
#endif

namespace roomie_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ButtonStatus_Request_
{
  using Type = ButtonStatus_Request_<ContainerAllocator>;

  explicit ButtonStatus_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->button_id = 0l;
    }
  }

  explicit ButtonStatus_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->button_id = 0l;
    }
  }

  // field types and members
  using _robot_id_type =
    int32_t;
  _robot_id_type robot_id;
  using _button_id_type =
    int32_t;
  _button_id_type button_id;

  // setters for named parameter idiom
  Type & set__robot_id(
    const int32_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__button_id(
    const int32_t & _arg)
  {
    this->button_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    roomie_msgs::srv::ButtonStatus_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const roomie_msgs::srv::ButtonStatus_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<roomie_msgs::srv::ButtonStatus_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<roomie_msgs::srv::ButtonStatus_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      roomie_msgs::srv::ButtonStatus_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<roomie_msgs::srv::ButtonStatus_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      roomie_msgs::srv::ButtonStatus_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<roomie_msgs::srv::ButtonStatus_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<roomie_msgs::srv::ButtonStatus_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<roomie_msgs::srv::ButtonStatus_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__roomie_msgs__srv__ButtonStatus_Request
    std::shared_ptr<roomie_msgs::srv::ButtonStatus_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__roomie_msgs__srv__ButtonStatus_Request
    std::shared_ptr<roomie_msgs::srv::ButtonStatus_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ButtonStatus_Request_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->button_id != other.button_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const ButtonStatus_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ButtonStatus_Request_

// alias to use template instance with default allocator
using ButtonStatus_Request =
  roomie_msgs::srv::ButtonStatus_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace roomie_msgs


// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__roomie_msgs__srv__ButtonStatus_Response __attribute__((deprecated))
#else
# define DEPRECATED__roomie_msgs__srv__ButtonStatus_Response __declspec(deprecated)
#endif

namespace roomie_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ButtonStatus_Response_
{
  using Type = ButtonStatus_Response_<ContainerAllocator>;

  explicit ButtonStatus_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->button_id = 0l;
      this->success = false;
      this->x = 0.0f;
      this->y = 0.0f;
      this->size = 0.0f;
      this->is_pressed = false;
    }
  }

  explicit ButtonStatus_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->button_id = 0l;
      this->success = false;
      this->x = 0.0f;
      this->y = 0.0f;
      this->size = 0.0f;
      this->is_pressed = false;
    }
  }

  // field types and members
  using _robot_id_type =
    int32_t;
  _robot_id_type robot_id;
  using _button_id_type =
    int32_t;
  _button_id_type button_id;
  using _success_type =
    bool;
  _success_type success;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _size_type =
    float;
  _size_type size;
  using _is_pressed_type =
    bool;
  _is_pressed_type is_pressed;
  using _timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_type timestamp;

  // setters for named parameter idiom
  Type & set__robot_id(
    const int32_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__button_id(
    const int32_t & _arg)
  {
    this->button_id = _arg;
    return *this;
  }
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__size(
    const float & _arg)
  {
    this->size = _arg;
    return *this;
  }
  Type & set__is_pressed(
    const bool & _arg)
  {
    this->is_pressed = _arg;
    return *this;
  }
  Type & set__timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    roomie_msgs::srv::ButtonStatus_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const roomie_msgs::srv::ButtonStatus_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<roomie_msgs::srv::ButtonStatus_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<roomie_msgs::srv::ButtonStatus_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      roomie_msgs::srv::ButtonStatus_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<roomie_msgs::srv::ButtonStatus_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      roomie_msgs::srv::ButtonStatus_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<roomie_msgs::srv::ButtonStatus_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<roomie_msgs::srv::ButtonStatus_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<roomie_msgs::srv::ButtonStatus_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__roomie_msgs__srv__ButtonStatus_Response
    std::shared_ptr<roomie_msgs::srv::ButtonStatus_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__roomie_msgs__srv__ButtonStatus_Response
    std::shared_ptr<roomie_msgs::srv::ButtonStatus_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ButtonStatus_Response_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->button_id != other.button_id) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->size != other.size) {
      return false;
    }
    if (this->is_pressed != other.is_pressed) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const ButtonStatus_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ButtonStatus_Response_

// alias to use template instance with default allocator
using ButtonStatus_Response =
  roomie_msgs::srv::ButtonStatus_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace roomie_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__roomie_msgs__srv__ButtonStatus_Event __attribute__((deprecated))
#else
# define DEPRECATED__roomie_msgs__srv__ButtonStatus_Event __declspec(deprecated)
#endif

namespace roomie_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ButtonStatus_Event_
{
  using Type = ButtonStatus_Event_<ContainerAllocator>;

  explicit ButtonStatus_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit ButtonStatus_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<roomie_msgs::srv::ButtonStatus_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<roomie_msgs::srv::ButtonStatus_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<roomie_msgs::srv::ButtonStatus_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<roomie_msgs::srv::ButtonStatus_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<roomie_msgs::srv::ButtonStatus_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<roomie_msgs::srv::ButtonStatus_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<roomie_msgs::srv::ButtonStatus_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<roomie_msgs::srv::ButtonStatus_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    roomie_msgs::srv::ButtonStatus_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const roomie_msgs::srv::ButtonStatus_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<roomie_msgs::srv::ButtonStatus_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<roomie_msgs::srv::ButtonStatus_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      roomie_msgs::srv::ButtonStatus_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<roomie_msgs::srv::ButtonStatus_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      roomie_msgs::srv::ButtonStatus_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<roomie_msgs::srv::ButtonStatus_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<roomie_msgs::srv::ButtonStatus_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<roomie_msgs::srv::ButtonStatus_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__roomie_msgs__srv__ButtonStatus_Event
    std::shared_ptr<roomie_msgs::srv::ButtonStatus_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__roomie_msgs__srv__ButtonStatus_Event
    std::shared_ptr<roomie_msgs::srv::ButtonStatus_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ButtonStatus_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const ButtonStatus_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ButtonStatus_Event_

// alias to use template instance with default allocator
using ButtonStatus_Event =
  roomie_msgs::srv::ButtonStatus_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace roomie_msgs

namespace roomie_msgs
{

namespace srv
{

struct ButtonStatus
{
  using Request = roomie_msgs::srv::ButtonStatus_Request;
  using Response = roomie_msgs::srv::ButtonStatus_Response;
  using Event = roomie_msgs::srv::ButtonStatus_Event;
};

}  // namespace srv

}  // namespace roomie_msgs

#endif  // ROOMIE_MSGS__SRV__DETAIL__BUTTON_STATUS__STRUCT_HPP_
