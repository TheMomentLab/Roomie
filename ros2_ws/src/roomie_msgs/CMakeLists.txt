cmake_minimum_required(VERSION 3.8)
project(roomie_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Find Python and numpy for rosidl Python generation
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
  OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)

if(NUMPY_INCLUDE_DIR)
  message(STATUS "Found numpy include directory: ${NUMPY_INCLUDE_DIR}")
  include_directories(${NUMPY_INCLUDE_DIR})
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I${NUMPY_INCLUDE_DIR}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I${NUMPY_INCLUDE_DIR}")
else()
  # Fallback to system numpy if available
  if(EXISTS "/usr/lib/python3/dist-packages/numpy/core/include")
    set(NUMPY_INCLUDE_DIR "/usr/lib/python3/dist-packages/numpy/core/include")
    message(STATUS "Using system numpy include directory: ${NUMPY_INCLUDE_DIR}")
    include_directories(${NUMPY_INCLUDE_DIR})
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I${NUMPY_INCLUDE_DIR}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I${NUMPY_INCLUDE_DIR}")
  else()
    message(WARNING "Could not find numpy include directory")
  endif()
endif()

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages - Robot Status
  "msg/robot_status/RobotState.msg"
  "msg/robot_status/BatteryStatus.msg"
  "msg/robot_status/RoomiePose.msg"
  "msg/robot_status/Arrival.msg"
  
  # Messages - Task Management
  "msg/task_management/TaskState.msg"
  "msg/task_management/PickupCompleted.msg"
  "msg/task_management/DeliveryCompleted.msg"
  
  # Messages - User Interface
  "msg/user_interface/RobotGuiEvent.msg"
  "msg/user_interface/TrackingEvent.msg"
  "msg/user_interface/Registered.msg"
  
  # Services - Robot Control
  "srv/robot_control/CreateTask.srv"
  "srv/robot_control/SetVSMode.srv"
  "srv/robot_control/Location.srv"
  "srv/robot_control/GetLocations.srv"
  
  # Services - Door/Elevator
  "srv/door_elevator/UnlockDoor.srv"
  "srv/door_elevator/ElevatorWidth.srv"
  "srv/door_elevator/ElevatorStatus.srv"
  "srv/door_elevator/DoorStatus.srv"
  
  # Services - Sensor
  "srv/sensor/ButtonStatus.srv"
  "srv/sensor/SpaceAvailability.srv"
  "srv/sensor/ReadCardInfo.srv"
  "srv/sensor/CheckItemLoaded.srv"
  "srv/sensor/CheckDoorState.srv"
  

  
  # Services - Security
  "srv/security/ControlLock.srv"
  
  # Actions
  "action/PerformTask.action"
  "action/PerformReturn.action"
  "action/StartCountdown.action"
  "action/ReturnCountdown.action"
  
  DEPENDENCIES
    geometry_msgs
    builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 