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

# Find Python3 and NumPy
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

# Get NumPy include dir
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
  OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

include_directories(${NUMPY_INCLUDE_DIR})
message(STATUS "NumPy include path: ${NUMPY_INCLUDE_DIR}")

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages - Robot Status
  "msg/robot_status/RobotState.msg"
  "msg/robot_status/BatteryStatus.msg"
  "msg/robot_status/RoomiePose.msg"
  "msg/robot_status/Obstacle.msg"
  "msg/robot_status/GlassDoorStatus.msg"
  
  # Messages - User Interface
  "msg/user_interface/RobotGuiEvent.msg"
  
  # Services - Robot Control
  "srv/robot_control/SetVSMode.srv"
  "srv/robot_control/Location.srv"
  "srv/robot_control/GetLocations.srv"
  
  # Services - Door/Elevator
  "srv/door_elevator/DoorStatus.srv"
  "srv/door_elevator/ElevatorStatus.srv"
  
  # Services - Sensor
  "srv/sensor/ButtonStatus.srv"
  "srv/sensor/ButtonStatus2.srv"
  "srv/sensor/CheckItemLoaded.srv"
  "srv/sensor/CheckDoorState.srv"
  
  # Services - Security
  "srv/security/ControlLock.srv"
  
  # Actions
  "action/PerformTask.action"
  "action/PerformReturn.action"
  "action/StartCountdown.action"
  "action/ReturnCountdown.action"
  "action/SetPose.action"
  "action/ClickButton.action"
  
  DEPENDENCIES
    geometry_msgs
    builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
