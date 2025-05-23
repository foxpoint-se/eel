cmake_minimum_required(VERSION 3.5)
project(eel_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImuStatus.msg"
  "msg/ImuOffsets.msg"
  "msg/Coordinate.msg"
  "msg/NavigationStatus.msg"
  "msg/TankStatus.msg"
  "msg/PressureStatus.msg"
  "msg/DepthControlStatus.msg"
  "msg/DepthControlCmd.msg"
  "msg/BatteryStatus.msg"
  "msg/PidDepthCmd.msg"
  "msg/PidPitchCmd.msg"
  "msg/ModemStatus.msg"
  "msg/NavigationAssignment.msg"
  "msg/NavigationMission.msg"
  "msg/SubmergedCoordinate.msg"
  "msg/TracedRoute.msg"
  "action/Dive.action"
  "action/Navigate.action"
  "srv/ModemStatus.srv"
  "srv/ModemPing.srv"
)

ament_package()
