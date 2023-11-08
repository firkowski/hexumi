cmake_minimum_required(VERSION 3.8)
project(hexumi_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs)
find_package(std_msgs)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EulerAngles.msg"
  "msg/PoseEuler.msg"
  "msg/MetaCommands.msg"
  "msg/HexapodMotion.msg"
  "msg/BodyControl.msg"
  "msg/FeetGroundContact.msg"
  "msg/LegPositions.msg"

  "msg/Current.msg"
  "msg/FeetContacts.msg"
  "msg/JointAngles.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()

