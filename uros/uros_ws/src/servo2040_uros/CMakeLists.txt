cmake_minimum_required(VERSION 3.8)
project(servo2040_uros)

set(BUILD_SHARED_LIBS
  OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointAngles.msg"
  "msg/FeetContacts.msg"
  "msg/Current.msg"
 )
 
ament_export_dependencies(rosidl_default_runtime)

ament_package()

install(
  PROGRAMS
    scripts/create_fwws.sh
  DESTINATION lib/${PROJECT_NAME}
)
