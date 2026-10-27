cmake_minimum_required(VERSION 3.8)
project(hexumi_teleop_joy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hexumi_msgs REQUIRED)

add_executable(hexumi_teleop_joy src/hexumi_teleop_joy.cpp)
ament_target_dependencies(hexumi_teleop_joy rclcpp sensor_msgs geometry_msgs hexumi_msgs)

install(TARGETS 
  hexumi_teleop_joy
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

