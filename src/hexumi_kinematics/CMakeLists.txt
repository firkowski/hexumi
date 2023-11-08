cmake_minimum_required(VERSION 3.8)
project(hexumi_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(hexumi_msgs REQUIRED)
find_package(servo2040_uros REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(eigen3_cmake_module REQUIRED)
# find_package(Eigen3)

add_executable(hexumi_kinematics src/kinematics.cpp)
ament_target_dependencies(hexumi_kinematics rclcpp std_msgs orocos_kdl hexumi_msgs servo2040_uros sensor_msgs)

install(TARGETS 
  hexumi_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
