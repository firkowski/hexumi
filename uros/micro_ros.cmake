if (DEFINED MICRO_ROS_PATH)
	message("Using Given MICRO_ROS_PATH '${MICRO_ROS_PATH}')")
else ()
	set(MICRO_ROS_PATH "${CMAKE_CURRENT_LIST_DIR}lib/micro_ros_raspberrypi_pico_sdk/libmicroros")
    message("Using local MICRO_ROS_PATH '${MICRO_ROS_PATH}')")
endif ()

add_library(micro_ros STATIC)

# Add include directory
target_include_directories(micro_ros PUBLIC 
    ${MICRO_ROS_PATH}/include
    ${MICRO_ROS_PATH}/include/rcl
    ${MICRO_ROS_PATH}/include/rcutils
    ${MICRO_ROS_PATH}/include/actionlib_msgs
	${MICRO_ROS_PATH}/include/action_msgs
	${MICRO_ROS_PATH}/include/builtin_interfaces
	${MICRO_ROS_PATH}/include/composition_interfaces
	${MICRO_ROS_PATH}/include/diagnostic_msgs
	${MICRO_ROS_PATH}/include/example_interfaces
	${MICRO_ROS_PATH}/include/geometry_msgs
	${MICRO_ROS_PATH}/include/servo2040_uros
	${MICRO_ROS_PATH}/include/lifecycle_msgs
	${MICRO_ROS_PATH}/include/micro_ros_msgs
	${MICRO_ROS_PATH}/include/nav_msgs
	${MICRO_ROS_PATH}/include/rcl
	${MICRO_ROS_PATH}/include/rcl_action
	${MICRO_ROS_PATH}/include/rcl_interfaces
	${MICRO_ROS_PATH}/include/rcl_lifecycle
	${MICRO_ROS_PATH}/include/rcl_logging_interface
	${MICRO_ROS_PATH}/include/rcutils
	${MICRO_ROS_PATH}/include/rmw
	${MICRO_ROS_PATH}/include/ros2_temperature
	${MICRO_ROS_PATH}/include/rosgraph_msgs
	${MICRO_ROS_PATH}/include/rosidl_runtime_c
	${MICRO_ROS_PATH}/include/rosidl_typesupport_c
	${MICRO_ROS_PATH}/include/rosidl_typesupport_interface
	${MICRO_ROS_PATH}/include/rosidl_typesupport_introspection_c
	${MICRO_ROS_PATH}/include/sensor_msgs
	${MICRO_ROS_PATH}/include/shape_msgs
	${MICRO_ROS_PATH}/include/statistics_msgs
	${MICRO_ROS_PATH}/include/std_msgs
	${MICRO_ROS_PATH}/include/std_srvs
	${MICRO_ROS_PATH}/include/stereo_msgs
	${MICRO_ROS_PATH}/include/test_msgs
	${MICRO_ROS_PATH}/include/tracetools
	${MICRO_ROS_PATH}/include/trajectory_msgs
	${MICRO_ROS_PATH}/include/unique_identifier_msgs
	${MICRO_ROS_PATH}/include/visualization_msgs
)

# Add the standard library to the build
target_link_libraries(micro_ros PUBLIC pico_stdlib microros)

link_directories(${MICRO_ROS_PATH})
