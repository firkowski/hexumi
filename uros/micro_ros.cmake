if (DEFINED MICRO_ROS_PATH)
	message("Using Given MICRO_ROS_PATH '${MICRO_ROS_PATH}')")
else ()
	set(MICRO_ROS_PATH "${CMAKE_CURRENT_LIST_DIR}lib/micro_ros_raspberrypi_pico_sdk/libmicroros")
    message("Using local MICRO_ROS_PATH '${MICRO_ROS_PATH}')")
endif ()

add_library(micro_ros STATIC)

# Add include directories. micro_ros_setup installs most ROS packages as
# include/<pkg>/<pkg>/*.h, so each of those needs its own include path. The set
# changes between distros (Jazzy added service_msgs, type_description_interfaces,
# rosidl_dynamic_typesupport, ...), so discover them instead of listing them.
if(NOT EXISTS "${MICRO_ROS_PATH}/include")
    message(FATAL_ERROR "micro-ROS headers not found in '${MICRO_ROS_PATH}/include'. Build the micro-ROS library first (uros_ws: ros2 run micro_ros_setup build_firmware.sh ...).")
endif()
set(MICRO_ROS_INCLUDE_DIRS "${MICRO_ROS_PATH}/include")
file(GLOB MICRO_ROS_INCLUDE_PKGS LIST_DIRECTORIES true RELATIVE "${MICRO_ROS_PATH}/include" "${MICRO_ROS_PATH}/include/*")
foreach(pkg ${MICRO_ROS_INCLUDE_PKGS})
    if(IS_DIRECTORY "${MICRO_ROS_PATH}/include/${pkg}/${pkg}")
        list(APPEND MICRO_ROS_INCLUDE_DIRS "${MICRO_ROS_PATH}/include/${pkg}")
    endif()
endforeach()
target_include_directories(micro_ros PUBLIC ${MICRO_ROS_INCLUDE_DIRS})

# Add the standard library to the build
target_link_libraries(micro_ros PUBLIC pico_stdlib microros)

link_directories(${MICRO_ROS_PATH})
