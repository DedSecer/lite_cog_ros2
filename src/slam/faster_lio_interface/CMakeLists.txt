# Copyright(c) 2019 livoxtech limited.
cmake_minimum_required(VERSION 3.5)

# we only need livox message definition here
project(faster_lio_interface)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/CustomPoint.msg"
    "msg/CustomMsg.msg"
    "msg/Pose6d.msg"    
)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif ()

ament_package()