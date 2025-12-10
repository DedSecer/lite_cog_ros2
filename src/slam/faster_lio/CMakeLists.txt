cmake_minimum_required(VERSION 3.5)
project(faster_lio)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(tf2_ros)
find_package(faster_lio_interface)
find_package(PCL)
find_package(nav_msgs)
find_package(sensor_msgs)

find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)

include_directories(
    include
)

#definitions
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# options
option(WITH_IVOX_NODE_TYPE_PHC "Use PHC instead of default ivox node" OFF)

if (WITH_IVOX_NODE_TYPE_PHC)
    message("USING_IVOX_NODE_TYPE_PHC")
    add_definitions(-DIVOX_NODE_TYPE_PHC)
else ()
    message("USING_IVOX_NODE_TYPE_DEFAULT")
endif()

add_library(${PROJECT_NAME}
    src/laser_mapping.cc
    src/imu_processing.cc
    src/pointcloud_preprocess.cc
    src/options.cc
    src/utils.cc
)
ament_target_dependencies(${PROJECT_NAME} tf2_ros faster_lio_interface sensor_msgs nav_msgs)

target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
    tbb
    glog
    yaml-cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})


add_executable(run_mapping_online 
    app/run_mapping_online.cc
)
ament_target_dependencies(run_mapping_online nav_msgs sensor_msgs)
target_link_libraries(run_mapping_online
    ${PROJECT_NAME} gflags
)

# add_executable(run_mapping_offline app/run_mapping_offline.cc)
# target_link_libraries(run_mapping_offline
#     ${PROJECT_NAME} gflags
# )

install(TARGETS
    ${PROJECT_NAME}
    run_mapping_online
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()