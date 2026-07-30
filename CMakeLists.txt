cmake_minimum_required(VERSION 3.16)
project(slam_to_mesh VERSION 0.0.1)

# Build in Release mode if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# Main dependency is LVR2
find_package(LVR2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

# Optional ROS2 support for reading bag files
# Only search for ROS2 packages if the environment appears to be sourced
if(DEFINED ENV{AMENT_PREFIX_PATH} OR DEFINED ENV{ROS_DISTRO})
    find_package(ament_cmake_core QUIET)
    if(ament_cmake_core_FOUND)
        find_package(rosbag2_cpp QUIET)
        find_package(sensor_msgs QUIET)
        find_package(nav_msgs QUIET)
        find_package(rclcpp QUIET)
    endif()
endif()

set(SLAM_TO_MESH_SOURCES
    src/slam_to_mesh.cpp
    src/options.cpp
    src/io/poses.cpp
    src/io/scans.cpp
    src/algorithm.cpp
    src/scan_filter.cpp
)

if(rosbag2_cpp_FOUND AND sensor_msgs_FOUND AND nav_msgs_FOUND AND rclcpp_FOUND)
    list(APPEND SLAM_TO_MESH_SOURCES src/io/rosbag_reader.cpp)
    message(STATUS "ROS2 bag reading support enabled")
else()
    message(STATUS "ROS2 bag reading support disabled (missing dependencies)")
endif()

# Add the tool(s)
add_executable(
    slam_to_mesh
    ${SLAM_TO_MESH_SOURCES}
)

target_include_directories(
    slam_to_mesh PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)
target_link_libraries(slam_to_mesh PRIVATE lvr2::lvr2 Eigen3::Eigen)

if(rosbag2_cpp_FOUND AND sensor_msgs_FOUND AND nav_msgs_FOUND AND rclcpp_FOUND)
    target_compile_definitions(slam_to_mesh PRIVATE HAS_ROS2_BAG_SUPPORT)
    target_link_libraries(slam_to_mesh PRIVATE
        rosbag2_cpp::rosbag2_cpp
        rclcpp::rclcpp
        sensor_msgs::sensor_msgs_library
        nav_msgs::nav_msgs__rosidl_generator_cpp
        nav_msgs::nav_msgs__rosidl_typesupport_cpp
    )
endif()

# Optional dependencies
if(OpenMP_CXX_FOUND)
    target_link_libraries(slam_to_mesh PRIVATE OpenMP::OpenMP_CXX)
endif()


set_target_properties(
    slam_to_mesh
    PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
)

# Install the tool(s)
install(
    TARGETS slam_to_mesh
    DESTINATION bin
)
