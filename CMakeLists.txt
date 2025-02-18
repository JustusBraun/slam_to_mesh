cmake_minimum_required(VERSION 3.16)
project(slam_to_mesh)


# Main dependency is LVR2
find_package(LVR2 REQUIRED)
find_package(Eigen3 REQUIRED)

# Add the tool(s)
add_executable(
    slam_to_mesh
    src/slam_to_mesh.cpp
    src/options.cpp
    src/io/poses.cpp
    src/io/scans.cpp
    src/algorithm.cpp
)

target_include_directories(
    slam_to_mesh PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)
target_link_libraries(slam_to_mesh PRIVATE lvr2::lvr2 Eigen3::Eigen)
set_target_properties(
    slam_to_mesh
    PROPERTIES
    CMAKE_CXX_STANDARD 23
    CMAKE_CXX_STANDARD_REQUIRED ON
)

# Install the tool(s)
install(
    TARGETS slam_to_mesh
    DESTINATION bin
)
