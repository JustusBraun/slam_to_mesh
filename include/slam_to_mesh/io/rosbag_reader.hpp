#pragma once

#include <slam_to_mesh/dataset.hpp>
#include <filesystem>

#ifdef HAS_ROS2_BAG_SUPPORT

/**
 *  @brief Read a ROS2 bagfile and extract paired pointclouds and odometry poses.
 *
 *  @param bag_path Path to the bagfile (directory or file)
 *  @param pointcloud_topic Topic containing sensor_msgs/PointCloud2
 *  @param odometry_topic Topic containing nav_msgs/Odometry
 *  @return Dataset with paired poses and scans in timestamp order
 */
Dataset read_rosbag(
    const std::filesystem::path& bag_path,
    const std::string& pointcloud_topic,
    const std::string& odometry_topic
);

#endif
