#pragma once

#include <slam_to_mesh/dataset.hpp>
#include <slam_to_mesh/scan_filter.hpp>
#include <filesystem>

#ifdef HAS_ROS2_BAG_SUPPORT

/**
 *  @brief Read a ROS2 bagfile in a single pass, applying a filter to each matched scan-pose pair.
 *
 *  @param bag_path Path to the bagfile (directory or file)
 *  @param pointcloud_topic Topic containing sensor_msgs/PointCloud2
 *  @param odometry_topic Topic containing nav_msgs/Odometry
 *  @param filter Incremental filter applied to each matched pair.
 *  @return Dataset with filtered poses and scans, or std::nullopt if the filter range is invalid.
 */
std::optional<Dataset> read_rosbag(
    const std::filesystem::path& bag_path,
    const std::string& pointcloud_topic,
    const std::string& odometry_topic,
    ScanFilter& filter);

#endif
