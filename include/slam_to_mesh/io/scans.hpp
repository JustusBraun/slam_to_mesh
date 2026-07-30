#pragma once
#include <filesystem>
#include <optional>
#include <vector>
#include <lvr2/types/PointBuffer.hpp>

#include <slam_to_mesh/dataset.hpp>
#include <slam_to_mesh/scan_filter.hpp>

/**
 *  @brief Read a set of scans/point clouds from the given path
 */
std::vector<lvr2::PointBufferPtr> read_scans(const std::filesystem::path& path);

/**
 *  @brief Read only the scans accepted by the filter from the given path.
 *
 *  @param path   Directory containing the individual scans.
 *  @param poses  Poses corresponding to the scans in natural-sorted order.
 *  @param filter Filter to apply to each pose/scan pair.
 *  @return Dataset with filtered poses and scans, or std::nullopt if the filter range is invalid.
 */
std::optional<Dataset> read_scans(
    const std::filesystem::path& path,
    const std::vector<Eigen::Isometry3f>& poses,
    ScanFilter& filter);

/**
 *  @brief Write a set of scans/point clouds to the given directory/file
 */
void write_scans(const std::filesystem::path& path, const std::vector<lvr2::PointBufferPtr>& scans, const std::string& format);
