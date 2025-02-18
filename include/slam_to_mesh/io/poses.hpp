#pragma once
#include <filesystem>
#include <vector>
#include <Eigen/Geometry>


/**
 *  @brief write a poses array to a file using the specified format
 *
 *  Supported formats are Kitti, TUM and HBA
 */
void write_poses(const std::filesystem::path& file, const std::vector<Eigen::Isometry3f>& poses, const std::string& format, const bool overwrite = false);

/**
 *  @brief read a poses array from a file using the specified format
 *
 *  Supported formats are Kitti, TUM and HBA
 *  If no format provided we try to infer it from the data
 *
 *  @param file The file to read from
 *  @param format The format of the file
 *  @return A vector of poses
 */
std::vector<Eigen::Isometry3f> read_poses(const std::filesystem::path& file, const std::string& format);
