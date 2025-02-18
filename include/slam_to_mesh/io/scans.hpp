#pragma once
#include <filesystem>
#include <vector>
#include <lvr2/types/PointBuffer.hpp>

/**
 *  @brief Read a set of scans/point clouds from the given path
 */
std::vector<lvr2::PointBufferPtr> read_scans(const std::filesystem::path& path);

/**
 *  @brief Write a set of scans/point clouds to the given directory/file
 */
void write_scans(const std::filesystem::path& path, const std::vector<lvr2::PointBufferPtr>& scans, const std::string& format);
