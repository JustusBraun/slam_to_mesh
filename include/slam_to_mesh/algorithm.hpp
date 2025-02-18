#pragma once

#include <slam_to_mesh/options.hpp>

#include <lvr2/types/PointBuffer.hpp>
#include <lvr2/geometry/BaseMesh.hpp>
#include <lvr2/geometry/BaseVector.hpp>


void deskew_scans(
    const std::vector<Eigen::Isometry3f>& poses,
    const std::vector<lvr2::PointBufferPtr>& scans
);


/**
 *  @brief Create a combined pointcloud from the dataset
 */
lvr2::PointBufferPtr combine_pointclouds(
    const std::vector<Eigen::Isometry3f>& poses,
    const std::vector<lvr2::PointBufferPtr>& scans
);


/**
 *  @brief Estimate the point normals using the poses
 */
void estimate_pointcloud_normals(
    const std::vector<Eigen::Isometry3f>& poses,
    lvr2::PointBufferPtr& points,
    const Options& opts
);


/**
 *  @brief Reconstruct a mesh from the combined pointcloud
 *
 *  @param points The pointcloud to reconstruct a mesh from. Needs to have normals!
 *  @return The reconstructed mesh
 */
std::shared_ptr<lvr2::BaseMesh<lvr2::BaseVector<float>>> reconstruct_mesh(
    const lvr2::PointBufferPtr points,
    const Options& opts
);


/**
 *  @brief Remove all NaN points from the pointcloud
 *
 *  @param buffer The pointcloud to filter
 *  @return A new buffer without the nan points
 */
lvr2::PointBufferPtr remove_nan(const lvr2::PointBufferPtr& points);
