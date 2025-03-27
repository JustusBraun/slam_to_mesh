#pragma once

#include <slam_to_mesh/dataset.hpp>
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
    const Dataset& dataset
);


/**
 *  @brief Estimate the point normals using the poses
 */
void estimate_pointcloud_normals(
    const std::vector<Eigen::Isometry3f>& poses,
    const lvr2::PointBufferPtr& points,
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


/**
 *  @brief Use voxel downsampling to reduce the pointcloud to a consistent density
 *
 */
lvr2::PointBufferPtr voxel_downsample(const lvr2::PointBufferPtr& points, const float voxel_size);


/**
 *  @brief Remove outliers using statistical outlier removal
 *
 *  This algorithm calculates the average distance and the standard deviation from each point to its neighbors.
 *  Then all points with an average distance larger than mean + stddev * factor are removed.
 *
 *  @param points The pointcloud to filter
 *  @param neighbors The number of nearest neighbors used in the distance calcualtion
 *  @param factor The factor to use in the outlier removal calculation
 *
 *  @return The filtered pointcloud
 *
 */
lvr2::PointBufferPtr statistical_outlier_removal(const lvr2::PointBufferPtr& points, const size_t neighbors, const float factor);
