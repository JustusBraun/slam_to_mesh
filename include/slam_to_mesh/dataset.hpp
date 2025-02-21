#pragma once

#include <vector>

#include <Eigen/Geometry>

#include <lvr2/types/PointBuffer.hpp>


struct Dataset
{
    using SharedPtr = typename std::shared_ptr<Dataset>;
    using ConstPtr = typename std::shared_ptr<const Dataset>;

    std::vector<Eigen::Isometry3f> poses;
    std::vector<lvr2::PointBufferPtr> scans;
};
