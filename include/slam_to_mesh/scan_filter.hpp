#pragma once

#include <optional>
#include <Eigen/Geometry>
#include "slam_to_mesh/options.hpp"

class ScanFilter {
public:
    ScanFilter(
        std::optional<std::pair<size_t, size_t>> range,
        std::optional<ScanSelectionSettings> selection);

    bool operator()(size_t index, const Eigen::Isometry3f& pose);

    bool is_valid(size_t max_count) const;

    void reset();

private:
    bool in_range(size_t index) const;
    bool passes_displacement(const Eigen::Isometry3f& pose);

    std::optional<std::pair<size_t, size_t>> range_;
    std::optional<ScanSelectionSettings> selection_;
    std::optional<Eigen::Isometry3f> last_kept_pose_;
};
