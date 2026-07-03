#include <slam_to_mesh/scan_filter.hpp>
#include <slam_to_mesh/logging.hpp>

ScanFilter::ScanFilter(
    std::optional<std::pair<size_t, size_t>> range,
    std::optional<ScanSelectionSettings> selection)
: range_(std::move(range))
, selection_(std::move(selection))
{
}

bool ScanFilter::operator()(size_t index, const Eigen::Isometry3f& pose)
{
    if (!in_range(index))
    {
        return false;
    }

    if (!passes_displacement(pose))
    {
        return false;
    }

    return true;
}

bool ScanFilter::is_valid(size_t max_count) const
{
    if (!range_)
    {
        return true;
    }

    if (range_->first >= range_->second)
    {
        return false;
    }

    if (range_->second > max_count)
    {
        return false;
    }

    return true;
}

void ScanFilter::reset()
{
    last_kept_pose_.reset();
}

bool ScanFilter::in_range(size_t index) const
{
    if (!range_)
    {
        return true;
    }

    return index >= range_->first && index < range_->second;
}

bool ScanFilter::passes_displacement(const Eigen::Isometry3f& pose)
{
    if (!selection_)
    {
        return true;
    }

    if (!last_kept_pose_)
    {
        last_kept_pose_ = pose;
        return true;
    }

    const float dist = (pose.translation() - last_kept_pose_->translation()).norm();
    const float rot = Eigen::Quaternionf(pose.rotation()).angularDistance(Eigen::Quaternionf(last_kept_pose_->rotation()));

    if (dist >= selection_->min_displacement || rot >= selection_->min_rotation)
    {
        last_kept_pose_ = pose;
        return true;
    }

    return false;
}
