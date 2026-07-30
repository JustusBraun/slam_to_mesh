#include <slam_to_mesh/io/scans.hpp>
#include <slam_to_mesh/sort.hpp>
#include <slam_to_mesh/logging.hpp>

#include <lvr2/types/Model.hpp>
#include <lvr2/io/ModelFactory.hpp>
#include <lvr2/util/Logging.hpp>

namespace fs = std::filesystem;

std::vector<lvr2::PointBufferPtr> read_scans(const fs::path& path)
{
    if (!fs::is_directory(path))
    {
        lvr2::logout::get() << lvr2::error << "[load_scans()] - " << path << " is not a directory" << lvr2::endl;
        return std::vector<lvr2::PointBufferPtr>();
    }

    std::vector<fs::path> scan_files;
    for (const auto& file: fs::directory_iterator(path))
    {
        if (!file.is_regular_file())
        {
            continue;
        }
        scan_files.push_back(file);
    }

    std::sort(scan_files.begin(), scan_files.end(), natural_compare());
    
    // Load the scans in order
    std::vector<lvr2::PointBufferPtr> scans;
    for (const auto& file: scan_files)
    {
        lvr2::ModelFactory io;
        const auto model = io.readModel(file);

        if (!model || !model->m_pointCloud)
        {
            lvr2::logout::get() << lvr2::warning << "Failed to read scan from " << file << lvr2::endl;
        }

        scans.push_back(model->m_pointCloud);
    }

    return scans;
}


std::optional<Dataset> read_scans(
    const fs::path& path,
    const std::vector<Eigen::Isometry3f>& poses,
    ScanFilter& filter)
{
    Dataset dataset;

    if (!fs::is_directory(path))
    {
        LOG_ERROR("{} is not a directory", path);
        return std::nullopt;
    }

    std::vector<fs::path> scan_files;
    for (const auto& file: fs::directory_iterator(path))
    {
        if (!file.is_regular_file())
        {
            continue;
        }
        scan_files.push_back(file);
    }

    std::sort(scan_files.begin(), scan_files.end(), natural_compare());

    const size_t max_count = std::min(poses.size(), scan_files.size());
    if (poses.size() != scan_files.size())
    {
        LOG_WARNING("Poses and scans differ in size! {} vs {}", poses.size(), scan_files.size());
        LOG_WARNING("This can have unintended side effects if the poses and scans do not belong together!");
        LOG_WARNING("Resizing to {} poses and scans!", max_count);
    }

    if (!filter.is_valid(max_count))
    {
        LOG_ERROR("Specified scan range is invalid for the available number of scans!");
        return std::nullopt;
    }

    for (size_t i = 0; i < max_count; ++i)
    {
        if (filter(i, poses[i]))
        {
            lvr2::ModelFactory io;
            const auto model = io.readModel(scan_files[i]);

            if (!model || !model->m_pointCloud)
            {
                LOG_WARNING("Failed to read scan from {}", scan_files[i]);
            }

            dataset.scans.push_back(model->m_pointCloud);
            dataset.poses.push_back(poses[i]);
        }
    }

    return dataset;
}


void write_scans(
    const std::filesystem::path& path,
    const std::vector<lvr2::PointBufferPtr>& scans,
    const std::string& format
)
{
    if (!fs::is_directory(path))
    {
        std::error_code ec;
        fs::create_directories(path, ec);
        if (ec)
        {
            lvr2::logout::get() << lvr2::error << "Failed to create output directory " << path << ": " << ec.message() << lvr2::endl;
            return;
        }
    }
    
    uint32_t i = 0;
    lvr2::ModelFactory io;
    auto model = std::make_shared<lvr2::Model>();
    for (const auto& scan: scans)
    {
        std::stringstream sstr;
        sstr << i++ << "." << format;
        model->m_pointCloud = scan;
        io.saveModel(model, path / sstr.str());
    }
}
