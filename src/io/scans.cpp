#include <slam_to_mesh/io/scans.hpp>
#include <slam_to_mesh/sort.hpp>

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
