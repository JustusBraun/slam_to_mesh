#include <slam_to_mesh/options.hpp>
#include <slam_to_mesh/sort.hpp>
#include <slam_to_mesh/io/poses.hpp>
#include <slam_to_mesh/io/scans.hpp>
#include <slam_to_mesh/algorithm.hpp>
#include <slam_to_mesh/logging.hpp>

#include <lvr2/util/Logging.hpp>
#include <lvr2/util/TransformUtils.hpp>
#include <lvr2/types/Model.hpp>
#include <lvr2/io/ModelFactory.hpp>
#include <lvr2/algorithm/FinalizeAlgorithms.hpp>

#include <filesystem>

#include <Eigen/Geometry>

namespace fs = std::filesystem;

int main(int argc, char** argv)
{
    Options options;
    if (!options.parse_arguments(argc, argv))
    {
        return -1;
    }

    lvr2::logout::get().setLogLevel(lvr2::LogLevel::info);

    LOG_INFO("Reading poses from {}", options.get_poses_path());

    std::vector<Eigen::Isometry3f> poses = read_poses(options.get_poses_path(), options.get_poses_filetype_hint());

    LOG_INFO("Got {} poses", poses.size());

    // Read the pointclouds
    LOG_INFO("Reading scans from {}", options.get_scans_path());

    std::vector<lvr2::PointBufferPtr> scans = read_scans(options.get_scans_path());

    LOG_INFO("Got {} scans", scans.size());

    if (scans.size() != poses.size())
    {
        LOG_WARNING("Poses and scans differ in size! {} vs {}", poses.size(), scans.size());
        LOG_WARNING("This can have unintended side effects if the poses and scans do not belong together!");
        const size_t min = std::min(poses.size(), scans.size());
        LOG_WARNING("Resizing to {} poses and scans!", min);
        poses.resize(min);
        scans.resize(min);
    }

    Dataset dataset;
    dataset.poses = poses;
    dataset.scans = scans;

    if (options.get_processing_range())
    {
        auto range = options.get_processing_range().value();

        if (range.first >= range.second)
        {
            LOG_ERROR("Specified range [{}, {}) is invalid!", range.first, range.second);
            return -1;
        }

        if (range.second > dataset.scans.size())
        {
            LOG_ERROR(
                "Specified range end '{}' cannot be larger than the number of scans & poses! ({})",
                range.second, dataset.scans.size()
            );
            return -1;
        }

        range.second = std::min(dataset.poses.size(), range.second);

        dataset.poses.erase(dataset.poses.begin(), dataset.poses.begin() + range.first);
        dataset.scans.erase(dataset.scans.begin(), dataset.scans.begin() + range.first);
        
        dataset.poses.resize(range.second - range.first);
        dataset.scans.resize(range.second - range.first);

        LOG_INFO("Using dataset range [{}, {})", range.first, range.second);
    }
    
    if (!options.output_directory().empty())
    {
        std::error_code ec;
        fs::create_directories(options.output_directory() / "pcd", ec);

        if (ec)
        {
            LOG_ERROR("Could not create output directory: {}", ec.message());
            return -1;
        }
    }

    // Remove all nan points from the scans
    for (auto& ptr: dataset.scans)
    {
        ptr = remove_nan(ptr);
    }

    // Merge pointcloud
    auto combined = combine_pointclouds(dataset);

    LOG_INFO("Build combined pointcloud");

    // Estimate normals
    estimate_pointcloud_normals(dataset.poses, combined, options);
    if (!combined->hasNormals())
    {
        LOG_ERROR("Failed to estimate point normals!");
        return -1;
    }
    LOG_INFO("Estimated point normals");

    // Reconstruct mesh
    auto mesh = reconstruct_mesh(combined, options);
    // Save the mesh
    {
        LOG_INFO("Num vertices: {}", mesh->numVertices());
        LOG_INFO("Num faces: {}", mesh->numFaces());
        fs::path mesh_file = options.output_directory();
        mesh_file = mesh_file.empty() ? "mesh.ply" : mesh_file / "mesh.ply";
        lvr2::SimpleFinalizer<lvr2::BaseVector<float>> fin;
        auto buffer = fin.apply(*mesh);
        lvr2::ModelFactory io;
        io.saveModel(std::make_shared<lvr2::Model>(buffer), mesh_file);
    }


    // Write hba format
    if (!options.output_directory().empty())
    {
        write_scans(options.output_directory() / "pcd", dataset.scans, "pcd");
    }

    if (options.save_poses())
    {
        LOG_INFO("Saving poses to {} (Format: {})", options.poses_output_file(), options.poses_output_format());
        write_poses(
            options.poses_output_file(),
            dataset.poses,
            options.poses_output_format()
        );
    }

    if (options.save_combined_points())
    {
        fs::path file;
        if (options.output_directory().empty())
        {
            file = "combined_points.ply";
        }
        else
        {
            file = options.output_directory() / "combined_points.ply";
        }
        lvr2::ModelFactory io;
        io.saveModel(std::make_shared<lvr2::Model>(combined), file);
    }

    return 0;
}
