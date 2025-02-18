#include <slam_to_mesh/options.hpp>
#include <slam_to_mesh/sort.hpp>
#include <slam_to_mesh/io/poses.hpp>
#include <slam_to_mesh/io/scans.hpp>
#include <slam_to_mesh/algorithm.hpp>

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

    lvr2::logout::get() << lvr2::info << "Reading poses from " << options.get_poses_path() << lvr2::endl;

    std::vector<Eigen::Isometry3f> poses = read_poses(options.get_poses_path(), options.get_poses_filetype_hint());

    lvr2::logout::get() << lvr2::info << "Got " << poses.size() << " poses" << lvr2::endl;

    // Read the pointclouds
    lvr2::logout::get() << lvr2::info << "Reading scans from " << options.get_scans_path() << lvr2::endl;

    std::vector<lvr2::PointBufferPtr> scans = read_scans(options.get_scans_path());

    lvr2::logout::get() << lvr2::info << "Got " << scans.size() << " scans" << lvr2::endl;

    if (scans.size() != poses.size())
    {
        lvr2::logout::get() << lvr2::warning << "Poses and scans differ in size! " << poses.size() << " vs " << scans.size();
        lvr2::logout::get() << "This can have unintended side effects if the poses and scans do not belong together!" << lvr2::endl;
        const size_t min = std::min(poses.size(), scans.size());
        lvr2::logout::get() << lvr2::warning << "Resizing to " << min << " poses and scans!" << lvr2::endl;
        poses.resize(min);
        scans.resize(min);
    }

    if (!options.output_directory().empty() && !fs::create_directories(options.output_directory() / "pcd"))
    {
        lvr2::logout::get() << lvr2::error << "Could not create output directory: " << std::strerror(errno) << lvr2::endl;
    }

    // Merge pointcloud
    auto combined = combine_pointclouds(poses, scans);

    // Estimate normals
    estimate_pointcloud_normals(poses, combined, options);

    // Reconstruct mesh
    auto mesh = reconstruct_mesh(combined, options);
    // Save the mesh
    {
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
        write_scans(options.output_directory() / "pcd", scans, "pcd");
    }

    if (options.save_poses())
    {
        lvr2::logout::get() << lvr2::info << "Saving poses to " << options.poses_output_file() << " (Format: " << options.poses_output_format() << ")" << lvr2::endl;
        write_poses(
            options.poses_output_file(),
            poses,
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
