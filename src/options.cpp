#include <slam_to_mesh/options.hpp>
#include <iostream>
#include <cstdint>

namespace po = boost::program_options;
namespace fs = std::filesystem;
using std::string;

Options::Options()
: po::options_description("slam_to_mesh options")
, normal_est_("Normal estimation")
, reconstruction_("Reconstruction")
{
    add_options()
    ("help", "output this message")
    ("poses", po::value<string>()->required(), "Path to a file containing a pose for each input scan. Supported formats are 'kitti' and 'tum'")
    ("poses-filetype-hint", po::value<string>()->default_value("kitti"), "An optional hint on the format of the poses file. Otherwise the program tries to infer from the data")
    ("scans", po::value<string>()->required(), "Path to a directory containing the individual lidar scans")
    ("poses-output-file", po::value<string>(), "Save the poses to the specified file")
    ("poses-output-format", po::value<string>()->default_value("kitty"), "The format to save the poses in")
    ("output-directory", po::value<string>(), "The directory to output converted data to")
    ("save-combined-points", "Save the combined pointcloud to the output directory")
    ;

    normal_est_.add_options()
    ("kn", po::value<uint32_t>(&normal_kn_)->default_value(50), "Number of nearest neighbor points used in normal estimation")
    ("normal-estimation-method", po::value<uint32_t>(&normal_estimation_method_)->default_value(0), "Normal estimation method to use. Choose from 0: PCA (default), 1: RANSAC, 2: IPCA ilikebigbits, 3: IPCA exact")
    ;

    reconstruction_.add_options()
    ("voxel-size,v", po::value<float>(&voxel_size_)->default_value(0.25), "The resolution of the mesh in meter")
    ("kd", po::value<uint32_t>(&reconstruction_kd_)->default_value(50), "The number of nearest neighbors used in distance calculation")
    ("remove-isolated-cluster-threshold", po::value<uint32_t>(&rda_thresh_)->default_value(10), "Remove connected triangle clusters with less then arg triangles")
    ;

    this->add(normal_est_);
    this->add(reconstruction_);
}

bool Options::parse_arguments(int argc, char** argv)
{
    po::store(po::parse_command_line(argc, argv, *this), vars_);

    if (vars_.count("help"))
    {
        std::cout << *this << std::endl;
        return false;
    }
    
    try
    {
        po::notify(vars_);
    }
    catch(std::exception const& ex)
    {
        std::cout << "Error parsing arguments: " << ex.what() << std::endl;
        std::cout << *this << std::endl;
        return false;
    }

    return true;
}

fs::path Options::get_poses_path() const
{
    return fs::path(vars_["poses"].as<std::string>());
}

std::string Options::get_poses_filetype_hint() const
{
    return vars_["poses-filetype-hint"].as<std::string>();
}

fs::path Options::get_scans_path() const
{
    return fs::path(vars_["scans"].as<std::string>());
}

bool Options::save_poses() const
{
    return vars_.count("poses-output-file");
}

fs::path Options::poses_output_file() const
{
    return vars_["poses-output-file"].as<std::string>();
}

std::string Options::poses_output_format() const
{
    return vars_["poses-output-format"].as<std::string>();
}

fs::path Options::output_directory() const
{
    if (vars_.count("output-directory"))
    {
        return vars_["output-directory"].as<std::string>();
    }
    return fs::path();
}

bool Options::save_combined_points() const
{
    return vars_.count("save-combined-points");
}

uint32_t Options::normal_estimation_kn() const
{
    return normal_kn_;
}

uint32_t Options::normal_estimation_method() const
{
    return normal_estimation_method_;
}

float Options::voxel_size() const
{
    return voxel_size_;
}

uint32_t Options::reconstruction_kd() const
{
    return reconstruction_kd_;
}

uint32_t Options::rda_threshold() const
{
    return rda_thresh_;
}
