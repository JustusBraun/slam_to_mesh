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
    ("poses", po::value<string>()->required(), "Path to a file containing a pose for each input scan. Supported formats are 'kitti', 'tum' and 'hba'")
    ("poses-filetype-hint", po::value<string>()->default_value("kitti"), "An optional hint on the format of the poses file. Otherwise the program tries to infer from the data")
    ("scans", po::value<string>()->required(), "Path to a directory containing the individual lidar scans")
    ("range", po::value<std::vector<size_t>>()->multitoken(),
     "Limits the range of scans & poses to process to [arg0, arg1)")
    ("displacement", po::value<std::vector<float>>()->multitoken(),
     "Select a subset of the input scans based on displacement (m) and orientation delta (rad). Example: '--displacement 0.3 1.0' includes a scan if it's pose has a displacement larger than or equal to 0.3m or an orientation delta larger than or equal to 1.0rad compared to the last included scan.")
    ("poses-output-file", po::value<string>(), "Save the poses to the specified file")
    ("poses-output-format", po::value<string>()->default_value("kitty"), "The format to save the poses in")
    ("output-directory", po::value<string>(), "The directory to output converted data to")
    ("save-combined-points", "Save the combined pointcloud to the output directory")
    ("disable-voxel-downsampling", "Do not downsample the combined pointcloud to a uniform cloud with a voxel resolution of 0.01 meter")
    ("disable-statistical-outlier-removal", "Do not apply statistical outlier removal to the combined pointcloud")
    ("sor-neighbors", po::value<size_t>(&sor_nn_)->default_value(50), "The number of neighbors to use in statistical outlier removal")
    ("sor-sigma-factor", po::value<float>(&sor_factor_)->default_value(2.0), "The factor to use in statistical outlier removal. Higher equals less strict, lower equals more agressive removal of points")
    ;

    normal_est_.add_options()
    ("recompute-normals", "Recompute the normals if the input scans already have them")
    ("kn", po::value<uint32_t>(&normal_kn_)->default_value(50), "Number of nearest neighbor points used in normal estimation")
    ("ki", po::value<uint32_t>(&normal_ki_)->default_value(50), "Number of nearest neighbor normals used in normal interpolation (smoothing)")
    ("normal-estimation-method", po::value<uint32_t>(&normal_estimation_method_)->default_value(3), "Normal estimation method to use. Choose from 0: PCA, 1: RANSAC, 2: IPCA ilikebigbits, 3: IPCA exact (default)")
    ;

    reconstruction_.add_options()
    ("voxel-size,v", po::value<float>(&voxel_size_)->default_value(0.25), "The resolution of the mesh in meter")
    ("kd", po::value<uint32_t>(&reconstruction_kd_)->default_value(50), "The number of nearest neighbors used in distance calculation")
    ("remove-isolated-cluster-threshold", po::value<uint32_t>(&rda_thresh_)->default_value(10), "Remove connected triangle clusters with less then arg triangles")
    ("fill-holes-threshold", po::value<uint32_t>(&fill_holes_thresh_)->default_value(10), "Fill holes with a perimeter of less than arg edges")
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

std::optional<std::pair<size_t, size_t>> Options::get_processing_range() const
{
    if (vars_.count("range"))
    {
        const auto range = vars_["range"].as<std::vector<size_t>>();
        return std::pair(range.at(0), range.at(1));
    }

    return std::nullopt;
}

std::optional<ScanSelectionSettings> Options::get_scan_selection_settings() const
{
    if (vars_.count("displacement"))
    {
        const auto args = vars_["displacement"].as<std::vector<float>>();
        return ScanSelectionSettings{.min_displacement = args[0], .min_rotation = args[1]};
    }

    return std::nullopt;
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

bool Options::disable_pcl_downsampling() const
{
    return vars_.count("disable-voxel-downsampling");
}

bool Options::disable_statistical_outlier_removal() const
{
    return vars_.count("disable-statistical-outlier-removal");
}

size_t Options::statistical_outlier_removal_neighbors() const
{
    return sor_nn_;
}

float Options::statistical_outlier_removal_factor() const
{
    return sor_factor_;
}

bool Options::recompute_normals() const
{
    return vars_.count("recompute-normals");
}

uint32_t Options::normal_estimation_kn() const
{
    return normal_kn_;
}

uint32_t Options::normal_estimation_ki() const
{
    return normal_ki_;
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

uint32_t Options::fill_holes_threshold() const
{
    return fill_holes_thresh_;
}
