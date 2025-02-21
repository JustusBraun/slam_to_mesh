#pragma once

#include <boost/program_options.hpp>
#include <filesystem>

class Options
: private boost::program_options::options_description
{
public:
    
    Options();
    
    /**
    *   @brief Parse the arguments into the options object
    *
    *   @return true if successful false otherwise
    */
    bool parse_arguments(int argc, char** argv);

    std::filesystem::path get_poses_path() const;

    std::string get_poses_filetype_hint() const;

    std::filesystem::path get_scans_path() const;

    std::optional<std::pair<size_t, size_t>> get_processing_range() const;

    bool save_poses() const;

    std::filesystem::path poses_output_file() const;

    std::string poses_output_format() const;

    std::filesystem::path output_directory() const;

    bool save_combined_points() const;

    /// Normal estimation

    uint32_t normal_estimation_kn() const;

    uint32_t normal_estimation_method() const;

    /// Reconstruction
    
    float voxel_size() const;

    uint32_t reconstruction_kd() const;

    uint32_t rda_threshold() const;

private:
    boost::program_options::variables_map vars_;

    boost::program_options::options_description normal_est_;

    boost::program_options::options_description reconstruction_;

    uint32_t normal_kn_;
    uint32_t normal_estimation_method_;
    
    float voxel_size_;
    uint32_t reconstruction_kd_;
    uint32_t rda_thresh_;
};
