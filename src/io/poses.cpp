#include <slam_to_mesh/io/poses.hpp>
#include <lvr2/util/Logging.hpp>
#include <cctype>
#include <fstream>
#include <boost/algorithm/string.hpp>

namespace fs = std::filesystem;

/**
 *  @brief write poses to the HBA format
 */
void write_hba(const std::filesystem::path& file, const std::vector<Eigen::Isometry3f>& poses)
{
    std::ofstream out;
    out.open(file);

    if (out.fail())
    {
        std::stringstream sstr;
        sstr << "Could not open file '" << file << "' for writing: " << std::strerror(errno);
        throw std::runtime_error(sstr.str());
    }

    for (const auto& pose: poses)
    {
        const Eigen::Vector3f trans = pose.translation();
        out << trans.x() << " " << trans.y() << " " << trans.z() << " ";
        const Eigen::Quaternionf ori(pose.rotation());
        out << ori.w() << " " << ori.x() << " " << ori.y() << " " << ori.z() << '\n';
    }
}


void write_poses(
    const std::filesystem::path& file,
    const std::vector<Eigen::Isometry3f>& poses,
    const std::string& format,
    const bool overwrite
)
{
    if (fs::exists(file) && !overwrite)
    {
        lvr2::logout::get() << lvr2::error << "File " << file << " already exists!" << lvr2::endl;
        return;
    }

    if ("tum" == format)
    {

    }
    else if ("kitti" == format)
    {

    }
    else if("hba" == format)
    {
        write_hba(file, poses);
    }
    else
    {
        std::stringstream sstr;
        sstr << "Unsupported pose file format: '" << format << "'";
        throw std::invalid_argument(sstr.str());
    }
}


bool read_kitty(
    const fs::path& file,
    std::vector<Eigen::Isometry3f>& out
)
{
    // The Kitti file format contains 12 numbers per line
    // each line is a 4x3 matrix representing the pose
    std::ifstream in;
    in.open(file);

    std::string line;
    while(std::getline(in, line))
    {
        boost::trim(line);
        std::stringstream stream(line);
        Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                if (!stream)
                {
                    lvr2::logout::get() << lvr2::error << "Failed to parse kitti format: not enough entries per line " << (row * 4 + col) << "/12" << lvr2::endl;
                    out.clear();
                    return false;
                }
                stream >> pose.matrix()(row, col);
            }
        }
        
        if(!stream.eof())
        {
            lvr2::logout::get() << lvr2::error << "Failed to parse kitti format: too much data per line" << lvr2::endl;
            out.clear();
            return false;
        }
        out.push_back(pose);
    }
    return true;
}


bool read_hba(
    const fs::path& file,
    std::vector<Eigen::Isometry3f>& out
)
{
    // The hba file format contains 7 numbers per line
    // tx ty tz w x y z
    std::ifstream in;
    in.open(file);

    std::string line;
    while(std::getline(in, line))
    {
        boost::trim(line);
        std::stringstream stream(line);
        Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();

        std::array<float, 7> data;

        for (int i = 0; i < 7; i++)
        {
            if (!stream)
            {
                lvr2::logout::get() << lvr2::error << "Failed to parse hba format: not enough entries per line " << i << "/7" << lvr2::endl;
                out.clear();
                return false;
            }
            stream >> data[i];
        }
        
        Eigen::Vector3f position(data[0], data[1], data[2]);
        Eigen::Quaternionf orientation(data[3], data[4], data[5], data[6]);
        pose.fromPositionOrientationScale(position, orientation, Eigen::Vector3f::Ones());
        
        if(!stream.eof())
        {
            lvr2::logout::get() << lvr2::error << "Failed to parse kitti format: too much data per line" << lvr2::endl;
            out.clear();
            return false;
        }
        out.push_back(pose);
    }
    return true;
}


std::vector<Eigen::Isometry3f> read_poses(
    const std::filesystem::path& file,
    const std::string& format
)
{
    if (!fs::exists(file))
    {
        lvr2::logout::get() << lvr2::error << "File " << file << " does not exist!" << lvr2::endl;
        return {};
    }

    std::vector<Eigen::Isometry3f> result;

    // if ("tum" == format)
    // {
    //
    // }
    // else if ("kitti" == format)
    if ("kitti" == format)
    {
        if (read_kitty(file, result))
        {
            return result;
        }
        else
        {
            std::stringstream sstr;
            sstr << "Failed to parse " << file << " as format 'kitty'";
            throw std::runtime_error(sstr.str());
        }
    }
    else if("hba" == format)
    {
        if (read_hba(file, result))
        {
            return result;
        }
        else
        {
            std::stringstream sstr;
            sstr << "Failed to parse " << file << " as format 'hba'";
            throw std::runtime_error(sstr.str());
        }
    }
    else
    {
        std::stringstream sstr;
        sstr << "Unsupported pose file format: '" << format << "'";
        throw std::invalid_argument(sstr.str());
    }
    return std::vector<Eigen::Isometry3f>();
}
