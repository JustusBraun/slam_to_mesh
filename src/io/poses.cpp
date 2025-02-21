#include <slam_to_mesh/io/poses.hpp>
#include <slam_to_mesh/logging.hpp>
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
        LOG_ERROR("The file {} already exists!", file);
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
                    LOG_ERROR("Failed to parse kitti format: not enough entries per line {}/12", row * 4 + col);
                    out.clear();
                    return false;
                }
                stream >> pose.matrix()(row, col);
            }
        }
        
        if(!stream.eof())
        {
            LOG_ERROR("Failed to parse kitti format: too much data per line!");
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
                LOG_ERROR("Failed to parse hba format: not enough entries per line {}/7", i);
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
            LOG_ERROR("Failed to parse hba format: too much data per line!");
            out.clear();
            return false;
        }
        out.push_back(pose);
    }
    return true;
}


bool read_tum(
    const fs::path& file,
    std::vector<Eigen::Isometry3f>& out
)
{
    // The tum file format contains 8 numbers per line
    // time tx ty tz x y z w
    std::ifstream in;
    in.open(file);

    std::string line;
    while(std::getline(in, line))
    {
        boost::trim(line);
        std::stringstream stream(line);
        Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();

        std::array<float, 8> data;

        for (int i = 0; i < 8; i++)
        {
            if (!stream)
            {
                LOG_ERROR("Failed to parse tum format: not enough entries per line {}/8", i);
                out.clear();
                return false;
            }
            stream >> data[i];
        }
        
        Eigen::Vector3f position(data[1], data[2], data[3]);
        Eigen::Quaternionf orientation(data[7], data[4], data[5], data[6]);
        pose.fromPositionOrientationScale(position, orientation, Eigen::Vector3f::Ones());
        
        if(!stream.eof())
        {
            LOG_ERROR("Failed to parse tum format: too much data per line!");
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
        LOG_ERROR("The file {} does not exist!", file);
        return {};
    }

    std::vector<Eigen::Isometry3f> result;

    if ("tum" == format)
    {
        if (read_tum(file, result))
        {
            return result;
        }
        else
        {
            std::stringstream sstr;
            sstr << "Failed to parse " << file << " as format 'tum'";
            throw std::runtime_error(sstr.str());
        }
    }
    else if ("kitti" == format)
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
