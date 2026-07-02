#include <slam_to_mesh/io/rosbag_reader.hpp>

#ifdef HAS_ROS2_BAG_SUPPORT

#include <slam_to_mesh/logging.hpp>

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <lvr2/types/PointBuffer.hpp>

#include <Eigen/Geometry>

#include <limits>

namespace fs = std::filesystem;

static int64_t ros_time_to_ns(const builtin_interfaces::msg::Time& t)
{
    return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
}

static lvr2::PointBufferPtr pointcloud2_to_buffer(const sensor_msgs::msg::PointCloud2& msg)
{
    size_t x_offset = 0;
    size_t y_offset = 0;
    size_t z_offset = 0;
    bool has_x = false;
    bool has_y = false;
    bool has_z = false;
    uint8_t x_datatype = 0;
    uint8_t y_datatype = 0;
    uint8_t z_datatype = 0;

    for (const auto& field : msg.fields)
    {
        if (field.name == "x")
        {
            x_offset = field.offset;
            x_datatype = field.datatype;
            has_x = true;
        }
        else if (field.name == "y")
        {
            y_offset = field.offset;
            y_datatype = field.datatype;
            has_y = true;
        }
        else if (field.name == "z")
        {
            z_offset = field.offset;
            z_datatype = field.datatype;
            has_z = true;
        }
    }

    if (!has_x || !has_y || !has_z)
    {
        LOG_ERROR("PointCloud2 message is missing x, y, or z field");
        return nullptr;
    }

    if (msg.is_bigendian)
    {
        LOG_ERROR("Big-endian PointCloud2 messages are not supported");
        return nullptr;
    }

    const size_t n_points = static_cast<size_t>(msg.height) * static_cast<size_t>(msg.width);
    std::vector<float> points;
    points.reserve(n_points * 3);

    auto read_float = [&](const uint8_t* ptr, uint8_t datatype) -> float
    {
        if (datatype == sensor_msgs::msg::PointField::FLOAT32)
        {
            return *reinterpret_cast<const float*>(ptr);
        }
        else if (datatype == sensor_msgs::msg::PointField::FLOAT64)
        {
            return static_cast<float>(*reinterpret_cast<const double*>(ptr));
        }
        else
        {
            LOG_WARNING("Unsupported PointCloud2 field datatype: {}", datatype);
            return std::numeric_limits<float>::quiet_NaN();
        }
    };

    for (size_t i = 0; i < n_points; ++i)
    {
        const uint8_t* point_base = msg.data.data() + i * msg.point_step;
        float x = read_float(point_base + x_offset, x_datatype);
        float y = read_float(point_base + y_offset, y_datatype);
        float z = read_float(point_base + z_offset, z_datatype);

        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
        {
            points.push_back(x);
            points.push_back(y);
            points.push_back(z);
        }
    }

    const size_t n_valid = points.size() / 3;
    auto float_array = lvr2::floatArr(new float[points.size()]);
    std::copy(points.begin(), points.end(), float_array.get());

    auto buffer = std::make_shared<lvr2::PointBuffer>();
    buffer->setPointArray(float_array, n_valid);
    return buffer;
}

static Eigen::Isometry3f odometry_to_pose(const nav_msgs::msg::Odometry& msg)
{
    Eigen::Vector3f position(
        -static_cast<float>(msg.pose.pose.position.x),
        -static_cast<float>(msg.pose.pose.position.y),
        -static_cast<float>(msg.pose.pose.position.z)
    );

    Eigen::Quaternionf orientation(
        static_cast<float>(msg.pose.pose.orientation.w),
        static_cast<float>(msg.pose.pose.orientation.x),
        static_cast<float>(msg.pose.pose.orientation.y),
        static_cast<float>(msg.pose.pose.orientation.z)
    );

    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.fromPositionOrientationScale(position, orientation, Eigen::Vector3f::Ones());
    return pose;
}

Dataset read_rosbag(
    const fs::path& bag_path,
    const std::string& pointcloud_topic,
    const std::string& odometry_topic
)
{
    Dataset dataset;

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path.string();
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    rosbag2_cpp::readers::SequentialReader reader;
    reader.open(storage_options, converter_options);

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_serialization;
    rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serialization;

    std::vector<std::pair<int64_t, lvr2::PointBufferPtr>> scans;
    std::vector<std::pair<int64_t, Eigen::Isometry3f>> poses;

    while (reader.has_next())
    {
        auto msg = reader.read_next();
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

        if (msg->topic_name == pointcloud_topic)
        {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pc_serialization.deserialize_message(&serialized_msg, &cloud_msg);
            auto buffer = pointcloud2_to_buffer(cloud_msg);
            if (buffer)
            {
                int64_t ts = ros_time_to_ns(cloud_msg.header.stamp);
                scans.emplace_back(ts, std::move(buffer));
            }
        }
        else if (msg->topic_name == odometry_topic)
        {
            nav_msgs::msg::Odometry odom_msg;
            odom_serialization.deserialize_message(&serialized_msg, &odom_msg);
            int64_t ts = ros_time_to_ns(odom_msg.header.stamp);
            poses.emplace_back(ts, odometry_to_pose(odom_msg));
        }
    }

    reader.close();

    LOG_INFO("Read {} pointclouds and {} odometry poses from bag", scans.size(), poses.size());

    size_t i = 0;
    size_t j = 0;
    while (i < scans.size() && j < poses.size())
    {
        if (scans[i].first == poses[j].first)
        {
            dataset.scans.push_back(std::move(scans[i].second));
            dataset.poses.push_back(std::move(poses[j].second));
            ++i;
            ++j;
        }
        else if (scans[i].first < poses[j].first)
        {
            LOG_WARNING("No matching odometry for pointcloud at t={}", scans[i].first);
            ++i;
        }
        else
        {
            LOG_WARNING("No matching pointcloud for odometry at t={}", poses[j].first);
            ++j;
        }
    }

    LOG_INFO("Matched {} paired scans and poses", dataset.scans.size());

    return dataset;
}

#endif
