#include <slam_to_mesh/algorithm.hpp>

#include <boost/smart_ptr/shared_array.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <lvr2/algorithm/CleanupAlgorithms.hpp>
#include <lvr2/reconstruction/AdaptiveKSearchSurface.hpp>
#include <lvr2/reconstruction/FastReconstruction.hpp>
#include <lvr2/reconstruction/FastBox.hpp>
#include <lvr2/reconstruction/PointsetGrid.hpp>
#include <lvr2/geometry/PMPMesh.hpp>

lvr2::PointBufferPtr combine_pointclouds(
    const std::vector<Eigen::Isometry3f>& poses,
    const std::vector<lvr2::PointBufferPtr>& scans
)
{
    // Count the number of points
    size_t n_points = 0;
    for (const auto& scan: scans)
    {
        n_points += scan->numPoints();
    }

    lvr2::floatArr points(new float[n_points * 3]);
    auto out = std::make_shared<lvr2::PointBuffer>(points, n_points);
    out->addEmptyIndexChannel("frame_id", n_points, 1);
    auto frame_ids = out->getIndexChannel("frame_id").get();

    size_t idx = 0;
    for (size_t pos_idx = 0; pos_idx < std::min(poses.size(), scans.size()); pos_idx++)
    {
        const Eigen::Isometry3f pose = poses[pos_idx];
        const lvr2::PointBufferPtr& scan = scans[pos_idx];

        // Copy all points
        const auto pts = scan->getPointArray();
        for (size_t i = 0; i < scan->numPoints(); i++)
        {
            Eigen::Vector3f point = pose * Eigen::Vector3f(pts.get() + i * 3);
            points[(idx + i) * 3 + 0] = point.x();
            points[(idx + i) * 3 + 1] = point.y();
            points[(idx + i) * 3 + 2] = point.z();
            
            // Need this cast because otherwise template argument deduction failes
            // in lvr2/types/ElementProxy.hpp
            frame_ids[idx + i] = (unsigned int) pos_idx;
        }
        idx += scan->numPoints();
    }

    // TODO: Copy all other sort of data
    return out;
}


void estimate_pointcloud_normals(
    const std::vector<Eigen::Isometry3f>& poses,
    lvr2::PointBufferPtr& cloud,
    const Options& opts
)
{
    using Vector = lvr2::BaseVector<float>;
    using Normal = lvr2::Normal<float>;

    lvr2::logout::get() << lvr2::info << "[estimate_pointcloud_normals] Nearest Neighbors: " << opts.normal_estimation_kn() << "; Normal Estimation Method: " << opts.normal_estimation_method() << lvr2::endl;

    lvr2::AdaptiveKSearchSurface<Vector> surface(
        cloud,
        "LVR2",
        opts.normal_estimation_kn(),
        0,
        0,
        opts.normal_estimation_method()
    );

    surface.calculateSurfaceNormals();

    if (!cloud->hasNormals())
    {
        lvr2::logout::get() << lvr2::error << "[estimate_pointcloud_normals] Buffer has no normals after normal calculation!" << lvr2::endl;
        return;
    }

    // Flip all normals towards the scanposition from which the point was aquired
    size_t n, w;
    const auto frame_id = cloud->getIndexArray("frame_id", n, w);
    const auto pts = cloud->getPointArray();
    const auto normals = cloud->getNormalArray();

    lvr2::Monitor monitor(lvr2::LogLevel::info, "Flipping Normals", cloud->numPoints());
    for (size_t i = 0; i < cloud->numPoints(); i++)
    {
        const unsigned int pos = frame_id[i];
        // The scan origin
        const Eigen::Vector3f tmp = poses[pos].translation();
        const Vector origin(tmp.x(), tmp.y(), tmp.z());
        // The point
        const Vector point(pts[i * 3 + 0], pts[i * 3 + 1], pts[i * 3 + 2]);

        const Vector dir = origin - point;
        // The calculated normal
        const Normal normal(normals[i * 3 + 0], normals[i * 3 + 1], normals[i * 3 + 2]);
        
        // If they do not point in the same direction flip the normal
        if (dir.dot(normal) < 0)
        {
            normals[i * 3 + 0] = -normal.x;
            normals[i * 3 + 1] = -normal.y;
            normals[i * 3 + 2] = -normal.z;
        }

        ++monitor;
    }
    monitor.terminate();
}


std::shared_ptr<lvr2::BaseMesh<lvr2::BaseVector<float>>> reconstruct_mesh(
    const lvr2::PointBufferPtr points,
    const Options& opts
)
{
    using Vector = lvr2::BaseVector<float>;
    using Box = lvr2::BilinearFastBox<Vector>;
    using Grid = lvr2::PointsetGrid<Vector, Box>;

    auto surface = std::make_shared<lvr2::AdaptiveKSearchSurface<Vector>>(
        points,
        "LVR2",
        0,
        0,
        opts.reconstruction_kd(),
        0
    );

    auto grid = std::make_shared<Grid>(
        opts.voxel_size(),
        surface,
        surface->getBoundingBox(),
        true,
        true
    );
    
    grid->calcDistanceValues();

    lvr2::FastReconstruction<Vector, Box> reconstruction(grid);

    auto mesh = std::make_shared<lvr2::PMPMesh<Vector>>();

    reconstruction.getMesh(*mesh);
    
    if (opts.rda_threshold() > 0)
    {
        lvr2::removeDanglingCluster(*mesh, opts.rda_threshold());
    }

    return mesh;
}


void deskew_scans(
    const std::vector<Eigen::Isometry3f>& poses,
    const std::vector<lvr2::PointBufferPtr>& scans
)
{
    
    for (size_t i = 1; i < std::min(poses.size(), scans.size()); i++)
    {
        Eigen::Isometry3f tf = poses[i] * poses[i - 1].inverse();
        auto scan = scans[i];
        size_t n, w;
        auto ts = scan->getFloatArray("time", n, w);

        if (!ts)
        {
            lvr2::logout::get() << lvr2::warning << "No time column!" << lvr2::endl;
            for (auto channel: *scan)
            {
                lvr2::logout::get() << lvr2::info << channel.first << lvr2::endl;
            }
        }

        auto pts = scan->getPointArray();
        
        // Normalize the timestamps
        const auto& [min, max] = std::minmax_element(ts.get(), ts.get() + n);
        const float delta = *max - *min;

        if (!delta)
        {
            continue;
        }

        const auto normalize = [min_val = *min, delta](const float ts){ return (ts - min_val) / delta;};
        
        // Transform all the points
        for (size_t i = 0; i < scan->numPoints(); i++)
        {
            auto time = normalize(ts[i]);
            lvr2::BaseVector<float> point(pts[i * 3 + 0], pts[i * 3 + 1], pts[i * 3 + 2]);
            
            // Interpolate SE3
            Eigen::Isometry3f interp = tf;
            Eigen::Quaternionf rot(interp.rotation());
            interp.matrix().topLeftCorner<3, 3>() = rot.slerp(time, Eigen::Quaternionf::Identity()).toRotationMatrix();
            interp.translation() = interp.translation() * time;
            point = interp.matrix() * point;

            pts[i * 3 + 0] = point.x;
            pts[i * 3 + 1] = point.y;
            pts[i * 3 + 2] = point.z;
        }
    }



}
