#include <slam_to_mesh/algorithm.hpp>
#include <slam_to_mesh/iterator.hpp>
#include <slam_to_mesh/variant_channel.hpp>
#include <slam_to_mesh/logging.hpp>

#include <boost/smart_ptr/shared_array.hpp>
#include <boost/variant/apply_visitor.hpp>

#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <lvr2/algorithm/CleanupAlgorithms.hpp>
#include <lvr2/reconstruction/AdaptiveKSearchSurface.hpp>
#include <lvr2/reconstruction/FastBox.hpp>
#include <lvr2/reconstruction/FastBoxTables.hpp>
#include <lvr2/reconstruction/FastReconstruction.hpp>
#include <lvr2/reconstruction/PointsetGrid.hpp>
#include <lvr2/registration/OctreeReduction.hpp>
#include <lvr2/geometry/PMPMesh.hpp>

lvr2::PointBufferPtr combine_pointclouds(
    const Dataset& ds
)
{
    if (ds.scans.empty())
    {
        LOG_ERROR("Input dataset contains no scans!");
        return nullptr;
    }

    if (ds.poses.size() < ds.scans.size())
    {
        LOG_ERROR("Input dataset is missing poses for some scans! Poses: {} Scans: {}", ds.poses.size(), ds.scans.size());
        return nullptr;
    }

    // Count the number of points
    size_t n_points = 0;
    for (const auto& scan: ds.scans)
    {
        n_points += scan->numPoints();
    }
    
    // Allocate new buffer
    lvr2::PointBufferPtr out = std::make_shared<lvr2::PointBuffer>();
    
    // Create a new channel with capacity n_points for each channel
    for (const auto& [name, channel]: *ds.scans[0])
    {
        CreateSameTypeChannelWithSize<lvr2::PointBuffer::mapped_type> creator(n_points);
        out->insert_or_assign(name, boost::apply_visitor(creator, channel));
    }
    out->addEmptyIndexChannel("frame_id", n_points, 1);
    
    // Copy the data
    detail::PointBufferIterator pts_out(out->getPointArray().get());
    uint32_t* ids_out = out->getIndexChannel("frame_id").get().dataPtr().get();
    size_t out_idx = 0;
    for (size_t pos_idx = 0; pos_idx < std::min(ds.poses.size(), ds.scans.size()); pos_idx++)
    {
        const Eigen::Isometry3f pose = ds.poses[pos_idx];
        const lvr2::PointBufferPtr& scan = ds.scans[pos_idx];

        auto to_map = [pose](const lvr2::BaseVector<float>& vec)
        {
            return pose.matrix() * vec;
        };

        auto range = PointBufferRange(*scan);
        // Copy and transform all points
        pts_out = std::transform(range.begin(), range.end(), pts_out, to_map);

        // Store the pose index
        ids_out = std::fill_n(ids_out, scan->numPoints(), pos_idx);

        // If the input scans have normals a normal array has been allocated
        if (const auto normals = out->getNormalArray())
        {
            const auto scan_normals = scan->getNormalArray();
            if (!scan_normals)
            {
                LOG_WARNING("Not all input scans have normals. Scan {} is missing normals", pos_idx);
                if (!out->removeFloatChannel("normals"))
                {
                    LOG_ERROR("Failed to remove normal array 'normals' from PointBuffer");
                }
            }
            // Transform the normals
            for (size_t i = 0; i < scan->numPoints(); i++)
            {
                lvr2::Normal<float> normal(
                    scan_normals[i * 3 + 0],
                    scan_normals[i * 3 + 1],
                    scan_normals[i * 3 + 2]
                );

                normal = pose.matrix() * normal;

                normals[(out_idx + i) * 3 + 0] = normal.x;
                normals[(out_idx + i) * 3 + 1] = normal.y;
                normals[(out_idx + i) * 3 + 2] = normal.z;
            }
        }

        // Copy the rest of the channels
        for (auto& [name, channel]: *out)
        {
            if ("points" == name || "frame_id" == name || "normals" == name)
            {
                continue;
            }
            boost::apply_visitor(CopyChannel(channel, out_idx), scan->at(name));
        }
        out_idx += scan->numPoints();
    }

    return out;
}


void estimate_pointcloud_normals(
    const std::vector<Eigen::Isometry3f>& poses,
    const lvr2::PointBufferPtr& cloud,
    const Options& opts
)
{
    using Vector = lvr2::BaseVector<float>;
    using Normal = lvr2::Normal<float>;

    LOG_INFO("Nearest Neighbors: {}; Normal Estimation Method: {}", opts.normal_estimation_kn(), opts.normal_estimation_method());

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
        LOG_ERROR("Buffer has no normals after normal calculation!");
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

    if (opts.normal_estimation_ki())
    {
        surface.setKi(opts.normal_estimation_ki());
        surface.interpolateSurfaceNormals();
    }
}


template <typename BoxT>
inline void link_neighbours(std::unordered_map<Eigen::Vector3i, BoxT>& shells, const Eigen::Vector3i& index, BoxT& box)
{
    size_t neighbor_index = 0;
    for (int dx = -1; dx <= 1; dx++)
    {
        for (int dy = -1; dy <= 1; dy++)
        {
            for (int dz = -1; dz <= 1; dz++)
            {
                auto neighbor_it = shells.find(index + Eigen::Vector3i(dx, dy, dz));

                // If it exists, save pointer in box
                if (neighbor_it != shells.end())
                {
                    const auto& neighbor = neighbor_it->second;
                    box.setNeighbor(neighbor_index, &neighbor_it->second);

                    // Update the m_intersections array which is normally set during the getSurface calls.
                    // By filling this data the cell knows if a corner vertex was already created by
                    // a neighbor cell.

                    // Iterate all 12 edges. Maybe this can be done without iterating all edges.
                    for (int isec = 0; isec < 12; isec++)
                    {
                        // 3 neighbors per edge
                        for ( int i = 0; i < 3; i++)
                        {
                            const int idx = lvr2::neighbor_table[isec][i];
                            if (idx != neighbor_index)
                            {
                                continue;
                            }

                            if (!neighbor.m_intersections[lvr2::neighbor_vertex_table[isec][i]])
                            {
                                continue;
                            }
                            box.m_intersections[isec] = neighbor.m_intersections[lvr2::neighbor_vertex_table[isec][i]];
                        }
                    }
                }
                neighbor_index++;
            }
        }
    }
}


void iterate_outer_shell(
    const Eigen::Vector3i& min,
    const Eigen::Vector3i& max,
    std::function<void(const Eigen::Vector3i&)> func
)
{
    // Iterate the cube walls
    // XY plane with z max and z min
    for (int x = min.x(); x < max.x(); x++)
    {
        for (int y = min.y(); y < max.y(); y++)
        {
            func(Eigen::Vector3i(x, y, min.z()));
            func(Eigen::Vector3i(x, y, max.z()));
        }
    }
    // XZ plane with y max and y min
    for (int x = min.x(); x < max.x(); x++)
    {
        for (int z = min.z() + 1; z < max.z() - 1; z++)
        {
            func(Eigen::Vector3i(x, min.y(), z));
            func(Eigen::Vector3i(x, max.y(), z));
        }
    }
    // YZ plane with x max and x min
    for (int y = min.y() + 1; y < max.y() - 1; y++)
    {
        for (int z = min.z() + 1; z < max.z() - 1; z++)
        {
            func(Eigen::Vector3i(min.x(), y, z));
            func(Eigen::Vector3i(max.x(), y, z));
        }
    }
}


std::shared_ptr<lvr2::BaseMesh<lvr2::BaseVector<float>>> reconstruct_mesh(
    const lvr2::PointBufferPtr points,
    const Options& opts
)
{
    using Vector = lvr2::BaseVector<float>;
    using Eigen::Vector3i;
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

    const lvr2::BoundingBox<Vector> bb = surface->getBoundingBox();

    // Slice the bounding box in 25m^3 cubes
    const float edge_length = 25.0;
    std::vector<lvr2::BoundingBox<Vector>> bboxes;
    const Vector minimum = bb.getMin();
    const Vector maximum = bb.getMax();

    const size_t x_steps = std::ceil(bb.getXSize() / edge_length);
    const size_t y_steps = std::ceil(bb.getYSize() / edge_length);
    const size_t z_steps = std::ceil(bb.getZSize() / edge_length);

    for (size_t z = 0; z < z_steps; z++)
    {
        for (size_t y = 0; y < y_steps; y++)
        {
            for (size_t x = 0; x < x_steps; x++)
            {
                const Vector displacement(x * edge_length, y * edge_length, z * edge_length);
                const Vector min = bb.getMin() + displacement;
                Vector max = min + Vector(edge_length, edge_length, edge_length);

                max.x = std::min(max.x, maximum.x);
                max.y = std::min(max.y, maximum.y);
                max.z = std::min(max.z, maximum.z);

                bboxes.push_back(lvr2::BoundingBox<Vector>(min, max));
            }
        }
    }


    // The output mesh instance
    auto mesh = std::make_shared<lvr2::PMPMesh<Vector>>();

    std::unordered_map<Vector3i, Box> shells;
    // Reconstruct the mesh one chunk at a time. This consumes less memory than
    // creating one big hash grid and therefore fits into ram, which makes it
    // faster as the need for swapping pages is eliminated.
    // TODO: Figure out how to suppress the output from the loop
    for (const auto& bounds: bboxes)
    {
        auto grid = std::make_shared<Grid>(
            opts.voxel_size(),
            surface,
            bounds,
            true,
            true
        );

        // Add neighbour entries for the boxes at the edge of the bounding box to boxes in the
        // next bounding box. This ensures that the mesh parts are connected
        // Calculate the shell of the current grid
        Vector half_voxel_size(opts.voxel_size() / 2.0, opts.voxel_size() / 2.0, opts.voxel_size() / 2.0);
        Vector3i min = grid->calcIndex(bounds.getMin() + half_voxel_size);
        Vector3i max = grid->calcIndex(bounds.getMax() - half_voxel_size);

        // Link the shell cells to the neighbor cells already created by processed bounding boxes
        iterate_outer_shell(
            min, max,
            [&](const Eigen::Vector3i& index)
            {
                auto it = grid->getCells().find(index);
                if (it != grid->getCells().end())
                {
                    link_neighbours(shells, index, *it->second);
                }
            }
        );

        // Reconstruct the part
        grid->calcDistanceValues();
        lvr2::FastReconstruction<Vector, Box> reconstruction(grid);
        reconstruction.getMesh(*mesh);

        // Save the shell cells of the current grid for later
        iterate_outer_shell(
            min, max,
            [&](const Eigen::Vector3i& index)
            {
                auto it = grid->getCells().find(index);
                if (it != grid->getCells().end())
                {
                    shells.insert_or_assign(index, *it->second);
                }
            }
        );
    }

    if (opts.rda_threshold() > 0)
    {
        lvr2::removeDanglingCluster(*mesh, opts.rda_threshold());
    }
    
    if (opts.fill_holes_threshold() > 0)
    {
        // The lvr2::naiveFillSmallHoles function does raise exceptions
        // so we just use the PMPMesh method instead
        mesh->fillHoles(opts.fill_holes_threshold());
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
            LOG_WARNING("The scan {} has no time column!", i);
            continue;
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


lvr2::PointBufferPtr remove_nan(const lvr2::PointBufferPtr& buffer)
{
    // Create a valid points mask
    auto has_nan = [](const lvr2::BaseVector<float>& vec)
    {
        return std::isnan(vec.x)
            || std::isnan(vec.y)
            || std::isnan(vec.z)
            || std::isinf(vec.x)
            || std::isinf(vec.y)
            || std::isinf(vec.z);
    };
    auto range = PointBufferRange(*buffer);
    std::vector<bool> mask(buffer->numPoints());
    std::transform(range.begin(), range.end(), mask.begin(), has_nan);
    const size_t n_valid = std::count_if(range.begin(), range.end(), std::not_fn(has_nan));

    auto out = std::make_shared<lvr2::PointBuffer>();

    // Remove all invalid points using the mask
    for (const auto& [name, channel]: *buffer)
    {
        out->insert_or_assign(name, boost::apply_visitor(
            CreateSameTypeChannelWithSize<lvr2::PointBuffer::mapped_type>(n_valid),
            channel
        ));
        
        boost::apply_visitor(RemoveCopyIf(out->at(name), mask.begin()), channel);
    }

    return out;
}


lvr2::PointBufferPtr voxel_downsample(const lvr2::PointBufferPtr& points, const float voxel_size)
{
    lvr2::OctreeReductionAlgorithm reduction(voxel_size, 1, lvr2::OctreeType(lvr2::NEAREST_CENTER));

    reduction.setPointBuffer(points);

    return reduction.getReducedPoints();
}


lvr2::PointBufferPtr statistical_outlier_removal(const lvr2::PointBufferPtr& points, const size_t neighbors, const float factor)
{
    using Vector = lvr2::BaseVector<float>;
    lvr2::SearchTreeFlann<Vector> tree(points);
    std::vector<float> distances(points->numPoints());
    lvr2::floatArr pts = points->getPointArray();
    
    lvr2::Monitor monitor(lvr2::LogLevel::info, std::format("[{}] Calculating average distances", __func__), points->numPoints());

    // Calculate the average distance of each point to its n neighbors.
    #pragma omp parallel
    {
    std::vector<size_t> _indices;
    std::vector<float> _dists;
        #pragma omp for
        for (size_t i = 0; i < points->numPoints(); i++)
        {
            const Vector vec(pts[i * 3 + 0], pts[i * 3 + 1], pts[i * 3 + 2]);
            tree.kSearch(vec, neighbors, _indices, _dists);
            
            distances[i] = std::accumulate(_dists.begin(), _dists.end(), 0.0) / neighbors;

            #pragma omp critical
            ++monitor;
        }
    }
    monitor.terminate();

    // Calculate the distance Mean and stddev
    const float mean = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
    const float variance = std::transform_reduce(
        distances.begin(), distances.end(), 0.0, std::plus<float>(),
        [mean](const float& dist){return std::pow(dist - mean, 2);}
    ) / distances.size();
    
    // Create a mask for point removal, points with an average distance to their
    // neighbors larger than thresh will be removed
    const float thresh = mean + factor * std::sqrt(variance);
    std::vector<bool> mask(distances.size());
    std::transform(distances.begin(), distances.end(), mask.begin(), [thresh](const float dist){ return dist > thresh;});

    // Create the new filtered buffer and return
    const size_t n_keep = std::count(mask.begin(), mask.end(), false);
    auto out = std::make_shared<lvr2::PointBuffer>();
    for (const auto& [name, channel]: *points)
    {
        out->insert_or_assign(name, boost::apply_visitor(
            CreateSameTypeChannelWithSize<lvr2::PointBuffer::mapped_type>(n_keep),
            channel
        ));
        
        boost::apply_visitor(RemoveCopyIf(out->at(name), mask.begin()), channel);
    }

    return out;
}
