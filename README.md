# Introduction

`slam_to_mesh` is a cli tool to create 3D triangle meshes from the output of 3D SLAM systems.
It is build upon the algorithms implemented in the [LVR2](https://github.com/uos/lvr2) reconstruction toolkit.
The tool takes pairs of raw sensor point clouds (obtained from a LiDAR or 3D depth camera) and their corresponding 3D pose (obtained from a SLAM or LIO system) as an input and reconstructs a 3D triangle mesh.
I decided to use the registered individual scans instead of a combined point cloud as the input since the per-point surface normal estimation is more accurate when the position from which a point was measured is known (this information is lost when creating a combined point cloud).

# Compilation

This tool can be build by cloning it into a colcon workspace.
Make sure to also clone the [LVR2](https://github.com/uos/lvr2) project into the same workspace or make sure LVR2 is installed on your system.

```bash
# Create a new workspace (skip this if you are using an existing workspace)
mkdir -p stm_ws/src
cd stm_ws/src

# Clone this repository
git clone https://github.com/JustusBraun/slam_to_mesh.git 
# (Optional) Clone LVR2
git clone https://github.com/uos/lvr2.git

# Return to the workspace directory and build the workspace
cd ..
colcon build
```

# Usage

To reconstruct a mesh you need a directory containing the individual point clouds (the point clouds have to be relative to sensor coordinate system since the tool assumes [0, 0, 0] as the sensor origin!) and a file with the corresponding poses.
Supported file types for the point clouds are all types supported by the LVR2 toolkit.
The pose file supports the `kitty` and `tum` formats.
To match the point clouds to their corresponding poses the point clouds are sorted by their filenames using natural sort and the poses are read from the pose file in order.
The first point cloud in the sorted scan list is then assumed to belong to the first pose in the poses file.
If the numbers of point clouds and poses does not match the tool truncates the longer list.

```bash
# This command will read the point clouds and poses and save the reconstructed triangle mesh to 'mesh.ply'
slam_to_mesh --scans /PATH/TO/POINT_CLOUD_DIR/ --poses /PATH/TO/POSES_FILE.xyz --poses-filetype-hint tum

# To explore the available program options run
slam_to_mesh --help
```

## ROS2 Bag files

We also support the use of ROS2 Bag files as input.
The bag file is expected to contain a `sensor_msgs/PointCloud2` topic and an `nav_msgs/Odometry` topic with the corresponding poses.
The point cloud and odometry messages need to have the exact same timestamp, which is expected when using Lidar Odometry.

```bash
# This command will read the point clouds and poses from the bag file topics and save the reconstructed triangle mesh to 'mesh.ply'
slam_to_mesh --bag /PATH/TO/BAG/FILE --bag-pointcloud-topic /registered --bag-odometry-topic /odom
```

> [!NOTE]
> ROS2 Bag input is only available when compiling with a sourced ROS2 environment.

## Tuning the parameters

### Mesh "resolution"

`slam_to_mesh` utilizes LVR2's implementation of the Marching-Cubes surface reconstruction algorithm.
The Marching-Cubes algorithm uses a regular voxel grid to partition the space and approximates a surface patch for each voxel.
This results in a triangle mesh with a uniform triangle size.
To obtain a mesh with high fidelity in areas with small details, the Marching-Cubes voxel resolution has to be set sufficiently small.
However, choosing a small voxel resolution results in a suboptimal mesh with lots of triangles on surfaces that could just as well be represented using fewer larger triangles.
To get a high-fidelity mesh with fewer triangles in flat areas, I recommend reconstructing a mesh with a small voxel size and then simplifying the resulting mesh using remeshing algorithms implemented in tools like [MeshLab](https://www.meshlab.net/).

The voxel size used by `slam_to_mesh` defaults to `0.25 meter` can be set using the `--voxel-size` cli argument.

```bash
# This command will read the point clouds and poses and reconstruct a mesh using a 0.1 meter voxel resolution
slam_to_mesh --scans /PATH/TO/POINT_CLOUD_DIR/ --poses /PATH/TO/POSES_FILE.xyz --poses-filetype-hint tum --voxel-size 0.1
```
