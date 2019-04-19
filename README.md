# lidar_ptcloud

This repository is modified from HPS3D160 LiDAR SDK from its manufacturer Hypersen Techonologies to generate a livestream of LiDAR point cloud data on ROS from a HPS3D160 LiDAR. 

## Getting Started
This repository contains a .so file (compiled library file) that encapsulates the commands to the LiDAR. On the first deployment of this repo, you must copy and load this library on your working Linux machine.
```
sudo cp libhps3d641712.so /usr/local/lib/
sudo ldconfig
```
Then run the following:
```
roscore
rosrun lidar_ptcloud ptcloud_gen
```
This generates a ROS node named `ptcloud_gen` that publishes point cloud data in `sensor_msgs/PointCloud` format.
