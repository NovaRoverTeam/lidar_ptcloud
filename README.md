# lidar_ptcloud

This repository is modified from HPS3D160 LiDAR SDK from its manufacturer Hypersen Techonologies to generate a livestream of a merged LiDAR point cloud on ROS from TWO HPS3D160 LiDAR simultaneously. 

## Getting Started
This repository contains a .so file (compiled library file) that encapsulates the commands to the LiDAR. On the first deployment of this repo, you must copy and load this library on your working Linux machine.
```
sudo cp libhps3d641712.so /usr/local/lib/
sudo ldconfig
```
Then plug the LiDARs by the order as instructed on the label on the USB plug and enable the read/write permission of the lidar
```
sudo chmod 777 /dev/ttyACM*
```
finally run the following:
```
roslaunch lidar_ptcloud lidar.launch
```
This generates two ROS nodes named `ptcloud_gen` and `ptcloud_merge`, with the latter publishes a merged point cloud data in `sensor_msgs/PointCloud2` format.
