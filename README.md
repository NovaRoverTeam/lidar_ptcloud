# lidar_ptcloud

This repository is modified from HPS3D160 LiDAR SDK from its manufacturer Hypersen Techonologies to generate a livestream of a merged LiDAR point cloud on ROS from TWO HPS3D160 LiDAR simultaneously. 

## Getting Started
This repository contains a .so file (compiled library file) that encapsulates the commands to the LiDAR. On the first deployment of this repo, you must copy and load this library on your working Linux machine.
```
#for ARM processor like Jetson
sudo cp libhps3d641714_arm.so /usr/local/lib/ 
sudo chmod +x /usr/local/lib/libhps3d641714_arm.so
sudo ldconfig

#for x86-64 processor
sudo cp libhps3d641712.so /usr/local/lib/ #for x86-64 processor
sudo chmod +x /usr/local/lib/libhps3d641712.so
sudo ldconfig
```
(For x86-64 processor users) The CMakelist.txt in this repo is set to read `libhps3d641714_arm.so` by default. As an additional step, change lines 28 and 29 to as follows
```
target_link_libraries(ptcloud_gen ${catkin_LIBRARIES} hps3d641712)
#target_link_libraries(ptcloud_gen ${catkin_LIBRARIES} hps3d641714_arm)
```
 
then build it
```
catkin build lidar_ptcloud
```
Plug the LiDARs by the order as instructed on the label on the USB plug and check if they are registered as `/dev/ttyACM0` (Left) and `/dev/ttyACM1` (Right) by executing this line
```
ll /dev/ttyACM*
```
then enable the read/write permission of the lidar
```
sudo chmod 777 /dev/ttyACM*
```
finally run the following:
```
roslaunch lidar_ptcloud lidar.launch
```
This generates two ROS nodes named `ptcloud_gen` and `ptcloud_merge`, with the latter publishes a merged point cloud data in `sensor_msgs/PointCloud2` format.
