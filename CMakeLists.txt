cmake_minimum_required(VERSION 2.8.3)
project(lidar_ptcloud)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  roslaunch
)

catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ptcloud_gen src/ptcloud_gen.cpp)
#target_link_libraries(ptcloud_gen ${catkin_LIBRARIES} hps3d641712)
target_link_libraries(ptcloud_gen ${catkin_LIBRARIES} hps3d641714_arm)

add_executable(ptcloud_merge src/ptcloud_merge.cpp)
target_link_libraries(ptcloud_merge ${catkin_LIBRARIES})

roslaunch_add_file_check(launch)
