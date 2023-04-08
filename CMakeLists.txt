cmake_minimum_required(VERSION 3.0.2)
project(myros_tools)

add_compile_options(-std=c++17 -O3)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  pcl_conversions
  tf
  tf_conversions
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# add hashmap
include_directories("/home/yzh/ros_ws/src/hash_map_v2/include")
file(GLOB_RECURSE HASH_MAP_SOURCES "/home/yzh/ros_ws/src/hash_map_v2/src/hash_map.cpp")

# pub .pcd to rviz
add_executable(pcd_pub src/pcd_pub.cpp)
target_link_libraries(pcd_pub
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

# generate hash_map.pcd
add_executable(bin2global_map ${HASH_MAP_SOURCES} src/bin2global_map.cpp)
target_link_libraries(bin2global_map
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    stdc++fs
)

# play kitti data
add_executable(play_kitti src/play_kitti.cpp)
target_link_libraries(play_kitti
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    stdc++fs
)