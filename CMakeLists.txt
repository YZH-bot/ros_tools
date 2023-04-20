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
  image_transport
  cv_bridge
)

find_package(PCL 1.7 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED QUIET)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  # ${GTSAM_LIBRARY_DIRS}
)

# pub .pcd to rviz
add_executable(pcd_pub src/kitti/pcd_pub.cpp)
target_link_libraries(pcd_pub
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

# generate pcd
add_executable(bin2global_map src/kitti/bin2global_map.cpp)
target_link_libraries(bin2global_map
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    stdc++fs
)

# play kitti data
add_executable(play_kitti src/kitti/play_kitti.cpp)
target_link_libraries(play_kitti
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    stdc++fs
)

# kitti data to range images
add_executable(kitti2range_image src/kitti/kitti2range_image.cpp)
target_compile_options(kitti2range_image PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(kitti2range_image
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OpenMP_CXX_FLAGS}
    stdc++fs
)

# keyboard listening
add_executable(kblistening src/others/kblistening.cpp)
target_link_libraries(kblistening -lncurses)