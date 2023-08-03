#include <termio.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include "kitti_function.hpp"
#include <pcl/filters/passthrough.h>
#include "octomap_building.hpp"
#include <octomap/octomap.h>

using namespace std;
using PointType = pcl::PointXYZI;

int main(int argc, char *argv[])
{
    int start_idx_ = 2350;
    int end_idx_ = 2460;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans_;
    std::vector<Eigen::Matrix4d> scan_poses_;
    std::string velodyne_path = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_velodyne/dataset/sequences/05/velodyne/";
    std::string calibration_file = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_calib/dataset/sequences/05/calib.txt";
    std::string time_file = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_calib/dataset/sequences/05/times.txt";
    std::string pose_file = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_labels/dataset/sequences/05/poses.txt";
    string lablespath = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_labels/dataset/sequences/05/labels/";
    // std::string velodyne_path = "/media/yzh/YZH2/KITTI Semantic/data_odometry_velodyne/dataset/sequences/05/velodyne/";
    // std::string calibration_file = "/media/yzh/YZH2/KITTI Semantic/data_odometry_calib/dataset/sequences/05/calib.txt";
    // std::string time_file = "/media/yzh/YZH2/KITTI Semantic/data_odometry_calib/dataset/sequences/05/times.txt";
    // std::string pose_file = "/media/yzh/YZH2/KITTI Semantic/data_odometry_labels/dataset/sequences/05/poses.txt";
    // string lablespath = "/media/yzh/YZH2/KITTI Semantic/data_odometry_labels/dataset/sequences/05/labels/";

    read_scans_and_poses(start_idx_, end_idx_, velodyne_path, pose_file, lablespath, calibration_file, scans_, scan_poses_);
    auto scans_size = scans_.size();

    ros::init(argc, argv, "kitti_publisher");
    ros::NodeHandle nh;
    ros::Publisher scans_pub = nh.advertise<sensor_msgs::PointCloud2>("/kitti_velo", 1);
    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map2", 1);
    ros::Publisher octomap_pub = nh.advertise<sensor_msgs::PointCloud2>("/octomap", 1);
    ros::Rate rate(10);
    int scan_idx = 0;

    //* hash map
    tf::TransformBroadcaster ego_tf_broadcaster;
    OctomapServer server;
    octomap::OcTree tree(0.1);

    bool paused = false;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>());
    while (ros::ok() && scan_idx < scans_size)
    {
        rate.sleep();

        pcl::PointCloud<PointType>::Ptr scan_global_coord(new pcl::PointCloud<PointType>());
        auto ii_scan = scans_.at(scan_idx);      // pcl::PointCloud<PointType>::Ptr
        auto ii_pose = scan_poses_.at(scan_idx); // Eigen::Matrix4d

        sensor_msgs::PointCloud2 lidar_msg;
        pcl::toROSMsg(*ii_scan, lidar_msg);
        lidar_msg.header.stamp = ros::Time::now();
        lidar_msg.header.frame_id = "ego_car";
        scans_pub.publish(lidar_msg);
        std::cout << "Publishing" << scan_idx << "\r" << std::endl;

        // ?tf2eigen https://blog.csdn.net/zz123456zzss/article/details/81914675
        Eigen::Quaterniond eigen_quat(ii_pose.block<3, 3>(0, 0).cast<double>());
        Eigen::Vector3d eigen_trans(ii_pose.block<3, 1>(0, 3).cast<double>());
        tf::Quaternion tf_quat;
        tf::Vector3 tf_trans;
        tf::quaternionEigenToTF(eigen_quat, tf_quat);
        tf::vectorEigenToTF(eigen_trans, tf_trans);
        tf::StampedTransform tf_map2scan(tf::Transform(tf_quat, tf_trans), ros::Time::now(), "world", "ego_car");
        ego_tf_broadcaster.sendTransform(tf_map2scan);
        scan_idx++;

        pcl::transformPointCloud(*ii_scan, *scan_global_coord, ii_pose);
        pcl::VoxelGrid<pcl::PointXYZI> sor1;
        sor1.setLeafSize(0.2, 0.2, 0.2);
        sor1.setInputCloud(scan_global_coord);
        sor1.filter(*scan_global_coord);

        *map += *scan_global_coord;
        pcl::VoxelGrid<pcl::PointXYZI> sor2;
        sor2.setLeafSize(0.2, 0.2, 0.2);
        sor2.setInputCloud(map);
        sor2.filter(*map);

        // 定义裁剪器，设置过滤范围
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(map);
        pass.setFilterFieldName("y");     // 按照z轴方向裁剪
        pass.setFilterLimits(10.0, 40.0); // 裁剪z轴范围为[0, 1]
        // 执行滤波操作，获取裁剪后的点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pass.filter(*filtered_cloud);

        sensor_msgs::PointCloud2 msg_ground;
        pcl::toROSMsg(*filtered_cloud, msg_ground);
        msg_ground.is_dense = true;
        msg_ground.header.frame_id = "world";
        map_pub.publish(msg_ground);
        ROS_INFO_STREAM(GREEN << "- sended map pcd: " << map->size());

        // PCLPointCloud pc_ground;    // segmented ground plane
        // PCLPointCloud pc_nonground; // everything else
        // pcl::copyPointCloud(*scan_global_coord, pc_ground);
        // tf::Vector3 p = tf_map2scan.getOrigin();
        // server.insertScan(p, pc_ground, pc_nonground);
        // server.toPointcloud();

        // sensor_msgs::PointCloud2 msg_octomap;
        // pcl::toROSMsg(server.pclCloud, msg_octomap);
        // msg_octomap.is_dense = true;
        // msg_octomap.header.frame_id = "world";
        // octomap_pub.publish(msg_octomap);
        // ROS_INFO_STREAM(GREEN << "- sended msg_octomap pcd: " << server.pclCloud.size());
        octomap::Pointcloud cloud_octo;
        for (auto pt : scan_global_coord->points)
        {
            // tree.updateNode(octomap::point3d(pt.x, pt.y, pt.z), true);
            cloud_octo.push_back(pt.x, pt.y, pt.z);
        }
        tree.insertPointCloud(cloud_octo, octomap::point3d(ii_pose(0,3), ii_pose(1,3), ii_pose(2,3)));
        tree.updateInnerOccupancy();
    }
    string output_file = "/media/robot-nuc12/T7/bag_map/comparison/octomap2.bt";
    tree.writeBinary(output_file);
    cout << "done." << endl;

    return 0;
}