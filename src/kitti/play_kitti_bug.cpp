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

using namespace std;
using PointType = pcl::PointXYZI;

int main(int argc, char *argv[])
{
    int start_idx_ = 2350;
    int end_idx_ = 2760;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans_;
    std::vector<Eigen::Matrix4d> scan_poses_;
    // std::string velodyne_path = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_velodyne/dataset/sequences/05/velodyne/";
    // std::string calibration_file = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_calib/dataset/sequences/05/calib.txt";
    // std::string time_file = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_calib/dataset/sequences/05/times.txt";
    // std::string pose_file = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_labels/dataset/sequences/05/poses.txt";
    std::string velodyne_path = "/media/yzh/YZH2/KITTI Semantic/data_odometry_velodyne/dataset/sequences/05/velodyne/";
    std::string calibration_file = "/media/yzh/YZH2/KITTI Semantic/data_odometry_calib/dataset/sequences/05/calib.txt";
    std::string time_file = "/media/yzh/YZH2/KITTI Semantic/data_odometry_calib/dataset/sequences/05/times.txt";
    std::string pose_file = "/media/yzh/YZH2/KITTI Semantic/data_odometry_labels/dataset/sequences/05/poses.txt";
    string lablespath = "/media/yzh/YZH2/KITTI Semantic/data_odometry_labels/dataset/sequences/05/labels/";

    read_scans_and_poses(start_idx_, end_idx_, velodyne_path, pose_file, lablespath, calibration_file, scans_, scan_poses_);
    auto scans_size = scans_.size();

    ros::init(argc, argv, "kitti_publisher");
    ros::NodeHandle nh;
    ros::Publisher scans_pub = nh.advertise<sensor_msgs::PointCloud2>("/kitti_velo", 1);
    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map2", 1);
    ros::Rate rate(10);
    int scan_idx = 0;

    //* hash map
    tf::TransformBroadcaster ego_tf_broadcaster;

    bool paused = false;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>());
    while (ros::ok() && scan_idx < scans_size)
    {
        rate.sleep();

        pcl::PointCloud<PointType>::Ptr scan_global_coord(new pcl::PointCloud<PointType>());
        auto ii_scan = scans_.at(scan_idx);      // pcl::PointCloud<PointType>::Ptr
        auto ii_pose = scan_poses_.at(scan_idx); // Eigen::Matrix4d
                                                 // DONE: remove in range of 0.5m
                                                 //* remove in range of 0.5m
        for (size_t i = 0; i < ii_scan->points.size(); i++)
        {
            auto point = ii_scan->points.at(i);
            point.z = point.z;
            if (point.z < -3)
                continue;
            if (std::hypot(point.x, point.y, point.z) < 2)
            {
                continue;
            }
            // if (std::atan2(point.y, point.x) * 180 / M_PI > 15 || std::atan2(point.y, point.x) * 180 / M_PI < -15)
            //     continue;
            scan_global_coord->points.emplace_back(point);
        }
        cout << "scan_global_coord size: " << scan_global_coord->size() << endl;
        // done: ring
        std::array<float, 360> roi_distances_max; // to filter out not roi points
        roi_distances_max.fill(100);
        float fov_down = -24.9 / 180.0 * M_PI;
        float fov = 26.9 / 180.0 * M_PI;
        for (auto &pt : scan_global_coord->points)
        {
            int index_360 = std::floor(std::atan2(pt.y, pt.x) * 180 / M_PI + 180);
            float distance = std::hypot(pt.x, pt.y);
            float vertical_angle = std::atan2(pt.z, distance);
            int ring = (vertical_angle + abs(fov_down)) / fov * 64;
            if (ring > 60)
            {
                pt.intensity = 63; // Debug: pink max ring
                float distance_roi = std::hypot(pt.x, pt.y);
                roi_distances_max[index_360] = std::min(roi_distances_max[index_360], distance_roi);
                if (roi_distances_max[index_360] < 2)
                {
                    cout << " xyz=" << pt.x << " " << pt.y << " " << pt.z;
                }
            }
            else
                pt.intensity = 0; // Debug: red ground
        }
        for (auto &pt : scan_global_coord->points)
        {
            if (pt.intensity == 0)
            {
                int index_360 = std::floor(std::atan2(pt.y, pt.x) * 180 / M_PI + 180);
                float ego_2d_dis = std::hypot(pt.x, pt.y); // 当前点的2d dis
                // done: 去除中间的not ROI
                // TODO: not roi filter out: 11 感觉有点大，因为有部分点很奇怪，没滤掉
                if (roi_distances_max[index_360] > 5 && ego_2d_dis > roi_distances_max[index_360])
                {
                    pt.intensity = 10; // Debug: yellow中间的not ROI
                    // cout << roi_distances_max[index_360] << " ";
                }
                // DONE: 取出平地上方存在遮掩的问题，去除max ring附近区域，不作为ROI -5~5度之间
                else
                {
                    for (int i = -3; i <= 3; i++)
                    {
                        if (roi_distances_max[index_360 + i] > 11 && std::abs(ego_2d_dis - roi_distances_max[index_360 + i]) < 1)
                        {
                            pt.intensity = 30; // debug: green block ground
                            // cout << " tree region :" << roi_distances_max[index_360 + i];
                        }
                    }
                }
            }
        }

        sensor_msgs::PointCloud2 lidar_msg;
        pcl::toROSMsg(*scan_global_coord, lidar_msg);
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

        pcl::transformPointCloud(*scan_global_coord, *scan_global_coord, ii_pose);
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
    }

    return 0;
}