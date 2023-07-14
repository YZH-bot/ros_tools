// #include <pcl/common/transforms.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <fstream>
// #include <iomanip>
// #include <iostream>
// #include <vector>
// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
// #include "kitti_function.hpp"
// #include <pcl/filters/passthrough.h>

// int main(int argc, char **argv)
// {
//     //     std::string velodyne_path = "/media/yzh/YZH2/KITTI Semantic/data_odometry_velodyne/dataset/sequences/05/velodyne/";
//     //     std::string calibration_file = "/media/yzh/YZH2/KITTI Semantic/data_odometry_calib/dataset/sequences/05/calib.txt";
//     //     std::string time_file = "/media/yzh/YZH2/KITTI Semantic/data_odometry_calib/dataset/sequences/05/times.txt";
//     //     std::string pose_file = "/media/yzh/YZH2/KITTI Semantic/data_odometry_labels/dataset/sequences/05/poses.txt";
//     //     string label_file = "/media/yzh/YZH2/KITTI Semantic/data_odometry_labels/dataset/sequences/05/labels/";
//     std::string velodyne_path = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_velodyne/dataset/sequences/05/velodyne/";
//     std::string calibration_file = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_calib/dataset/sequences/05/calib.txt";
//     std::string time_file = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_calib/dataset/sequences/05/times.txt";
//     std::string pose_file = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_labels/dataset/sequences/05/poses.txt";
//     std::string label_file = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_labels/dataset/sequences/05/labels/";

//     std::ifstream calibration_fin(calibration_file);
//     std::string value;

//     // 创建 AngleAxis 对象表示绕 X 轴旋转 -90 弧度
//     Eigen::AngleAxisd rotation_vector_x(M_PI / 2, Eigen::Vector3d::UnitX());
//     Eigen::Matrix3d rotation_x = rotation_vector_x.toRotationMatrix();
//     Eigen::Matrix4d transformation_matrix_x = Eigen::Matrix4d::Identity();
//     transformation_matrix_x.block<3, 3>(0, 0) = rotation_x;
//     // 创建 AngleAxis 对象表示绕 Z 轴旋转 180 弧度
//     Eigen::AngleAxisd rotation_vector_z(M_PI / 2, Eigen::Vector3d::UnitZ());
//     Eigen::Matrix3d rotation_z = rotation_vector_z.toRotationMatrix();
//     Eigen::Matrix4d transformation_matrix_z = Eigen::Matrix4d::Identity();
//     transformation_matrix_z.block<3, 3>(0, 0) = rotation_z;

//     // done: get cam_velo T
//     for (int i = 0; i < 4; i++)
//     {
//         std::getline(calibration_fin, value);
//     }
//     Eigen::Matrix<double, 4, 3> matrix_in;
//     Eigen::Matrix<double, 4, 4> t_cam_velo(Eigen::Matrix<double, 4, 4>::Identity());
//     std::getline(calibration_fin, value, ' ');
//     for (int i = 0; i < 11; i++)
//     {
//         std::getline(calibration_fin, value, ' ');
//         matrix_in(i) = stof(value);
//     }
//     std::getline(calibration_fin, value, '\n');
//     matrix_in(11) = stof(value);
//     t_cam_velo.block<3, 4>(0, 0) = matrix_in.transpose();

//     std::ifstream time_fin(time_file);
//     std::ifstream pose_fin(pose_file);
//     int line_num = 0;
//     std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans_;
//     std::vector<Eigen::Matrix4d> scan_poses_;
//     while (std::getline(time_fin, value))
//     {
//         if (line_num < 2350)
//         {
//             line_num++; //
//             for (int i = 0; i < 11; i++)
//             {
//                 std::getline(pose_fin, value, ' ');
//                 matrix_in(i) = stof(value);
//             }
//             std::getline(pose_fin, value, '\n');
//             continue;
//         }
//         std::stringstream velo_ss, label_ss;
//         velo_ss << velodyne_path << "/" << std::setfill('0') << std::setw(6)
//                 << line_num << ".bin";

//         int32_t num = 1000000;
//         float *data = (float *)malloc(num * sizeof(float));

//         float *px = data + 0;
//         float *py = data + 1;
//         float *pz = data + 2;
//         float *pr = data + 3;

//         FILE *stream;
//         stream = fopen(velo_ss.str().c_str(), "rb");
//         num = fread(data, sizeof(float), num, stream) / 4;

//         pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(
//             new pcl::PointCloud<pcl::PointXYZI>);
//         for (int32_t i = 0; i < num; i++)
//         {
//             pcl::PointXYZI point_xyzi;
//             point_xyzi.x = *px;
//             point_xyzi.y = *py;
//             point_xyzi.z = *pz;
//             point_xyzi.intensity = *pr;
//             point_cloud->push_back(point_xyzi);
//             px += 4;
//             py += 4;
//             pz += 4;
//             pr += 4;
//         }
//         fclose(stream);
//         delete data;

//         // ##################### load pose ########################
//         for (int i = 0; i < 11; i++)
//         {
//             std::getline(pose_fin, value, ' ');
//             matrix_in(i) = stof(value);
//         }
//         std::getline(pose_fin, value, '\n');
//         matrix_in(11) = stof(value);
//         Eigen::Matrix4d pose_cam(Eigen::Matrix4d::Identity());
//         pose_cam.block<3, 4>(0, 0) = matrix_in.transpose();
//         Eigen::Matrix4d pose_velo = pose_cam * t_cam_velo;
//         // pcl::transformPointCloud(*point_cloud, *point_cloud, pose_velo);
//         // pcl::transformPointCloud(*point_cloud, *point_cloud, t_cam_velo.inverse().cast<float>());
//         scans_.push_back(point_cloud);

//         Eigen::Matrix4d pose_pub = t_cam_velo.inverse() * pose_cam * transformation_matrix_x * transformation_matrix_z;
//         scan_poses_.push_back(pose_cam);
//         std::cout << line_num++ << '\r' << std::flush;
//         // if (line_num > 400)
//         // {
//         //     break;
//         // }
//     }

//     ros::init(argc, argv, "kitti_node");
//     ros::NodeHandle nh;
//     ros::Publisher scans_pub = nh.advertise<sensor_msgs::PointCloud2>("scans_pub", 1);
//     ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);
//     tf::TransformBroadcaster tf_broadcaster;
//     ros::Rate rate(10);
//     auto scans_size = scans_.size();
//     int scan_idx = 0;
//     pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>());

//     // Eigen::Matrix4d velo2cam, cam2velo;
//     // cam2velo << 0, 0, 1, 0,
//     //     -1, 0, 0, 0,
//     //     0, -1, 0, 0.08,
//     //     0, 0, 0, 1;
//     // velo2cam << 0, -1, 0, 0,
//     //     0, 0, -1, 0,
//     //     1, 0, 0, -0.08,
//     //     0, 0, 0, 1;
//     Eigen::Matrix4d t_cam_velo_inverse;
//     t_cam_velo_inverse << -0.00185774, -0.00648147, 0.999977, 0.3335048572608868,
//                         -0.999966, 0.00805186, -0.00180553, -0.004796109355192,
//                         -0.00803998, -0.999947, -0.0064962, -0.0755785859790194,
//                         0, 0, 0, 1;
//     while (ros::ok())
//     {
//         rate.sleep();
//         if (scan_idx < scans_size - 1)
//         {
//             scan_idx++;
//             auto ii_scan = scans_.at(scan_idx);
//             auto ii_pose = scan_poses_.at(scan_idx);

//             ii_pose = t_cam_velo_inverse * ii_pose * t_cam_velo;
//             cout << t_cam_velo << endl;
//             cout << t_cam_velo.inverse() << endl;
//             pcl::transformPointCloud(*ii_scan, *ii_scan, ii_pose);
//             // ?calculate the rings for points
//             add_rings_information(ii_scan);

//             sensor_msgs::PointCloud2 lidar_msg;
//             pcl::toROSMsg(*ii_scan, lidar_msg);
//             lidar_msg.header.stamp = ros::Time::now();
//             lidar_msg.header.frame_id = "world";
//             scans_pub.publish(lidar_msg);

//             // ?TF publishing
//             tf::StampedTransform tf_map2ego = Matrix4dpose2TF(ii_pose);
//             tf_broadcaster.sendTransform(tf_map2ego);

//             std::cout << "Publishing" << std::endl;

//             pcl::VoxelGrid<pcl::PointXYZI> sor1;
//             sor1.setLeafSize(0.5, 0.5, 0.5);
//             sor1.setInputCloud(ii_scan);
//             sor1.filter(*ii_scan);
//             *map += *ii_scan;
//             sor1.setInputCloud(map);
//             sor1.filter(*map);
//             sensor_msgs::PointCloud2 msg_ground;
//             // // 定义裁剪器，设置过滤范围
//             // pcl::PassThrough<pcl::PointXYZI> pass;
//             // pass.setInputCloud(map);
//             // pass.setFilterFieldName("y");   // 按照z轴方向裁剪
//             // pass.setFilterLimits(10.0, 40.0); // 裁剪z轴范围为[0, 1]
//             // // 执行滤波操作，获取裁剪后的点云
//             // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//             // pass.filter(*filtered_cloud);
//             pcl::toROSMsg(*map, msg_ground);
//             msg_ground.header.frame_id = "world";
//             map_pub.publish(msg_ground);
//             ROS_INFO_STREAM(GREEN << "- sended map pcd: " << map->size());
//         }
//         else
//         {
//             scan_idx = 0;
//             std::exit(0);
//         }
//     }
//     return 0;
// }

#include <termio.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include "kitti_function.hpp"

using namespace std;
using PointType = pcl::PointXYZI;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kitti_publisher");
    ros::NodeHandle nh("~");
    ros::Publisher scans_pub = nh.advertise<sensor_msgs::PointCloud2>("/kitti_velo", 1);
    ros::Rate rate(10);
    tf::TransformBroadcaster ego_tf_broadcaster;

    int start_idx_ = 2350;
    int end_idx_ = 2760;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans_;
    std::vector<Eigen::Matrix4d> scan_poses_;
    string velodyne_path;
    string lablespath;
    string pose_file;
    string calibration_file;
    string time_file;
    string save_pcd_directory_;
    // nh.param<std::string>("lidar_datapath", velodyne_path, "/media/yzh/YZH2/KITTI Semantic/data_odometry_velodyne/dataset/sequences/05/velodyne/");
    // nh.param<std::string>("lables_path", lablespath, "/media/yzh/YZH2/KITTI Semantic/data_odometry_labels/dataset/sequences/05/labels/");
    // nh.param<std::string>("pose_file", pose_file, "/media/yzh/YZH2/KITTI Semantic/data_odometry_labels/dataset/sequences/05/poses.txt");
    // nh.param<std::string>("calibration_file", calibration_file, "/media/yzh/YZH2/KITTI Semantic/data_odometry_calib/dataset/sequences/05/calib.txt");
    // nh.param<std::string>("time_file", time_file, "/media/yzh/YZH2/KITTI Semantic/data_odometry_calib/dataset/sequences/05/times.txt");
    nh.param("start_idx_", start_idx_, 2350);
    nh.param("end_idx_", end_idx_, 2760);
    nh.param<std::string>("lidar_datapath", velodyne_path, "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_velodyne/dataset/sequences/05/velodyne/");
    nh.param<std::string>("lables_path", lablespath, "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_labels/dataset/sequences/05/labels/");
    nh.param<std::string>("pose_file", pose_file, "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_labels/dataset/sequences/05/poses.txt");
    nh.param<std::string>("calibration_file", calibration_file, "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_calib/dataset/sequences/05/calib.txt");
    nh.param<std::string>("time_file", time_file, "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_calib/dataset/sequences/05/times.txt");
    nh.param<std::string>("save_pcd_directory_", save_pcd_directory_, "/media/robot-nuc12/T7/bag_map/kitti/.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>());
    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);

    ROS_INFO_STREAM(GREEN << velodyne_path);
    ROS_INFO_STREAM(GREEN << lablespath);
    ROS_INFO_STREAM(GREEN << pose_file);
    ROS_INFO_STREAM(GREEN << calibration_file);
    ROS_INFO_STREAM(GREEN << time_file);
    // read_scans_and_poses(start_idx_, end_idx_, velodyne_path, pose_file, lablespath, calibration_file, scans_, scan_poses_);
    // read_scans_and_poses(start_idx_, end_idx_, scans_, scan_poses_, time_file, pose_file, velodyne_path, calibration_file, lablespath);
    std::ifstream time_fin(time_file);
    std::ifstream pose_fin(pose_file);
    std::ifstream calibration_fin(calibration_file);
    std::string value;
    int line_num = 0;
    Eigen::Matrix<double, 4, 3> matrix_in;
    Eigen::Matrix<double, 4, 4> t_cam_velo(Eigen::Matrix<double, 4, 4>::Identity());

    // DONE: get cam_velo T
    for (int i = 0; i < 4; i++)
    {
        std::getline(calibration_fin, value);
    }
    std::getline(calibration_fin, value, ' ');
    for (int i = 0; i < 11; i++)
    {
        std::getline(calibration_fin, value, ' ');
        matrix_in(i) = stof(value);
    }
    std::getline(calibration_fin, value, '\n');
    matrix_in(11) = stof(value);
    t_cam_velo.block<3, 4>(0, 0) = matrix_in.transpose();

    // DONE: read scans and poses
    while (ros::ok() && std::getline(time_fin, value))
    {
        if (line_num < start_idx_)
        {
            line_num++;
            for (int i = 0; i < 11; i++)
            {
                std::getline(pose_fin, value, ' ');
                matrix_in(i) = stof(value);
            }
            std::getline(pose_fin, value, '\n');
            continue;
        }
        if (line_num > end_idx_)
            break;
        std::stringstream velo_ss;
        std::stringstream label_ss;
        velo_ss << velodyne_path << std::setfill('0') << std::setw(6)
                << line_num << ".bin";
        label_ss << lablespath << std::setfill('0') << std::setw(6)
                 << line_num << ".label";
        // ROS_INFO_STREAM(DEBUG << velo_ss.str());
        // ROS_INFO_STREAM(DEBUG << label_ss.str());
        int32_t num = 1000000;
        float *data = new float[num];
        float *px = data + 0;
        float *py = data + 1;
        float *pz = data + 2;
        float *pr = data + 3;

        FILE *stream;
        stream = fopen(velo_ss.str().c_str(), "rb");
        num = fread(data, sizeof(float), num, stream) / 4;

        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(
            new pcl::PointCloud<pcl::PointXYZI>);
        for (int32_t i = 0; i < num; i++)
        {
            pcl::PointXYZI point_xyzi;
            point_xyzi.x = *px;
            point_xyzi.y = *py;
            point_xyzi.z = *pz;
            point_xyzi.intensity = *pr;
            point_cloud->push_back(point_xyzi);
            px += 4;
            py += 4;
            pz += 4;
            pr += 4;
        }
        fclose(stream);
        delete data;

        for (int i = 0; i < 11; i++)
        {
            std::getline(pose_fin, value, ' ');
            matrix_in(i) = stof(value);
        }
        std::getline(pose_fin, value, '\n');
        matrix_in(11) = stof(value);
        Eigen::Matrix4d pose_cam(Eigen::Matrix4d::Identity());
        pose_cam.block<3, 4>(0, 0) = matrix_in.transpose();
        // std::cout << line_num++ << '\r' << std::flush;
        std::cout << line_num++ << std::endl;

        load_label(label_ss.str(), point_cloud); // For KITTI (.label)
        transform_pose_to_world(pose_cam, t_cam_velo);

        // DONE: ros
        // DONE: remove in range of 0.5m
        pcl::PointCloud<PointType>::Ptr scan_global_coord(new pcl::PointCloud<PointType>());
        for (size_t i = 0; i < point_cloud->points.size(); i++)
        {
            auto point = point_cloud->points.at(i);
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
        sensor_msgs::PointCloud2 lidar_msg;
        pcl::toROSMsg(*scan_global_coord, lidar_msg);
        lidar_msg.header.stamp = ros::Time::now();
        lidar_msg.header.frame_id = "ego_car";
        scans_pub.publish(lidar_msg);
        std::cout << "Publishing " << line_num << "\r" << std::endl;

        // ?tf2eigen https://blog.csdn.net/zz123456zzss/article/details/81914675
        Eigen::Quaterniond eigen_quat(pose_cam.block<3, 3>(0, 0).cast<double>());
        Eigen::Vector3d eigen_trans(pose_cam.block<3, 1>(0, 3).cast<double>());
        tf::Quaternion tf_quat;
        tf::Vector3 tf_trans;
        tf::quaternionEigenToTF(eigen_quat, tf_quat);
        tf::vectorEigenToTF(eigen_trans, tf_trans);
        tf::StampedTransform tf_map2scan(tf::Transform(tf_quat, tf_trans), ros::Time::now(), "world", "ego_car");
        ego_tf_broadcaster.sendTransform(tf_map2scan);

        Eigen::Vector3f lidar_pos(pose_cam.block<3, 1>(0, 3).cast<float>());
        pcl::PointCloud<PointType>::Ptr scan_global_frame(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*scan_global_coord, *scan_global_frame, pose_cam);

        voxelize_preserving_labels(scan_global_frame, *scan_global_frame, 1.0);
        *map += *scan_global_frame;
        voxelize_preserving_labels(map, *map, 1.0);

        sensor_msgs::PointCloud2 msg_ground;
        pcl::toROSMsg(*map, msg_ground);
        msg_ground.header.frame_id = "world";
        // map_pub.publish(msg_ground);
        ROS_INFO_STREAM(GREEN << "- sended map pcd: " << map->size());
        rate.sleep();
    }
    ROS_INFO_STREAM(GREEN << "- Saving map pcd: " << map->size());
    // pcl::transformPointCloud(*map, *map, t_cam_velo.inverse().cast<float>());
    map->width = map->size();
    map->height = 1;
    map->resize(map->width * map->height);
    pcl::io::savePCDFile(save_pcd_directory_, *map);

    return 0;
}