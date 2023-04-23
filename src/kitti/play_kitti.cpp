#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "kitti_function.hpp"

int main(int argc, char **argv)
{
    std::string velodyne_path = "/home/yzh/learning/SLAM/datas/kitti_dateset/sequences/00/velodyne";
    std::string calibration_file = "/home/yzh/learning/SLAM/datas/kitti_dateset/sequences/00/calib.txt";
    std::string time_file = "/home/yzh/learning/SLAM/datas/kitti_dateset/sequences/00/times.txt";
    std::string pose_file = "/home/yzh/learning/SLAM/datas/kitti_dateset/sequences/00/poses.txt";

    std::ifstream calibration_fin(calibration_file);
    std::string value;

    // Eigen::Matrix4d rotation_z = Eigen::Matrix4d::Identity();
    // rotation_z(0, 0) = 0;
    // rotation_z(0, 1) = 1;
    // rotation_z(1, 0) = -1;
    // rotation_z(1, 1) = 0;
    // rotation_z(2, 3) = 0;

    // Eigen::Matrix4d rotation_x = Eigen::Matrix4d::Identity();
    // rotation_x(1, 1) = 0;
    // rotation_x(2, 1) = -1;
    // rotation_x(1, 2) = 1;
    // rotation_x(2, 2) = 0;

    // 创建 AngleAxis 对象表示绕 X 轴旋转 -90 弧度
    Eigen::AngleAxisd rotation_vector_x(M_PI / 2, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d rotation_x = rotation_vector_x.toRotationMatrix();
    Eigen::Matrix4d transformation_matrix_x = Eigen::Matrix4d::Identity();
    transformation_matrix_x.block<3, 3>(0, 0) = rotation_x;
    // 创建 AngleAxis 对象表示绕 Z 轴旋转 180 弧度
    Eigen::AngleAxisd rotation_vector_z(M_PI / 2, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rotation_z = rotation_vector_z.toRotationMatrix();
    Eigen::Matrix4d transformation_matrix_z = Eigen::Matrix4d::Identity();
    transformation_matrix_z.block<3, 3>(0, 0) = rotation_z;

    // 定义平移向量
    Eigen::Translation3d translation_vector(0.0, 0.0, 1.73);
    Eigen::Matrix4d transformation_matrix_height = Eigen::Matrix4d::Identity();
    transformation_matrix_height.block<3, 1>(0, 3) = translation_vector.vector();

    // done: get cam_velo T
    for (int i = 0; i < 4; i++)
    {
        std::getline(calibration_fin, value);
    }
    Eigen::Matrix<double, 4, 3> matrix_in;
    Eigen::Matrix<double, 4, 4> t_cam_velo(Eigen::Matrix<double, 4, 4>::Identity());
    std::getline(calibration_fin, value, ' ');
    for (int i = 0; i < 11; i++)
    {
        std::getline(calibration_fin, value, ' ');
        matrix_in(i) = stof(value);
    }
    std::getline(calibration_fin, value, '\n');
    matrix_in(11) = stof(value);
    t_cam_velo.block<3, 4>(0, 0) = matrix_in.transpose();

    std::ifstream time_fin(time_file);
    std::ifstream pose_fin(pose_file);
    int line_num = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans_;
    std::vector<Eigen::Matrix4d> scan_poses_;
    while (std::getline(time_fin, value))
    {
        std::stringstream velo_ss;
        velo_ss << velodyne_path << "/" << std::setfill('0') << std::setw(6)
                << line_num << ".bin";

        int32_t num = 1000000;
        float *data = (float *)malloc(num * sizeof(float));

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
        Eigen::Matrix4d pose_velo = transformation_matrix_height * t_cam_velo.inverse() * pose_cam * transformation_matrix_x * transformation_matrix_z;
        pcl::transformPointCloud(*point_cloud, *point_cloud, pose_velo);

        scans_.push_back(point_cloud);
        scan_poses_.push_back(pose_velo);
        std::cout << line_num++ << '\r' << std::flush;
        if (line_num > 400)
        {
            break;
        }
    }

    ros::init(argc, argv, "kitti_node");
    ros::NodeHandle nh;
    ros::Publisher scans_pub = nh.advertise<sensor_msgs::PointCloud2>("scans_pub", 1);
    tf::TransformBroadcaster tf_broadcaster;
    ros::Rate rate(5);
    auto scans_size = scans_.size();
    int scan_idx = 0;
    while (ros::ok())
    {
        rate.sleep();
        if (scan_idx < scans_size - 1)
        {
            scan_idx++;
            auto ii_scan = scans_.at(scan_idx);
            auto ii_pose = scan_poses_.at(scan_idx);

            // ?calculate the rings for points
            add_rings_information(ii_scan);

            sensor_msgs::PointCloud2 lidar_msg;
            pcl::toROSMsg(*ii_scan, lidar_msg);
            lidar_msg.header.stamp = ros::Time::now();
            lidar_msg.header.frame_id = "world";
            scans_pub.publish(lidar_msg);

            // ?TF publishing
            tf::StampedTransform tf_map2ego = Matrix4dpose2TF(ii_pose);
            tf_broadcaster.sendTransform(tf_map2ego);

            std::cout << "Publishing" << std::endl;
        }
        else
        {
            scan_idx = 0;
            std::exit(0);
        }
    }
    return 0;
}