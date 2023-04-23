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
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "kitti_function.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kitti_map_node");
    ros::NodeHandle nh;
    ros::Publisher kitti_map_pub = nh.advertise<sensor_msgs::PointCloud2>("kitti_map_pub", 1);
    ros::Rate rate(10);
    tf::TransformBroadcaster tf_broadcaster;

    std::string velodyne_path = "/home/yzh/learning/SLAM/datas/kitti_dateset/sequences/00/velodyne";
    std::string calibration_file = "/home/yzh/learning/SLAM/datas/kitti_dateset/sequences/00/calib.txt";
    std::string time_file = "/home/yzh/learning/SLAM/datas/kitti_dateset/sequences/00/times.txt";
    std::string pose_file = "/home/yzh/learning/SLAM/datas/kitti_dateset/sequences/00/poses.txt";
    std::ifstream calibration_fin(calibration_file);
    std::string value;

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
    Eigen::Translation3d translation_vector(0.0, 0.0, 0.0);
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_pub(new pcl::PointCloud<pcl::PointXYZI>);
    while (std::getline(time_fin, value))
    {
        std::stringstream velo_ss;
        velo_ss << velodyne_path << "/" << std::setfill('0') << std::setw(6)
                << line_num << ".bin";
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
        // Eigen::Matrix4d pose_velo = pose_cam * t_cam_velo;
        Eigen::Matrix4d pose_velo = pose_cam * t_cam_velo;
        pcl::transformPointCloud(*point_cloud, *point_cloud, pose_velo);
        std::cout << line_num++ << '\r' << std::flush;

        *map += *point_cloud;
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setLeafSize(.5, .5, .5);
        pcl::PointCloud<pcl::PointXYZI> filtered;
        sor.setInputCloud(map);
        sor.filter(*map);

        pcl::transformPointCloud(*map, *map_pub, t_cam_velo.inverse().cast<float>());
        // if (line_num > 300)
        // {
        //     break;
        // }

        sensor_msgs::PointCloud2 lidar_msg;
        pcl::toROSMsg(*map_pub, lidar_msg);
        lidar_msg.header.stamp = ros::Time::now();
        lidar_msg.header.frame_id = "world";
        kitti_map_pub.publish(lidar_msg);

        // DONE: TF publishing
        Eigen::Matrix4d pose_pub = transformation_matrix_height * t_cam_velo.inverse() * pose_cam * transformation_matrix_x * transformation_matrix_z;
        tf::StampedTransform tf_map2ego = Matrix4dpose2TF(pose_pub);
        tf_broadcaster.sendTransform(tf_map2ego);
        rate.sleep();
    }
    std::cout << std::endl;
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setLeafSize(.4, .4, .4);
    pcl::PointCloud<pcl::PointXYZI> filtered;
    sor.setInputCloud(map);
    sor.filter(filtered);
    *map = filtered;

    std::string save_pcd_directory_ = "/home/yzh/learning/SLAM/datas/pcd_map/";
    std::string static_global_file_name = save_pcd_directory_ + "00_0_400.pcd";
    pcl::transformPointCloud(*map, *map, t_cam_velo.inverse().cast<float>());
    pcl::io::savePCDFile(static_global_file_name, *map);
    return 0;
}