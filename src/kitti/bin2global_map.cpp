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

int main(int argc, char **argv)
{
    std::string velodyne_path = "/media/yzh/T7/Study/SLAM/Dataset/data_odometry_velodyne/dataset/sequences/05/velodyne";
    std::string calibration_file = "/media/yzh/T7/Study/SLAM/Dataset/data_odometry_calib/dataset/sequences/05/calib.txt";
    std::string time_file = "/media/yzh/T7/Study/SLAM/Dataset/data_odometry_calib/dataset/sequences/05/times.txt";
    std::string pose_file = "/media/yzh/T7/Study/SLAM/Dataset/odometry_ground_truth_poses/poses/05.txt";

    std::ifstream calibration_fin(calibration_file);
    std::string value;

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
        Eigen::Matrix4d pose_velo = pose_cam * t_cam_velo;
        pcl::transformPointCloud(*point_cloud, *point_cloud, pose_velo);
        *map += *point_cloud;
        std::cout << line_num++ << '\r' << std::flush;
        if (line_num > 400)
        {
            break;
        }
    }
    std::cout << std::endl;
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setLeafSize(.2, .2, .2);
    pcl::PointCloud<pcl::PointXYZI> filtered;
    sor.setInputCloud(map);
    sor.filter(filtered);
    *map = filtered;

    std::string save_pcd_directory_ = "/media/yzh/T7/bag_map/kitti/";
    std::string static_global_file_name = save_pcd_directory_ + "05_0_400.pcd";
    pcl::transformPointCloud(*map, *map, t_cam_velo.inverse().cast<float>());
    pcl::io::savePCDFile(static_global_file_name, *map);
    return 0;
}