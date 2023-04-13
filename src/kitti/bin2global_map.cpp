#include <iostream>
#include <string>
#include <filesystem> // requires gcc version >= 8
#include <vector>
#include <ros/ros.h>
#include <fstream>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include "hash_map.hpp"

using namespace std;
namespace fs = std::filesystem;
using PointType = pcl::PointXYZI;

std::vector<std::string> sequence_scan_names_;
std::vector<std::string> sequence_scan_paths_;
int num_total_scans_of_sequence_;
std::vector<Eigen::Matrix4d> sequence_scan_poses_;
std::vector<Eigen::Matrix4d> sequence_scan_inverse_poses_; // used for global to local
float kDownsampleVoxelSize = 0.1;
// target region to removerting
int start_idx_ = 2350;
int end_idx_ = 2670;
bool use_keyframe_gap_ = false;
int keyframe_gap_ = 1;
std::vector<std::string> sequence_valid_scan_names_;
std::vector<std::string> sequence_valid_scan_paths_;
std::vector<pcl::PointCloud<PointType>::Ptr> scans_;
std::vector<pcl::PointCloud<PointType>::Ptr> scans_static_;
std::vector<pcl::PointCloud<PointType>::Ptr> scans_dynamic_;
std::vector<Eigen::Matrix4d> scan_poses_;
std::vector<Eigen::Matrix4d> scan_inverse_poses_;
// sequence bin files
bool isScanFileKITTIFormat_ = true;
pcl::PointCloud<PointType>::Ptr map_global_orig_;
pcl::PointCloud<PointType>::Ptr map_global_curr_; // the M_i. i.e., removert is: M1 -> S1 + D1, D1 -> M2 , M2 -> S2 + D2 ... repeat ...
std::vector<double> kVecExtrinsicLiDARtoPoseBase =
    {-1.857e-03, -9.999e-01, -8.039e-03, -4.784e-03,
     -6.481e-03, 8.0518e-03, -9.999e-01, -7.337e-02,
     9.999e-01, -1.805e-03, -6.496e-03, -3.339e-01,
     0.0, 0.0, 0.0, 1.0};
// std::vector<double> kVecExtrinsicLiDARtoPoseBase = {1.0, 0.0, 0.0, 0.0,
//                                                     0.0, 1.0, 0.0, 0.0,
//                                                     0.0, 0.0, 1.0, 0.0,
//                                                     0.0, 0.0, 0.0, 1.0};
Eigen::Matrix4d kSE3MatExtrinsicLiDARtoPoseBase = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(kVecExtrinsicLiDARtoPoseBase.data(), 4, 4);
bool kFlagSaveMapPointcloud = true;
std::string save_pcd_directory_ = "/media/yzh/T7/bag_map/kitti/";
// function
std::vector<double> splitPoseLine(std::string _str_line, char _delimiter);
void parseValidScanInfo();
void readValidScans();
void makeGlobalMap(void);

int main(int argc, char *argv[])
{
    map_global_orig_.reset(new pcl::PointCloud<PointType>());
    map_global_curr_.reset(new pcl::PointCloud<PointType>());
    cout << "kSE3MatExtrinsicLiDARtoPoseBase: " << kSE3MatExtrinsicLiDARtoPoseBase << endl;

    // bin
    // string sequence_scan_dir_ = "/media/robot-nuc12/T7/Study/SLAM/Dataset/own/bin/";
    string sequence_scan_dir_ = "/media/yzh/T7/Study/SLAM/Dataset/data_odometry_velodyne/dataset/sequences/05/velodyne/";
    for (auto &_entry : fs::directory_iterator(sequence_scan_dir_))
    {
        sequence_scan_names_.emplace_back(_entry.path().filename());
        sequence_scan_paths_.emplace_back(_entry.path());
    }
    std::sort(sequence_scan_names_.begin(), sequence_scan_names_.end());
    std::sort(sequence_scan_paths_.begin(), sequence_scan_paths_.end());
    for (size_t i = 0; i < sequence_scan_names_.size(); i++)
    {
        cout << sequence_scan_names_.at(i) << " ";
    }
    num_total_scans_of_sequence_ = sequence_scan_paths_.size();
    ROS_INFO_STREAM("\033[1;32m Total : " << num_total_scans_of_sequence_ << " scans in the directory.\033[0m");

    // pose
    // string sequence_pose_path_ = "/media/robot-nuc12/T7/Study/SLAM/Dataset/own/bin/poses.txt";
    string sequence_pose_path_ = "/media/yzh/T7/Study/SLAM/Dataset/data_odometry_labels/dataset/sequences/05/poses.txt";
    std::ifstream pose_file_handle(sequence_pose_path_);
    int num_poses{0};
    std::string strOneLine;

    // TODO: rotate to world(for kitti)
    Eigen::Matrix4d rotation_z = Eigen::Matrix4d::Identity();
    rotation_z(0, 0) = 0;
    rotation_z(0, 1) = 1;
    rotation_z(1, 0) = -1;
    rotation_z(1, 1) = 0;
    rotation_z(2, 3) = 1.73;

    Eigen::Matrix4d rotation_x = Eigen::Matrix4d::Identity();
    rotation_x(1, 1) = 0;
    rotation_x(2, 1) = -1;
    rotation_x(1, 2) = 1;
    rotation_x(2, 2) = 0;

    while (getline(pose_file_handle, strOneLine))
    {
        // str to vec
        std::vector<double> ith_pose_vec = splitPoseLine(strOneLine, ' ');
        if (ith_pose_vec.size() == 12)
        {
            ith_pose_vec.emplace_back(double(0.0));
            ith_pose_vec.emplace_back(double(0.0));
            ith_pose_vec.emplace_back(double(0.0));
            ith_pose_vec.emplace_back(double(1.0));
        }

        // vec to eig
        Eigen::Matrix4d ith_pose = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(ith_pose_vec.data(), 4, 4);
        Eigen::Matrix4d ith_pose_inverse = ith_pose.inverse();
        ith_pose = rotation_z * rotation_x * ith_pose;
        sequence_scan_poses_.emplace_back(ith_pose);
        sequence_scan_inverse_poses_.emplace_back(ith_pose_inverse);

        num_poses++;
    }
    cout << "num_poses = " << num_poses << endl;

    // global map
    // load scan and poses
    parseValidScanInfo();
    readValidScans();
    makeGlobalMap();

    ros::init(argc, argv, "globalmap");
    ros::NodeHandle nh;
    ros::Publisher pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("globalmap", 1);
    ros::Publisher scans_pub = nh.advertise<sensor_msgs::PointCloud2>("scans_pub", 1);
    // FIXME: my mapp
    dynamic_mapping::DiscreteSpace space{{-3000, -3000, -3000}, {3000, 3000, 3000}, nh};

    ros::Rate rate(5);
    int scan_idx = 0;
    while (ros::ok())
    {
        sensor_msgs::PointCloud2 lidar_msg;
        pcl::toROSMsg(*map_global_curr_, lidar_msg);
        lidar_msg.header.stamp = ros::Time::now();
        lidar_msg.header.frame_id = "world";
        pcd_pub.publish(lidar_msg);
        auto scans_size = scans_.size();
        if (scan_idx < scans_size - 1)
        {
            scan_idx++;
            auto ii_scan = scans_.at(scan_idx);      // pcl::PointCloud<PointType>::Ptr
            auto ii_pose = scan_poses_.at(scan_idx); // Eigen::Matrix4d
            // local to global (local2global)
            pcl::PointCloud<PointType>::Ptr scan_global_coord(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*ii_scan, *scan_global_coord, kSE3MatExtrinsicLiDARtoPoseBase);
            pcl::transformPointCloud(*scan_global_coord, *scan_global_coord, ii_pose);

            pcl::PointCloud<pcl::PointXYZ>::Ptr my_scan(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::copyPointCloud(*scan_global_coord, *my_scan);
            pcl::PointXYZ lidarLocation{ii_pose(0, 3), ii_pose(1, 3), ii_pose(2, 3)};
            space.addPointCloud(*my_scan, lidarLocation);

            sensor_msgs::PointCloud2 lidar_msg;
            pcl::toROSMsg(*scan_global_coord, lidar_msg);
            lidar_msg.header.stamp = ros::Time::now();
            lidar_msg.header.frame_id = "camera_init";
            scans_pub.publish(lidar_msg);

            // 进度条
            float progress = scan_idx / float(scans_size) * 100;
            cout << "\r"
                 << "Hash_map building: ["; // \r 意思是回车
            for (int j = 0; j < progress; j += 2)
            {
                cout << "#";
            }
            for (int j = progress; j < 100; j += 2)
            {
                cout << "-";
            }
            cout << "] " << progress << "%" << flush;
        }
        else
        {
            std::string static_global_file_name = save_pcd_directory_ + "hash_map.pcd";
            pcl::PointCloud<pcl::PointXYZ>::Ptr global_hash_map(new pcl::PointCloud<pcl::PointXYZ>());
            space.sendMessageToRviz();
            for (auto &[_, region] : space.regions)
            {
                region.genPointCloud();
                *global_hash_map += region.get_ground_PointCloud();
                *global_hash_map += region.get_nonground_PointCloud();
            }
            ROS_INFO_STREAM("\033[1;32m Size " << global_hash_map->size()
                                               << "\033[0m");
            pcl::io::savePCDFileBinary(static_global_file_name, *global_hash_map);
            ROS_INFO_STREAM("\033[1;32m The original pointcloud is saved (global coord): " << static_global_file_name << "\033[0m");
            return 0;
        }
    }

    return 0;
}

std::vector<double> splitPoseLine(std::string _str_line, char _delimiter)
{
    std::vector<double> parsed;
    std::stringstream ss(_str_line);
    std::string temp;
    while (getline(ss, temp, _delimiter))
    {
        parsed.push_back(std::stod(temp)); // convert string to "double"
    }
    return parsed;
}

void parseValidScanInfo(void)
{
    int num_valid_parsed{0};
    float movement_counter{0.0};

    for (int curr_idx = 0; curr_idx < int(sequence_scan_paths_.size()); curr_idx++)
    {
        // check the scan idx within the target idx range
        if (curr_idx > end_idx_ || curr_idx < start_idx_)
        {
            curr_idx++;
            continue;
        }

        // check enough movement occured (e.g., parse every 2m)
        if (use_keyframe_gap_)
        {
            if (remainder(num_valid_parsed, keyframe_gap_) != 0)
            {
                num_valid_parsed++;
                continue;
            }
        }

        // save the info (reading scan bin is in makeGlobalMap)
        sequence_valid_scan_paths_.emplace_back(sequence_scan_paths_.at(curr_idx));
        sequence_valid_scan_names_.emplace_back(sequence_scan_names_.at(curr_idx));

        scan_poses_.emplace_back(sequence_scan_poses_.at(curr_idx));                 // used for local2global
        scan_inverse_poses_.emplace_back(sequence_scan_inverse_poses_.at(curr_idx)); // used for global2local

        //
        num_valid_parsed++;
    }

    if (use_keyframe_gap_)
    {
        ROS_INFO_STREAM("\033[1;32m Total " << sequence_valid_scan_paths_.size()
                                            << " nodes are used from the index range [" << start_idx_ << ", " << end_idx_ << "]"
                                            << " (every " << keyframe_gap_ << " frames parsed)\033[0m");
    }
    else
    {
        ROS_INFO_STREAM("\033[1;32m Total " << sequence_valid_scan_paths_.size()
                                            << " nodes are used from the index range [" << start_idx_ << ", " << end_idx_ << "]"
                                            << " (every " << keyframe_gap_ << " frames parsed)\033[0m");
    }
} // parseValidScanInfo

void readBin(std::string _bin_path, pcl::PointCloud<PointType>::Ptr _pcd_ptr)
{
    std::fstream input(_bin_path.c_str(), ios::in | ios::binary);
    if (!input.good())
    {
        cerr << "Could not read file: " << _bin_path << endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, ios::beg);

    for (int ii = 0; input.good() && !input.eof(); ii++)
    {
        PointType point;

        input.read((char *)&point.x, sizeof(float));
        input.read((char *)&point.y, sizeof(float));
        input.read((char *)&point.z, sizeof(float));
        input.read((char *)&point.intensity, sizeof(float));

        _pcd_ptr->push_back(point);
    }
    input.close();
}

void readValidScans(void)
// for target range of scan idx
{
    const int cout_interval{10};
    int cout_counter{0};

    for (auto &_scan_path : sequence_valid_scan_paths_)
    {
        // read bin files and save
        pcl::PointCloud<PointType>::Ptr points(new pcl::PointCloud<PointType>); // pcl::PointCloud Ptr is a shared ptr so this points will be automatically destroyed after this function block (because no others ref it).
        if (isScanFileKITTIFormat_)
        {
            readBin(_scan_path, points); // For KITTI (.bin)
        }
        else
        {
            pcl::io::loadPCDFile<PointType>(_scan_path, *points); // saved from SC-LIO-SAM's pcd binary (.pcd)
        }

        // pcdown
        pcl::VoxelGrid<PointType> downsize_filter;
        downsize_filter.setLeafSize(kDownsampleVoxelSize, kDownsampleVoxelSize, kDownsampleVoxelSize);
        downsize_filter.setInputCloud(points);

        pcl::PointCloud<PointType>::Ptr downsampled_points(new pcl::PointCloud<PointType>);
        downsize_filter.filter(*downsampled_points);

        // save downsampled pointcloud
        scans_.emplace_back(downsampled_points);

        // cout for debug
        cout_counter++;
        if (remainder(cout_counter, cout_interval) == 0)
        {
            cout << _scan_path << endl;
            cout << "Read a pointcloud with " << points->points.size() << " points." << endl;
            cout << "downsample the pointcloud: " << downsampled_points->points.size() << " points." << endl;
            cout << " ... (display every " << cout_interval << " readings) ..." << endl;
        }
    }
    cout << endl;
} // readValidScans

void mergeScansWithinGlobalCoord(
    const std::vector<pcl::PointCloud<PointType>::Ptr> &_scans,
    const std::vector<Eigen::Matrix4d> &_scans_poses,
    pcl::PointCloud<PointType>::Ptr &_ptcloud_to_save)
{
    // NOTE: _scans must be in local coord
    for (std::size_t scan_idx = 0; scan_idx < _scans.size(); scan_idx++)
    {
        auto ii_scan = _scans.at(scan_idx);       // pcl::PointCloud<PointType>::Ptr
        auto ii_pose = _scans_poses.at(scan_idx); // Eigen::Matrix4d

        // local to global (local2global)
        pcl::PointCloud<PointType>::Ptr scan_global_coord(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*ii_scan, *scan_global_coord, kSE3MatExtrinsicLiDARtoPoseBase);
        pcl::transformPointCloud(*scan_global_coord, *scan_global_coord, ii_pose);

        // merge the scan into the global map
        *_ptcloud_to_save += *scan_global_coord;
    }
} // mergeScansWithinGlobalCoord

void octreeDownsampling(const pcl::PointCloud<PointType>::Ptr &_src, pcl::PointCloud<PointType>::Ptr &_to_save)
{
    pcl::octree::OctreePointCloudVoxelCentroid<PointType> octree(kDownsampleVoxelSize);
    octree.setInputCloud(_src);
    octree.defineBoundingBox();
    octree.addPointsFromInputCloud();
    pcl::octree::OctreePointCloudVoxelCentroid<PointType>::AlignedPointTVector centroids;
    octree.getVoxelCentroids(centroids);

    // init current map with the downsampled full cloud
    _to_save->points.assign(centroids.begin(), centroids.end());
    _to_save->width = 1;
    _to_save->height = _to_save->points.size(); // make sure again the format of the downsampled point cloud
    ROS_INFO_STREAM("\033[1;32m Downsampled pointcloud have: " << _to_save->points.size() << " points.\033[0m");
    cout << endl;
} // octreeDownsampling

void makeGlobalMap(void)
{
    // transform local to global and merging the scans
    map_global_orig_->clear();
    map_global_curr_->clear();

    mergeScansWithinGlobalCoord(scans_, scan_poses_, map_global_orig_);
    ROS_INFO_STREAM("\033[1;32m Map pointcloud (having redundant points) have: " << map_global_orig_->points.size() << " points.\033[0m");
    ROS_INFO_STREAM("\033[1;32m Downsampling leaf size is " << kDownsampleVoxelSize << " m.\033[0m");

    // remove repeated (redundant) points
    // - using OctreePointCloudVoxelCentroid for downsampling
    // - For a large-size point cloud should use OctreePointCloudVoxelCentroid rather VoxelGrid
    octreeDownsampling(map_global_orig_, map_global_curr_);
    // save the original cloud
    if (kFlagSaveMapPointcloud)
    {
        // in global coord
        std::string static_global_file_name = save_pcd_directory_ + "OriginalNoisyMapGlobal.pcd";
        pcl::io::savePCDFileBinary(static_global_file_name, *map_global_curr_);
        ROS_INFO_STREAM("\033[1;32m The original pointcloud is saved (global coord): " << static_global_file_name << "\033[0m");
    }
} // makeGlobalMap
