#include <vector>
#include <filesystem> // requires gcc version >= 8
#include <string>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <fstream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <tf_conversions/tf_eigen.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using     PointType = pcl::PointXYZI;
namespace fs        = std::filesystem;

#define CAR_BODY_SIZE 2.7
#define RED "\033[31m"
#define GREEN "\033[32m"
#define DEBUG "\033[33m"

std::vector<int> DYNAMIC_CLASSES = {251, 252, 253, 254, 255, 256, 257, 258, 259};

void load_cam_to_vel(string calibration_file, Eigen::Matrix<double, 4, 4> &t_cam_velo)
{
    std::ifstream calibration_fin(calibration_file);
    std::string value;
    for (int i = 0; i < 4; i++)
    {
        std::getline(calibration_fin, value);
    }
    Eigen::Matrix<double, 4, 3> matrix_in;
    t_cam_velo = Eigen::Matrix<double, 4, 4>::Identity();
    std::getline(calibration_fin, value, ' ');
    for (int i = 0; i < 11; i++)
    {
        std::getline(calibration_fin, value, ' ');
        matrix_in(i) = stof(value);
    }
    std::getline(calibration_fin, value, '\n');
    matrix_in         (11)       = stof(value);
    t_cam_velo.block<3, 4>(0, 0) = matrix_in.transpose();
}

std::vector<double> splitPoseLine(std::string _str_line, char _delimiter)
{
    std::vector<double> parsed;
    std::stringstream ss(_str_line);
    std::string temp;
    while (getline(ss, temp, _delimiter))
    {
        parsed.push_back(std::stod(temp));  // convert string to "double"
    }
    return parsed;
}

void transform_pose_to_world(Eigen::Matrix4d &ith_pose, Eigen::Matrix<double, 4, 4> t_cam_velo)
{
      // 创建 AngleAxis 对象表示绕 X 轴旋转 -90 弧度
    Eigen::AngleAxisd rotation_vector_x(M_PI / 2, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d                 rotation_x              = rotation_vector_x.toRotationMatrix();
    Eigen::Matrix4d                 transformation_matrix_x = Eigen::Matrix4d::Identity();
    transformation_matrix_x.block<3, 3>(0, 0)               = rotation_x;
      // 创建 AngleAxis 对象表示绕 Z 轴旋转 180 弧度
    Eigen::AngleAxisd rotation_vector_z(M_PI / 2, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d                 rotation_z              = rotation_vector_z.toRotationMatrix();
    Eigen::Matrix4d                 transformation_matrix_z = Eigen::Matrix4d::Identity();
    transformation_matrix_z.block<3, 3>(0, 0)               = rotation_z;

    ith_pose = t_cam_velo.inverse() * ith_pose * transformation_matrix_x * transformation_matrix_z;
}

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

void load_label(std::string _label_path, pcl::shared_ptr<pcl::PointCloud<PointType>> _pcd_ptr)
{
      // ##################### load label ########################
    std::ifstream inputfile;
    inputfile.open(_label_path.c_str(), ios::binary);

    vector<uint32_t> label;
    if (!inputfile)
    {
        cerr << "ERROR: Cannot open file " << _label_path
             << "! Aborting..." << endl;
    }
    inputfile.seekg(0, ios::beg);  // 将文件读取指针移动到文件的开头位置
    for (int i = 0; inputfile.good() && !inputfile.eof(); i++)
    {
        uint32_t data;
        inputfile.read(reinterpret_cast<char *>(&data), sizeof(data));
        label.push_back(data);
    }

    vector<uint16_t> instance;
    vector<uint16_t> semantic_label;

    for (int i = 0; i < label.size(); ++i)
    {
        uint16_t slabel = label[i] & 0xFFFF;
        uint16_t inst   = label[i] >> 16;
        instance.push_back(inst);
        semantic_label.push_back(slabel);
        bool is_static = true;
        for (int class_num : DYNAMIC_CLASSES)
        {
            if (slabel == class_num)
            { // 1. check it is in the moving object classes
                _pcd_ptr->points[i].intensity = 63;
                                 is_static    = false;
            }
        }
        if (is_static)
        {
            _pcd_ptr->points[i].intensity = 0;
        }
    }
}

void load_points_in_certain_angle(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out)
{
    for (auto &pt : cloud_in->points)
    {
        auto angle = std::atan2(pt.y, pt.x) * 180 / M_PI;
          // if (angle > 15 || angle < -15)
          //     // if (std::atan2(pt.y, pt.x) * 180 / M_PI < 165 || std::atan2(pt.y, pt.x) * 180 / M_PI > -165)
          //     continue;
        if (hypot(pt.y, pt.x) > 50)
            continue;
        cloud_out->points.push_back(pt);
    }
}

void remove_inner_points(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out, double min_distance = CAR_BODY_SIZE)
{
      // To remove some noisy points in the vicinity of the vehicles
    for (auto const &pt : cloud_in->points)
    {
        double dist_square = std::hypot(pt.x, pt.y);
        if (dist_square < min_distance)
        {
            continue;
        }
        else
        {
            cloud_out->points.push_back(pt);
        }
    }
}

void read_scans_and_poses(int start_idx_,
                          int end_idx_,
                          std::string sequence_scan_dir_in,
                          std::string sequence_pose_path_,
                          std::vector<pcl::PointCloud<PointType>::Ptr> &scans_,
                          std::vector<Eigen::Matrix4d> &scan_poses_,
                          float kDownsampleVoxelSize = 0.1,
                          bool use_keyframe_gap_ = false,
                          int keyframe_gap_ = 1)
{
    std::vector<std::string> sequence_scan_paths_;
    for (auto file : fs::directory_iterator(sequence_scan_dir_in))
    {
        sequence_scan_paths_.emplace_back(file.path());
    }
    std::sort(sequence_scan_paths_.begin(), sequence_scan_paths_.end());
    // read poses
    std::vector<Eigen::Matrix4d> sequence_scan_poses_; // all poses
    std::ifstream pose_file_handle(sequence_pose_path_);
    std::string strOneLine;
    int num_poses{0};
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
        ith_pose = rotation_z * rotation_x * ith_pose;
        sequence_scan_poses_.emplace_back(ith_pose);
        num_poses++;
    }
    cout << "num_poses = " << num_poses << endl;

    // ?parseValidScanInfo
    int num_valid_parsed{0};
    float movement_counter{0.0};
    std::vector<std::string> sequence_valid_scan_paths_;
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
        scan_poses_.emplace_back(sequence_scan_poses_.at(curr_idx)); // used for local2global

        //
        num_valid_parsed++;
    }

    if (use_keyframe_gap_)
    {
        std::cout << "\033[1;32m Total " << sequence_valid_scan_paths_.size()
                  << " nodes are used from the index range [" << start_idx_ << ", " << end_idx_ << "]"
                  << " (every " << keyframe_gap_ << " frames parsed)\033[0m";
    }
    else
    {
        std::cout << "\033[1;32m Total " << sequence_valid_scan_paths_.size()
                  << " nodes are used from the index range [" << start_idx_ << ", " << end_idx_ << "]"
                  << " (every " << keyframe_gap_ << " frames parsed)\033[0m";
    }

    // ?readValidScans;
    const int cout_interval{10};
    int cout_counter{0};

    for (auto &_scan_path : sequence_valid_scan_paths_)
    {
        // read bin files and save
        pcl::PointCloud<PointType>::Ptr points(new pcl::PointCloud<PointType>); // pcl::PointCloud Ptr is a shared ptr so this points will be automatically destroyed after this function block (because no others ref it).

        readBin(_scan_path, points); // For KITTI (.bin)

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
}

void read_scans_and_poses(int start_idx_,
                          int end_idx_,
                          std::string sequence_scan_dir_in,
                          std::string sequence_pose_path_,
                          std::string sequence_label_path_,
                          std::string sequence_calibration_path_,
                          std::vector<pcl::PointCloud<PointType>::Ptr> &scans_,
                          std::vector<Eigen::Matrix4d> &scan_poses_,
                          float kDownsampleVoxelSize = 0.1,
                          bool  use_keyframe_gap_    = false,
                          int   keyframe_gap_        = 1)
{
    Eigen::Matrix<double, 4, 4> t_cam_velo;
    load_cam_to_vel(sequence_calibration_path_, t_cam_velo);

    std::vector<std::string> sequence_scan_paths_;
    std::vector<std::string> sequence_label_paths_;
    for (auto file : fs::directory_iterator(sequence_scan_dir_in))
    {
        sequence_scan_paths_.emplace_back(file.path());
    }
    for (auto file : fs::directory_iterator(sequence_label_path_))
    {
        sequence_label_paths_.emplace_back(file.path());
    }
    std::sort(sequence_scan_paths_.begin(), sequence_scan_paths_.end());
    std::sort(sequence_label_paths_.begin(), sequence_label_paths_.end());
    std::cout << "\033[1;32m Read scans and labels size: " << sequence_scan_paths_.size()
              << " and " << sequence_label_paths_.size() << "\033[0m" << std::endl;
    if (sequence_scan_paths_.size() != sequence_label_paths_.size())
    {
        std::cerr << RED
                  << "scans and labels not match" << std::endl;
    }

    // read poses
    std::vector<Eigen::Matrix4d> sequence_scan_poses_; // all poses
    std::ifstream pose_file_handle(sequence_pose_path_);
    std::string strOneLine;
    int num_poses{0};
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
        transform_pose_to_world(ith_pose, t_cam_velo);
        sequence_scan_poses_.emplace_back(ith_pose);
        num_poses++;
    }

    // ?parseValidScanInfo
    int num_valid_parsed{0};
    float movement_counter{0.0};
    std::vector<std::string> sequence_valid_scan_paths_;
    std::vector<std::string> sequence_valid_label_paths_;
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
        sequence_valid_label_paths_.emplace_back(sequence_label_paths_.at(curr_idx));
        scan_poses_.emplace_back(sequence_scan_poses_.at(curr_idx)); // used for local2global

        //
        num_valid_parsed++;
    }

    if (use_keyframe_gap_)
    {
        std::cout << "\033[1;32m Total " << sequence_valid_scan_paths_.size()
                  << " nodes are used from the index range [" << start_idx_ << ", " << end_idx_ << "]"
                  << " (every " << keyframe_gap_ << " frames parsed)\033[0m";
    }
    else
    {
        std::cout << "\033[1;32m Total " << sequence_valid_scan_paths_.size()
                  << " nodes are used from the index range [" << start_idx_ << ", " << end_idx_ << "]"
                  << " (every " << keyframe_gap_ << " frames parsed)\033[0m" << std::endl;
    }

    // ?readValidScans;
    const int cout_interval{10};
    int cout_counter{0};
    scans_.clear();
    for (int curr_idx = 0; curr_idx < int(sequence_valid_scan_paths_.size()); curr_idx++)
    {
        // read bin files and save
        auto _scan_path = sequence_valid_scan_paths_[curr_idx];
        auto _label_path = sequence_valid_label_paths_[curr_idx];
        pcl::PointCloud<PointType>::Ptr points(new pcl::PointCloud<PointType>);     // pcl::PointCloud Ptr is a shared ptr so this points will be automatically destroyed after this function block (because no others ref it).
        pcl::PointCloud<PointType>::Ptr points_out(new pcl::PointCloud<PointType>); // pcl::PointCloud Ptr is a shared ptr so this points will be automatically destroyed after this function block (because no others ref it).

        readBin(_scan_path, points);     // For KITTI (.bin)
        load_label(_label_path, points); // For KITTI (.label)
        remove_inner_points(points, points_out);

        pcl::PointCloud<PointType>::Ptr points_in_certain_angle(new pcl::PointCloud<PointType>);
        load_points_in_certain_angle(points_out, points_in_certain_angle);
        scans_.emplace_back(points_in_certain_angle);

        // // pcdown
        // pcl::VoxelGrid<PointType> downsize_filter;
        // downsize_filter.setLeafSize(kDownsampleVoxelSize, kDownsampleVoxelSize, kDownsampleVoxelSize);
        // downsize_filter.setInputCloud(points);
        // pcl::PointCloud<PointType>::Ptr downsampled_points(new pcl::PointCloud<PointType>);
        // downsize_filter.filter(*downsampled_points);

        // save downsampled pointcloud
        // scans_.emplace_back(points_out);

        // cout for debug
        cout_counter++;
        // if (remainder(cout_counter, cout_interval) == 0)
        // {
        //     cout << _scan_path << endl;
        //     cout << "Read a pointcloud with " << points_out->points.size() << " points." << endl;
        //     cout << "downsample the pointcloud: " << points_out->points.size() << " points." << endl;
        //     cout << " ... (display every " << cout_interval << " readings) ..." << endl;
        // }
    }
    std::cout << GREEN << "Scan size: " << scans_.size() << std::endl;
}

void add_rings_information(pcl::shared_ptr<pcl::PointCloud<PointType>> &ii_scan)
{
    // 计算每个点的ring值
    float fov_down = -24.9 / 180.0 * M_PI;
    float fov = (abs(-24.9) + abs(2.0)) / 180.0 * M_PI;
    int max = 0;
    int min = 10;
    for (auto &pt : *ii_scan)
    {
        float distance = std::hypot(pt.x, pt.y);
        float vertical_angle = std::atan2(pt.z, distance);
        int ring = (vertical_angle + abs(fov_down)) / fov * 64;
        ring = ring > 63 ? 63 : ring;
        // ! kitti 数据集挺多数据打到地面的下面去了
        ring = ring < 0 ? 0 : ring;
        pt.intensity = ring;
    }
}

tf::StampedTransform Matrix4dpose2TF(Eigen::Matrix4d &ego_car_link)
{
    Eigen::Quaterniond eigen_quat(ego_car_link.block<3, 3>(0, 0).cast<double>());
    Eigen::Vector3d eigen_trans(ego_car_link.block<3, 1>(0, 3).cast<double>());
    tf::Quaternion tf_quat;
    tf::Vector3 tf_trans;
    tf::quaternionEigenToTF(eigen_quat, tf_quat);
    tf::vectorEigenToTF(eigen_trans, tf_trans);
    tf::StampedTransform tf_map2ego(tf::Transform(tf_quat, tf_trans), ros::Time::now(), "world", "ego_car");
    return tf_map2ego;
}

void pointcloud_ROI(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out)
{
    // DONE: remove in range of 0.5m
    //* remove in range of 0.5m
    for (size_t i = 0; i < cloud_in->points.size(); i++)
    {
        auto point = cloud_in->points.at(i);
        point.z = point.z;
        if (point.z < -3)
            continue;
        if (std::hypot(point.x, point.y, point.z) < 2)
        {
            continue;
        }
        // if (std::atan2(point.y, point.x) * 180 / M_PI > 15 || std::atan2(point.y, point.x) * 180 / M_PI < -15)
        //     continue;
        cloud_out->points.emplace_back(point);
    }
    cout << "scan_global_coord size: " << cloud_out->size() << endl;
    // done: ring
    std::array<float, 360> roi_distances_max; // to filter out not roi points
    roi_distances_max.fill(100);
    float fov_down = -24.9 / 180.0 * M_PI;
    float fov = 26.9 / 180.0 * M_PI;
    for (auto &pt : cloud_out->points)
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
    for (auto &pt : cloud_out->points)
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
}

// ?range images generation
struct SphericalPoint
{
    float az; // azimuth
    float el; // elevation
    float r;  // radius
};

SphericalPoint cart2sph(const PointType & _cp)
{ // _cp means cartesian point

    SphericalPoint sph_point {
         std::atan2(_cp.y, _cp.x), 
         std::atan2(_cp.z, std::sqrt(_cp.x*_cp.x + _cp.y*_cp.y)),
         std::sqrt(_cp.x*_cp.x + _cp.y*_cp.y + _cp.z*_cp.z)
    };    
    return sph_point;
}

inline float rad2deg(float radians) 
{ 
    return radians * 180.0 / M_PI; 
}

std::pair<int, int> resetRimgSize(const std::pair<float, float> _fov, const float _resize_ratio)
{
    // default is 1 deg x 1 deg 
    float alpha_vfov = _resize_ratio;    
    float alpha_hfov = _resize_ratio;    

    float V_FOV = _fov.first;
    float H_FOV = _fov.second;

    int NUM_RANGE_IMG_ROW = std::round(V_FOV*alpha_vfov);
    int NUM_RANGE_IMG_COL = std::round(H_FOV*alpha_hfov);

    std::pair<int, int> rimg {NUM_RANGE_IMG_ROW, NUM_RANGE_IMG_COL};
    return rimg;
}

const float kFlagNoPOINT = 10000.0; // no point constant, 10000 has no meaning, but must be larger than the maximum scan range (e.g., 200 meters)

cv::Mat scan2RangeImg(const pcl::PointCloud<PointType>::Ptr& _scan, 
                      const std::pair<float, float> _fov, /* e.g., [vfov = 50 (upper 25, lower 25), hfov = 360] */
                      const std::pair<int, int> _rimg_size)
{
    const float kVFOV = _fov.first;
    const float kHFOV = _fov.second;
    
    const int kNumRimgRow = _rimg_size.first;
    const int kNumRimgCol = _rimg_size.second;
    // cout << "rimg size is: [" << _rimg_size.first << ", " << _rimg_size.second << "]." << endl;

    // @ range image initizliation 
    cv::Mat rimg = cv::Mat(kNumRimgRow, kNumRimgCol, CV_32FC1, cv::Scalar::all(kFlagNoPOINT)); // float matrix

    // @ points to range img 
    int num_points = _scan->points.size();
    // #pragma omp parallel for num_threads(4)
    for (int pt_idx = 0; pt_idx < num_points; ++pt_idx)
    {   
        PointType this_point = _scan->points[pt_idx];
        SphericalPoint sph_point = cart2sph(this_point);

        // @ note about vfov: e.g., (+ V_FOV/2) to adjust [-15, 15] to [0, 30]
        // @ min and max is just for the easier (naive) boundary checks. 
        int lower_bound_row_idx {0}; 
        int lower_bound_col_idx {0};
        int upper_bound_row_idx {kNumRimgRow - 1}; 
        int upper_bound_col_idx {kNumRimgCol - 1};
        int pixel_idx_row = int(std::min(std::max(std::round(kNumRimgRow * (1 - (rad2deg(sph_point.el) + (kVFOV/float(2.0))) / (kVFOV - float(0.0)))), float(lower_bound_row_idx)), float(upper_bound_row_idx)));
        int pixel_idx_col = int(std::min(std::max(std::round(kNumRimgCol * ((rad2deg(sph_point.az) + (kHFOV/float(2.0))) / (kHFOV - float(0.0)))), float(lower_bound_col_idx)), float(upper_bound_col_idx)));

        float curr_range = sph_point.r;

        // @ Theoretically, this if-block would have race condition (i.e., this is a critical section), 
        // @ But, the resulting range image is acceptable (watching via Rviz), 
        // @      so I just naively applied omp pragma for this whole for-block (2020.10.28)
        // @ Reason: because this for loop is splited by the omp, points in a single splited for range do not race among them, 
        // @         also, a point A and B lied in different for-segments do not tend to correspond to the same pixel, 
        // #               so we can assume practically there are few race conditions.     
        // @ P.S. some explicit mutexing directive makes the code even slower ref: https://stackoverflow.com/questions/2396430/how-to-use-lock-in-openmp
        if ( curr_range < rimg.at<float>(pixel_idx_row, pixel_idx_col) ) {
            rimg.at<float>(pixel_idx_row, pixel_idx_col) = curr_range;
        }
    }

    return rimg;
} // scan2RangeImg

// ? publishing images

template<typename T>
cv::Mat convertColorMappedImg (const cv::Mat &_src, std::pair<T, T> _caxis)
{
  T min_color_val = _caxis.first;
  T max_color_val = _caxis.second;

  cv::Mat image_dst;
  image_dst = 255 * (_src - min_color_val) / (max_color_val - min_color_val);
  image_dst.convertTo(image_dst, CV_8UC1);
  
  cv::applyColorMap(image_dst, image_dst, cv::COLORMAP_JET);

  return image_dst;
}

sensor_msgs::ImagePtr cvmat2msg(const cv::Mat &_img)
{
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _img).toImageMsg();
  return msg;
}

void pubRangeImg(cv::Mat& _rimg, 
                sensor_msgs::ImagePtr& _msg,
                image_transport::Publisher& _publiser,
                std::pair<float, float> _caxis)
{
    cv::Mat scan_rimg_viz = convertColorMappedImg(_rimg, _caxis);
    _msg = cvmat2msg(scan_rimg_viz);
    _publiser.publish(_msg);    
} // pubRangeImg

  void voxelize_preserving_labels(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI> &dst, double leaf_size)
  {
    /**< IMPORTANT
     * Because PCL voxlizaiton just does average the intensity of point cloud,
     * so this function is to conduct voxelization followed by nearest points search to re-assign the label of each point */
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_reassigned(new pcl::PointCloud<pcl::PointXYZI>);

    // 1. Voxelization
    static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(src);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*ptr_voxelized);

    // 2. Find nearest point to update intensity (index and id)
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(src);

    ptr_reassigned->points.reserve(ptr_voxelized->points.size());

    int K = 1;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    // Set dst <- output
    for (const auto &pt : ptr_voxelized->points)
    {
      if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        auto updated = pt;
        // Update meaned intensity to original intensity
        updated.intensity = (*src)[pointIdxNKNSearch[0]].intensity;
        ptr_reassigned->points.emplace_back(updated);
      }
    }
    dst = *ptr_reassigned;
  }
