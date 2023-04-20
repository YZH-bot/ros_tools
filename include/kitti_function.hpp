#include <vector>
#include <filesystem> // requires gcc version >= 8
#include <string>
#include <iostream>
#include <filesystem>
#include <vector>
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
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
using namespace std;
using PointType = pcl::PointXYZI;
namespace fs = std::filesystem;

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

tf::StampedTransform Matrix4dpose2TF(Eigen::Product<Eigen::Matrix4d, Eigen::Matrix4d, 0> &ego_car_link)
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
