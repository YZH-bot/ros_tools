
#include "kitti_function.hpp"

namespace fs = std::filesystem;

std::vector<std::string> sequence_scan_names_;
std::vector<std::string> sequence_scan_paths_;
std::vector<std::string> sequence_valid_scan_names_;
std::vector<std::string> sequence_valid_scan_paths_;
std::vector<Eigen::Matrix4d> scan_poses_;
std::vector<Eigen::Matrix4d> sequence_scan_poses_;
int start_idx_ = 2350;
int end_idx_ = 2670;
bool use_keyframe_gap_ = false;
int keyframe_gap_ = 1;
std::vector<double> kVecExtrinsicLiDARtoPoseBase =
    {-1.857e-03, -9.999e-01, -8.039e-03, -4.784e-03,
     -6.481e-03, 8.0518e-03, -9.999e-01, -7.337e-02,
     9.999e-01, -1.805e-03, -6.496e-03, -3.339e-01,
     0.0, 0.0, 0.0, 1.0};
Eigen::Matrix4d kSE3MatExtrinsicLiDARtoPoseBase = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(kVecExtrinsicLiDARtoPoseBase.data(), 4, 4);

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
} // parseValidScanInfo

void readValidScans(std::vector<pcl::PointCloud<PointType>::Ptr> &scans_, float kDownsampleVoxelSize = 0.1)
// for target range of scan idx
{
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
} // readValidScans

int main(int argc, char *argv[])
{

    string sequence_scan_dir_ = "/media/yzh/T7/Study/SLAM/Dataset/data_odometry_velodyne/dataset/sequences/05/velodyne/";
    for (auto file : fs::directory_iterator(sequence_scan_dir_))
    {
        sequence_scan_names_.emplace_back(file.path().filename());
        sequence_scan_paths_.emplace_back(file.path());
    }
    std::sort(sequence_scan_names_.begin(), sequence_scan_names_.end());
    std::sort(sequence_scan_paths_.begin(), sequence_scan_paths_.end());
    for (size_t i = 0; i < sequence_scan_names_.size(); i++)
    {
        cout << sequence_scan_names_.at(i) << " ";
    }
    // pose
    string sequence_pose_path_ = "/media/yzh/T7/Study/SLAM/Dataset/data_odometry_labels/dataset/sequences/05/poses.txt";
    std::ifstream pose_file_handle(sequence_pose_path_);

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
        ith_pose = rotation_z * rotation_x * ith_pose;
        sequence_scan_poses_.emplace_back(ith_pose);

        num_poses++;
    }
    cout << "num_poses = " << num_poses << endl;

    // read all the scans
    std::vector<pcl::PointCloud<PointType>::Ptr> scans_;
    parseValidScanInfo();
    readValidScans(scans_);
    auto scans_size = scans_.size();

    ros::init(argc, argv, "globalmap");
    ros::NodeHandle nh;
    ros::Publisher scans_pub = nh.advertise<sensor_msgs::PointCloud2>("scans_pub", 1);
    ros::Rate rate(2);
    int scan_idx = 0;
    while (ros::ok())
    {
        rate.sleep();
        if (scan_idx < scans_size - 1)
        {
            scan_idx++;
            auto ii_scan = scans_.at(scan_idx);      // pcl::PointCloud<PointType>::Ptr
            auto ii_pose = scan_poses_.at(scan_idx); // Eigen::Matrix4d
            // local to global (local2global)
            pcl::PointCloud<PointType>::Ptr scan_global_coord(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*ii_scan, *scan_global_coord, kSE3MatExtrinsicLiDARtoPoseBase);
            pcl::transformPointCloud(*scan_global_coord, *scan_global_coord, ii_pose);
            sensor_msgs::PointCloud2 lidar_msg;
            pcl::toROSMsg(*scan_global_coord, lidar_msg);
            lidar_msg.header.stamp = ros::Time::now();
            lidar_msg.header.frame_id = "world";
            scans_pub.publish(lidar_msg);
            std::cout << "Publishing" << std::endl;
        }
        else
        {
            scan_idx = 0;
        }
    }
    return 0;
}
