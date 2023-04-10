#include "kitti_function.hpp"

std::vector<double> kVecExtrinsicLiDARtoPoseBase =
    {-1.857e-03, -9.999e-01, -8.039e-03, -4.784e-03,
     -6.481e-03, 8.0518e-03, -9.999e-01, -7.337e-02,
     9.999e-01, -1.805e-03, -6.496e-03, -3.339e-01,
     0.0, 0.0, 0.0, 1.0};
Eigen::Matrix4d kSE3MatExtrinsicLiDARtoPoseBase = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(kVecExtrinsicLiDARtoPoseBase.data(), 4, 4);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kitti2range_image");
    ros::NodeHandle nh;
    image_transport::ImageTransport ROSimg_transporter_(nh);
    image_transport::Publisher scan_rimg_msg_publisher_ = ROSimg_transporter_.advertise("/scan_rimg_single", 10);

    // ? image parameters
    // fov
    float kVFOV;
    float kHFOV;
    float rimg_color_min_;
    float rimg_color_max_;
    float _res_alpha = 1;
    sensor_msgs::ImagePtr scan_rimg_msg_;
    nh.param<float>("removert/sequence_vfov", kVFOV, 50.0);
    nh.param<float>("removert/sequence_hfov", kHFOV, 360.0);
    nh.param<float>("removert/rimg_color_min", rimg_color_min_, 0.0);
    nh.param<float>("removert/rimg_color_max", rimg_color_max_, 10.0);
    std::pair<float, float> kFOV = std::pair<float, float>(kVFOV, kHFOV);
    std::pair<int, int> rimg_shape = resetRimgSize(kFOV, _res_alpha);
    std::pair<float, float> kRangeColorAxis = std::pair<float, float>{rimg_color_min_, rimg_color_max_}; // meter

    // ? scan read parameters
    int start_idx_ = 2350;
    int end_idx_ = 2670;
    string sequence_scan_dir_;
    string sequence_pose_path_;
    std::vector<pcl::PointCloud<PointType>::Ptr> scans_;
    std::vector<Eigen::Matrix4d> scan_poses_;
    nh.param<string>("sequence_scan_dir_", sequence_scan_dir_, "/media/yzh/T7/Study/SLAM/Dataset/data_odometry_velodyne/dataset/sequences/05/velodyne/");
    nh.param<string>("sequence_pose_path_", sequence_pose_path_, "/media/yzh/T7/Study/SLAM/Dataset/data_odometry_labels/dataset/sequences/05/poses.txt");
    cout << sequence_scan_dir_ << std::endl;
    cout << sequence_pose_path_ << std::endl;

    // read scans and poses
    read_scans_and_poses(start_idx_, end_idx_, sequence_scan_dir_, sequence_pose_path_, scans_, scan_poses_);

    ros::Rate rate(5);
    int scan_idx = 0;
    auto scans_size = scans_.size();
    while (ros::ok())
    {
        rate.sleep();
        if (scan_idx < scans_size - 1)
        {
            scan_idx++;
            auto ii_scan = scans_.at(scan_idx);
            auto ii_pose = scan_poses_.at(scan_idx);

            cv::Mat scan_rimg = scan2RangeImg(ii_scan, kFOV, rimg_shape); // openMP inside
            cout << "rimg shape" << rimg_shape.first << " " << rimg_shape.second << endl;
            // visualization
            pubRangeImg(scan_rimg, scan_rimg_msg_, scan_rimg_msg_publisher_, kRangeColorAxis);
            cout << "visulization range images!" << endl;
        }
        else
        {
            scan_idx = 0;
        }
    }
    return 0;
}