
#include "kitti_function.hpp"

std::vector<double> kVecExtrinsicLiDARtoPoseBase =
    {-1.857e-03, -9.999e-01, -8.039e-03, -4.784e-03,
     -6.481e-03, 8.0518e-03, -9.999e-01, -7.337e-02,
     9.999e-01, -1.805e-03, -6.496e-03, -3.339e-01,
     0.0, 0.0, 0.0, 1.0};
Eigen::Matrix4d kSE3MatExtrinsicLiDARtoPoseBase = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(kVecExtrinsicLiDARtoPoseBase.data(), 4, 4);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kitti_node");
    ros::NodeHandle nh;

    // ? parameters set
    int start_idx_ = 2350;
    int end_idx_ = 2670;
    // pose & scan
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

    auto scans_size = scans_.size();
    ros::Publisher scans_pub = nh.advertise<sensor_msgs::PointCloud2>("scans_pub", 1);
    tf::TransformBroadcaster tf_broadcaster;
    ros::Rate rate(5);
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

            // local to global (local2global)
            pcl::PointCloud<PointType>::Ptr scan_global_coord(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*ii_scan, *scan_global_coord, kSE3MatExtrinsicLiDARtoPoseBase);
            pcl::transformPointCloud(*scan_global_coord, *scan_global_coord, ii_pose);
            sensor_msgs::PointCloud2 lidar_msg;
            pcl::toROSMsg(*scan_global_coord, lidar_msg);
            lidar_msg.header.stamp = ros::Time::now();
            lidar_msg.header.frame_id = "world";
            scans_pub.publish(lidar_msg);

            // ?TF publishing
            auto ego_car_link = ii_pose * kSE3MatExtrinsicLiDARtoPoseBase;
            tf::StampedTransform tf_map2ego = Matrix4dpose2TF(ego_car_link);
            tf_broadcaster.sendTransform(tf_map2ego);

            std::cout << "Publishing" << std::endl;
        }
        else
        {
            scan_idx = 0;
        }
    }
    return 0;
}
