#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <string>

using namespace std;
string pcd_path_;
#define RED "\033[31m"
#define GREEN "\033[32m"
#define DEBUG "\033[33m"

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh("~");

    nh.param<string>("path_to_pcd", pcd_path_, " ");

    // Load the .pcd file
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path_, *cloud) == -1)
    {
        ROS_ERROR_STREAM(pcd_path_ << " Failed to load PCD file");
        return -1;
    }
    ROS_INFO_STREAM(DEBUG << pcd_path_);
    // Convert the PointCloud to PointCloud2
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);

    // Set the frame ID and timestamp
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros::Time::now();

    // Create a publisher for the PointCloud2 message
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("pcd_pub", 1);

    // Loop at 10 Hz and publish the PointCloud2 message
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        cloud_pub.publish(cloud_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
