/*
 * @Description: mapping
 * @Version: 1.0
 * @Author: Xiaoyi Wu
 * @Date: 2023-02-16 10:06:35
 * @LastEditors: Xiaoyi Wu
 * @LastEditTime: 2023-02-16 11:02:37
 */
#include <iostream>
#include <cmath>
#include <ros/ros.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

#include <sensor_msgs/PointCloud2.h>

ros::Publisher pcl_ring_pub;
pcl::PointCloud<pcl::PointXYZI> Pointcloud_ring;
struct MyPoint
{
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring = 0;
    double timestamp = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(MyPoint, // 注册点类型宏
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

/*
    回调函数定义...略，假设回调函数入口参数是 （const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr）
*/
void lidar_pcd_callback(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    Pointcloud_ring.clear();
    pcl::PointXYZI temp;
    // 定义接收变量
    pcl::PointCloud<MyPoint> laserCloudIn;
    // 将ROS格式点云转化为PCL格式
    pcl::fromROSMsg(*in_cloud_ptr, laserCloudIn);
    // for (auto pt : laserCloudIn)
    // {
    //     std::cout << pt.ring << " ";
    // }
    // 打印每个点的ring信息
    int count = 0;
    for (auto pt : laserCloudIn)
    {
        if (isnan(pt.x) || isnan(pt.y) || isnan(pt.z))
        {
            // std::cout << laserCloudIn.points[i].x << " " << laserCloudIn.points[i].y << std::endl;
            count++;
            continue;
        }
        if(pt.getVector3fMap().norm() < 1.5)
            continue;
        // 注意这里要使用printf函数打印输出查看，使用cout终端打印会出错
        // printf("%d ", laserCloudIn.points[i].ring);
        // ROS_INFO_STREAM("")
        temp.x = pt.x;
        temp.y = pt.y;
        temp.z = pt.z;
        temp.intensity = pt.ring;
        Pointcloud_ring.emplace_back(temp);
    }
    Pointcloud_ring.is_dense = false;
    Pointcloud_ring.width = Pointcloud_ring.size();
    Pointcloud_ring.height = 1;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(Pointcloud_ring, Pointcloud_ring, indices);
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(Pointcloud_ring, cloud);
    cloud.header.frame_id = "body";
    cloud.header.stamp = ros::Time::now();
    pcl_ring_pub.publish(cloud);
    ROS_INFO_STREAM("pcl_ring receive. original size: " << laserCloudIn.points.size() << "after: " << Pointcloud_ring.size() << " nan size: " << count);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_ring");
    ros::NodeHandle nh;
    ros::Subscriber lidar_pcd_sub = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 10, &lidar_pcd_callback);
    pcl_ring_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl_ring", 1);
    ros::Rate loop_rate(20);
    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}