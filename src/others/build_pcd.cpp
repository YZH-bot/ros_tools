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

ros::Publisher map_pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr map;

/*
    回调函数定义...略，假设回调函数入口参数是 （const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr）
*/
void lidar_pcd_callback(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    // 定义接收变量
    pcl::PointCloud<pcl::PointXYZI>::Ptr Scan(new pcl::PointCloud<pcl::PointXYZI>());
    // 将ROS格式点云转化为PCL格式
    pcl::fromROSMsg(*in_cloud_ptr, *Scan);
    ROS_INFO_STREAM("pcl_ring receive. original size: " << Scan->size() << " register ID: " << Scan->header.frame_id);

    pcl::VoxelGrid<pcl::PointXYZI> sor1;
    sor1.setLeafSize(0.2, 0.2, 0.2);
    sor1.setInputCloud(Scan);
    sor1.filter(*Scan);
    *map += *Scan;
    sor1.setInputCloud(map);
    sor1.filter(*map);

    sensor_msgs::PointCloud2 msg_ground;
    pcl::toROSMsg(*map, msg_ground);
    msg_ground.header.frame_id = Scan->header.frame_id;
    map_pub.publish(msg_ground);
    ROS_INFO_STREAM("- sended map pcd: " << map->size());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_building");
    ros::NodeHandle nh;
    ros::Subscriber lidar_pcd_sub = nh.subscribe<sensor_msgs::PointCloud2>("/registered_cloud", 10, &lidar_pcd_callback);
    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);
    map.reset(new pcl::PointCloud<pcl::PointXYZI>());
    ros::Rate loop_rate(5);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}