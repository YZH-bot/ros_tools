#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
import pcl

if __name__ == '__main__':
    rospy.init_node('pcl_publisher')

    # Load the PCD file
    cloud = pcl.load('/path/to/your/pcd/file.pcd')

    # Convert PCL PointCloud to ROS PointCloud2
    ros_msg = pc2.create_cloud_xyz32(header=None, cloud=cloud)

    # Publish the PointCloud2 message
    pub = rospy.Publisher('/your/pointcloud2/topic', PointCloud2, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(ros_msg)
        rate.sleep()
