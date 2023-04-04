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
using namespace std;
using PointType = pcl::PointXYZI;

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
