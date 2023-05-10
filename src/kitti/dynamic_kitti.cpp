#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <math.h>

// PCL
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp> //自定义点云类型时要加
#include <pcl/common/transforms.h>

// Eigen
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

// Boost
#include <boost/tokenizer.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

// ROS
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;
typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
int range_img_width_ = 2083;

struct PointXYZIR
{
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    uint16_t px;
    uint16_t py;
    uint16_t seglabel;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(uint16_t, px, px)(uint16_t, py, py)(uint16_t, seglabel, seglabel))

template <class Type>
Type stringToNum(const string &str)
{
    istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

template <typename T>
string toString(const T &t)
{
    ostringstream oss;
    oss << t;
    return oss.str();
}

void calPolarAngle(float x, float y, float &temp_tangle)
{
    if (x == 0 && y == 0)
    {
        temp_tangle = 0;
    }
    else if (y >= 0)
    {
        temp_tangle = (float)atan2(y, x);
    }
    else if (y <= 0)
    {
        temp_tangle = (float)atan2(y, x) + 2 * M_PI;
    }
}

int fileNameFilter(const struct dirent *cur)
{
    std::string str(cur->d_name);
    if (str.find(".bin") != std::string::npos || str.find(".velodata") != std::string::npos || str.find(".pcd") != std::string::npos || str.find(".png") != std::string::npos || str.find(".jpg") != std::string::npos || str.find(".txt") != std::string::npos)
    {
        return 1;
    }
    return 0;
}

bool get_all_files(const std::string &dir_in,
                   std::vector<std::string> &files)
{

    if (dir_in.empty())
    {
        return false;
    }
    struct stat s;
    stat(dir_in.c_str(), &s);
    if (!S_ISDIR(s.st_mode))
    {
        return false;
    }
    DIR *open_dir = opendir(dir_in.c_str());
    if (NULL == open_dir)
    {
        std::exit(EXIT_FAILURE);
    }
    dirent *p = nullptr;
    while ((p = readdir(open_dir)) != nullptr)
    {
        struct stat st;
        if (p->d_name[0] != '.')
        {
            std::string name = dir_in + std::string("/") + std::string(p->d_name);
            stat(name.c_str(), &st);
            if (S_ISDIR(st.st_mode))
            {
                get_all_files(name, files);
            }
            else if (S_ISREG(st.st_mode))
            {
                boost::char_separator<char> sepp{"."};
                tokenizer tokn(std::string(p->d_name), sepp);
                vector<string> filename_sep(tokn.begin(), tokn.end());
                string type_ = "." + filename_sep[1];
                break;
            }
        }
    }

    struct dirent **namelist;
    int n = scandir(dir_in.c_str(), &namelist, fileNameFilter, alphasort);
    if (n < 0)
    {
        return false;
    }
    for (int i = 0; i < n; ++i)
    {
        std::string filePath(namelist[i]->d_name);
        files.push_back(filePath);
        free(namelist[i]);
    };
    free(namelist);
    closedir(open_dir);
    return true;
}

bool Load_Sensor_Data_Path(std::vector<std::string> &lidarfile_name, string &path)
{
    string lidar_file_path = path;
    cout << lidar_file_path << endl;
    if (!get_all_files(lidar_file_path, lidarfile_name))
        return false;

    return true;
}

int get_quadrant(pcl::PointXYZI point)
{
    int res = 0;
    float x = point.x;
    float y = point.y;
    if (x > 0 && y >= 0)
        res = 1;
    else if (x <= 0 && y > 0)
        res = 2;
    else if (x < 0 && y <= 0)
        res = 3;
    else if (x >= 0 && y < 0)
        res = 4;
    return res;
}

void AddRingInfo(pcl::PointCloud<pcl::PointXYZI> &input, pcl::PointCloud<PointXYZIR> &output)
{
    int previous_quadrant = 0;
    uint16_t ring_ = (uint16_t)64 - 1;
    for (pcl::PointCloud<pcl::PointXYZI>::iterator pt = input.points.begin(); pt < input.points.end() - 1; ++pt)
    {
        int quadrant = get_quadrant(*pt);
        if (quadrant == 1 && previous_quadrant == 4 && ring_ > 0)
            ring_ -= 1;
        PointXYZIR point;
        point.x = pt->x;
        point.y = pt->y;
        point.z = pt->z;
        point.intensity = pt->intensity;
        point.ring = ring_;
        output.push_back(point);
        previous_quadrant = quadrant;
    }
}

bool LoadKittiPointcloud(pcl::PointCloud<pcl::PointXYZI> &cloud_IN, string path)
{
    string lidar_filename_path = path;
    ifstream inputfile;
    inputfile.open(lidar_filename_path, ios::binary);
    if (!inputfile)
    {
        cerr << "ERROR: Cannot open file " << lidar_filename_path
             << "! Aborting..." << endl;
        return false;
    }

    inputfile.seekg(0, ios::beg);
    for (int i = 0; inputfile.good() && !inputfile.eof(); i++)
    {
        pcl::PointXYZI p;
        inputfile.read((char *)&p.x, 3 * sizeof(float));
        inputfile.read((char *)&p.intensity, sizeof(float));
        cloud_IN.points.push_back(p);
    }
    return true;
}

void RangeProjection(pcl::PointCloud<PointXYZIR> &Cloud_IN)
{

    for (int i = 0; i < Cloud_IN.points.size(); ++i)
    {
        float u(0);
        calPolarAngle(Cloud_IN.points[i].x, Cloud_IN.points[i].y, u);
        int col = round((range_img_width_ - 1) * (u * 180.0 / M_PI) / 360.0);
        int ind = Cloud_IN.points[i].ring;
        Cloud_IN.points[i].py = ind;
        Cloud_IN.points[i].px = col;
    }
}

int main(int argc, char **argv)
{

    // ##################### label ########################
    unordered_map<int, string> labels;
    unordered_map<int, vector<int>> color_map;

    labels[0] = "unlabeled";
    labels[1] = "outlier";
    labels[10] = "car";
    labels[11] = "bicycle";
    labels[13] = "bus";
    labels[15] = "motorcycle";
    labels[16] = "on-rails";
    labels[18] = "truck";
    labels[20] = "other-vehicle";
    labels[30] = "person";
    labels[31] = "bicyclist";
    labels[32] = "motorcyclist";
    labels[40] = "road";
    labels[44] = "parking";
    labels[48] = "sidewalk";
    labels[49] = "other-ground";
    labels[50] = "building";
    labels[51] = "fence";
    labels[52] = "other-structure";
    labels[60] = "lane-marking";
    labels[70] = "vegetation";
    labels[71] = "trunk";
    labels[72] = "terrain";
    labels[80] = "pole";
    labels[81] = "traffic-sign";
    labels[99] = "other-object";
    labels[252] = "moving-car";
    labels[253] = "moving-bicyclist";
    labels[254] = "moving-person";
    labels[255] = "moving-motorcyclist";
    labels[256] = "moving-on-rails";
    labels[257] = "moving-bus";
    labels[258] = "moving-truck";
    labels[259] = "moving-other-vehicle";

    std::vector<int> DYNAMIC_CLASSES = {252, 253, 254, 255, 256, 257, 258, 259};

    ros::init(argc, argv, "semantic_kitti_node");
    ros::NodeHandle nh;

    ros::Publisher publidar = nh.advertise<sensor_msgs::PointCloud2>("/scans_pub", 1);
    tf::TransformBroadcaster ego_tf_broadcaster;

    // ##################### data path ########################
    // string datapath = "/media/yzh/YZH2/KITTI Semantic/data_odometry_velodyne/dataset/sequences/05/velodyne/";
    // string lablespath = "/media/yzh/YZH2/KITTI Semantic/data_odometry_labels/dataset/sequences/05/labels/";
    // string pose_file = "/media/yzh/YZH2/KITTI Semantic/data_odometry_labels/dataset/sequences/05/poses.txt";
    // string calibration_file = "/media/yzh/YZH2/KITTI Semantic/data_odometry_calib/dataset/sequences/05/calib.txt";

    string datapath = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_velodyne/dataset/sequences/05/velodyne/";
    string lablespath = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_labels/dataset/sequences/05/labels/";
    string pose_file = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_labels/dataset/sequences/05/poses.txt";
    string calibration_file = "/media/robot-nuc12/T7/Study/SLAM/Dataset/data_odometry_calib/dataset/sequences/05/calib.txt";

    // 创建 AngleAxis 对象表示绕 X 轴旋转 -90 弧度
    Eigen::AngleAxisd rotation_vector_x(M_PI / 2, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d rotation_x = rotation_vector_x.toRotationMatrix();
    Eigen::Matrix4d transformation_matrix_x = Eigen::Matrix4d::Identity();
    transformation_matrix_x.block<3, 3>(0, 0) = rotation_x;
    // 创建 AngleAxis 对象表示绕 Z 轴旋转 180 弧度
    Eigen::AngleAxisd rotation_vector_z(M_PI / 2, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rotation_z = rotation_vector_z.toRotationMatrix();
    Eigen::Matrix4d transformation_matrix_z = Eigen::Matrix4d::Identity();
    transformation_matrix_z.block<3, 3>(0, 0) = rotation_z;

    // done: get cam_velo T
    std::ifstream calibration_fin(calibration_file);
    std::ifstream pose_fin(pose_file);
    std::string value;
    for (int i = 0; i < 4; i++)
    {
        std::getline(calibration_fin, value);
    }
    Eigen::Matrix<double, 4, 3> matrix_in;
    Eigen::Matrix<double, 4, 4> t_cam_velo(Eigen::Matrix<double, 4, 4>::Identity());
    std::getline(calibration_fin, value, ' ');
    for (int i = 0; i < 11; i++)
    {
        std::getline(calibration_fin, value, ' ');
        matrix_in(i) = stof(value);
    }
    std::getline(calibration_fin, value, '\n');
    matrix_in(11) = stof(value);
    t_cam_velo.block<3, 4>(0, 0) = matrix_in.transpose();

    vector<string> lidarname;
    if (!Load_Sensor_Data_Path(lidarname, datapath))
    {
        cout << "Detecion file wrong!" << endl;
        std::abort();
    }

    cout << lidarname.size() << endl;
    int maxframe = lidarname.size();
    vector<int> frame_num;
    boost::char_separator<char> sep{" "};
    sort(frame_num.begin(), frame_num.end(), [](int a, int b)
         { return a < b; });

    int frame = 0;
    ros::Rate r(5);

    while (ros::ok() && frame < maxframe)
    {
        if (frame < 2350)
        {
            frame++;
            for (int i = 0; i < 11; i++)
            {
                std::getline(pose_fin, value, ' ');
                matrix_in(i) = stof(value);
            }
            std::getline(pose_fin, value, '\n');
            continue;
        }
        // ##################### load data ########################
        string cloudpath = datapath + lidarname[frame];

        boost::char_separator<char> sept{"."};
        tokenizer tokn(lidarname[frame], sept);
        vector<string> filename_sep(tokn.begin(), tokn.end());
        std::cout << filename_sep[0] << std::endl;

        string lablepath = lablespath + filename_sep[0] + ".label";

        pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud(
            new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<PointXYZIR>::Ptr cloud_ring(
            new pcl::PointCloud<PointXYZIR>);

        LoadKittiPointcloud(*Cloud, cloudpath);
        // AddRingInfo(*Cloud, *cloud_ring);
        // RangeProjection(*cloud_ring);

        // ##################### load label ########################
        std::ifstream inputfile;
        inputfile.open(lablepath.c_str(), ios::binary);

        vector<uint32_t> label;

        if (!inputfile)
        {
            cerr << "ERROR: Cannot open file " << lablepath
                 << "! Aborting..." << endl;
        }
        inputfile.seekg(0, ios::beg); // 将文件读取指针移动到文件的开头位置
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
            uint16_t inst = label[i] >> 16;
            instance.push_back(inst);
            semantic_label.push_back(slabel);
            bool is_static = true;
            for (int class_num : DYNAMIC_CLASSES)
            {
                if (slabel == class_num)
                { // 1. check it is in the moving object classes
                    Cloud->points[i].intensity = 63;
                    is_static = false;
                }
            }
            if (is_static)
            {
                Cloud->points[i].intensity = 0;
            }
        }

        // ##################### load pose ########################
        for (int i = 0; i < 11; i++)
        {
            std::getline(pose_fin, value, ' ');
            matrix_in(i) = stof(value);
        }
        std::getline(pose_fin, value, '\n');
        matrix_in(11) = stof(value);
        Eigen::Matrix4d pose_cam(Eigen::Matrix4d::Identity());
        pose_cam.block<3, 4>(0, 0) = matrix_in.transpose();
        Eigen::Matrix4d lidar = t_cam_velo.inverse() * pose_cam * transformation_matrix_x * transformation_matrix_z;
        pcl::transformPointCloud(*Cloud, *Cloud, lidar);

        // ##################### TF ########################
        // auto lidar = t_cam_velo * pose_cam;
        Eigen::Affine3d affine_transform(lidar); // 转换为Affine3d类型
        tf::Transform tf_transform;
        tf::transformEigenToTF(affine_transform, tf_transform);
        tf::StampedTransform tf_map2scan(tf_transform, ros::Time::now(), "world", "ego_car");
        ego_tf_broadcaster.sendTransform(tf_map2scan);

        // ##################### visual ########################
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(*Cloud, ros_cloud);
        ros_cloud.header.frame_id = "world";
        publidar.publish(ros_cloud);

        r.sleep();
        frame++;
    }

    return 0;
}