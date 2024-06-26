#pragma once

#include <iostream>
#include <vector>
#include <sstream>
#include <array>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include "yaml-cpp/yaml.h"
// ros
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#define RESET "\033[0m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"

enum problem
{
    // bag
    rosbag_cloud_pro1, // 没有找到标准topic及type的点云
    rosbag_cloud_pro2, // 点云消息频率与10hz相差甚远
    rosbag_cloud_pro3, // 点云时间序列错误
    rosbag_cloud_pro4, // 点的时间序列错误
    rosbag_cloud_pro5, // 点云长宽与size不匹配
    rosbag_cloud_pro6, // 点云的时间戳分布不均匀
    rosbag_cloud_pro7, // is_dense?
    rosbag_cloud_pro8, // inf nan?
    rosbag_cloud_pro9, // 点云实际情况符合设备信息
                       //  pcd
    pcd_cloud_pro1,    // pcd文件的类型，如ascii，binary
    pcd_cloud_pro2,    // pcd点云的type是否符合需求
    pcd_cloud_pro3,    // 点的时间序列错误
    pcd_cloud_pro4,    // 点云长宽与size不匹配
    pcd_cloud_pro5,    // inf nan?
    pcd_cloud_pro6,    // cloud不是444x28的xyzirt
    pcd_cloud_pro7,    // 点云实际情况符合设备信息 small_pt_sum
                       // imu
    imu_msg_pro1,      // 没有找到标准topic及type的imu msg
    imu_msg_pro2,      // 消息频率与100hz相差甚远
    imu_msg_pro3,      // imu msg时间序列错误
    imu_msg_pro4,      // imu msg时间戳分布不均匀
    imu_msg_pro5,      // 9 axis data or 6 axis data
    imu_msg_pro6,      // g error or g bias
                       // gnss
    gnss_msg_pro1,     // 没有找到标准topic及type的gnss msg
    gnss_msg_pro2,     // 消息频率与100hz相差甚远
    gnss_msg_pro3,     // gnss msg时间序列错误
    gnss_msg_pro4,     // gnss msg时间戳分布不均匀
    gnss_msg_pro5      // gnss 消息符合使用LBH的需求
};

enum PCDType
{
    ASCII,
    BINARY,
    BINARY_COMPRESSED,
    UNKNOWN
};

struct PointXYZIRT_444128
{
    PCL_ADD_POINT4D;
    std::uint8_t intensity;
    std::uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT_444128,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))

struct PointXYZIRT_444228
{
    PCL_ADD_POINT4D;
    std::uint16_t intensity;
    std::uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT_444228,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint16_t, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))

struct PointXYZIRT_444428
{
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT_444428,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))

std::string match_index(int index);