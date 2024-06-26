#pragma once

#include "common.h"

class RosbagChecker
{
public:
    RosbagChecker(const std::string &filePath, std::vector<problem> &data);
    bool check_type_flag = false;
    bool wrong_cld_order = false;
    bool bad_cld_stamp_distribution = false;
    bool wrong_pt_order = false;
    bool wrong_HW_size = false;
    bool is_dense = false;
    bool is_nan = false;
    bool small_pt_sum = false;
    double LastCloudHeaderStamp = -1;
    std::array<int, 6> arr;

private:
    bool found_cloud_topic_flag = false;
    ros::Time startTime, endTime;
    int messageCount = 0;
    std::string rosbag_pointcloud_path;
    std::vector<problem> &problem_item_index;
    int check_rosbag_pointcloud(std::string rosbag_pointcloud_path, std::vector<problem> &problem_item_index);
    std::array<int, 6> checkPointCloudType(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) const;
    int printFieldType(const sensor_msgs::PointField &field) const;
};