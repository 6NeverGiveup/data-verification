#pragma once

#include "common.h"

class ImuChecker
{

public:
    ImuChecker(const std::string &filePath, std::vector<problem> &data);
    double LastImuHeaderStamp = -1;
    bool wrong_imu_order = false;
    bool bad_imu_stamp_distribution = false;
    bool wrong_axis_num_flag = false;
    bool g_check = false;
    bool is_g = false;
    bool is_m2s = false;

private:
    int check_imu_msg(std::string imu_msg_path, std::vector<problem> &problem_item_index);
    bool isValidOrientation(const geometry_msgs::Quaternion &orientation);
    bool isValidCovariance(const boost::array<double, 9> &covariance);
    bool checkProximity(double number);
    bool found_imu_topic_flag = false;
    int messageCount = 0;
    int exceeding_num = 0;
    ros::Time startTime, endTime;
    std::string imu_msg_path;
    std::vector<problem> &problem_item_index;
};
