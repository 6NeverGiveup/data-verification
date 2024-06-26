#pragma once

#include "common.h"

class GnssChecker
{
public:
    GnssChecker(const std::string &filePath, std::vector<problem> &data);
    double LastGnssHeaderStamp = -1;
    bool found_gnss_topic_flag = false;
    bool wrong_gnss_order = false;
    bool bad_gnss_stamp_distribution = false;
    bool bad_data_num = false;

private:
    bool isValidNavSatFix(const sensor_msgs::NavSatFix &msg);
    int messageCount = 0;
    ros::Time startTime, endTime;
    std::string gnss_msg_path;
    std::vector<problem> &problem_item_index;
    int check_gnss_msg(std::string gnss_msg_path, std::vector<problem> &problem_item_index);
};
