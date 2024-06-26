#include "gnss_check.h"

GnssChecker::GnssChecker(const std::string &filePath, std::vector<problem> &data)
    : gnss_msg_path(filePath), problem_item_index(data)
{
    check_gnss_msg(gnss_msg_path, problem_item_index);
}

bool GnssChecker::isValidNavSatFix(const sensor_msgs::NavSatFix &msg)
{
    // 检查GPS状态
    if (msg.status.status < sensor_msgs::NavSatStatus::STATUS_FIX)
        return false;
    // 检查经纬度范围
    if (std::abs(msg.latitude) > 90.0 || std::abs(msg.longitude) > 180.0)
        return false;

    // 检查高度范围，可以根据具体应用场景调整
    if (msg.altitude < -1000.0 || msg.altitude > 1000.0)
        return false;

    // 检查位置协方差类型
    if (msg.position_covariance_type == sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
        return false;

    // 检查协方差矩阵的值是否合理，这里简单检查不为负数
    for (double cov : msg.position_covariance)
    {
        if (cov < 0)
        {
            return false;
        }
    }
    return true; // 如果通过了所有检查，则认为消息有效且合理
}

int GnssChecker::check_gnss_msg(std::string gnss_msg_path, std::vector<problem> &problem_item_index)
{
    rosbag::Bag in_bag;
    in_bag.open(gnss_msg_path, rosbag::bagmode::Read);

    rosbag::View view(in_bag, rosbag::TopicQuery("/gps/data"));
    for (rosbag::MessageInstance const &msg : view)
    {
        if (msg.isType<sensor_msgs::NavSatFix>())
        {
            if (!found_gnss_topic_flag)
                found_gnss_topic_flag = true;
            if (messageCount == 0)
            {
                startTime = msg.getTime();
            }
            endTime = msg.getTime();
            messageCount++;

            sensor_msgs::NavSatFixPtr current_gnss_msg = msg.instantiate<sensor_msgs::NavSatFix>();
            if (current_gnss_msg != nullptr)
            {
                double CurGnssHeaderStamp = current_gnss_msg->header.stamp.toSec();
                if (!wrong_gnss_order)
                {
                    if (CurGnssHeaderStamp < LastGnssHeaderStamp)
                    {
                        wrong_gnss_order = true;
                        std::cout << RED << "CurGnssHeaderStamp < LastGnssHeaderStamp! " << RESET << std::endl;
                        std::cout << "data cur stamp is : " << std::setprecision(19) << CurGnssHeaderStamp << " data last stamp is : " << LastGnssHeaderStamp << std::endl;
                        problem_item_index.push_back(gnss_msg_pro3);
                    }
                }

                if (!bad_gnss_stamp_distribution)
                {
                    if (LastGnssHeaderStamp > 0)
                    {
                        if (fabs((CurGnssHeaderStamp - LastGnssHeaderStamp) - 0.01) > 0.005)
                        {
                            bad_gnss_stamp_distribution = true;
                            std::cout << YELLOW << "fabs((CurGnssHeaderStamp - LastGnssHeaderStamp) - 0.01) > 0.005 ..." << RESET << std::endl;
                            std::cout << "pro msg num is : " << messageCount << std::endl;
                            std::cout << "data cur stamp is : " << std::setprecision(19) << CurGnssHeaderStamp << ", and data last stamp is : " << LastGnssHeaderStamp << std::endl;
                            problem_item_index.push_back(gnss_msg_pro4);
                        }
                    }
                }

                if (!bad_data_num)
                {
                    if (!isValidNavSatFix(*current_gnss_msg))
                    {
                        bad_data_num = true;
                        std::cout << RED << "GPS message is invalid or unreasonable." << RESET << std::endl;
                        problem_item_index.push_back(gnss_msg_pro5);
                    }
                }

                LastGnssHeaderStamp = current_gnss_msg->header.stamp.toSec();
            }
        }
    }

    if (found_gnss_topic_flag)
    {
        std::cout << GREEN << "Input bag contains sensor_msgs/NavSatFix type messages, and topic is 'gps/data'" << RESET << std::endl;
        if (messageCount > 1)
        {
            double duration = (endTime - startTime).toSec();
            double averageFrequency = (messageCount - 1) / duration;
            if (fabs(averageFrequency - 100) < 2.5)
                std::cout << GREEN << "Average frequency of sensor_msgs/NavSatFix messages in topic 'gps/data' is : " << std::setprecision(4) << averageFrequency << " Hz" << RESET << std::endl;
            else
            {
                std::cout << YELLOW << "Average frequency of sensor_msgs/NavSatFix messages in topic 'gps/data' is not quite equal 100 hz, calculation is : " << std::setprecision(4) << averageFrequency << " Hz" << RESET << std::endl;
                problem_item_index.push_back(gnss_msg_pro2);
            }
        }
    }
    else
    {
        std::cout << RED << "Can not found Gnss matching information in bag" << RESET << std::endl;
        problem_item_index.push_back(gnss_msg_pro1);
    }

    in_bag.close();
    return 1;
}