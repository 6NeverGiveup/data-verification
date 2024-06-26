#include "imu_check.h"

ImuChecker::ImuChecker(const std::string &filePath, std::vector<problem> &data)
    : imu_msg_path(filePath), problem_item_index(data)
{
    check_imu_msg(imu_msg_path, problem_item_index);
}

bool ImuChecker::isValidOrientation(const geometry_msgs::Quaternion &orientation)
{
    // Check if orientation values are not NaN or infinite
    return !(std::isnan(orientation.x) || std::isnan(orientation.y) || std::isnan(orientation.z) || std::isnan(orientation.w) ||
             std::isinf(orientation.x) || std::isinf(orientation.y) || std::isinf(orientation.z) || std::isinf(orientation.w));
}

bool ImuChecker::isValidCovariance(const boost::array<double, 9> &covariance)
{
    // Check if covariance values are valid (not -1, NaN, or infinite)
    for (int i = 0; i < 9; ++i)
    {
        if (covariance[i] == -1 || std::isnan(covariance[i]) || std::isinf(covariance[i]))
        {
            return false;
        }
    }
    return true;
}

bool ImuChecker::checkProximity(double number)
{
    double distanceTo1 = std::abs(number - 1.0);
    double distanceTo10 = std::abs(number - 10.0);
    if (distanceTo1 < distanceTo10)
        return false;
    else
        return true;
}

int ImuChecker::check_imu_msg(std::string imu_msg_path, std::vector<problem> &problem_item_index)
{
    rosbag::Bag in_bag;
    in_bag.open(imu_msg_path, rosbag::bagmode::Read);

    rosbag::View view(in_bag, rosbag::TopicQuery("/imu/data"));
    for (rosbag::MessageInstance const &msg : view)
    {
        if (msg.isType<sensor_msgs::Imu>())
        {
            if (!found_imu_topic_flag)
                found_imu_topic_flag = true;
            if (messageCount == 0)
            {
                startTime = msg.getTime();
            }
            endTime = msg.getTime();
            messageCount++;

            sensor_msgs::ImuConstPtr current_imu_msg = msg.instantiate<sensor_msgs::Imu>();
            if (current_imu_msg != nullptr)
            {
                double CurImuHeaderStamp = current_imu_msg->header.stamp.toSec();
                if (!wrong_imu_order)
                {
                    if (CurImuHeaderStamp < LastImuHeaderStamp)
                    {
                        wrong_imu_order = true;
                        std::cout << RED << "CurImuHeaderStamp < LastImuHeaderStamp! " << RESET << std::endl;
                        std::cout << "data cur stamp is : " << std::setprecision(19) << CurImuHeaderStamp << " data last stamp is : " << LastImuHeaderStamp << std::endl;
                        problem_item_index.push_back(imu_msg_pro3);
                    }
                }

                if (!bad_imu_stamp_distribution)
                {
                    if (LastImuHeaderStamp > 0)
                    {
                        if (fabs((CurImuHeaderStamp - LastImuHeaderStamp) - 0.01) > 0.005)
                        {
                            bad_imu_stamp_distribution = true;
                            std::cout << YELLOW << "fabs((CurImuHeaderStamp - LastImuHeaderStamp) - 0.01) > 0.005 ..." << RESET << std::endl;
                            std::cout << "pro msg num is : " << messageCount << std::endl;
                            std::cout << "data cur stamp is : " << std::setprecision(19) << CurImuHeaderStamp << ", and data last stamp is : " << LastImuHeaderStamp << std::endl;
                            problem_item_index.push_back(imu_msg_pro4);
                        }
                    }
                }

                if (!wrong_axis_num_flag)
                {
                    //? 如何判断一个imu msg是6还是9
                    bool has_valid_orientation = isValidOrientation(current_imu_msg->orientation);
                    bool has_valid_orientation_covariance = isValidCovariance(current_imu_msg->orientation_covariance);
                    if (has_valid_orientation && has_valid_orientation_covariance)
                    {
                        std::cout << GREEN << "Imu is 9 axis" << RESET << std::endl;
                    }
                    else
                    {
                        std::cout << RED << "IMU is not matched, its 6-axis or others" << RESET << std::endl;
                        problem_item_index.push_back(imu_msg_pro5);
                    }
                    wrong_axis_num_flag = true;
                }

                if (!g_check)
                {
                    if (checkProximity(current_imu_msg->linear_acceleration.z))
                    {
                        is_m2s = true;
                        std::cout << GREEN << "Imu acc data unit is m2/s " << RESET << std::endl;
                    }
                    else
                    {
                        is_g = true;
                        std::cout << GREEN << "Imu acc data unit is g " << RESET << std::endl;
                    }
                    g_check = true;
                }
                if (is_m2s)
                {
                    if (current_imu_msg->linear_acceleration.z >= 12.25 || current_imu_msg->linear_acceleration.z <= 7.35) // 1.25 0.75
                    {
                        exceeding_num++;
                    }
                }
                else if (is_g)
                {
                    if (current_imu_msg->linear_acceleration.z >= 1.25 || current_imu_msg->linear_acceleration.z <= 0.75)
                    {
                        exceeding_num++;
                    }
                }
                LastImuHeaderStamp = current_imu_msg->header.stamp.toSec();
            }
        }
    }

    if (found_imu_topic_flag)
    {
        std::cout << GREEN << "Input bag contains sensor_msgs/Imu type messages, and topic is 'imu/data'" << RESET << std::endl;
        if (messageCount > 1)
        {
            double duration = (endTime - startTime).toSec();
            double averageFrequency = (messageCount - 1) / duration;
            if (fabs(averageFrequency - 100) < 2.5)
                std::cout << GREEN << "Average frequency of sensor_msgs/Imu messages in topic 'imu/data' is : " << std::setprecision(4) << averageFrequency << " Hz" << RESET << std::endl;
            else
            {
                std::cout << YELLOW << "Average frequency of sensor_msgs/Imu messages in topic 'imu/data' is not quite equal 100 hz, calculation is : " << std::setprecision(4) << averageFrequency << " Hz" << RESET << std::endl;
                problem_item_index.push_back(imu_msg_pro2);
            }
            if (1.0 * exceeding_num / messageCount > 0.05)
            {
                std::cout << YELLOW << "Imu z acc data exceeding num is more than 5% " << RESET << std::endl;
                problem_item_index.push_back(imu_msg_pro6);
            }
        }
    }
    else
    {
        std::cout << RED << "Can not found Imu matching information in bag" << RESET << std::endl;
        problem_item_index.push_back(imu_msg_pro1);
    }

    in_bag.close();
    return 1;
}