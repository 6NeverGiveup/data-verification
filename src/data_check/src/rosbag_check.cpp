#include "rosbag_check.h"

RosbagChecker::RosbagChecker(const std::string &filePath, std::vector<problem> &data)
    : rosbag_pointcloud_path(filePath), problem_item_index(data)
{
    check_rosbag_pointcloud(rosbag_pointcloud_path, problem_item_index);
}

int RosbagChecker::check_rosbag_pointcloud(std::string rosbag_pointcloud_path, std::vector<problem> &problem_item_index)
{
    // todo 校验是否以.bag结尾，否则弹出
    rosbag::Bag in_bag;
    in_bag.open(rosbag_pointcloud_path, rosbag::bagmode::Read);

    //? ProA. topic校验，信息类型校验，基本频率校验
    rosbag::View view(in_bag, rosbag::TopicQuery("/points_raw"));
    for (rosbag::MessageInstance const &msg : view)
    {
        if (msg.isType<sensor_msgs::PointCloud2>())
        {
            if (!found_cloud_topic_flag)
                found_cloud_topic_flag = true;
            if (messageCount == 0)
            {
                startTime = msg.getTime();
            }
            endTime = msg.getTime();
            messageCount++;

            //? ProB. bag中点云的时间序列查验，点云时间排列均匀性，点的时间序列查验
            //? ProC. bag中的点云依次检查宽、高，数目
            sensor_msgs::PointCloud2ConstPtr current_pt_msg = msg.instantiate<sensor_msgs::PointCloud2>();
            if (current_pt_msg != nullptr)
            {
                //? check PointCloudType. no pro here
                if (!check_type_flag)
                {
                    arr = checkPointCloudType(current_pt_msg);
                    check_type_flag = true;
                }
                if (arr[3] == 1)
                {
                    using PointXYZIRT = PointXYZIRT_444128;
                    pcl::PointCloud<PointXYZIRT>::Ptr Cur_cloud(new pcl::PointCloud<PointXYZIRT>);
                    pcl::fromROSMsg(*current_pt_msg, *Cur_cloud);
                    if (!wrong_pt_order)
                    {
                        for (size_t i = 0; i < Cur_cloud->size() - 1; i++)
                        {
                            if (Cur_cloud->points[i].timestamp > Cur_cloud->points[i + 1].timestamp)
                            {
                                wrong_pt_order = true;
                                std::cout << RED << "Cur_cloud->points[i].timestamp > Cur_cloud->points[i + 1].timestamp! " << RESET << std::endl;
                                std::cout << "msg num is : " << messageCount << ", and current cloud size is : "  << Cur_cloud->size() << ", and point index is : " << i << std::endl;
                                std::cout << "Cur_cloud->points[i].timestamp is : " << std::setprecision(19) << Cur_cloud->points[i].timestamp << ", and Cur_cloud->points[i + 1].timestamp is : " << Cur_cloud->points[i + 1].timestamp << std::endl;
                                problem_item_index.push_back(rosbag_cloud_pro4);
                                break;
                            }
                        }
                    }
                    if (!wrong_HW_size)
                    {
                        if (Cur_cloud->height * Cur_cloud->width != Cur_cloud->size())
                        {
                            wrong_HW_size = true;
                            std::cout << RED << "Cur_cloud->height * Cur_cloud->width not match Cur_cloud->size()! " << RESET << std::endl;
                            std::cout << "Cur_cloud->height is : " << Cur_cloud->height << " and Cur_cloud->width is : " << Cur_cloud->width << " and Cur_cloud->size() is : " << Cur_cloud->size() << std::endl;
                            problem_item_index.push_back(rosbag_cloud_pro5);
                        }
                    }
                    if (!is_dense)
                    {
                        if (!Cur_cloud->is_dense)
                        {
                            is_dense = true;
                            std::cout << RED << "is_dense false! " << RESET << std::endl;
                            problem_item_index.push_back(rosbag_cloud_pro7);
                        }
                    }
                    if (!is_nan)
                    {
                        for (size_t i = 0; i < Cur_cloud->size() - 1; i++)
                        {
                            if (isnan(Cur_cloud->points[i].x) || isnan(Cur_cloud->points[i].y) || isnan(Cur_cloud->points[i].z))
                            {
                                is_nan = true;
                                std::cout << RED << "is_nan false! " << RESET << std::endl;
                                problem_item_index.push_back(rosbag_cloud_pro8);
                                break;
                            }
                        }
                    }
                    if (!small_pt_sum)
                    {
                        if (Cur_cloud->size() < 300)
                        {
                            small_pt_sum = true;
                            std::cout << RED << "small_pt_sum! " << RESET << std::endl;
                            problem_item_index.push_back(rosbag_cloud_pro9);
                        }
                    }
                }
                else if (arr[3] == 2)
                {
                    using PointXYZIRT = PointXYZIRT_444228;
                    pcl::PointCloud<PointXYZIRT>::Ptr Cur_cloud(new pcl::PointCloud<PointXYZIRT>);
                    pcl::fromROSMsg(*current_pt_msg, *Cur_cloud);
                    if (!wrong_pt_order)
                    {
                        for (size_t i = 0; i < Cur_cloud->size() - 1; i++)
                        {
                            if (Cur_cloud->points[i].timestamp > Cur_cloud->points[i + 1].timestamp)
                            {
                                wrong_pt_order = true;
                                std::cout << RED << "Cur_cloud->points[i].timestamp > Cur_cloud->points[i + 1].timestamp! " << RESET << std::endl;
                                std::cout << "msg num is : " << messageCount << ", and current cloud size is : "  << Cur_cloud->size() << ", and point index is : " << i << std::endl;
                                std::cout << "Cur_cloud->points[i].timestamp is : " << std::setprecision(19) << Cur_cloud->points[i].timestamp << ", and Cur_cloud->points[i + 1].timestamp is : " << Cur_cloud->points[i + 1].timestamp << std::endl;
                                problem_item_index.push_back(rosbag_cloud_pro4);
                                break;
                            }
                        }
                    }
                    if (!wrong_HW_size)
                    {
                        if (Cur_cloud->height * Cur_cloud->width != Cur_cloud->size())
                        {
                            wrong_HW_size = true;
                            std::cout << RED << "Cur_cloud->height * Cur_cloud->width not match Cur_cloud->size()! " << RESET << std::endl;
                            std::cout << "Cur_cloud->height is : " << Cur_cloud->height << " and Cur_cloud->width is : " << Cur_cloud->width << " and Cur_cloud->size() is : " << Cur_cloud->size() << std::endl;
                            problem_item_index.push_back(rosbag_cloud_pro5);
                        }
                    }
                    if (!is_dense)
                    {
                        if (!Cur_cloud->is_dense)
                        {
                            is_dense = true;
                            std::cout << RED << "is_dense false! " << RESET << std::endl;
                            problem_item_index.push_back(rosbag_cloud_pro7);
                        }
                    }
                    if (!is_nan)
                    {
                        for (size_t i = 0; i < Cur_cloud->size() - 1; i++)
                        {
                            if (isnan(Cur_cloud->points[i].x) || isnan(Cur_cloud->points[i].y) || isnan(Cur_cloud->points[i].z))
                            {
                                is_nan = true;
                                std::cout << RED << "is_nan false! " << RESET << std::endl;
                                problem_item_index.push_back(rosbag_cloud_pro8);
                                break;
                            }
                        }
                    }
                    if (!small_pt_sum)
                    {
                        if (Cur_cloud->size() < 300)
                        {
                            small_pt_sum = true;
                            std::cout << RED << "small_pt_sum! " << RESET << std::endl;
                            problem_item_index.push_back(rosbag_cloud_pro9);
                        }
                    }
                }
                else if (arr[3] == 4)
                {
                    using PointXYZIRT = PointXYZIRT_444428;
                    pcl::PointCloud<PointXYZIRT>::Ptr Cur_cloud(new pcl::PointCloud<PointXYZIRT>);
                    pcl::fromROSMsg(*current_pt_msg, *Cur_cloud);
                    if (!wrong_pt_order)
                    {
                        for (size_t i = 0; i < Cur_cloud->size() - 1; i++)
                        {
                            if (Cur_cloud->points[i].timestamp > Cur_cloud->points[i + 1].timestamp)
                            {
                                wrong_pt_order = true;
                                std::cout << RED << "Cur_cloud->points[i].timestamp > Cur_cloud->points[i + 1].timestamp! " << RESET << std::endl;
                                std::cout << "msg num is : " << messageCount << ", and current cloud size is : "  << Cur_cloud->size() << ", and point index is : " << i << std::endl;
                                std::cout << "Cur_cloud->points[i].timestamp is : " << std::setprecision(19) << Cur_cloud->points[i].timestamp << ", and Cur_cloud->points[i + 1].timestamp is : " << Cur_cloud->points[i + 1].timestamp << std::endl;
                                problem_item_index.push_back(rosbag_cloud_pro4);
                                break;
                            }
                        }
                    }
                    if (!wrong_HW_size)
                    {
                        if (Cur_cloud->height * Cur_cloud->width != Cur_cloud->size())
                        {
                            wrong_HW_size = true;
                            std::cout << RED << "Cur_cloud->height * Cur_cloud->width not match Cur_cloud->size()! " << RESET << std::endl;
                            std::cout << "Cur_cloud->height is : " << Cur_cloud->height << " and Cur_cloud->width is : " << Cur_cloud->width << " and Cur_cloud->size() is : " << Cur_cloud->size() << std::endl;
                            problem_item_index.push_back(rosbag_cloud_pro5);
                        }
                    }
                    if (!is_dense)
                    {
                        if (!Cur_cloud->is_dense)
                        {
                            is_dense = true;
                            std::cout << RED << "is_dense false! " << RESET << std::endl;
                            problem_item_index.push_back(rosbag_cloud_pro7);
                        }
                    }
                    if (!is_nan)
                    {
                        for (size_t i = 0; i < Cur_cloud->size() - 1; i++)
                        {
                            if (isnan(Cur_cloud->points[i].x) || isnan(Cur_cloud->points[i].y) || isnan(Cur_cloud->points[i].z))
                            {
                                is_nan = true;
                                std::cout << RED << "is_nan false! " << RESET << std::endl;
                                problem_item_index.push_back(rosbag_cloud_pro8);
                                break;
                            }
                        }
                    }
                    if (!small_pt_sum)
                    {
                        if (Cur_cloud->size() < 300)
                        {
                            small_pt_sum = true;
                            std::cout << RED << "small_pt_sum! " << RESET << std::endl;
                            problem_item_index.push_back(rosbag_cloud_pro9);
                        }
                    }
                }
                else
                {
                    std::cout << RED << "data type is not standard! " << RESET << std::endl;
                }

                double CurCloudHeaderStamp = current_pt_msg->header.stamp.toSec();
                // control element maximum occurrence once in problem_item_index
                if (!wrong_cld_order)
                {
                    if (CurCloudHeaderStamp < LastCloudHeaderStamp)
                    {
                        wrong_cld_order = true;
                        std::cout << RED << "CurCloudHeaderStamp < LastCloudHeaderStamp! " << RESET << std::endl;
                        std::cout << "data cur stamp is : " << std::setprecision(19) << CurCloudHeaderStamp << " data last stamp is : " << LastCloudHeaderStamp << std::endl;
                        problem_item_index.push_back(rosbag_cloud_pro3);
                    }
                }

                if (!bad_cld_stamp_distribution)
                {
                    if (LastCloudHeaderStamp > 0)
                    {
                        if (fabs((CurCloudHeaderStamp - LastCloudHeaderStamp) - 0.1) > 0.05)
                        {
                            bad_cld_stamp_distribution = true;
                            std::cout << YELLOW << "fabs((CurCloudHeaderStamp - LastCloudHeaderStamp) - 0.1) > 0.05 ..." << RESET << std::endl;
                            std::cout << "pro msg num is : " << messageCount << std::endl;
                            std::cout << "data cur stamp is : " << std::setprecision(19) << CurCloudHeaderStamp << ", and data last stamp is : " << LastCloudHeaderStamp << std::endl;
                            problem_item_index.push_back(rosbag_cloud_pro6);
                        }
                    }
                }
                LastCloudHeaderStamp = current_pt_msg->header.stamp.toSec();
            }
        }
    }
    if (found_cloud_topic_flag)
    {
        std::cout << GREEN << "Input bag contains sensor_msgs/PointCloud2 type messages, and topic is '/points_raw'" << RESET << std::endl;
        if (messageCount > 1)
        {
            // 计算指定话题消息的频率
            double duration = (endTime - startTime).toSec();
            double averageFrequency = (messageCount - 1) / duration;
            if (fabs(averageFrequency - 10) < 0.5)
                std::cout << GREEN << "Average frequency of sensor_msgs/PointCloud2 messages in topic '/points_raw' is : " << std::setprecision(4) << averageFrequency << " Hz" << RESET << std::endl;
            else
            {
                std::cout << YELLOW << "Average frequency of sensor_msgs/PointCloud2 messages in topic '/points_raw' is not quite equal 10 hz, calculation is : " << std::setprecision(4) << averageFrequency << " Hz" << RESET << std::endl;
                problem_item_index.push_back(rosbag_cloud_pro2);
            }
        }
    }
    else
    {
        std::cout << RED << "Can not found pointcloud matching information in bag" << RESET << std::endl;
        problem_item_index.push_back(rosbag_cloud_pro1);
    }

    in_bag.close();
    return 1;
}

int RosbagChecker::printFieldType(const sensor_msgs::PointField &field) const
{
    std::cout << "Field name: " << field.name << ", Data type: ";
    switch (field.datatype)
    {
    case sensor_msgs::PointField::INT8:
        std::cout << "INT8 (1 byte)" << std::endl;
        return 1;
    case sensor_msgs::PointField::UINT8:
        std::cout << "UINT8 (1 byte)" << std::endl;
        return 1;
    case sensor_msgs::PointField::INT16:
        std::cout << "INT16 (2 bytes)" << std::endl;
        return 2;
    case sensor_msgs::PointField::UINT16:
        std::cout << "UINT16 (2 bytes)" << std::endl;
        return 2;
    case sensor_msgs::PointField::INT32:
        std::cout << "INT32 (4 bytes)" << std::endl;
        return 4;
    case sensor_msgs::PointField::UINT32:
        std::cout << "UINT32 (4 bytes)" << std::endl;
        return 4;
    case sensor_msgs::PointField::FLOAT32:
        std::cout << "FLOAT32 (4 bytes)" << std::endl;
        return 4;
    case sensor_msgs::PointField::FLOAT64:
        std::cout << "FLOAT64 (8 bytes)" << std::endl;
        return 8;
    default:
        std::cout << "UNKNOWN" << std::endl;
        return 0;
    }
}

std::array<int, 6> RosbagChecker::checkPointCloudType(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) const
{
    bool has_x = false;
    bool has_y = false;
    bool has_z = false;
    bool has_intensity = false;
    bool has_ring = false;
    bool has_time = false;
    std::array<int, 6> arr;

    for (const auto &field : cloud_msg->fields)
    {
        if (field.name == "x")
        {
            int x_type = printFieldType(field);
            arr[0] = x_type;
            has_x = true;
        }
        if (field.name == "y")
        {
            int y_type = printFieldType(field);
            arr[1] = y_type;
            has_y = true;
        }
        if (field.name == "z")
        {
            int z_type = printFieldType(field);
            arr[2] = z_type;
            has_z = true;
        }
        if (field.name == "intensity")
        {
            int intensity_type = printFieldType(field);
            arr[3] = intensity_type;
            has_intensity = true;
        }
        if (field.name == "ring" || field.name == "r")
        {
            int ring_type = printFieldType(field);
            arr[4] = ring_type;
            has_ring = true;
        }
        if (field.name == "time" || field.name == "timestamp" || field.name == "stamp" || field.name == "t")
        {
            int time_type = printFieldType(field);
            arr[5] = time_type;
            has_time = true;
        }
    }

    if (has_intensity && !has_ring && !has_time)
    {
        std::cout << "PointCloud type: pcl::PointXYZI" << std::endl;
    }
    else if (has_intensity && has_ring && has_time)
    {
        std::cout << "PointCloud type: pcl::PointXYZIRT" << std::endl;
    }
    else
    {
        std::cout << "Unknown PointCloud type" << std::endl;
    }
    return arr;
}