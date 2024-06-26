#include "common.h"
#include "rosbag_check.h"
#include "pcd_check.h"
#include "imu_check.h"
#include "gnss_check.h"
#include "yaml_check.h"

int main(int argc, char **argv)
{
    //? 0.读取本程序需要的yaml信息
    std::string yaml_path;
    if (!argv[1])
    {
        std::cout << RED << "Input Yaml Path is invalid! " << RESET << std::endl;
        return -1;
    }
    yaml_path = argv[1];
    YAML::Node basenode = YAML::LoadFile(yaml_path);
    std::string rosbag_pointcloud_path = basenode["rosbag_pointcloud"].as<std::string>();
    std::string pcd_pointcloud_path = basenode["pcd_path"].as<std::string>();
    std::string rosbag_imu_path = basenode["rosbag_imu"].as<std::string>();
    std::string rosbag_gnss_path = basenode["rosbag_gps"].as<std::string>();
    std::string config_file_path = basenode["config_file"].as<std::string>();

    //? 1.读取rosbag的点云消息，检查点云相关内容
    if (rosbag_pointcloud_path.empty())
    {
        std::cout << RED << "YAML INFO rosbag_pointcloud_path is invalid!" << RESET << std::endl;
        return -1;
    }
    std::cout << GREEN << "[Info]: " << RESET << "rosbag check func start !" << std::endl;
    std::vector<problem> rosbag_pro;
    RosbagChecker rosbag_check(rosbag_pointcloud_path, rosbag_pro);
    std::cout << GREEN << "[Info]: " << RESET << "rosbag check done !" << std::endl;

    //? 2.读取pcd的文件，检查点云相关内容
    if (pcd_pointcloud_path.empty())
    {
        std::cout << RED << "YAML INFO pcd_pointcloud_path is invalid! " << RESET << std::endl;
        return -1;
    }
    std::vector<problem> pcd_pro;
    PcdChecker pcd_check(pcd_pointcloud_path, pcd_pro);

    //? 3.读取rosbag的gps和imu信息
    if (rosbag_imu_path.empty())
    {
        std::cout << RED << "YAML INFO rosbag_imu_path is invalid! " << RESET << std::endl;
        return -1;
    }
    std::vector<problem> imu_pro;
    ImuChecker imu_check(rosbag_imu_path, imu_pro);

    if (rosbag_gnss_path.empty())
    {
        std::cout << RED << "YAML INFO rosbag_gnss_path is invalid! " << RESET << std::endl;
        return -1;
    }
    std::vector<problem> gnss_pro;
    GnssChecker gnss_check(rosbag_gnss_path, gnss_pro);

    //? 4.读取配置文件yaml，检查相关内容
    if (config_file_path.empty())
    {
        std::cout << RED << "YAML INFO config_file_path is invalid! " << RESET << std::endl;
        return -1;
    }
    std::vector<problem> yaml_pro;
    YamlChecker yaml_check(config_file_path, yaml_pro);

    //--------------------------------------------------------------------------------------------------------------
    //? 写出到文件
    if (!argv[2])
    {
        std::cout << RED << "Output Path is invalid! " << RESET << std::endl;
        return -1;
    }

    std::string output_path = argv[2];
    std::ofstream pro_file(output_path + "pro_file.txt", std::ios::app | std::ios::out);
    if (!pro_file)
    {
        std::cout << RED << "[PRO] :" << RESET << "pro_file.txt Error opening file!" << std::endl;
        return -1;
    }

    std::cout << GREEN << "[Info] :" << RESET << " rosbag_pro size is : " << rosbag_pro.size() << std::endl;
    if (rosbag_pro.size() > 0)
    {
        for (size_t i = 0; i < rosbag_pro.size(); i++)
        {
            std::cout << RED << "[PRO] :" << RESET << " rosbag_pro is : " << rosbag_pro[i] << std::endl;
            pro_file << match_index(rosbag_pro[i]);
        }
    }

    std::cout << GREEN << "[Info] :" << RESET << " pcd_pro size is : " << pcd_pro.size() << std::endl;
    if (pcd_pro.size() > 0)
    {
        for (size_t i = 0; i < pcd_pro.size(); i++)
        {
            std::cout << RED << "[PRO] :" << RESET << "pcd_pro is : " << pcd_pro[i] << std::endl;
            pro_file << match_index(pcd_pro[i]);
        }
    }

    std::cout << GREEN << "[Info] :" << RESET << " imu_pro size is : " << imu_pro.size() << std::endl;
    if (imu_pro.size() > 0)
    {
        for (size_t i = 0; i < imu_pro.size(); i++)
        {
            std::cout << RED << "[PRO] :" << RESET << "imu_pro is : " << imu_pro[i] << std::endl;
            pro_file << match_index(imu_pro[i]);
        }
    }

    std::cout << GREEN << "[Info] :" << RESET << " gnss_pro size is : " << gnss_pro.size() << std::endl;
    if (gnss_pro.size() > 0)
    {
        for (size_t i = 0; i < gnss_pro.size(); i++)
        {
            std::cout << RED << "[PRO] :" << RESET << "gnss_pro is : " << gnss_pro[i] << std::endl;
            pro_file << match_index(gnss_pro[i]);
        }
    }

    // std::cout << GREEN << "[Info] :" << RESET << " yaml_pro size is : " << yaml_pro.size() << std::endl;
    // if (yaml_pro.size() > 0)
    // {
    //     for (size_t i = 0; i < yaml_pro.size(); i++)
    //     {
    //         std::cout << RED << "[PRO] :" << RESET << "yaml_pro is : " << yaml_pro[i] << std::endl;
    //     }
    // }

    pro_file.close();
    //--------------------------------------------------------------------------------------------------------------

    return 1;
}

std::string match_index(int index)
{
    switch (index)
    {
    case 0:
        return "rosbag包中没有找到标准topic及type的点云\n";
    case 1:
        return "rosbag包中的点云消息频率与10hz相差甚远\n";
    case 2:
        return "rosbag包中存在点云时间序列错误\n";
    case 3:
        return "rosbag包中存在点的时间序列错误\n";
    case 4:
        return "rosbag包中的点云长宽与size不匹配\n";
    case 5:
        return "rosbag包中的点云时间戳分布不均匀\n";
    case 6:
        return "rosbag包中的点云is_dense校验失败\n";
    case 7:
        return "rosbag包中的点云中出现了inf或nan\n";
    case 8:
        return "rosbag包中的点云中点数过少\n";
    case 9:
        return "pcd文件的类型非ascii或binary\n";
    case 10:
        return "pcd点云的type不符合slam需求\n";
    case 11:
        return "pcd点云的点的时间序列错误\n";
    case 12:
        return "pcd点云的长宽与size不匹配\n";
    case 13:
        return "pcd点云的中中出现了inf或nan\n";
    case 14:
        return "pcd点云不是444x28类型\n";
    case 15:
        return "pcd点云中点数过少\n";
    case 16:
        return "rosbag包中没有找到标准topic及type的imu msg\n";
    case 17:
        return "rosbag包中imu频率与100hz相差甚远\n";
    case 18:
        return "rosbag包中imu msg时间序列错误\n";
    case 19:
        return "rosbag包中imu msg时间戳分布不均匀\n";
    case 20:
        return "rosbag包中imu msg非9 axis或6 axis\n";
    case 21:
        return "rosbag包中imu msg存在g error或g bias\n";
    case 22:
        return "rosbag包中没有找到标准topic及type的gnss msg\n";
    case 23:
        return "rosbag包中gnss消息频率与100hz相差甚远\n";
    case 24:
        return "rosbag包中gnss msg时间序列错误\n";
    case 25:
        return "rosbag包中gnss msg时间戳分布不均匀\n";
    case 26:
        return "rosbag包中gnss 消息符合使用LBH的需求\n";
    default:
        std::cout << RED << "match_index func shows unexpected result! " << RESET << std::endl;
        return "\n";
    }
}