#pragma once

#include "common.h"

class PcdChecker
{
public:
    PcdChecker(const std::string &filePath, std::vector<problem> &data);
    bool print_flag_ascii = false;
    bool print_flag_binary = false;
    bool print_flag_binary_compressed = false;
    bool print_flag_unknown = false;

    bool flag_pcd_cloud_pro1 = false;
    bool flag_pcd_cloud_pro2 = false;
    bool flag_pcd_cloud_wrong_pt_order = false;
    bool flag_pcd_wrong_HW_size = false;
    bool flag_pcd_is_nan = false;
    bool flag_small_pt_sum = false;

    bool print_xyzirt = false;
    bool print_xyzi = false;
    bool print_xyz = false;
    bool print_no_matched_type = false;
    bool not_cloud_444x28 = false;

private:
    std::string pcd_pointcloud_path;
    std::vector<problem> &problem_item_index;
    int check_pcd_pointcloud(std::string pcd_pointcloud_path, std::vector<problem> &problem_item_index);
    int pcdTypeCheck(const std::string &filename);
    int cloudTypeCheck(const std::string &filename);
    std::string normalizeFieldName(const std::string &field);
    std::vector<std::string> split(const std::string &str, char delimiter);
    PCDType getPCDType(const std::string &filename);
};