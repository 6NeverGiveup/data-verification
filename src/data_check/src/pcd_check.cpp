#include "pcd_check.h"

PcdChecker::PcdChecker(const std::string &filePath, std::vector<problem> &data)
    : pcd_pointcloud_path(filePath), problem_item_index(data)
{
    check_pcd_pointcloud(pcd_pointcloud_path, problem_item_index);
}

PCDType PcdChecker::getPCDType(const std::string &filename)
{
    std::ifstream file(filename.c_str());
    if (!file.is_open())
    {
        std::cout << RED << "Failed to open file: " << filename << RESET << std::endl;
        return UNKNOWN;
    }
    std::string line;
    while (std::getline(file, line))
    {
        if (line.find("DATA") != std::string::npos)
        {
            if (line.find("ascii") != std::string::npos)
                return ASCII;
            else if (line.find("binary_compressed") != std::string::npos)
                return BINARY_COMPRESSED;
            else if (line.find("binary") != std::string::npos)
                return BINARY;
            else
                return UNKNOWN;
        }
    }
    return UNKNOWN;
}

// Helper function to split a string by a delimiter
std::vector<std::string> PcdChecker::split(const std::string &str, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

// Function to normalize field names
std::string PcdChecker::normalizeFieldName(const std::string &field)
{
    static std::unordered_map<std::string, std::string> fieldMap = {
        {"x", "x"},
        {"y", "y"},
        {"z", "z"},
        {"i", "intensity"},
        {"intensity", "intensity"},
        {"r", "ring"},
        {"ring", "ring"},
        {"t", "timestamp"},
        {"timestamp", "timestamp"},
        {"stamp", "timestamp"},
        {"time", "timestamp"}};

    auto it = fieldMap.find(field);
    if (it != fieldMap.end())
    {
        return it->second;
    }
    return "";
}

// Function to check if the PCD file has the necessary fields
int PcdChecker::pcdTypeCheck(const std::string &filename)
{
    std::ifstream file(filename.c_str());
    if (!file.is_open())
    {
        std::cout << RED << "Failed to open file: " << filename << RESET << std::endl;
        return -1;
    }

    std::unordered_set<std::string> requiredFieldsXYZIRT = {"x", "y", "z", "intensity", "ring", "timestamp"};
    std::unordered_set<std::string> requiredFieldsXYZI = {"x", "y", "z", "intensity"};
    std::unordered_set<std::string> requiredFieldsXYZ = {"x", "y", "z"};
    std::unordered_set<std::string> fileFields;

    std::string line;
    while (std::getline(file, line))
    {
        // Find the line that starts with "FIELDS"
        if (line.find("FIELDS") != std::string::npos)
        {
            // Split the line to get the fields
            std::vector<std::string> fields = split(line, ' ');
            // Remove the first element "FIELDS"
            fields.erase(fields.begin());
            // Normalize and collect the fields
            for (const auto &field : fields)
            {
                std::string normalizedField = normalizeFieldName(field);
                if (!normalizedField.empty())
                    fileFields.insert(normalizedField);
                // std::cout << "cout fields : " << normalizedField << std::endl;
            }
            // Check if the fields match any of the required field sets
            auto containsAll = [](const std::unordered_set<std::string> &fileFields, const std::unordered_set<std::string> &requiredFields)
            {
                for (const auto &field : requiredFields)
                {
                    if (fileFields.find(field) == fileFields.end())
                    {
                        return false;
                    }
                }
                return true;
            };
            if (containsAll(fileFields, requiredFieldsXYZIRT))
            {
                return 1;
            }
            if (containsAll(fileFields, requiredFieldsXYZI))
            {
                return 2;
            }
            if (containsAll(fileFields, requiredFieldsXYZ))
            {
                return 3;
            }

            return -1; // Return -1 if none of the combinations match
        }
    }
    return -1;
}

int PcdChecker::cloudTypeCheck(const std::string &filename)
{
    std::ifstream file(filename.c_str());
    if (!file.is_open())
    {
        std::cout << RED << "Failed to open file: " << filename << RESET << std::endl;
        return -1;
    }
    std::vector<std::string> fields;
    std::vector<std::string> sizes;
    std::string line;

    while (std::getline(file, line))
    {
        // Find the lines that start with "FIELDS" and "SIZE"
        if (line.find("FIELDS") != std::string::npos)
        {
            fields = split(line, ' ');
            if (!fields.empty())
            {
                fields.erase(fields.begin()); // Remove the "FIELDS" keyword
            }
        }
        else if (line.find("SIZE") != std::string::npos)
        {
            sizes = split(line, ' ');
            if (!sizes.empty())
            {
                sizes.erase(sizes.begin()); // Remove the "SIZE" keyword
            }
        }
    }
    // Ensure that we have valid FIELDS and SIZE lines
    if (fields.empty() || sizes.empty() || fields.size() != sizes.size())
    {
        std::cout << RED << "Invalid PCD file format: missing SIZE information." << RESET << std::endl;
        return -1;
    }
    for (size_t i = 0; i < fields.size(); ++i)
    {
        fields[i] = normalizeFieldName(fields[i]);
    }
    auto it = std::find(fields.begin(), fields.end(), "intensity");
    if (it != fields.end())
    {
        size_t index = std::distance(fields.begin(), it);
        int size = std::stoi(sizes[index]);
        return size;
    }
    return -1;
}

int PcdChecker::check_pcd_pointcloud(std::string pcd_pointcloud_path, std::vector<problem> &problem_item_index)
{
    DIR *dir;
    struct dirent *ptr;
    std::vector<std::string> pcd_file_list;
    const char *p = pcd_pointcloud_path.c_str();
    dir = opendir(p);
    while ((ptr = readdir(dir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
            continue;
        if (strcmp(ptr->d_name, "pcd") == 0)
            continue;
        pcd_file_list.push_back(ptr->d_name);
    }
    closedir(dir);
    std::sort(pcd_file_list.begin(), pcd_file_list.end());

    for (int num_index = 0; num_index < pcd_file_list.size(); num_index++)
    {
        std::string pcd_fileName = pcd_file_list[num_index];
        std::string pcd_filePath = pcd_pointcloud_path + pcd_fileName;
        PCDType type = getPCDType(pcd_filePath);
        switch (type)
        {
        case ASCII:
            if (!print_flag_ascii)
            {
                std::cout << GREEN << "PCD file type: ASCII" << RESET << std::endl;
                std::cout << "current pcd file num_index is : " << num_index << std::endl;
                print_flag_ascii = true;
            }
            break;
        case BINARY:
            if (!print_flag_binary)
            {
                std::cout << "PCD file num_index is : " << num_index << std::endl;
                std::cout << GREEN << "PCD file type: Binary" << RESET << std::endl;
                print_flag_binary = true;
            }
            break;
        case BINARY_COMPRESSED:
            if (!print_flag_binary_compressed)
            {
                std::cout << "PCD file num_index is : " << num_index << std::endl;
                std::cout << GREEN << "PCD file type: Binary Compressed" << RESET << std::endl;
                print_flag_binary_compressed = true;
            }
            break;
        default:
            if (!print_flag_unknown)
            {
                std::cout << "PCD file num_index is : " << num_index << std::endl;
                std::cout << RED << "PCD file type: Unknown" << RESET << std::endl;
                print_flag_unknown = true;
            }
            flag_pcd_cloud_pro1 = true;
            break;
        }

        int pcd_type = pcdTypeCheck(pcd_filePath);
        if (pcd_type == 1)
        {
            if (!print_xyzirt)
            {
                std::cout << GREEN << "PCD file cloud type: xyzirt " << RESET << std::endl;
                print_xyzirt = true;
            }
        }
        else if (pcd_type == 2)
        {
            if (!print_xyzi)
            {
                std::cout << RED << "PCD file cloud type: xyzi " << RESET << std::endl;
                print_xyzi = true;
            }
            flag_pcd_cloud_pro2 = true;
        }
        else if (pcd_type == 3)
        {
            if (!print_xyz)
            {
                std::cout << RED << "PCD file cloud type: xyz " << RESET << std::endl;
                print_xyz = true;
            }
            flag_pcd_cloud_pro2 = true;
        }
        else if (pcd_type == -1)
        {
            if (!print_no_matched_type)
            {
                std::cout << RED << "PCD file cloud type not matched ! " << RESET << std::endl;
                print_no_matched_type = true;
            }
            flag_pcd_cloud_pro2 = true;
        }

        int cloud_type = cloudTypeCheck(pcd_filePath);
        if (cloud_type == 1)
        {
            using PointXYZIRT = PointXYZIRT_444128;
            pcl::PointCloud<PointXYZIRT>::Ptr pcd_pointcloud(new pcl::PointCloud<PointXYZIRT>);
            if (pcl::io::loadPCDFile<PointXYZIRT>(pcd_filePath, *pcd_pointcloud) == -1)
            {
                PCL_ERROR("Couldn't read the pcd file!\n");
                return -1;
            }
            if (!flag_pcd_cloud_wrong_pt_order)
            {
                for (size_t i = 0; i < pcd_pointcloud->size() - 1; i++)
                {
                    if (pcd_pointcloud->points[i].timestamp > pcd_pointcloud->points[i + 1].timestamp)
                    {
                        flag_pcd_cloud_wrong_pt_order = true;
                        std::cout << RED << "pcd_pointcloud->points[i].timestamp > pcd_pointcloud->points[i + 1].timestamp! " << RESET << std::endl;
                        std::cout << "i is : " << i << std::endl;
                        std::cout << "pcd_pointcloud->points[i].timestamp is : " << pcd_pointcloud->points[i].timestamp << " and pcd_pointcloud->points[i + 1].timestamp is : " << pcd_pointcloud->points[i + 1].timestamp << std::endl;
                        problem_item_index.push_back(pcd_cloud_pro3);
                        break;
                    }
                }
            }
            if (!flag_pcd_wrong_HW_size)
            {
                if (pcd_pointcloud->height * pcd_pointcloud->width != pcd_pointcloud->size())
                {
                    flag_pcd_wrong_HW_size = true;
                    std::cout << RED << "pcd_pointcloud->height * pcd_pointcloud->width not match pcd_pointcloud->size()! " << RESET << std::endl;
                    std::cout << "pcd_pointcloud->height is : " << pcd_pointcloud->height << " and pcd_pointcloud->width is : " << pcd_pointcloud->width << " and pcd_pointcloud->size() is : " << pcd_pointcloud->size() << std::endl;
                    problem_item_index.push_back(pcd_cloud_pro4);
                }
            }
            if (!flag_pcd_is_nan)
            {
                for (size_t i = 0; i < pcd_pointcloud->size() - 1; i++)
                {
                    if (isnan(pcd_pointcloud->points[i].x) || isnan(pcd_pointcloud->points[i].y) || isnan(pcd_pointcloud->points[i].z))
                    {
                        flag_pcd_is_nan = true;
                        std::cout << RED << "flag_pcd_is_nan false! " << RESET << std::endl;
                        problem_item_index.push_back(pcd_cloud_pro5);
                        break;
                    }
                }
            }
            if (!flag_small_pt_sum)
            {
                if (pcd_pointcloud->size() < 300)
                {
                    flag_small_pt_sum = true;
                    std::cout << RED << "small_pt_sum! " << RESET << std::endl;
                    problem_item_index.push_back(pcd_cloud_pro7);
                }
            }
        }
        else if (cloud_type == 2)
        {
            using PointXYZIRT = PointXYZIRT_444228;
            pcl::PointCloud<PointXYZIRT>::Ptr pcd_pointcloud(new pcl::PointCloud<PointXYZIRT>);
            if (pcl::io::loadPCDFile<PointXYZIRT>(pcd_filePath, *pcd_pointcloud) == -1)
            {
                PCL_ERROR("Couldn't read the pcd file!\n");
                return -1;
            }
            if (!flag_pcd_cloud_wrong_pt_order)
            {
                for (size_t i = 0; i < pcd_pointcloud->size() - 1; i++)
                {
                    if (pcd_pointcloud->points[i].timestamp > pcd_pointcloud->points[i + 1].timestamp)
                    {
                        flag_pcd_cloud_wrong_pt_order = true;
                        std::cout << RED << "pcd_pointcloud->points[i].timestamp > pcd_pointcloud->points[i + 1].timestamp! " << RESET << std::endl;
                        std::cout << "i is : " << i << std::endl;
                        std::cout << "pcd_pointcloud->points[i].timestamp is : " << pcd_pointcloud->points[i].timestamp << " and pcd_pointcloud->points[i + 1].timestamp is : " << pcd_pointcloud->points[i + 1].timestamp << std::endl;
                        problem_item_index.push_back(pcd_cloud_pro3);
                        break;
                    }
                }
            }
            if (!flag_pcd_wrong_HW_size)
            {
                if (pcd_pointcloud->height * pcd_pointcloud->width != pcd_pointcloud->size())
                {
                    flag_pcd_wrong_HW_size = true;
                    std::cout << RED << "pcd_pointcloud->height * pcd_pointcloud->width not match pcd_pointcloud->size()! " << RESET << std::endl;
                    std::cout << "pcd_pointcloud->height is : " << pcd_pointcloud->height << " and pcd_pointcloud->width is : " << pcd_pointcloud->width << " and pcd_pointcloud->size() is : " << pcd_pointcloud->size() << std::endl;
                    problem_item_index.push_back(pcd_cloud_pro4);
                }
            }
            if (!flag_pcd_is_nan)
            {
                for (size_t i = 0; i < pcd_pointcloud->size() - 1; i++)
                {
                    if (isnan(pcd_pointcloud->points[i].x) || isnan(pcd_pointcloud->points[i].y) || isnan(pcd_pointcloud->points[i].z))
                    {
                        flag_pcd_is_nan = true;
                        std::cout << RED << "flag_pcd_is_nan false! " << RESET << std::endl;
                        problem_item_index.push_back(pcd_cloud_pro5);
                        break;
                    }
                }
            }
            if (!flag_small_pt_sum)
            {
                if (pcd_pointcloud->size() < 300)
                {
                    flag_small_pt_sum = true;
                    std::cout << RED << "small_pt_sum! " << RESET << std::endl;
                    problem_item_index.push_back(pcd_cloud_pro7);
                }
            }
        }
        else if (cloud_type == 4)
        {
            using PointXYZIRT = PointXYZIRT_444428;
            pcl::PointCloud<PointXYZIRT>::Ptr pcd_pointcloud(new pcl::PointCloud<PointXYZIRT>);
            if (pcl::io::loadPCDFile<PointXYZIRT>(pcd_filePath, *pcd_pointcloud) == -1)
            {
                PCL_ERROR("Couldn't read the pcd file!\n");
                return -1;
            }
            if (!flag_pcd_cloud_wrong_pt_order)
            {
                for (size_t i = 0; i < pcd_pointcloud->size() - 1; i++)
                {
                    if (pcd_pointcloud->points[i].timestamp > pcd_pointcloud->points[i + 1].timestamp)
                    {
                        flag_pcd_cloud_wrong_pt_order = true;
                        std::cout << RED << "pcd_pointcloud->points[i].timestamp > pcd_pointcloud->points[i + 1].timestamp! " << RESET << std::endl;
                        std::cout << "i is : " << i << std::endl;
                        std::cout << "pcd_pointcloud->points[i].timestamp is : " << pcd_pointcloud->points[i].timestamp << " and pcd_pointcloud->points[i + 1].timestamp is : " << pcd_pointcloud->points[i + 1].timestamp << std::endl;
                        problem_item_index.push_back(pcd_cloud_pro3);
                        break;
                    }
                }
            }
            if (!flag_pcd_wrong_HW_size)
            {
                if (pcd_pointcloud->height * pcd_pointcloud->width != pcd_pointcloud->size())
                {
                    flag_pcd_wrong_HW_size = true;
                    std::cout << RED << "pcd_pointcloud->height * pcd_pointcloud->width not match pcd_pointcloud->size()! " << RESET << std::endl;
                    std::cout << "pcd_pointcloud->height is : " << pcd_pointcloud->height << " and pcd_pointcloud->width is : " << pcd_pointcloud->width << " and pcd_pointcloud->size() is : " << pcd_pointcloud->size() << std::endl;
                    problem_item_index.push_back(pcd_cloud_pro4);
                }
            }
            if (!flag_pcd_is_nan)
            {
                for (size_t i = 0; i < pcd_pointcloud->size() - 1; i++)
                {
                    if (isnan(pcd_pointcloud->points[i].x) || isnan(pcd_pointcloud->points[i].y) || isnan(pcd_pointcloud->points[i].z))
                    {
                        flag_pcd_is_nan = true;
                        std::cout << RED << "flag_pcd_is_nan false! " << RESET << std::endl;
                        problem_item_index.push_back(pcd_cloud_pro5);
                        break;
                    }
                }
            }
            if (!flag_small_pt_sum)
            {
                if (pcd_pointcloud->size() < 300)
                {
                    flag_small_pt_sum = true;
                    std::cout << RED << "small_pt_sum! " << RESET << std::endl;
                    problem_item_index.push_back(pcd_cloud_pro7);
                }
            }
        }
        else if (cloud_type == -1)
        {
            if (!not_cloud_444x28)
            {
                not_cloud_444x28 = true;
                problem_item_index.push_back(pcd_cloud_pro6);
            }
        }
    }
    //? pcd文件的类型，如ascii，binary
    if (flag_pcd_cloud_pro1)
        problem_item_index.push_back(pcd_cloud_pro1);

    //? pcd点云的type是否符合需求
    if (flag_pcd_cloud_pro2)
        problem_item_index.push_back(pcd_cloud_pro2);

    return 1;
}