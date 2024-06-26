#include "yaml_check.h"

YamlChecker::YamlChecker(const std::string &filePath, std::vector<problem> &data)
    : yaml_path(filePath), problem_item_index(data)
{
    check_yaml(yaml_path, problem_item_index);
}

int YamlChecker::check_yaml(std::string yaml_path, std::vector<problem> problem_item_index)
{
    YAML::Node basenode = YAML::LoadFile(yaml_path);
    std::string DataType = basenode["lio_sam"]["DataType"].as<std::string>();
    std::string IgnoredRingNumeberFrom_0_Range = basenode["lio_sam"]["IgnoredRingNumeberFrom_0_Range"].as<std::string>();
    std::string TranslationCompensation = basenode["lio_sam"]["TranslationCompensation"].as<std::string>();
    std::string N_SCAN = basenode["lio_sam"]["N_SCAN"].as<std::string>();
    std::string gpsAddInterval = basenode["lio_sam"]["gpsAddInterval"].as<std::string>();
    std::string gps_noise = basenode["lio_sam"]["gps_noise"].as<std::string>();

    std::cout << GREEN << "[Info]: " << RESET << "DataType is : " << DataType << std::endl;
    std::cout << GREEN << "[Info]: " << RESET << "IgnoredRingNumeberFrom_0_Range is : " << IgnoredRingNumeberFrom_0_Range << std::endl;
    std::cout << GREEN << "[Info]: " << RESET << "TranslationCompensation trigger is : " << TranslationCompensation << std::endl;
    std::cout << GREEN << "[Info]: " << RESET << "N_SCAN is : " << N_SCAN << std::endl;
    std::cout << GREEN << "[Info]: " << RESET << "gpsAddInterval is : " << gpsAddInterval << std::endl;
    std::cout << GREEN << "[Info]: " << RESET << "gps_noise is : " << gps_noise << std::endl;

    return 1;
}