#pragma once

#include "common.h"

class YamlChecker
{

public:
    int check_yaml(std::string yaml_path, std::vector<problem> problem_item_index);
    YamlChecker(const std::string &filePath, std::vector<problem> &data);
private:
    std::string yaml_path;
    std::vector<problem> &problem_item_index;
};
