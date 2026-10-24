#pragma once
#include <string>

#include "navigation2d/configuration/navigation_config.h"
namespace YAML {
class Node;
}
namespace navigation2d::configuration_detail {
NavigationConfig ReadNavigationYaml(const std::string&);
void ReadPlanningYaml(const YAML::Node&, NavigationConfig&);
void ReadControlYaml(const YAML::Node&, NavigationConfig&);
void ReadSafetyYaml(const YAML::Node&, NavigationConfig&);
}  // namespace navigation2d::configuration_detail
