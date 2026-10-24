#pragma once
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <string>
namespace navigation2d::configuration_detail {
inline double Number(const YAML::Node& node, const char* key) {
  if (!node[key]) {
    throw std::runtime_error(std::string("missing configuration key: ") + key);
  }
  const double value = node[key].as<double>();
  if (!std::isfinite(value)) {
    throw std::runtime_error(std::string("non-finite configuration key: ") + key);
  }
  return value;
}
inline void ExactKeys(const YAML::Node& node, std::initializer_list<const char*> expected,
                      const std::string& path) {
  if (!node.IsMap()) {
    throw std::runtime_error(path + " must be a mapping");
  }
  for (const auto& item : node) {
    const std::string key = item.first.as<std::string>();
    bool known = false;
    for (const char* value : expected) {
      known = known || key == value;
    }
    if (!known) {
      throw std::runtime_error("unknown configuration key: " + path + "." + key);
    }
  }
  for (const char* key : expected) {
    if (!node[key]) {
      throw std::runtime_error("missing configuration key: " + path + "." + key);
    }
  }
}
}  // namespace navigation2d::configuration_detail
