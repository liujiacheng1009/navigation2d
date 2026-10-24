#pragma once
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

#include "navigation2d/test/temporary_directory.h"
namespace navigation2d::test {
class NavigationConfigurationFixture {
 public:
  NavigationConfigurationFixture()
      : original_(
            YAML::LoadFile(std::string(NAVIGATION2D_SOURCE_DIR) + "/config/navigation2d.yaml")) {
    std::ofstream map(directory_.path() / "map.json");
    map << R"({"width":160,"height":100,"resolution":)" << std::setprecision(17)
        << original_["map"]["resolution"].as<double>() << R"(,"cells":[)";
    for (int i = 0; i < 16000; ++i) {
      map << (i ? ",0" : "0");
    }
    map << "]}";
    if (!map) {
      throw std::runtime_error("cannot write fixture map");
    }
  }
  NavigationConfigurationFixture(const NavigationConfigurationFixture&) = delete;
  NavigationConfigurationFixture& operator=(const NavigationConfigurationFixture&) = delete;
  YAML::Node Fresh() const { return YAML::Clone(original_); }
  std::string Write(const YAML::Node& node) const {
    const auto path = directory_.path() / "configuration.yaml";
    std::ofstream output(path);
    output << YAML::Dump(node);
    if (!output) {
      throw std::runtime_error("cannot write configuration fixture");
    }
    return path.string();
  }
  std::string map() const { return (directory_.path() / "map.json").string(); }

 private:
  TemporaryDirectory directory_;
  YAML::Node original_;
};
}  // namespace navigation2d::test
