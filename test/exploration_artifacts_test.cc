#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/ros/exploration/artifact_writer.h"

namespace {
struct Directory {
  std::filesystem::path path;
  Directory() {
    char name[] = "/tmp/exploration-artifacts-XXXXXX";
    const char* created = mkdtemp(name);
    if (!created) {
      throw std::runtime_error("mkdtemp failed");
    }
    path = created;
  }
  ~Directory() {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }
};
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("artifact contract failed");
  }
}
}  // namespace

int main() {
  using namespace navigation2d;
  Directory directory;
  const auto result = (directory.path / "result.json").string();
  const auto snapshot = (directory.path / "snapshot.json").string();
  const std::string message = "returned", source = "localization2d", backend = "rpp";
  const std::string global = "astar", local = "custom_local";
  ExplorationGrid map{2, 2, .1, -1., -2., {0, -1, 100, 0}};
  const std::vector<std::pair<double, double>> trail{{0., 0.}, {.1, .2}};
  exploration_io::WriteArtifacts(
      {result, snapshot},
      {"COMPLETE", message, source, backend, global, local, 0, 4, .5, 3, 1, 2, .03}, &map, trail);
  const auto report = YAML::LoadFile(result);
  Require(report["status"].as<std::string>() == "COMPLETE");
  Require(report["planning"].as<std::string>() == "navigation2d/astar+custom_local");
  Require(report["return_error_m"].as<double>() == .03);
  const auto observed = YAML::LoadFile(snapshot);
  Require(observed["cells"].as<std::vector<int>>() == std::vector<int>({0, -1, 100, 0}));
  Require(observed["trajectory"].size() == 2 && observed["origin_y"].as<double>() == -2.);
}
