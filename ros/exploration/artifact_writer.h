#pragma once
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <utility>

#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration_io {
struct ArtifactPaths {
  const std::string& result;
  const std::string& snapshot;
};
struct MissionReportView {
  const char* status;
  const std::string& message;
  const std::string& pose_source;
  const std::string& backend;
  const std::string& global_planner;
  const std::string& local_controller;
  std::uint64_t mppi_commands, rpp_commands;
  double total_distance_m;
  std::size_t known_cells;
  int completed_goals, failed_goals;
  double return_error;
};
// Serialization only. The mission decides success and stops the base before
// this call. Preserves the existing file schema and write ordering.
void WriteArtifacts(const ArtifactPaths& paths, const MissionReportView& report,
                    const ExplorationGrid* map,
                    std::span<const std::pair<double, double>> trajectory);
}  // namespace navigation2d::exploration_io
