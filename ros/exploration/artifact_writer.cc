#include "navigation2d/ros/exploration/artifact_writer.h"

#include <filesystem>
#include <fstream>
namespace navigation2d::exploration_io {
void WriteArtifacts(const ArtifactPaths& paths, const MissionReportView& report,
                    const ExplorationGrid* map,
                    std::span<const std::pair<double, double>> trajectory) {
  std::filesystem::create_directories(std::filesystem::path(paths.result).parent_path());
  if (map) {
    // Persist the exact online occupancy grid used by exploration, together
    // with the trajectory from the selected pose source.  The host-side renderer turns this
    // into the reviewable PNG artifact after the mapper exits.
    std::ofstream snapshot(paths.snapshot);
    snapshot << "{\"width\":" << map->width << ",\"height\":" << map->height
             << ",\"resolution\":" << map->resolution << ",\"origin_x\":" << map->origin_x
             << ",\"origin_y\":" << map->origin_y << ",\"cells\":[";
    for (std::size_t index = 0; index < map->cells.size(); ++index) {
      if (index) {
        snapshot << ',';
      }
      snapshot << static_cast<int>(map->cells[index]);
    }
    snapshot << "],\"trajectory\":[";
    for (std::size_t index = 0; index < trajectory.size(); ++index) {
      if (index) {
        snapshot << ',';
      }
      snapshot << '[' << trajectory[index].first << ',' << trajectory[index].second << ']';
    }
    snapshot << "]}";
  }
  std::ofstream output(paths.result);

  output << "{\n  \"status\": \"" << report.status << "\",\n"
         << "  \"message\": \"" << report.message << "\",\n"
         << "  \"pose_source\": \"" << report.pose_source << "\",\n"
         << "  \"map_source\": \""
         << (report.pose_source == "ground_truth" ? "ground_truth_scan_insertion"
                                                  : "localization2d_scan_matching")
         << "\",\n"
         << "  \"planning\": \"navigation2d/" << report.global_planner << "+"
         << report.local_controller << "\",\n"
         << "  \"last_controller_backend\": \"" << report.backend << "\",\n"
         << "  \"mppi_commands\": " << report.mppi_commands << ",\n"
         << "  \"rpp_commands\": " << report.rpp_commands << ",\n"
         << "  \"travel_distance_m\": " << report.total_distance_m << ",\n"
         << "  \"known_cells\": " << report.known_cells << ",\n"
         << "  \"completed_goals\": " << report.completed_goals << ",\n"
         << "  \"failed_goals\": " << report.failed_goals << ",\n"
         << "  \"return_error_m\": " << report.return_error << "\n}\n";
}
}  // namespace navigation2d::exploration_io
