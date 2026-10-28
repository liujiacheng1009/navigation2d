#include "navigation2d/ros/exploration/debug_reporter.h"

#include "navigation2d/application/exploration/debug_overlay.h"
#include "navigation2d/ros/exploration/map_observation.h"

namespace navigation2d::exploration_ros {
void DebugReporter::ClearPaths(DebugReportSink& sink, const Clock& clock) {
  nav_msgs::msg::Path empty;
  empty.header.stamp = clock();
  empty.header.frame_id = "map";
  sink.SendPath(DebugPathChannel::kGlobal, empty);
  sink.SendPath(DebugPathChannel::kLocal, empty);
  sink.SendPath(DebugPathChannel::kTour, empty);
  paths_.Clear();
}

void DebugReporter::Paths(DebugReportSink& sink, const RouteDebugView& route,
                          const TourDebugView& tour, const Clock& clock) {
  const auto global = paths_.Global(route.route, route.origin_x, route.origin_y, clock());
  sink.SendPath(DebugPathChannel::kGlobal, global);
  const auto local = paths_.Local(global, route.progress_m);
  sink.SendPath(DebugPathChannel::kLocal, local);
  sink.SendCarrot(paths_.Carrot(global.header));
  const auto [tour_path, frontiers] =
      paths_.Tour(global.header, tour.map, tour.robot_x, tour.robot_y, tour.active, tour.goals);
  sink.SendPath(DebugPathChannel::kTour, tour_path);
  sink.SendFrontiers(frontiers);
}

void DebugReporter::PlanningMap(DebugReportSink& sink, const nav_msgs::msg::OccupancyGrid& map,
                                const Pose2d& robot_local, const Clock& clock) const {
  // Unknown is not a wall: only known-free cells receive debug inflation.
  const auto grid = ConvertOccupancyGrid(map);
  nav_msgs::msg::OccupancyGrid input = map;
  input.header.stamp = clock();
  input.header.frame_id = "map";
  exploration::PaintPlanningDebugInput(grid, robot_local, input.data);
  sink.SendMap(DebugMapChannel::kPlanningInput, input);
  nav_msgs::msg::OccupancyGrid inflated = input;
  exploration::PaintInflatedDebugMap(grid, input.data, inflated.data);
  sink.SendMap(DebugMapChannel::kInflated, inflated);
}

void DebugReporter::Frontiers(DebugReportSink& sink, FrontierExplorer& explorer) const {
  // Refresh live FR cells; CG/SG still reflect the last goal-selection snapshot.
  explorer.RefreshDebugFrontierCells();
  std_msgs::msg::String message;
  message.data = exploration::SerializeFrontierDebug(explorer.debug_snapshot());
  sink.SendFrontierDebug(message);
}
}  // namespace navigation2d::exploration_ros
