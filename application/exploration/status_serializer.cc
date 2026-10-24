#include "navigation2d/application/exploration/status_serializer.h"

#include <cmath>

#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
namespace {
std::string EscapeJson(const std::string& value) {
  std::string out;
  out.reserve(value.size() + 8);
  for (const char ch : value) {
    if (ch == '"' || ch == '\\') {
      out.push_back('\\');
    }
    out.push_back(ch);
  }
  return out;
}

}  // namespace
std::string SerializeExplorationStatus(const ExplorationStatusView& view) {
  std::string tour_json = "[";
  bool first_tour = true;
  auto append_tour_goal = [&](const ExplorationGoal& goal) {
    if (!first_tour) {
      tour_json += ",";
    }
    first_tour = false;
    tour_json += "{\"x\":" + std::to_string(goal.x) + ",\"y\":" + std::to_string(goal.y) +
                 ",\"yaw\":" + std::to_string(goal.yaw) +
                 ",\"frontier_x\":" + std::to_string(goal.frontier_x) +
                 ",\"frontier_y\":" + std::to_string(goal.frontier_y) +
                 ",\"frontier_cells\":" + std::to_string(goal.frontier_cells) +
                 ",\"information_gain\":" + std::to_string(goal.information_gain) + "}";
  };
  bool status_published_active = false;
  if (std::isfinite(view.active_goal.x) && std::isfinite(view.active_goal.y)) {
    append_tour_goal(view.active_goal);
    status_published_active = true;
  }
  for (const ExplorationGoal& goal : view.tour) {
    if (status_published_active &&
        std::hypot(goal.x - view.active_goal.x, goal.y - view.active_goal.y) < .05 &&
        std::hypot(goal.frontier_x - view.active_goal.frontier_x,
                   goal.frontier_y - view.active_goal.frontier_y) < .05) {
      status_published_active = false;
      continue;
    }
    append_tour_goal(goal);
  }

  std::string result;
  result =
      "{\"state\":\"" + EscapeJson(view.state) + "\",\"message\":\"" + EscapeJson(view.message) +
      "\",\"mission_phase\":\"" + EscapeJson(view.phase) +
      "\",\"known_cells\":" + std::to_string(view.known_cells) +
      ",\"completed_goals\":" + std::to_string(view.completed_goals) +
      ",\"failed_goals\":" + std::to_string(view.failed_goals) +
      ",\"raw_frontier_cells\":" + std::to_string(view.raw_frontier_cells) +
      ",\"unresolved_frontier_cells\":" + std::to_string(view.unresolved_frontier_cells) +
      ",\"candidates\":" + std::to_string(view.candidates) +
      ",\"tour_remaining\":" + std::to_string(view.tour.size()) + ",\"tour\":" + tour_json + "]" +
      ",\"current_goal\":" +
      (std::isfinite(view.active_goal.x) && std::isfinite(view.active_goal.y)
           ? ("{\"x\":" + std::to_string(view.active_goal.x) +
              ",\"y\":" + std::to_string(view.active_goal.y) +
              ",\"yaw\":" + std::to_string(view.active_goal.yaw) +
              ",\"frontier_x\":" + std::to_string(view.active_goal.frontier_x) +
              ",\"frontier_y\":" + std::to_string(view.active_goal.frontier_y) +
              ",\"frontier_cells\":" + std::to_string(view.active_goal.frontier_cells) +
              ",\"information_gain\":" + std::to_string(view.active_goal.information_gain) + "}")
           : "null") +
      ",\"carrot\":{\"x\":" + std::to_string(view.carrot_x) +
      ",\"y\":" + std::to_string(view.carrot_y) + ",\"yaw\":" + std::to_string(view.carrot_yaw) +
      ",\"valid\":" + (view.carrot_valid ? "true" : "false") +
      "},\"path_progress_m\":" + std::to_string(view.navigation.path_progress_m) +
      ",\"path_length_m\":" + std::to_string(view.navigation.path_length_m) +
      ",\"path_min_clearance_m\":" + std::to_string(view.path_min_clearance_m) +
      ",\"path_max_curvature\":" + std::to_string(view.navigation.path_max_curvature) +
      ",\"replans\":" + std::to_string(view.navigation.replans) +
      ",\"incremental_replans\":" + std::to_string(view.navigation.incremental_replans) +
      ",\"recoveries\":" + std::to_string(view.navigation.recoveries) +
      ",\"emergency_stops\":" + std::to_string(view.navigation.emergency_stops) +
      ",\"controller_backend\":\"" + EscapeJson(view.navigation.backend) + "\",\"nav_status\":\"" +
      EscapeJson(view.navigation.nav_status) + "\",\"nav_phase\":\"" +
      EscapeJson(view.navigation.nav_phase) + "\",\"maneuver\":\"" +
      EscapeJson(view.navigation.maneuver) + "\",\"monitor_action\":\"" +
      EscapeJson(view.navigation.monitor_action) + "\",\"planning_failure_reason\":\"" +
      EscapeJson(view.planning_failure) + "\",\"controller_commanded_motion\":" +
      (view.navigation.controller_commanded ? "true" : "false") +
      ",\"safety_stopped_motion\":" + (view.navigation.safety_stopped ? "true" : "false") +
      ",\"intentional_stop\":" + (view.navigation.intentional_stop ? "true" : "false") +
      ",\"fallback_level\":" + std::to_string(view.navigation.fallback_level) +
      ",\"controller_solve_us\":" + std::to_string(view.navigation.controller_solve_us) +
      ",\"minimum_ttc_s\":" + std::to_string(view.navigation.minimum_ttc_s) +
      ",\"planner_expansions\":" + std::to_string(view.navigation.planner_expansions) +
      ",\"planner_elapsed_s\":" + std::to_string(view.navigation.planner_elapsed_s) +
      ",\"robot_radius_m\":" + std::to_string(navigation2d::kPlanningRadius()) +
      ",\"cmd_vel\":{\"linear\":" + std::to_string(view.navigation.cmd_linear) +
      ",\"angular\":" + std::to_string(view.navigation.cmd_angular) +
      "},\"requested_cmd\":{\"linear\":" + std::to_string(view.navigation.requested_linear) +
      ",\"angular\":" + std::to_string(view.navigation.requested_angular) +
      "},\"published_cmd\":{\"linear\":" + std::to_string(view.navigation.published_linear) +
      ",\"angular\":" + std::to_string(view.navigation.published_angular) +
      "},\"measured_vel\":{\"linear\":" + std::to_string(view.navigation.measured_linear) +
      ",\"angular\":" + std::to_string(view.navigation.measured_angular) + "},\"pose_source\":\"" +
      EscapeJson(view.pose_source) + "\"}";
  return result;
}
}  // namespace navigation2d::exploration
