#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigation2d/application/navigation_system.h"
#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/geometry/robot_geometry.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

namespace {

constexpr double kExploreGoalTolerance = .18;

double Yaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2. * (q.w * q.z + q.x * q.y),
                    1. - 2. * (q.y * q.y + q.z * q.z));
}

double NormalizeAngle(double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

double RequiredPositiveEnvironment(const char* name, double fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') return fallback;
  const double parsed = std::stod(value);
  if (!std::isfinite(parsed) || parsed <= 0.)
    throw std::runtime_error(std::string(name) + " must be positive");
  return parsed;
}

using Goal = navigation2d::ExplorationGoal;

navigation2d::FrontierExplorerConfig ExplorerConfig() {
  navigation2d::FrontierExplorerConfig config;
  // Maze doorways often produce short but useful frontier components.  A
  // larger threshold silently drops those branches before they reach the
  // planner; noise is handled by the stability observer below.
  config.minimum_frontier_cells = 6;
  // Keep frontier generation consistent with the navigation costmap: a goal
  // must leave room for the physical footprint and its obstacle buffer, not
  // merely be free in the raw occupancy image. CAD max footprint radius is
  // ~0.207 m; planning uses kPlanningRadius + soft inflation (same as production).
  // Open-space standoff stays well off walls — lidar covers the rim without
  // driving there. Narrow doorways still use kCloseStandoff via SafeViewpoint.
  config.footprint_clearance = .38;
  config.minimum_standoff = .45;
  config.maximum_standoff = 1.20;
  config.blacklist_radius = 1.20;
  config.cover_residuals = true;
  return config;
}

const char* BackendName(navigation2d::ControllerBackend backend) {
  switch (backend) {
    case navigation2d::ControllerBackend::kAcados: return "acados";
    case navigation2d::ControllerBackend::kMppi: return "mppi-fallback";
    case navigation2d::ControllerBackend::kRpp: return "rpp-fallback";
    case navigation2d::ControllerBackend::kDwa: return "dwa";
    case navigation2d::ControllerBackend::kNone: return "none";
  }
  return "unknown";
}

const char* NavStatusName(navigation2d::NavigationStatus status) {
  switch (status) {
    case navigation2d::NavigationStatus::kIdle: return "idle";
    case navigation2d::NavigationStatus::kNavigating: return "navigating";
    case navigation2d::NavigationStatus::kSucceeded: return "succeeded";
    case navigation2d::NavigationStatus::kBlocked: return "blocked";
  }
  return "unknown";
}

const char* NavPhaseName(navigation2d::NavigationPhase phase) {
  switch (phase) {
    case navigation2d::NavigationPhase::kAlignToPath: return "align_to_path";
    case navigation2d::NavigationPhase::kTrackPath: return "track_path";
    case navigation2d::NavigationPhase::kDockToGoal: return "dock_to_goal";
  }
  return "unknown";
}

const char* ManeuverName(navigation2d::ControllerManeuver maneuver) {
  switch (maneuver) {
    case navigation2d::ControllerManeuver::kTracking: return "tracking";
    case navigation2d::ControllerManeuver::kStopping: return "stopping";
    case navigation2d::ControllerManeuver::kRotateToPath: return "rotate_to_path";
    case navigation2d::ControllerManeuver::kRouteInfeasible: return "route_infeasible";
  }
  return "unknown";
}

const char* MonitorActionName(navigation2d::CollisionMonitorAction action) {
  switch (action) {
    case navigation2d::CollisionMonitorAction::kNone: return "none";
    case navigation2d::CollisionMonitorAction::kSlowdown: return "slowdown";
    case navigation2d::CollisionMonitorAction::kStop: return "stop";
    case navigation2d::CollisionMonitorAction::kBlindZoneStop: return "blind_zone_stop";
    case navigation2d::CollisionMonitorAction::kSourceTimeout: return "source_timeout";
  }
  return "unknown";
}

geometry_msgs::msg::Pose PoseFromXyYaw(double x, double y, double yaw) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.z = std::sin(yaw / 2.);
  pose.orientation.w = std::cos(yaw / 2.);
  return pose;
}

// Per-goal recovery budget. Counters survive resume/replan of the same
// viewpoint so side-speck peel cannot reset itself into an infinite loop.
struct GoalRecoverySession {
  double goal_x = 0.;
  double goal_y = 0.;
  int side_peels = 0;
  int open_replans = 0;
  // Open-forward wall scrape: peel→resume same chord is the red-box sawtooth.
  int wall_scrapes = 0;
  bool resume_after_escape = false;
  bool active = false;

  bool SameGoal(double x, double y, double tol = 0.5) const {
    return active && std::hypot(x - goal_x, y - goal_y) <= tol;
  }

  void Begin(double x, double y) {
    if (SameGoal(x, y)) return;
    goal_x = x;
    goal_y = y;
    side_peels = 0;
    open_replans = 0;
    wall_scrapes = 0;
    resume_after_escape = false;
    active = true;
  }

  void Clear() {
    active = false;
    side_peels = 0;
    open_replans = 0;
    wall_scrapes = 0;
    resume_after_escape = false;
  }
};

class AutonomousExplorer final : public rclcpp::Node {
 public:
  AutonomousExplorer() : Node("sweepnav_autonomous_explorer"), explorer_(ExplorerConfig()) {
    pose_source_ = declare_parameter<std::string>("pose_source", "localization2d");
    if (pose_source_ != "ground_truth" && pose_source_ != "localization2d") {
      throw std::runtime_error(
          "pose_source must be 'ground_truth' or 'localization2d'");
    }
    result_path_ = std::getenv("SWEEPNAV_EXPLORATION_RESULT") != nullptr
        ? std::getenv("SWEEPNAV_EXPLORATION_RESULT") : "/tmp/exploration-result.json";
    snapshot_path_ = (std::filesystem::path(result_path_).parent_path() /
                      "exploration_snapshot.json").string();
    max_duration_s_ = RequiredPositiveEnvironment("SWEEPNAV_EXPLORATION_TIMEOUT", 600.);
    simulation_speed_ = RequiredPositiveEnvironment("SWEEPNAV_SIMULATION_SPEED", 1.);
    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_publisher_ = create_publisher<std_msgs::msg::String>(
        "/exploration/status", rclcpp::QoS(1).transient_local().reliable());
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
        "/navigation/path", rclcpp::QoS(1).transient_local().reliable());
    local_trajectory_publisher_ = create_publisher<nav_msgs::msg::Path>(
        "/navigation/local_trajectory", rclcpp::QoS(1).transient_local().reliable());
    tour_publisher_ = create_publisher<nav_msgs::msg::Path>(
        "/navigation/tour", rclcpp::QoS(1).transient_local().reliable());
    carrot_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/navigation/carrot", rclcpp::QoS(1).transient_local().reliable());
    frontiers_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
        "/exploration/frontier_goals", rclcpp::QoS(1).transient_local().reliable());
    frontier_debug_publisher_ = create_publisher<std_msgs::msg::String>(
        "/exploration/frontier_debug", rclcpp::QoS(1).transient_local().reliable());
    planning_input_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/exploration/global_planning_input", rclcpp::QoS(1).transient_local().reliable());
    inflated_costmap_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/exploration/global_costmap_inflated", rclcpp::QoS(1).transient_local().reliable());
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/localization2d/map", rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr value) {
          map_ = *value;
          navigation2d::ExplorationGrid grid;
          grid.width = static_cast<int>(value->info.width);
          grid.height = static_cast<int>(value->info.height);
          grid.resolution = value->info.resolution;
          grid.origin_x = value->info.origin.position.x;
          grid.origin_y = value->info.origin.position.y;
          grid.cells = value->data;
          explorer_.UpdateMap(std::move(grid));
          ++map_revision_;
        });
    if (pose_source_ == "ground_truth") {
      // Planning-only evaluation deliberately bypasses Localization2D's pose.
      // The online mapper still supplies its live occupancy grid, while the
      // controller and explorer receive the simulator's exact pose expressed in
      // the same initial-odometry/map coordinate system used by that grid.
      truth_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
          "/ground_truth", rclcpp::SensorDataQoS(),
          [this](nav_msgs::msg::Odometry::ConstSharedPtr value) {
          const auto& source = value->pose.pose;
          const double source_yaw = Yaw(source.orientation);
          if (!truth_origin_) {
            truth_origin_x_ = source.position.x;
            truth_origin_y_ = source.position.y;
            truth_origin_yaw_ = source_yaw;
            truth_origin_ = true;
          }
          const double dx = source.position.x - truth_origin_x_;
          const double dy = source.position.y - truth_origin_y_;
          geometry_msgs::msg::PoseStamped value_in_map;
          value_in_map.header = value->header;
          value_in_map.header.frame_id = "map";
          value_in_map.pose.position.x = std::cos(truth_origin_yaw_) * dx +
              std::sin(truth_origin_yaw_) * dy;
          value_in_map.pose.position.y = -std::sin(truth_origin_yaw_) * dx +
              std::cos(truth_origin_yaw_) * dy;
          const double yaw = source_yaw - truth_origin_yaw_;
          value_in_map.pose.orientation.z = std::sin(yaw / 2.);
          value_in_map.pose.orientation.w = std::cos(yaw / 2.);
          pose_ = value_in_map;
          const auto point = std::pair<double, double>{
              value_in_map.pose.position.x, value_in_map.pose.position.y};
          if (trajectory_.empty() || std::hypot(point.first - trajectory_.back().first,
                                                point.second - trajectory_.back().second) > .05) {
            if (!trajectory_.empty()) total_distance_m_ += std::hypot(
                point.first - trajectory_.back().first, point.second - trajectory_.back().second);
            trajectory_.push_back(point);
          }
          });
    } else {
      // In the joint localization/navigation evaluation the map and every
      // planning/control decision use Localization2D's scan-matched map pose.
      // Do not subscribe to /ground_truth here: it remains an offline oracle.
      localization_pose_subscription_ =
          create_subscription<geometry_msgs::msg::PoseStamped>(
              "/localization2d/pose", rclcpp::SensorDataQoS(),
              [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr value) {
                pose_ = *value;
                const auto point = std::pair<double, double>{
                    value->pose.position.x, value->pose.position.y};
                if (trajectory_.empty() ||
                    std::hypot(point.first - trajectory_.back().first,
                               point.second - trajectory_.back().second) > .05) {
                  if (!trajectory_.empty()) total_distance_m_ += std::hypot(
                      point.first - trajectory_.back().first,
                      point.second - trajectory_.back().second);
                  trajectory_.push_back(point);
                }
              });
    }
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr value) {
          velocity_ = {value->twist.twist.linear.x, value->twist.twist.angular.z};
          velocity_stamp_ = rclcpp::Time(value->header.stamp);
        });
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr value) {
          scan_ = *value;
          if (navigation_ && pose_) FeedScan();
        });
    timer_ = create_wall_timer(
        std::chrono::duration<double>(.06 / simulation_speed_), [this]() { Tick(); });
    PublishStatus("WAITING", "waiting for " + pose_source_ + " pose and online map");
  }

 private:
  std::size_t KnownCells() const {
    return explorer_.KnownCells();
  }

  navigation2d::Pose2d LocalPose(const geometry_msgs::msg::Pose& pose) const {
    return navigation2d::MakePose2d(pose.position.x - navigation_origin_x_,
                                    pose.position.y - navigation_origin_y_, Yaw(pose.orientation));
  }

  navigation2d::Twist2d FreshVelocity(const rclcpp::Time& reference) const {
    if (velocity_stamp_.nanoseconds() == 0) return {};
    const double age = (reference - velocity_stamp_).seconds();
    // Do not feed a stale encoder sample into acceleration limiting or stall
    // diagnosis. Pose feedback remains the authoritative accumulated-motion
    // signal when the odometry publisher is delayed by simulation scheduling.
    if (!std::isfinite(age) || age < -.20 || age > .20) return {};
    return velocity_;
  }

  navigation2d::LaserScan ConvertScan() const {
    navigation2d::LaserScan result;
    result.angle_min = scan_->angle_min;
    result.angle_increment = scan_->angle_increment;
    result.range_min = scan_->range_min;
    result.range_max = scan_->range_max;
    result.ranges.assign(scan_->ranges.begin(), scan_->ranges.end());
    return result;
  }

  void FeedScan() {
    auto robot = LocalPose(pose_->pose);
    const double yaw = navigation2d::Yaw(robot);
    const auto sensor = navigation2d::MakePose2d(
        navigation2d::X(robot) + .18 * std::cos(yaw),
        navigation2d::Y(robot) + .18 * std::sin(yaw), yaw);
    // The online mapper has already fused this scan into the static map.
    // Do not let instantaneous endpoint noise disconnect its global route;
    // retain the scan for near-field collision monitoring during execution.
    navigation_->UpdateCollisionMonitorLaserScan(sensor, ConvertScan());
  }

  bool StartNavigation(const Goal& requested_goal, bool returning,
                       bool latest_map_retry = false) {
    if (!latest_map_retry) latest_map_replans_ = 0;
    Goal goal = requested_goal;
    if (!returning) {
      if (const auto settled = explorer_.SettleGoal(goal)) {
        if (std::hypot(settled->x - goal.x, settled->y - goal.y) > 1e-6)
          RCLCPP_INFO(get_logger(),
                      "Goal (%.2f, %.2f) too close to a mapped wall; moved to (%.2f, %.2f)",
                      goal.x, goal.y, settled->x, settled->y);
        goal = *settled;
      }
    }
    navigation_origin_x_ = map_->info.origin.position.x;
    navigation_origin_y_ = map_->info.origin.position.y;
    map_snapshot_path_ = "/tmp/sweepnav-navigation-map-" + std::to_string(map_revision_) + ".json";
    std::ofstream output(map_snapshot_path_);
    output << "{\"width\":" << map_->info.width << ",\"height\":" << map_->info.height
           << ",\"resolution\":" << map_->info.resolution << ",\"cells\":[";
    const int robot_col = static_cast<int>(std::floor(
        (pose_->pose.position.x - navigation_origin_x_) / map_->info.resolution));
    const int robot_row = static_cast<int>(std::floor(
        (pose_->pose.position.y - navigation_origin_y_) / map_->info.resolution));
    // The simulator's ground-truth pose guarantees that the complete base
    // footprint is free even when SLAM has not yet converted the cells under
    // the robot from unknown.  Keep that local evidence slightly wider than
    // the planner radius (the body circle), so
    // a valid current pose is not rejected as "start occupied" on a replan
    // after a tight-corner approach.  This only clears the measured current
    // footprint; all cells ahead remain lethal unless observed free.
    // Unknown next to the robot is not yet observed. Occupied is cleared only
    // inside the planning circle — the same disk PathCuts ignores. Clearing
    // occupied out to this wider disk let the planner thread a door jamb and
    // PathCuts then reject every entry as a first-segment replay.
    const double snapshot_clearance = .40;
    const int robot_clearance = static_cast<int>(
        std::ceil(snapshot_clearance / map_->info.resolution));
    const int body_cells = static_cast<int>(
        std::ceil(navigation2d::kPlanningRadius / map_->info.resolution));
    // Near-field lidar is fresher than the online occupancy grid. Preserve
    // only ray interiors (never the hit endpoint) as temporary free evidence
    // for this plan, allowing a certified narrow passage to be entered before
    // the mapper has committed every cell. The hard footprint check remains
    // authoritative and the evidence expires with this plan.
    std::unordered_set<std::size_t> near_field_free;
    if (scan_ && pose_) {
      const double yaw = Yaw(pose_->pose.orientation);
      const double sensor_x = pose_->pose.position.x + .18 * std::cos(yaw);
      const double sensor_y = pose_->pose.position.y + .18 * std::sin(yaw);
      const double evidence_range = std::min(3.0, static_cast<double>(scan_->range_max));
      const double step = std::max(.5 * map_->info.resolution, .02);
      for (std::size_t beam = 0; beam < scan_->ranges.size(); ++beam) {
        const double range = scan_->ranges[beam];
        if (!std::isfinite(range) || range < scan_->range_min || range > evidence_range) continue;
        const double angle = yaw + scan_->angle_min + static_cast<double>(beam) * scan_->angle_increment;
        const double usable = std::max(0., range - std::max(.06, .5 * map_->info.resolution));
        for (double distance = 0.; distance < usable; distance += step) {
          const double x = sensor_x + distance * std::cos(angle);
          const double y = sensor_y + distance * std::sin(angle);
          const int col = static_cast<int>(std::floor((x - navigation_origin_x_) / map_->info.resolution));
          const int row = static_cast<int>(std::floor((y - navigation_origin_y_) / map_->info.resolution));
          if (col < 0 || row < 0 || col >= static_cast<int>(map_->info.width) ||
              row >= static_cast<int>(map_->info.height)) continue;
          near_field_free.insert(static_cast<std::size_t>(row) * map_->info.width + col);
        }
      }
    }
    driven_corridor_.clear();
    if (clear_start_pinch_) {
      // Robot sits inside inflation at a wall corner: reopen the driven trail
      // so the next plan can leave instead of reject-looping every frontier.
      paint_driven_corridor_ = true;
      clear_start_pinch_ = false;
    }
    const bool entry_plan = pending_entry_goal_ || locked_entry_mouth_;
    if (paint_driven_corridor_ && map_->info.resolution > 0. && trajectory_.size() >= 2) {
      // The chassis already drove this corridor. A later scan can pinch the
      // costmap below the planning radius and trap the robot; reopen the
      // driven trail for this one plan. Return uses a long budget so door
      // seals along the outbound path stay plannable. Entry must NOT force
      // occupied cells free — that invented through-wall chords PathCuts then
      // rejected (live: Entry probe rejected reason=path intersects…).
      paint_driven_corridor_ = false;
      const double trail_budget_m =
          paint_trail_budget_m_ > 0. ? paint_trail_budget_m_ : 2.0;
      paint_trail_budget_m_ = 0.;
      const double paint_radius_m =
          entry_plan ? .12 : ((returning || trail_budget_m > 4.) ? .28 : .16);
      const int radius = static_cast<int>(std::ceil(paint_radius_m / map_->info.resolution));
      double walked = 0.;
      for (std::size_t i = trajectory_.size(); i-- > 0;) {
        if (i + 1 < trajectory_.size())
          walked += std::hypot(trajectory_[i].first - trajectory_[i + 1].first,
                                trajectory_[i].second - trajectory_[i + 1].second);
        if (walked > trail_budget_m) break;
        const int trail_col = static_cast<int>(std::floor(
            (trajectory_[i].first - navigation_origin_x_) / map_->info.resolution));
        const int trail_row = static_cast<int>(std::floor(
            (trajectory_[i].second - navigation_origin_y_) / map_->info.resolution));
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            if (dx * dx + dy * dy > radius * radius) continue;
            const int x = trail_col + dx, y = trail_row + dy;
            if (x < 0 || y < 0 || x >= static_cast<int>(map_->info.width) ||
                y >= static_cast<int>(map_->info.height))
              continue;
            const std::size_t index =
                static_cast<std::size_t>(y) * map_->info.width + x;
            // Entry: only reopen unknown / free trail cells, never occupied.
            if (entry_plan && map_->data[index] >= 50) continue;
            driven_corridor_.insert(index);
          }
        }
      }
    }
    // Planning uses mapper occupancy as-is (no door-seal carving).
    navigation2d::ExplorationGrid planning_grid;
    planning_grid.width = static_cast<int>(map_->info.width);
    planning_grid.height = static_cast<int>(map_->info.height);
    planning_grid.resolution = map_->info.resolution;
    planning_grid.origin_x = map_->info.origin.position.x;
    planning_grid.origin_y = map_->info.origin.position.y;
    planning_grid.cells = map_->data;
    // Locked-mouth entry: paint unknown along frontier → this navigation goal
    // (the room-side probe). Hallway-standoff→frontier pointed the wrong way
    // when the locked mouth was already room-side.
    if (entry_plan && map_->info.resolution > 0.) {
      const Goal& mouth = pending_entry_goal_.value_or(
          locked_entry_mouth_.value_or(goal));
      const double fx = mouth.frontier_x;
      const double fy = mouth.frontier_y;
      double ax = goal.x - fx;
      double ay = goal.y - fy;
      if (std::hypot(ax, ay) < .12) {
        ax = fx - mouth.x;
        ay = fy - mouth.y;
      }
      const double alen = std::hypot(ax, ay);
      if (alen > .05) {
        const double ux = ax / alen, uy = ay / alen;
        const double lx = -uy, ly = ux;
        const double res = map_->info.resolution;
        const int width = static_cast<int>(map_->info.width);
        const int height = static_cast<int>(map_->info.height);
        for (double depth = -.10; depth <= 1.20; depth += res) {
          for (double lat = -.14; lat <= .14 + 1e-9; lat += res) {
            const double wx = fx + depth * ux + lat * lx;
            const double wy = fy + depth * uy + lat * ly;
            const int col = static_cast<int>(
                std::floor((wx - navigation_origin_x_) / res));
            const int row = static_cast<int>(
                std::floor((wy - navigation_origin_y_) / res));
            if (col < 0 || row < 0 || col >= width || row >= height) continue;
            const std::size_t index =
                static_cast<std::size_t>(row) * map_->info.width + col;
            if (planning_grid.cells[index] < 0) planning_grid.cells[index] = 0;
          }
        }
      }
    }
    for (std::size_t index = 0; index < planning_grid.cells.size(); ++index) {
      if (index) output << ',';
      // Free is 0. A real obstacle is 254 and inflates. Unknown is 255:
      // search may not enter it, and it is not a wall, so a frontier
      // viewpoint beside it stays a valid goal.
      const int row = static_cast<int>(index / map_->info.width);
      const int col = static_cast<int>(index % map_->info.width);
      const bool under_robot = (col - robot_col) * (col - robot_col) +
          (row - robot_row) * (row - robot_row) <= robot_clearance * robot_clearance;
      const bool under_body = (col - robot_col) * (col - robot_col) +
          (row - robot_row) * (row - robot_row) <= body_cells * body_cells;
      // The robot's current footprint is physically known to be free. SLAM's
      // confidence threshold can otherwise leave isolated unknown cells below
      // the base and make every first plan fail its start-collision check.
      // Occupied outside that circle stays lethal so a doorway plan cannot
      // ride a jamb the live-map check will cut.
      const auto cell = planning_grid.cells[index];
      const bool lidar_evidence = cell < 0 && near_field_free.count(index) > 0;
      const bool driven = driven_corridor_.count(index) > 0;
      const bool unknown_near_robot = under_robot && cell < 0;
      const bool forced_free = cell == 0 || under_body || unknown_near_robot ||
                               lidar_evidence || driven;
      output << (forced_free ? 0 : (cell < 0 ? 255 : 254));
    }
    output << "]}";
    output.close();
    driven_corridor_.clear();
    PublishPlanningDebugMap(returning);
    navigation2d::NavigationConfig config;
    config.map_resolution = map_->info.resolution;
    // Grid search stays inside the explorer Tick budget.  The SE(2) lattice
    // plus its obstacle heuristic can allocate a yaw-binned graph the size of
    // the whole localization map and never return, so the node produces no
    // result.json.  First-segment cooldown, escape, and A* replay rejection
    // already stop rotate-in-place stalls without that planner.
    // Regulated Pure Pursuit is the Nav2 controller used for robust service
    // robot path following. Its velocity regulation around curvature and
    // obstacles avoids MPPI fallback oscillation on partially observed maps.
    config.controller = "rpp";
    // Prefer forward tracking, but allow a short reverse so a U-alcove tip
    // can back out a few tens of centimetres while turning toward the exit.
    config.max_reverse_velocity = .10;
    // CAD: chassis 290×230 mm, wheel envelope half-width 0.147 m, max
    // footprint radius ~0.207 m. Match production navigation2d_node (+margin).
    // Always plan at kPlanningRadius. The old entry 0.14 m / pinch acceptance
    // <0.15 m let GLOBAL thread furniture V-gaps the body cannot fit.
    const bool entry_drive = pending_entry_goal_ || locked_entry_mouth_;
    // Entry: A* only. Theta* long chords cut nearby wall tips even when the
    // locked mouth is metres wide (live: gap=2.15 Entry probe rejected
    // path-intersects → abandon → tour loop → failed_goals thrash).
    config.planner = entry_drive ? "astar" : "theta_star";
    config.robot_radius = navigation2d::kPlanningRadius;
    // Keep the chassis rectangle on a door crawl. Clearing it made the
    // collision monitor take the planning circle as the body width, so a
    // 0.29 m robot was stopped as though it were 0.44 m wide.
    config.footprint = {{-.145, -.147}, {.115, -.147}, {.145, -.132},
                        {.145, .132}, {.115, .147}, {-.145, .147}};
    // Clearance tiers (strategy, not scene knobs):
    //   entry  — locked mouth crawl (softer inflation only)
    //   pinch  — automatic retry when open-tier finds no route through a
    //            door-width throat (any map, not kitchen-specific)
    //   open   — default explore: prefer mid-aisle over wall feet
    // Lethal radius stays kPlanningRadius in all tiers; tiers only change
    // soft inflation.
    const bool pinch_plan = narrow_passage_retry_ && !entry_drive && !returning;
    // Soft inflation must stay below half a typical doorway (~0.35 m for a
    // 0.70 m mouth). Open 0.42 sealed those doors in cost (UI: tan band fills
    // the gap) and made pinch/open retries report no path.
    if (entry_drive) {
      config.inflation_radius = .26;
      config.inflation_cost_scaling = 8.0;
    } else if (pinch_plan) {
      config.inflation_radius = .28;
      config.inflation_cost_scaling = 7.0;
    } else {
      // Stronger soft inflation than pinch: mid-aisle preference. Doors that
      // open-tier cannot thread still fall through via no-route → pinch.
      config.inflation_radius = .36;
      config.inflation_cost_scaling = 9.0;
    }
    // The global plan already enforces the footprint buffer.  A long local
    // collision projection into sparse unknown cells caused false stops in
    // visibly clear aisles; retain near-field collision checking only.
    config.collision_horizon = .50;
    // Doorway crawl: jamb returns sit at ~body half-width and latched STOP
    // with the open-space travel corridor. Passage mode narrows that band.
    if (entry_drive) {
      // Passage mode: keep monitor on so a true bumper hit still stops, but
      // do not treat door jambs as forward obstacles. stop_distance was 0.02 m
      // (contact) — the mouth-crawl override then shoved 0.16 m/s into the
      // wall after the monitor had already zeroed the command (UI: LOC on wall).
      config.collision_monitor_enabled = true;
      config.collision_monitor_passage_mode = true;
      config.collision_monitor_slowdown_distance = .18;
      config.collision_monitor_approach_horizon = .50;
      config.collision_monitor_stop_distance = .08;
    }
    // Entry crawl must clear the sim/base deadzone. Live: RPP published
    // 0.048 m/s (accel-limited from rest), measured velocity stayed 0, and
    // CommandStalled fired "no measured motion" before the chassis moved.
    config.desired_linear_velocity = entry_drive ? .22 : .28;
    config.regulated_min_speed = entry_drive ? .14 : .10;
    if (entry_drive) {
      config.max_linear_acceleration = 2.4;
      config.regulated_min_radius = .28;
      config.regulated_clearance_min_scale = .90;
      config.regulated_clearance_min_scale_trusted = .95;
      config.rotate_to_heading_min_angle = .85;
      config.progress_timeout = 40.;
    }
    // Soft target in open space: pull paths off speckles/walls when the room
    // is wide. Doorways cannot reach 0.50 m clearance — that smoother pulled
    // entry chords into jambs / PathCuts rejects (live: probe rejected
    // path-intersects while the free throat was clear).
    if (entry_drive) {
      // No soft pull on entry — smoother bows cut wall tips and PathCuts then
      // rejects a perfectly wide mouth (live gap=2.15 thrash).
      config.smoother_enabled = false;
    } else if (pinch_plan) {
      // Enough to thread a certified doorway; not a wall-foot preference.
      config.smoother_min_clearance = .28;
      config.smoother_obstacle_weight = 1.0;
      config.smoother_data_weight = .08;
      config.smoother_max_deviation = .40;
      config.smoother_max_step = .05;
    } else {
      config.smoother_min_clearance = .55;
      config.smoother_obstacle_weight = 1.9;
      config.smoother_data_weight = .04;
      config.smoother_max_deviation = .65;
      config.smoother_max_step = .06;
    }
    // Frontier goals are observation viewpoints, but home is a docking pose.
    // Keep their completion contracts separate: return must converge to the
    // recorded ground-truth start instead of stopping at the edge of the
    // ordinary navigation tolerance disk.
    config.goal_xy_tolerance = returning ? .04 : kExploreGoalTolerance;
    // LD14 observes 360 degrees, so frontier visits need no stop-and-turn
    // terminal orientation. Preserve precise heading only when docking home.
    // Backtrack edges are positional checkpoints; requiring a docking yaw at
    // every 0.35 m checkpoint creates needless stop-turn-stop motion.
    // Returning means recovering the recorded start point. With a 360-degree
    // lidar there is no mission requirement to reproduce the arbitrary start
    // yaw; enforcing it caused needless terminal spin after position docking.
    config.goal_yaw_tolerance = 3.13;
    try {
      navigation_ = std::make_unique<navigation2d::NavigationSystem>(config, map_snapshot_path_);
      reported_recoveries_ = 0;
      navigation_->BanPathSignatures(std::vector<std::uint64_t>(
          cooled_signatures_.begin(), cooled_signatures_.end()));
      navigation_->SetGoal(navigation2d::MakePose2d(
          goal.x - navigation_origin_x_, goal.y - navigation_origin_y_, goal.yaw));
      // Do not commit active_goal_/returning_/recovery_ until the plan is
      // accepted. Early assign left ghost GOALs outside painted free and kept
      // recovery sessions alive after open-area plan rejects (stuck mid-room).
      if (scan_) FeedScan();
      const auto initial_time = now();
      const auto initial_state = navigation_->ComputeCommand(
          LocalPose(pose_->pose), FreshVelocity(initial_time), initial_time.seconds());
      const double goal_distance = std::hypot(
          goal.x - pose_->pose.position.x, goal.y - pose_->pose.position.y);
      if (goal_distance > config.goal_xy_tolerance &&
          initial_state.global_path_length_m <= 0.) {
        last_planning_failure_ = initial_state.planning_failure_reason.empty()
                                     ? "no route"
                                     : initial_state.planning_failure_reason;
        RCLCPP_WARN(get_logger(),
                    "Navigation2D rejected goal before execution: %s",
                    last_planning_failure_.c_str());
        navigation_.reset();
        // Pinch only when open/entry finds NO route (sealed throat). Never
        // pinch-retry wall-scrape — that re-plans the whole map with looser
        // inflation and accepts wall-foot GLOBAL (static-map collisions).
        // Never pinch-retry "goal occupied" (corner-tip viewpoints).
        const bool goal_in_obstacle =
            last_planning_failure_.find("goal occupied") != std::string::npos;
        const bool pinch_worthy =
            !goal_in_obstacle &&
            (last_planning_failure_.find("no route") != std::string::npos ||
             last_planning_failure_.find("no path") != std::string::npos ||
             (entry_drive &&
              last_planning_failure_.find("first-segment") != std::string::npos));
        if (!returning && !retreat_goal_ && !narrow_passage_retry_ &&
            pinch_worthy) {
          narrow_passage_retry_ = true;
          RCLCPP_INFO(get_logger(),
                      "%s no route (%s); retrying pinch clearance",
                      entry_drive ? "Entry" : "Open",
                      last_planning_failure_.c_str());
          const bool ok = StartNavigation(goal, returning, latest_map_retry);
          narrow_passage_retry_ = false;
          if (ok) return true;
        }
        return false;
      }
      // Always reject through-wall chords on the live map (UI dashed purple
      // through solid black). SLAM occupied is never carved open for entry.
      if (PathCutsMappedObstacle()) {
        last_planning_failure_ = "path intersects mapped obstacle";
        RCLCPP_WARN(get_logger(),
                    "Navigation2D rejected goal before execution: %s",
                    last_planning_failure_.c_str());
        // Open explore bans the chord before its one retry so the replay is a
        // different first segment. Entry shares that hallway prefix with every
        // probe into the room; banning it made the retry "failed first-segment
        // replay" and left the mouth in the same second (gap=1.60).
        const auto cut_signature = initial_state.path_signature;
        navigation_.reset();
        if (!mapped_obstacle_retry_) {
          mapped_obstacle_retry_ = true;
          if (!entry_drive && cut_signature != 0)
            cooled_signatures_.insert(cut_signature);
          const bool ok = StartNavigation(goal, returning, true);
          mapped_obstacle_retry_ = false;
          if (ok) return true;
        }
        if (cut_signature != 0) cooled_signatures_.insert(cut_signature);
        if (!entry_drive) NoteMappedObstacleReject(goal);
        return false;
      }
      mapped_obstacle_rejects_ = 0;
      last_path_signature_ = initial_state.path_signature;
      last_path_min_clearance_m_ = initial_state.global_planning.path_min_clearance_m;
      // path_min_clearance_m is EXTRA margin past the planning circle, to
      // occupied cells only. Any strictly positive margin is executable.
      // Margin == 0 is the circle itself. Pinch only for true mouths —
      // whole-map pinch on scrape produced the static-map wall collisions.
      if (!entry_drive && !returning && !retreat_goal_ && !pinch_plan &&
          std::isfinite(last_path_min_clearance_m_) &&
          !navigation2d::PathClearsOpen(last_path_min_clearance_m_)) {
        if (initial_state.path_signature != 0)
          cooled_signatures_.insert(initial_state.path_signature);
        last_planning_failure_ = "wall-scrape path in open clearance";
        RCLCPP_WARN(get_logger(),
                    "Navigation2D rejected goal before execution: %s "
                    "(margin=%.3f)",
                    last_planning_failure_.c_str(), last_path_min_clearance_m_);
        navigation_.reset();
        if (!open_scrape_open_retry_) {
          open_scrape_open_retry_ = true;
          const bool ok = StartNavigation(goal, returning, latest_map_retry);
          open_scrape_open_retry_ = false;
          if (ok) return true;
        }
        // Door mouths: open inflation may only fit with thin margin — pinch.
        if (!narrow_passage_retry_ && PassableMouthGoal(goal)) {
          narrow_passage_retry_ = true;
          RCLCPP_INFO(get_logger(),
                      "Open scrape on corridor mouth (margin=%.3f); pinch",
                      last_path_min_clearance_m_);
          const bool ok = StartNavigation(goal, returning, latest_map_retry);
          narrow_passage_retry_ = false;
          if (ok) return true;
        }
        return false;
      }
      // Pinch may thread a door but must not accept a corner-tip scrape.
      if (!entry_drive && !returning && !retreat_goal_ && pinch_plan &&
          std::isfinite(last_path_min_clearance_m_) &&
          !navigation2d::PathClearsPinch(last_path_min_clearance_m_)) {
        if (initial_state.path_signature != 0)
          cooled_signatures_.insert(initial_state.path_signature);
        last_planning_failure_ = "pinch path still wall-scrape";
        RCLCPP_WARN(get_logger(),
                    "Navigation2D rejected goal before execution: %s "
                    "(margin=%.3f)",
                    last_planning_failure_.c_str(), last_path_min_clearance_m_);
        navigation_.reset();
        return false;
      }
      // Do not classify a frontier from the controller's first sample. At a
      // continuous goal handoff the measured velocity still belongs to the
      // previous route, so acceleration and collision constraints may
      // correctly emit one zero command before tracking starts. Executability
      // is a temporal property owned by NavigationSystem's progress and
      // bounded-rotation state machines, not a one-frame threshold.
      // RPP slows in curves and near occupied space. Exploration goals get a
      // tighter budget so a mid-doorway crawl cannot burn 2+ minutes; return
      // home keeps the longer travel allowance.
      if (returning) {
        goal_timeout_s_ = std::max(55., initial_state.global_path_length_m / .10 + 20.);
      } else {
        goal_timeout_s_ = std::min(
            75., std::max(40., initial_state.global_path_length_m / .12 + 15.));
      }
      active_goal_ = goal;
      returning_ = returning;
      // Resume / latest-map replan of the same viewpoint must keep peel and
      // open-replan counters; only a materially new goal opens a session.
      recovery_.Begin(goal.x, goal.y);
      goal_started_ = now();
      best_goal_path_progress_m_ = 0.;
      last_goal_path_progress_time_ = now();
      immobility_open_replans_ = 0;
      scrape_reject_goal_.reset();
      scrape_reject_count_ = 0;
      PublishStatus(returning ? "RETURNING" :
                        (goal.leftover_approach ? "COMPLETING" : "NAVIGATING"),
                    returning ? "returning to exploration start" :
                        (retreat_goal_ ? "leaving a pinched corridor along the driven trail" :
                        (goal.leftover_approach
                             ? "one leftover approach after exploring tour emptied"
                             : "navigating to frontier")));
      // Accepted dispatch commits the DFS room context. Inspecting candidates
      // must not Enter — only a path that will actually be tracked.
      if (!returning && !retreat_goal_) explorer_.CommitRegion(goal);
      RCLCPP_INFO(get_logger(),
                  "Navigation2D goal: pose=(%.2f, %.2f) goal=(%.2f, %.2f) "
                  "frontier_cells=%d region=%llu path_signature=%llu %s",
                  pose_->pose.position.x, pose_->pose.position.y, goal.x, goal.y,
                  goal.frontier_cells,
                  static_cast<unsigned long long>(goal.region_id),
                  static_cast<unsigned long long>(initial_state.path_signature),
                  explorer_.RegionAudit().c_str());
      return true;
    } catch (const std::exception& error) {
      last_planning_failure_ = error.what();
      RCLCPP_WARN(get_logger(), "Navigation2D rejected goal: %s", error.what());
      navigation_.reset();
      return false;
    }
  }

  void PublishPlanningDebugMap(bool returning) {
    if (!map_ || !pose_) return;
    (void)returning;
    // Mirror LayeredCostmap::Reinflate for the UI: unknown is not a wall and
    // must not be painted by inflation (max keeps 255 over inscribed). Only
    // free cells receive inscribed/soft cost. The previous debug path dilated
    // into unknown and sealed passable doors in the overlay.
    navigation2d::ExplorationGrid planning_grid;
    planning_grid.width = static_cast<int>(map_->info.width);
    planning_grid.height = static_cast<int>(map_->info.height);
    planning_grid.resolution = map_->info.resolution;
    planning_grid.origin_x = map_->info.origin.position.x;
    planning_grid.origin_y = map_->info.origin.position.y;
    planning_grid.cells.assign(map_->data.begin(), map_->data.end());

    nav_msgs::msg::OccupancyGrid input = *map_;
    input.header.stamp = now();
    input.header.frame_id = "map";
    const int robot_col = static_cast<int>(std::floor(
        (pose_->pose.position.x - navigation_origin_x_) / map_->info.resolution));
    const int robot_row = static_cast<int>(std::floor(
        (pose_->pose.position.y - navigation_origin_y_) / map_->info.resolution));
    const int evidence_cells = static_cast<int>(std::ceil(.30 / map_->info.resolution));
    input.data.resize(map_->data.size());
    for (std::size_t index = 0; index < map_->data.size(); ++index) {
      const int row = static_cast<int>(index / map_->info.width);
      const int col = static_cast<int>(index % map_->info.width);
      const bool under_robot = (col - robot_col) * (col - robot_col) +
          (row - robot_row) * (row - robot_row) <= evidence_cells * evidence_cells;
      const auto cell = planning_grid.cells[index];
      if (cell == 0 || under_robot) input.data[index] = 0;
      else if (cell < 0) input.data[index] = -1;
      else input.data[index] = 100;
    }
    planning_input_publisher_->publish(input);

    nav_msgs::msg::OccupancyGrid inflated = input;
    constexpr double kRobotRadius = navigation2d::kPlanningRadius;
    constexpr double kInflationRadius = .30;
    constexpr double kInflationCostScaling = 8.;
    const int inflation_cells =
        static_cast<int>(std::ceil(kInflationRadius / map_->info.resolution));
    for (int row = 0; row < static_cast<int>(map_->info.height); ++row) {
      for (int col = 0; col < static_cast<int>(map_->info.width); ++col) {
        if (input.data[static_cast<std::size_t>(row) * map_->info.width + col] != 100)
          continue;
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            const int x = col + dx, y = row + dy;
            if (x < 0 || y < 0 || x >= static_cast<int>(map_->info.width) ||
                y >= static_cast<int>(map_->info.height))
              continue;
            const double distance = std::hypot(dx, dy) * map_->info.resolution;
            if (distance > kInflationRadius || distance == 0.) continue;
            const std::size_t index =
                static_cast<std::size_t>(y) * map_->info.width + static_cast<std::size_t>(x);
            // Match Reinflate: unknown stays unknown; only free is painted.
            if (input.data[index] != 0) continue;
            const int8_t value = distance <= kRobotRadius
                ? static_cast<int8_t>(99)
                : static_cast<int8_t>(std::clamp(
                      static_cast<int>(std::lround(
                          98. * std::exp(-kInflationCostScaling *
                                         (distance - kRobotRadius)))),
                      1, 98));
            inflated.data[index] = std::max(inflated.data[index], value);
          }
        }
      }
    }
    inflated_costmap_publisher_->publish(inflated);
  }

  void BuildTour() {
    committed_tour_.clear();
    const double yaw = pose_ ? Yaw(pose_->pose.orientation)
                             : std::numeric_limits<double>::quiet_NaN();
    const auto goals = explorer_.BuildTour(pose_->pose.position.x,
                                           pose_->pose.position.y, yaw);
    for (const auto& goal : goals) committed_tour_.push_back(goal);
    ++tour_revision_;
    PublishFrontierDebug();
    RCLCPP_INFO(get_logger(),
                "Frontier graph tour %zu: raw_cells=%zu raw_clusters=%zu candidates=%zu "
                "stable=%zu %s",
                tour_revision_, explorer_.raw_frontier_cells(),
                explorer_.raw_frontier_clusters(), explorer_.candidate_frontier_goals(),
                committed_tour_.size(), explorer_.RegionAudit().c_str());
  }

  void PublishFrontierDebug() {
    // Rebuild FR cells from the live map every publish. SelectGoals snapshots
    // go stale while navigating (unknown→free), which left the overlay empty
    // after prune-only. CG/SG markers still come from the last SelectGoals.
    explorer_.RefreshDebugFrontierCells();
    const auto& snap = explorer_.debug_snapshot();
    std::string json = "{\"cells\":[";
    bool first = true;
    constexpr std::size_t kMaxCells = 4000;
    const std::size_t cell_step =
        snap.cells.size() > kMaxCells ? snap.cells.size() / kMaxCells + 1 : 1;
    for (std::size_t i = 0; i < snap.cells.size(); i += cell_step) {
      const auto& c = snap.cells[i];
      if (!first) json += ",";
      first = false;
      json += "[" + std::to_string(c.x) + "," + std::to_string(c.y) + "," +
              std::to_string(c.cluster_id) + "]";
    }
    json += "],\"cluster_goals\":[";
    first = true;
    for (const auto& g : snap.cluster_goals) {
      if (!first) json += ",";
      first = false;
      json += "{\"x\":" + std::to_string(g.x) + ",\"y\":" + std::to_string(g.y) +
              ",\"fx\":" + std::to_string(g.frontier_x) +
              ",\"fy\":" + std::to_string(g.frontier_y) +
              ",\"cid\":" + std::to_string(g.cluster_id) +
              ",\"score\":" + std::to_string(g.score) +
              ",\"gain\":" + std::to_string(g.information_gain) +
              ",\"n\":" + std::to_string(g.frontier_cells) + "}";
    }
    json += "],\"goals\":[";
    first = true;
    for (const auto& g : snap.goals) {
      if (!first) json += ",";
      first = false;
      json += "{\"x\":" + std::to_string(g.x) + ",\"y\":" + std::to_string(g.y) +
              ",\"fx\":" + std::to_string(g.frontier_x) +
              ",\"fy\":" + std::to_string(g.frontier_y) +
              ",\"cid\":" + std::to_string(g.cluster_id) +
              ",\"score\":" + std::to_string(g.score) +
              ",\"gain\":" + std::to_string(g.information_gain) +
              ",\"n\":" + std::to_string(g.frontier_cells) + "}";
    }
    json += "]}";
    std_msgs::msg::String msg;
    msg.data = std::move(json);
    frontier_debug_publisher_->publish(msg);
  }

  void LockEntryMouth(const Goal& mouth) {
    locked_entry_mouth_ = mouth;
    pending_entry_goal_ = mouth;
    entry_probe_attempts_ = 0;
    entry_open_loop_ticks_ = 0;
    // Latch easy/corridor at lock time so mid-approach map growth cannot flip
    // IsCorridorMouth false and abandon a real doorway.
    locked_entry_easy_ = explorer_.IsCorridorMouth(mouth.frontier_x,
                                                   mouth.frontier_y) &&
                         explorer_.MouthGapWidth(mouth.frontier_x,
                                                 mouth.frontier_y) >= .50 &&
                         explorer_.MouthGapWidth(mouth.frontier_x,
                                                 mouth.frontier_y) <= 1.40;
    entry_probe_initial_known_cells_ = KnownCells();
    entry_probe_initial_map_revision_ = map_revision_;
  }

  void ClearEntryMouthLock() {
    locked_entry_mouth_.reset();
    pending_entry_goal_.reset();
    entry_probe_attempts_ = 0;
    entry_open_loop_ticks_ = 0;
    locked_entry_easy_ = false;
  }

  // Entry budget exhausted or mouth unplannable: short demote so the tour
  // cannot immediately re-drive the same hall→threshold shuttle. Keep the
  // remaining tour — clearing it forced BuildTour to hop across the map to a
  // far sibling while the kitchen mouth was only briefly demoted (冤枉路).
  // Far mouths that were already queued stay, but local unfinished mouths
  // are rotated to the front so we do not leave the pocket for a detour.
  void AbandonLockedEntryMouth(const std::string& reason) {
    Goal mouth = pending_entry_goal_.value_or(
        locked_entry_mouth_.value_or(active_goal_));
    // Do not 1.1 m / 4000-cell blacklist the frontier: that hid the sealed
    // room for the rest of the run after a first-segment miss. Cool only the
    // viewpoint briefly so the tour can re-approach after open-loop / peel.
    explorer_.RecordAttempt(mouth, false, true, false);
    explorer_.CoolRegion(mouth.x, mouth.y, 0.55, 1200u);
    recently_demoted_ = mouth;
    recently_demoted_until_ = now() + rclcpp::Duration::from_seconds(8.0);
    ClearEntryMouthLock();
    constexpr double kLocalHopM = 4.0;
    const double rx = pose_ ? pose_->pose.position.x : mouth.x;
    const double ry = pose_ ? pose_->pose.position.y : mouth.y;
    const auto is_local = [&](const Goal& goal) {
      return std::hypot(goal.x - rx, goal.y - ry) <= kLocalHopM ||
             std::hypot(goal.frontier_x - mouth.frontier_x,
                        goal.frontier_y - mouth.frontier_y) <= kLocalHopM ||
             explorer_.GoalInActiveRegion(goal);
    };
    std::deque<Goal> ahead_local;
    std::deque<Goal> behind_local;
    std::deque<Goal> far;
    const auto place = [&](Goal goal) {
      if (SameEntryMouth(goal, mouth) || RecentlyDemoted(goal)) return;
      if (!is_local(goal)) {
        far.push_back(std::move(goal));
        return;
      }
      if (GoalIsBehindTrail(goal))
        behind_local.push_back(std::move(goal));
      else
        ahead_local.push_back(std::move(goal));
    };
    for (const auto& goal : committed_tour_) place(goal);
    // Pull in other executable mouths near the abandoned doorway.
    if (pose_) {
      auto mouths = explorer_.SelectGoals(rx, ry);
      mouths = explorer_.OrderByRegion(std::move(mouths), rx, ry,
                                       Yaw(pose_->pose.orientation));
      for (const auto& candidate : mouths) {
        if (SameEntryMouth(candidate, mouth) || RecentlyDemoted(candidate))
          continue;
        const bool duplicate =
            std::any_of(ahead_local.begin(), ahead_local.end(),
                        [&](const Goal& prior) {
                          return SameEntryMouth(prior, candidate);
                        }) ||
            std::any_of(behind_local.begin(), behind_local.end(),
                        [&](const Goal& prior) {
                          return SameEntryMouth(prior, candidate);
                        }) ||
            std::any_of(far.begin(), far.end(), [&](const Goal& prior) {
              return SameEntryMouth(prior, candidate);
            });
        if (duplicate) continue;
        place(candidate);
      }
      if (ahead_local.empty() && behind_local.empty() && far.empty()) {
        for (const auto& candidate : mouths) {
          if (SameEntryMouth(candidate, mouth) || RecentlyDemoted(candidate))
            continue;
          far.push_back(candidate);
          break;
        }
      }
    }
    committed_tour_.clear();
    // Prefer goals ahead of the outbound trail so we do not U-turn home.
    for (auto& goal : ahead_local) committed_tour_.push_back(std::move(goal));
    for (auto& goal : far) committed_tour_.push_back(std::move(goal));
    for (auto& goal : behind_local) committed_tour_.push_back(std::move(goal));
    next_selection_ = now();
    PublishStatus("SELECTING", reason);
  }

  bool SameEntryMouth(const Goal& a, const Goal& b) const {
    return std::hypot(a.frontier_x - b.frontier_x, a.frontier_y - b.frontier_y) <
               .75 ||
           std::hypot(a.x - b.x, a.y - b.y) < .55;
  }

  // Real door/alcove only: CorridorMouthAt AND throat in [0.50, 1.40] m.
  // Open-room free span (live gap=1.65 along a wall) is NOT a mouth — treating
  // it as "easy door" locked entry and thrash-abandoned the wrong place.
  // Once locked, trust the latch: flooded throats flip IsCorridorMouth false.
  bool EasyLockedMouth() const {
    if (!pending_entry_goal_ && !locked_entry_mouth_) return false;
    if (locked_entry_mouth_ && locked_entry_easy_) return true;
    const Goal& mouth =
        pending_entry_goal_.value_or(*locked_entry_mouth_);
    if (!explorer_.IsCorridorMouth(mouth.frontier_x, mouth.frontier_y))
      return false;
    const double gap =
        explorer_.MouthGapWidth(mouth.frontier_x, mouth.frontier_y);
    return gap >= .50 && gap <= 1.40;
  }

  bool PassableMouthGoal(const Goal& goal) const {
    if (!std::isfinite(goal.frontier_x) || !std::isfinite(goal.frontier_y))
      return false;
    if (!explorer_.IsCorridorMouth(goal.frontier_x, goal.frontier_y))
      return false;
    const double gap =
        explorer_.MouthGapWidth(goal.frontier_x, goal.frontier_y);
    return gap >= .35 && gap <= 1.40;
  }

  int EntryAttemptBudget() const {
    // Wide free mouths must not be abandoned after a few PathCuts rejects —
    // live log: gap=2.15 failed_goals→69 while the opening stayed clear.
    return EasyLockedMouth() ? 14 : 6;
  }

  // Promote only true corridor mouths into locked entry.
  bool PromotePassableMouthEntry(const char* reason) {
    if (returning_ || retreat_goal_ || phase_ == MissionPhase::kReturning)
      return false;
    if (!std::isfinite(active_goal_.x) || !PassableMouthGoal(active_goal_))
      return false;
    if (locked_entry_mouth_ && !SameEntryMouth(*locked_entry_mouth_, active_goal_))
      return false;
    RCLCPP_WARN(get_logger(), "Passable-mouth promote: %s gap=%.2f", reason,
                explorer_.MouthGapWidth(active_goal_.frontier_x,
                                        active_goal_.frontier_y));
    LockEntryMouth(active_goal_);
    paint_driven_corridor_ = true;
    paint_trail_budget_m_ = 8.0;
    cooled_signatures_.clear();
    if (StartEntryProbe()) {
      PublishStatus("COMPLETING", reason);
      return true;
    }
    if (EasyLockedMouth() && entry_open_loop_ticks_ == 0) {
      entry_open_loop_ticks_ = 1;
      PublishStatus("COMPLETING", "passable mouth; mid-gap crawl");
      return true;
    }
    return false;
  }

  bool StartEntryProbe() {
    if (!pending_entry_goal_ || !pose_) return false;
    if (RecentlyDemoted(*pending_entry_goal_)) return false;
    // Commit to one mouth: hopping across kitchen / alcove frontiers while the
    // robot sits on the threshold produces the scribble thrash in the red box.
    if (!locked_entry_mouth_ || !SameEntryMouth(*locked_entry_mouth_, *pending_entry_goal_))
      LockEntryMouth(*pending_entry_goal_);
    else
      pending_entry_goal_ = *locked_entry_mouth_;
    // Far from the mouth: step along the ray toward the frontier until the
    // remaining gap is the poke distance. The stored viewpoint can sit behind
    // the robot; navigating there is the open-room reverse, and once the
    // chassis is already inside goal tolerance that navigation succeeds every
    // tick and the robot freezes (live: pose=(-1.69,-0.50) goal=(-1.81,-0.45)
    // while the frontier stayed 2.06 m south).
    {
      const double fx = pending_entry_goal_->frontier_x;
      const double fy = pending_entry_goal_->frontier_y;
      const double rx = pose_->pose.position.x;
      const double ry = pose_->pose.position.y;
      const double mouth_range = std::hypot(rx - fx, ry - fy);
      constexpr double kPokeRange = 1.55;
      const double travel = mouth_range - kPokeRange;
      if (travel > navigation2d::kSwingClearance) {
        Goal approach = *pending_entry_goal_;
        approach.x = rx + travel * (fx - rx) / mouth_range;
        approach.y = ry + travel * (fy - ry) / mouth_range;
        approach.yaw = std::atan2(fy - ry, fx - rx);
        paint_driven_corridor_ = true;
        paint_trail_budget_m_ = 8.0;
        cooled_signatures_.clear();
        if (StartNavigation(approach, false)) {
          RCLCPP_INFO(get_logger(),
                      "Entry approach standoff: frontier=(%.2f,%.2f) "
                      "range=%.2f step=(%.2f,%.2f)",
                      fx, fy, mouth_range, approach.x, approach.y);
          PublishStatus("COMPLETING", "approaching sealed-room mouth");
          return true;
        }
      }
    }
    // Adaptive entry depth: fixed 0.55 m rejected easy shallow rooms (UI:
    // wide free doorway, SelectEntryProbe nullopt → abandon). Wide gaps use
    // a mid-gap poke; pinch mouths keep deeper probes into the nose.
    const double gap = explorer_.MouthGapWidth(pending_entry_goal_->frontier_x,
                                               pending_entry_goal_->frontier_y);
    const double free_depth = explorer_.MouthFreeDepth(
        *pending_entry_goal_, pose_->pose.position.x, pose_->pose.position.y);
    const bool easy_door =
        explorer_.IsCorridorMouth(pending_entry_goal_->frontier_x,
                                  pending_entry_goal_->frontier_y) &&
        gap >= .50 && gap <= 1.40;
    std::optional<Goal> probe;
    double used_min_depth = 0.;
    // Wide mouths with almost no mapped free beyond the jamb (live: gap=0.70
    // free_depth=0.05) must try a shallow poke first — deep schedules only
    // return nullopt and abandon an enterable door.
    const double schedules[][2] = {
        {easy_door ? .10 : (.42 + .10 * entry_probe_attempts_),
         easy_door ? .30 : (entry_probe_attempts_ == 0 ? 1.30 : 1.60)},
        {easy_door ? (.22 + .08 * entry_probe_attempts_)
                   : (.42 + .10 * entry_probe_attempts_),
         easy_door ? .85 : (entry_probe_attempts_ == 0 ? 1.30 : 1.60)},
        {.18, .55},
        {.12, .35},
    };
    for (const auto& schedule : schedules) {
      double minimum_depth = schedule[0];
      double maximum_depth = schedule[1];
      if (free_depth > .05) {
        minimum_depth =
            std::min(minimum_depth, std::max(.10, free_depth - .05));
        maximum_depth = std::max(maximum_depth, minimum_depth + .10);
        maximum_depth = std::min(maximum_depth, std::max(minimum_depth + .10,
                                                         free_depth + .08));
      } else if (easy_door) {
        // Tiny free nose: keep the shallow window, do not demand 0.28 m depth.
        maximum_depth = std::max(maximum_depth, minimum_depth + .10);
      }
      if (maximum_depth < minimum_depth) continue;
      probe = explorer_.SelectEntryProbe(
          *pending_entry_goal_, pose_->pose.position.x, pose_->pose.position.y,
          minimum_depth, maximum_depth);
      if (probe) {
        used_min_depth = minimum_depth;
        break;
      }
    }
    if (!probe) {
      ++entry_probe_attempts_;
      RCLCPP_WARN(get_logger(),
                  "Entry probe unavailable: frontier=(%.2f,%.2f) attempts=%d "
                  "gap=%.2f free_depth=%.2f",
                  pending_entry_goal_->frontier_x, pending_entry_goal_->frontier_y,
                  entry_probe_attempts_, gap, free_depth);
      return false;
    }
    // Already past the jambs: close the mouth without StartNavigation (live:
    // already-past logged dozens of times/sec while re-locking the same door).
    {
      const Goal prior_active = active_goal_;
      active_goal_ = *probe;
      ++entry_probe_attempts_;
      const bool already_in = EntryProbeSatisfied();
      if (already_in) {
        explorer_.RecordAttempt(*pending_entry_goal_, true, false);
        ForgetEntryRoom(*pending_entry_goal_);
        recently_demoted_ = *pending_entry_goal_;
        recently_demoted_until_ = now() + rclcpp::Duration::from_seconds(14.0);
        // Do not CoolRegion the mouth — that hid kitchen leftovers and made
        // the tour soft-unlock out after a doorway peek.
        ClearEntryMouthLock();
        QueueActiveRegionFinish();
        active_goal_ = prior_active;
        next_selection_ = now();
        PublishStatus("SELECTING",
                      "room entered; finishing active region first");
        RCLCPP_INFO(get_logger(),
                    "room entered at pose=(%.2f,%.2f) (already past mouth)",
                    pose_->pose.position.x, pose_->pose.position.y);
        return false;  // mouth closed; caller should not treat as active drive
      }
      active_goal_ = prior_active;
      --entry_probe_attempts_;
    }
    ++entry_probe_attempts_;
    entry_probe_initial_known_cells_ = KnownCells();
    entry_probe_initial_map_revision_ = map_revision_;
    // Reopen only unknown/free trail cells (not occupied) so pinch escapes
    // still work without inventing through-wall entry chords.
    paint_driven_corridor_ = true;
    paint_trail_budget_m_ = 4.0;
    // Entry plans must not inherit cooled hallway chords — those forced
    // first-segment replay failures with progress=0 at the wrong pose.
    cooled_signatures_.clear();
    RememberEntryRoom(*pending_entry_goal_, *probe);
    // A target inside goal tolerance "arrives" on the first tick without
    // moving, and the retry picks the same spot again.
    if (std::hypot(probe->x - pose_->pose.position.x,
                   probe->y - pose_->pose.position.y) <= kExploreGoalTolerance) {
      RCLCPP_WARN(get_logger(),
                  "Entry probe rejected: target=(%.2f,%.2f) depth=%.2f attempt=%d "
                  "gap=%.2f reason=target within goal tolerance of robot",
                  probe->x, probe->y, used_min_depth, entry_probe_attempts_, gap);
      return false;
    }
    if (!StartNavigation(*probe, false)) {
      RCLCPP_WARN(get_logger(),
                  "Entry probe rejected: target=(%.2f,%.2f) depth=%.2f attempt=%d "
                  "gap=%.2f reason=%s",
                  probe->x, probe->y, used_min_depth, entry_probe_attempts_, gap,
                  last_planning_failure_.empty() ? "unknown" :
                      last_planning_failure_.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(),
                "Entry probe started: frontier=(%.2f,%.2f) target=(%.2f,%.2f) "
                "depth=%.2f attempt=%d gap=%.2f easy=%s",
                pending_entry_goal_->frontier_x, pending_entry_goal_->frontier_y,
                probe->x, probe->y, used_min_depth, entry_probe_attempts_, gap,
                easy_door ? "yes" : "no");
    PublishStatus("COMPLETING", "crossing frontier entry and waiting for map growth");
    return true;
  }

  double EntryProbeDepth() const {
    if (!pending_entry_goal_ || !pose_) return 0.;
    const double direction_x = pending_entry_goal_->frontier_x - pending_entry_goal_->x;
    const double direction_y = pending_entry_goal_->frontier_y - pending_entry_goal_->y;
    const double length = std::hypot(direction_x, direction_y);
    if (length < .05) return 0.;
    return ((pose_->pose.position.x - pending_entry_goal_->frontier_x) * direction_x +
            (pose_->pose.position.y - pending_entry_goal_->frontier_y) * direction_y) /
           length;
  }

  bool EntryProbeSatisfied() const {
    if (!pending_entry_goal_ || !pose_ || entry_probe_attempts_ < 1) return false;
    // Must be well inside the room — 0.22 m past the jambs was "just over the
    // doorway then leave" (kitchen white wall still unmapped).
    const Goal& mouth = locked_entry_mouth_.value_or(*pending_entry_goal_);
    const double fx = mouth.frontier_x;
    const double fy = mouth.frontier_y;
    const double rx = pose_->pose.position.x;
    const double ry = pose_->pose.position.y;
    if (std::hypot(rx - fx, ry - fy) > 1.35) return false;
    double into_x = active_goal_.x - fx;
    double into_y = active_goal_.y - fy;
    double length = std::hypot(into_x, into_y);
    if (length < .12) {
      into_x = fx - mouth.x;
      into_y = fy - mouth.y;
      length = std::hypot(into_x, into_y);
    }
    if (length < .05) return false;
    const double ux = into_x / length, uy = into_y / length;
    const double lateral = std::abs((rx - fx) * (-uy) + (ry - fy) * ux);
    if (lateral > .60) return false;
    const double robot_side = (rx - fx) * ux + (ry - fy) * uy;
    // Wide easy doors: a shallow mid-gap poke (~0.30 m) is enough to mark
    // entered. Narrow mouths still need ~0.48 m so we do not peek-and-leave.
    const double gap = explorer_.MouthGapWidth(fx, fy);
    const double need = gap >= .50 ? .30 : .48;
    return robot_side > need;
  }

  // After crossing a mouth: queue in-room leftovers before any hallway tour so
  // the kitchen white pocket is observed before soft-unlock hops out.
  void QueueActiveRegionFinish() {
    if (!pose_) return;
    const double rx = pose_->pose.position.x;
    const double ry = pose_->pose.position.y;
    explorer_.CommitRegion(rx, ry);
    committed_tour_.clear();
    auto leftovers = explorer_.SelectLeftoverApproaches(rx, ry, 6);
    leftovers = explorer_.OrderByRegion(std::move(leftovers), rx, ry,
                                        Yaw(pose_->pose.orientation));
    for (auto& goal : leftovers) {
      if (!explorer_.GoalInActiveRegion(goal)) continue;
      if (RecentlyDemoted(goal)) continue;
      committed_tour_.push_back(std::move(goal));
    }
    if (committed_tour_.empty()) {
      auto mouths = explorer_.SelectGoals(rx, ry);
      mouths = explorer_.OrderByRegion(std::move(mouths), rx, ry,
                                       Yaw(pose_->pose.orientation));
      for (auto& goal : mouths) {
        if (!explorer_.GoalInActiveRegion(goal)) continue;
        if (RecentlyDemoted(goal)) continue;
        committed_tour_.push_back(std::move(goal));
      }
    }
    RCLCPP_INFO(get_logger(),
                "active room finish queued: local_goals=%zu unknown=%zu %s",
                committed_tour_.size(),
                explorer_.ReachableUnknownCells(rx, ry),
                explorer_.RegionAudit().c_str());
  }

  std::pair<int, int> PoseCell() const {
    const double resolution = map_ ? map_->info.resolution : .05;
    return {static_cast<int>(std::floor(pose_->pose.position.x / resolution)),
            static_cast<int>(std::floor(pose_->pose.position.y / resolution))};
  }

  void RefreshStallContext() {
    if (!pose_) return;
    const auto cell = PoseCell();
    if (!stall_pose_initialized_ || cell.first != stall_pose_cell_x_ ||
        cell.second != stall_pose_cell_y_) {
      stall_pose_initialized_ = true;
      stall_pose_cell_x_ = cell.first;
      stall_pose_cell_y_ = cell.second;
      first_segment_failures_at_pose_ = 0;
      cooled_signatures_.clear();
    }
    const std::size_t known = KnownCells();
    if (known >= last_stall_known_cells_ + 500) {
      cooled_signatures_.clear();
      last_stall_known_cells_ = known;
    }
    NoteKnownProgress(known);
  }

  void NoteKnownProgress(std::size_t known) {
    if (last_progress_known_cells_ == 0 || known >= last_progress_known_cells_ + 500) {
      last_progress_known_cells_ = known;
      last_progress_time_ = now();
    }
  }

  bool NoteFirstSegmentFailure(const navigation2d::NavigationState& state) {
    RefreshStallContext();
    if (state.path_signature != 0) cooled_signatures_.insert(state.path_signature);
    ++first_segment_failures_at_pose_;
    ++execution_stall_cycles_;
    RCLCPP_WARN(get_logger(),
                "First-segment execution failed: signature=%llu failures_at_pose=%d/%d cooled=%zu",
                static_cast<unsigned long long>(state.path_signature),
                first_segment_failures_at_pose_, 3, cooled_signatures_.size());
    return first_segment_failures_at_pose_ >= 3;
  }

  double SectorMinRange(double center_angle, double half_width) const {
    if (!scan_) return 0.;
    double minimum = 10.;
    bool any = false;
    for (std::size_t beam = 0; beam < scan_->ranges.size(); ++beam) {
      const double range = scan_->ranges[beam];
      if (!std::isfinite(range) || range < scan_->range_min) continue;
      const double angle = scan_->angle_min + static_cast<double>(beam) * scan_->angle_increment;
      if (std::abs(NormalizeAngle(angle - center_angle)) > half_width) continue;
      minimum = std::min(minimum, range);
      any = true;
    }
    return any ? minimum : 0.;
  }

  // One aperture for every stall/escape sample (kBodyConeHalfAngle).
  struct BodyRanges {
    double front = 0.;
    double back = 0.;
    double left = 0.;
    double right = 0.;
    double side = 0.;
  };

  BodyRanges ReadBodyRanges() const {
    const double cone = navigation2d::kBodyConeHalfAngle;
    const double pi = std::acos(-1.);
    BodyRanges ranges;
    ranges.front = SectorMinRange(0., cone);
    ranges.back = SectorMinRange(pi, cone);
    // See kLidarForwardOffset: the back ray starts ahead of the chassis
    // centre. Keep a positive value so a wall between the lidar and the tail
    // still counts as inside the body rather than "no return".
    if (ranges.back > 0.)
      ranges.back = std::max(1e-3, ranges.back - navigation2d::kLidarForwardOffset);
    ranges.left = SectorMinRange(pi / 2., cone);
    ranges.right = SectorMinRange(-pi / 2., cone);
    ranges.side = std::min(ranges.left > 0. ? ranges.left : 10.,
                           ranges.right > 0. ? ranges.right : 10.);
    return ranges;
  }

  // Prefer the live route metric. A stored 0 after navigation_.reset() is not
  // a measurement — callers that require an active route check navigation_.
  double ActivePathMargin(double sample) const {
    if (navigation_ && std::isfinite(sample)) return sample;
    return last_path_min_clearance_m_;
  }

  // Body-frame lidar only — no map pose / yaw. LD14 distorts hard while
  // spinning and localization coasts on odom, so tip escape must prefer
  // reverse translation over in-place rotate.
  bool TryStartEscape() {
    if (!scan_ || !pose_ || now() < escape_until_) return false;
    const BodyRanges body = ReadBodyRanges();
    const double front = body.front;
    const double back = body.back;
    const double left = body.left;
    const double right = body.right;
    geometry_msgs::msg::Twist command;
    double hold_s = 1.6;
    // Mild side bias only (≤0.35 rad/s). Large w during reverse was the
    // "spin to free" habit that corrupted scan matching mid-escape.
    const double side_bias =
        (left > 0. && right > 0. && std::abs(left - right) > .12)
            ? ((left > right) ? .30 : -.30)
            : 0.;
    if (back >= navigation2d::kPlanningRadius) {
      // Priority 1: back straight out of the tip. Odom coasts fine; map yaw
      // does not matter. Tail must clear the planning circle.
      command.linear.x = -.20;
      command.angular.z = side_bias;
      hold_s = returning_ ? 2.2 : 1.8;
    } else if (navigation2d::AxisSwingClear(front)) {
      // Priority 2: creep forward into open, slight bias off the near wall.
      command.linear.x = .16;
      command.angular.z = side_bias;
      hold_s = 1.4;
    } else if (navigation2d::AxisSwingClear(std::max(left, right))) {
      // Priority 3 (last): short body-frame turn toward freer flank, then the
      // next escape tick will reverse. Keep spin ≤0.8 s.
      command.angular.z = left >= right ? .55 : -.55;
      hold_s = .70;
    } else {
      return false;
    }
    escape_command_ = command;
    escape_until_ = now() + rclcpp::Duration::from_seconds(hold_s);
    RCLCPP_WARN(get_logger(),
                "Starting body-frame escape: front=%.2f back=%.2f left=%.2f "
                "right=%.2f v=%.2f w=%.2f hold=%.1f",
                front, back, left, right, command.linear.x, command.angular.z,
                hold_s);
    return true;
  }

  bool TickEscape() {
    if (escape_until_.nanoseconds() == 0) return false;
    if (now() < escape_until_) {
      // Re-read body-frame lidar every tick. Do not trust the frozen command's
      // yaw bias after localization has coasted on odom.
      geometry_msgs::msg::Twist command = escape_command_;
      const BodyRanges body = ReadBodyRanges();
      const double front = body.front;
      const double back = body.back;
      const double left = body.left;
      const double right = body.right;
      if (std::abs(command.linear.x) > .05 && left > 0. && right > 0. &&
          std::abs(left - right) > .12) {
        const double bias = (left > right) ? .30 : -.30;
        // Keep the planned sign of rotation if any; otherwise gently bias.
        if (std::abs(command.angular.z) < .15) command.angular.z = bias;
        else if ((command.angular.z > 0.) != (bias > 0.))
          command.angular.z = .5 * command.angular.z + .5 * bias;
      }
      if (command.linear.x > 0. && front > 0. &&
          front < navigation2d::kPlanningRadius) {
        command.linear.x = 0.;
        // Prefer reverse over spin when the nose closes mid-escape.
        if (back >= navigation2d::kPlanningRadius) {
          command.linear.x = -.16;
          command.angular.z = 0.;
        } else if (std::abs(command.angular.z) < .20) {
          command.angular.z = (left >= right) ? .50 : -.50;
        }
      }
      if (command.linear.x < 0. && navigation2d::HitInsideBody(back)) {
        command.linear.x = 0.;
      }
      command_publisher_->publish(command);
      PublishStatus("RECOVERING", "body-frame tip escape (prefer reverse)");
      return true;
    }
    escape_until_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    geometry_msgs::msg::Twist stop;
    command_publisher_->publish(stop);
    const bool resume_return = escape_then_return_;
    escape_then_return_ = false;
    if (resume_return) {
      BeginReturn(true);
      return true;
    }
    // Session-owned peel exit: resume without clearing side_peels/open_replans.
    if (recovery_.resume_after_escape) {
      recovery_.resume_after_escape = false;
      if (phase_ == MissionPhase::kExploring && !returning_) {
        if (locked_entry_mouth_ || pending_entry_goal_) {
          if (!pending_entry_goal_) pending_entry_goal_ = locked_entry_mouth_;
          // One peel retry only. Further resumes of the same mouth + trail
          // reverse were the post-脱困 折返 loop.
          if (entry_probe_attempts_ < EntryAttemptBudget() && StartEntryProbe()) {
            PublishStatus("COMPLETING", "resumed locked mouth after peel");
            return true;
          }
          if (EasyLockedMouth()) {
            if (entry_open_loop_ticks_ == 0) entry_open_loop_ticks_ = 1;
            PublishStatus("COMPLETING",
                          "peel done; wide mouth crawl instead of abandon");
            return true;
          }
          AbandonLockedEntryMouth(
              "peel done; leave mouth and continue tour (no reverse)");
          next_selection_ = now();
          return true;
        }
        // Wall-scrape peels must not resume the banned chord — that rebuilds
        // the STOP→peel→scrape sawtooth along the inflation rim.
        if (BreakWallScrapeAfterPeel()) return true;
        if (StartNavigation(active_goal_, false)) {
          PublishStatus("NAVIGATING", "resumed goal after open-space peel");
          return true;
        }
      }
      DemoteActiveGoal("resume after peel failed; demoting viewpoint");
      return true;
    }
    if (phase_ == MissionPhase::kCompleting) {
      FinishCompleting();
      return true;
    }
    next_selection_ = now();
    return true;
  }

  void StartCompletingProbe() {
    ++completing_probe_rounds_;
    phase_ = MissionPhase::kCompleting;
    committed_tour_.clear();
    // Failure cools hid unfinished alcove mouths from the exploring tour.
    // Completing must see them again or PARTIAL leaves visible white tips.
    explorer_.ClearFailedBlacklist();
    cooled_signatures_.clear();
    auto leftovers = explorer_.SelectLeftoverApproaches(
        pose_->pose.position.x, pose_->pose.position.y, 3);
    leftovers = explorer_.OrderByRegion(
        std::move(leftovers), pose_->pose.position.x, pose_->pose.position.y,
        Yaw(pose_->pose.orientation));
    if (leftovers.empty()) {
      RCLCPP_WARN(get_logger(),
                  "Completing probe found no leftover approach: raw_cells=%zu rounds=%d",
                  explorer_.raw_frontier_cells(), completing_probe_rounds_);
      phase_ = MissionPhase::kExploring;
      // Tip pocket + empty leftovers: leave immediately and hunt far mouths
      // instead of burning completing rounds while wedged (下方卡住).
      if (TipPinchedNow() &&
          TryRescueDistantMouths("completing empty while tip-pinched"))
        return;
      if (ShouldKeepWaitingForFrontiers()) {
        PublishStatus("SELECTING",
                      "unresolved boundary; waiting for map before return");
        return;
      }
      BeginReturnOrRescue(explorer_.raw_frontier_cells() > 0);
      return;
    }
    for (const auto& leftover : leftovers) {
      // Skip only the threshold cell itself. In-room leftovers near the robot
      // (kitchen white pocket just past the door) must still be driven.
      const double near = std::hypot(leftover.x - pose_->pose.position.x,
                                     leftover.y - pose_->pose.position.y);
      if (near < .60 && !explorer_.GoalInActiveRegion(leftover)) {
        RCLCPP_WARN(get_logger(),
                    "Completing leftover is on the current doorway cell (%.2f, %.2f); skipping",
                    leftover.x, leftover.y);
        continue;
      }
      if (StartNavigation(leftover, false)) return;
      if (BreakMappedObstacleThrash()) return;
      // Doorway leftovers often need a room-side entry pose; hallway standoffs
      // fail StartNavigation while the mouth is still openable.
      LockEntryMouth(leftover);
      if (StartEntryProbe()) {
        RCLCPP_WARN(get_logger(),
                    "Completing leftover unplannable at (%.2f, %.2f); trying entry probe",
                    leftover.x, leftover.y);
        return;
      }
      RCLCPP_WARN(get_logger(),
                  "Completing leftover approach is unplannable at (%.2f, %.2f)",
                  leftover.x, leftover.y);
      explorer_.RecordAttempt(leftover, false);
      ClearEntryMouthLock();
    }
    // Keep exploring while raw frontiers remain — one failed leftover must
    // not abandon unfinished alcoves/rooms.
    phase_ = MissionPhase::kExploring;
    if (TipPinchedNow() &&
        TryRescueDistantMouths("leftovers unplannable while tip-pinched"))
      return;
    if (ShouldKeepWaitingForFrontiers()) {
      PublishStatus("SELECTING",
                    "leftover blocked; waiting for map update before return");
      return;
    }
    BeginReturnOrRescue(explorer_.raw_frontier_cells() > 0 ||
                        explorer_.ReachableUnknownCells(
                            pose_->pose.position.x, pose_->pose.position.y) > 0);
  }

  bool ShouldKeepWaitingForFrontiers() const {
    if (!explorer_.HasExploringWork()) return false;
    // Cap empty SELECTING loops. The old 36-cycle budget left the chassis
    // wedged in a tip for tens of seconds before PARTIAL (UI: 下方卡住).
    if (completing_probe_rounds_ < 3) return true;
    return no_executable_frontier_cycles_ < 8;
  }

  bool TipPinchedNow() const {
    if (!scan_ || !pose_) return false;
    const BodyRanges body = ReadBodyRanges();
    if (navigation_)
      return IsPinchGeometryStall(last_path_min_clearance_m_, body.front,
                                  body.side);
    // No route: both axes inside the planning circle. A near flank alone is
    // not a pocket.
    return navigation2d::HitBlocksSwing(body.front) &&
           navigation2d::HitBlocksSwing(body.side);
  }

  // Leave a tip pocket and re-queue far corridor mouths (kitchen) instead of
  // PARTIAL while raw frontiers remain. One shot per mission.
  bool TryRescueDistantMouths(const char* why) {
    if (tip_rescue_attempted_ || returning_ ||
        phase_ == MissionPhase::kReturning || !pose_)
      return false;
    if (!explorer_.HasExploringWork()) return false;
    tip_rescue_attempted_ = true;
    RCLCPP_WARN(get_logger(),
                "Tip/tour rescue (%s): peel + retry distant mouths; raw=%zu",
                why, explorer_.raw_frontier_cells());
    recently_demoted_.reset();
    recently_demoted_until_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    cooled_signatures_.clear();
    explorer_.ClearFailedBlacklist();
    completing_probe_rounds_ = 0;
    no_executable_frontier_cycles_ = 0;
    reject_streak_ = 0;
    ClearEntryMouthLock();
    navigation_.reset();
    ClearPublishedNavPaths();
    ClearDisplayedGoal();
    recovery_.Clear();
    if (TipPinchedNow() &&
        EscapeOrShortRetreat("tip rescue; peel into free",
                             "tip rescue; short trail retreat")) {
      PublishStatus("RECOVERING", why);
      next_selection_ = now() + rclcpp::Duration::from_seconds(0.6);
      return true;
    }
    const double rx = pose_->pose.position.x;
    const double ry = pose_->pose.position.y;
    BuildTour();
    std::deque<Goal> distant;
    for (const auto& goal : committed_tour_) {
      if (RecentlyDemoted(goal)) continue;
      if (std::hypot(goal.x - rx, goal.y - ry) < 1.8) continue;
      distant.push_back(goal);
    }
    auto leftovers = explorer_.SelectLeftoverApproaches(rx, ry, 8);
    leftovers = explorer_.OrderByRegion(std::move(leftovers), rx, ry,
                                        Yaw(pose_->pose.orientation));
    for (auto& goal : leftovers) {
      if (std::hypot(goal.x - rx, goal.y - ry) < 1.5) continue;
      if (!explorer_.IsCorridorMouth(goal.frontier_x, goal.frontier_y) &&
          explorer_.MouthGapWidth(goal.frontier_x, goal.frontier_y) < .50)
        continue;
      distant.push_front(std::move(goal));
    }
    committed_tour_.clear();
    for (auto& goal : distant) committed_tour_.push_back(std::move(goal));
    if (committed_tour_.empty()) {
      RCLCPP_WARN(get_logger(), "Tip rescue found no distant mouth to retry");
      return false;
    }
    phase_ = MissionPhase::kExploring;
    SelectNextGoal();
    return navigation_ != nullptr || pending_entry_goal_ || locked_entry_mouth_;
  }

  void BeginReturnOrRescue(bool force_partial) {
    if (force_partial && TryRescueDistantMouths("before PARTIAL return")) return;
    BeginReturn(force_partial);
  }

  void FinishCompleting() {
    phase_ = MissionPhase::kExploring;
    committed_tour_.clear();
    BuildTour();
    if (!committed_tour_.empty() || explorer_.candidate_frontier_goals() > 0) {
      RCLCPP_INFO(get_logger(),
                  "Completing probe finished; normal candidates=%zu tour=%zu",
                  explorer_.candidate_frontier_goals(), committed_tour_.size());
      SelectNextGoal();
      return;
    }
    if (explorer_.raw_frontier_cells() > 0 && completing_probe_rounds_ < 3) {
      StartCompletingProbe();
      return;
    }
    if (ShouldKeepWaitingForFrontiers()) {
      PublishStatus("SELECTING",
                    "completing emptied tour; waiting for unresolved frontiers");
      return;
    }
    BeginReturnOrRescue(explorer_.raw_frontier_cells() > 0);
  }

  // True pinch peel. Never fall back to trail reverse while a mouth is locked:
  // StartRetreat sends the robot back along the outbound path (折返) and that
  // is the inefficiency the UI shows after a failed doorway crawl.
  bool EscapeOrShortRetreat(const char* escape_msg, const char* retreat_msg) {
    if (TryStartEscape()) {
      recovery_.resume_after_escape = true;
      PublishStatus("RECOVERING", escape_msg);
      return true;
    }
    if (pending_entry_goal_ || locked_entry_mouth_) return false;
    if (StartRetreat()) {
      PublishStatus("NAVIGATING", retreat_msg);
      return true;
    }
    return false;
  }

  // Tip only when BOTH axes are inside the planning circle and the live path
  // does not already clear the open-tier margin. A near side wall with an
  // open nose is ordinary driving.
  bool IsPinchGeometryStall(double path_clearance_m, double front_range,
                            double side_range) const {
    if (pending_entry_goal_ || locked_entry_mouth_ || returning_ || retreat_goal_)
      return false;
    if (!navigation_ || navigation2d::PathClearsOpen(path_clearance_m))
      return false;
    return navigation2d::HitBlocksSwing(front_range) &&
           navigation2d::HitBlocksSwing(side_range);
  }

  bool IsOpenWallScrape(double path_clearance_m, double side_range,
                        bool forward_clearly_open) const {
    // Open nose, but the flank hit is inside the planning circle and the
    // path is not an open-tier (mid-aisle) route.
    if (!forward_clearly_open || !navigation_) return false;
    if (navigation2d::PathClearsOpen(path_clearance_m)) return false;
    return navigation2d::HitInsideBody(side_range);
  }

  bool IsCornerTipStall(double path_clearance_m, double front_range,
                        double side_range) const {
    return IsPinchGeometryStall(path_clearance_m, front_range, side_range);
  }

  // Peel off the jamb, cool the viewpoint, pick something else. Do not
  // replan-in-place while the chassis sits on the threshold (live: scrapes
  // counter Cleared every tick → forever wall_scrapes=1).
  bool UnstickFromPinch(const char* reason) {
    if (last_path_signature_ != 0)
      cooled_signatures_.insert(last_path_signature_);
    RCLCPP_WARN(get_logger(),
                "Pinch unstick: %s clearance=%.3f scrapes=%d pose=(%.2f,%.2f)",
                reason, last_path_min_clearance_m_, recovery_.wall_scrapes,
                pose_ ? pose_->pose.position.x : 0.,
                pose_ ? pose_->pose.position.y : 0.);
    geometry_msgs::msg::Twist stop;
    command_publisher_->publish(stop);
    if (phase_ != MissionPhase::kReturning && !returning_ &&
        std::isfinite(active_goal_.x) && std::isfinite(active_goal_.y)) {
      explorer_.RecordAttempt(pending_entry_goal_.value_or(active_goal_), false);
      recently_demoted_ = pending_entry_goal_.value_or(active_goal_);
      // Short cool — 20 s kept re-queuing the same tip while kitchen waited.
      recently_demoted_until_ = now() + rclcpp::Duration::from_seconds(6.0);
      explorer_.CoolRegion(active_goal_.x, active_goal_.y, 0.70, 1500u);
      explorer_.CoolRegion(active_goal_.frontier_x, active_goal_.frontier_y, 0.70,
                           1500u);
    }
    ClearEntryMouthLock();
    navigation_.reset();
    ClearPublishedNavPaths();
    ClearDisplayedGoal();
    if (!committed_tour_.empty()) committed_tour_.pop_front();
    recovery_.Clear();
    retreat_attempted_ = false;
    if (EscapeOrShortRetreat("pinch unstick; peel into free",
                             "pinch unstick; short trail retreat")) {
      PublishStatus("RECOVERING", reason);
      return true;
    }
    // After peel fails, immediately hunt a far mouth instead of SELECTING
    // idle in the tip until completing rounds burn out.
    if (TryRescueDistantMouths(reason)) return true;
    next_selection_ = now() + rclcpp::Duration::from_seconds(0.4);
    PublishStatus("SELECTING", reason);
    return true;
  }

  bool BreakWallScrapeThrash(const char* reason) {
    if (last_path_signature_ != 0)
      cooled_signatures_.insert(last_path_signature_);
    ++recovery_.wall_scrapes;
    navigation_.reset();
    RCLCPP_WARN(get_logger(),
                "%s; wall_scrapes=%d clearance=%.3f signature=%llu", reason,
                recovery_.wall_scrapes, last_path_min_clearance_m_,
                static_cast<unsigned long long>(last_path_signature_));
    // First scrape: same-goal latest-map replan. Second scrape / prior peel /
    // lethal tip → demote. Do not unstick on first open-front scrape (that
    // burned tour goals via "driving scrape path into corner tip").
    if (recovery_.wall_scrapes >= 2 || recovery_.side_peels > 0 ||
        !navigation2d::PathClearsPinch(last_path_min_clearance_m_)) {
      return UnstickFromPinch(reason);
    }
    latest_map_replans_ = 0;
    const int scrapes = recovery_.wall_scrapes;
    if (StartNavigation(active_goal_, false, true)) {
      recovery_.wall_scrapes = scrapes;  // Begin must not erase the budget
      if (!navigation2d::PathClearsPinch(last_path_min_clearance_m_)) {
        if (last_path_signature_ != 0)
          cooled_signatures_.insert(last_path_signature_);
        // Count as a failed replan scrape; next BreakWallScrapeThrash demotes.
        ++recovery_.wall_scrapes;
        return UnstickFromPinch("replanned path still pinch-scrape");
      }
      PublishStatus("NAVIGATING", "replanned off wall after scrape stop");
      return true;
    }
    return UnstickFromPinch("pinch replan failed");
  }

  bool BreakWallScrapeAfterPeel() {
    // Only after a confirmed scrape session. side_peels-only (open speck peel)
    // must resume/reselect — Unstick→Escape here was the fixed-pose thrash
    // (log: after wall peel clearance=0 scrapes=0 every ~2s).
    if (recovery_.wall_scrapes <= 0) return false;
    return UnstickFromPinch("after wall peel; leave pinch pocket");
  }

  // Goal lies on the outbound trail behind the robot — picking it after a
  // failed mouth is the post-脱困 折返.
  bool GoalIsBehindTrail(const Goal& goal) const {
    if (!pose_ || trajectory_.size() < 8) return false;
    const double rx = pose_->pose.position.x;
    const double ry = pose_->pose.position.y;
    const double to_goal = std::hypot(goal.x - rx, goal.y - ry);
    if (to_goal < .35) return false;
    // Sample older trail points (≥1.2 m back). If the goal is much closer to
    // those than to the robot, driving there is a U-turn reverse.
    double best_behind = 1e9;
    const double rx0 = trajectory_.back().first;
    const double ry0 = trajectory_.back().second;
    double walked = 0.;
    for (std::size_t i = trajectory_.size(); i-- > 0;) {
      if (i + 1 < trajectory_.size())
        walked += std::hypot(trajectory_[i].first - trajectory_[i + 1].first,
                             trajectory_[i].second - trajectory_[i + 1].second);
      if (walked < 1.20) continue;
      if (walked > 8.0) break;
      const double d = std::hypot(goal.x - trajectory_[i].first,
                                  goal.y - trajectory_[i].second);
      best_behind = std::min(best_behind, d);
    }
    (void)rx0;
    (void)ry0;
    return best_behind + .60 < to_goal;
  }

  bool StartRetreat() {
    if (trajectory_.size() < 4 || !pose_) return false;
    const auto cell = PoseCell();
    if (retreat_attempted_ && retreat_cell_x_ == cell.first && retreat_cell_y_ == cell.second)
      return false;
    // Wall-tip thrash: trail arc through dense stuck samples can be metres
    // while Euclidean progress is centimetres — require real displacement.
    // Entry / open-room: keep reverse short — long trail retreat is the
    // inefficient 往回跑 the UI shows.
    const BodyRanges body = ReadBodyRanges();
    const bool entry = pending_entry_goal_ || locked_entry_mouth_;
    const bool pinched = navigation2d::HitBlocksSwing(body.front) &&
                         navigation2d::HitBlocksSwing(body.side);
    const double want_euclid = entry ? navigation2d::kEntryRetreat
                                     : (pinched ? navigation2d::kPocketRetreat
                                                : navigation2d::kOpenRetreat);
    const double min_euclid = 0.5 * want_euclid;
    const double rx = pose_->pose.position.x;
    const double ry = pose_->pose.position.y;
    std::size_t index = trajectory_.size() - 1;
    double best_d = 0.;
    std::size_t best = index;
    while (index > 0) {
      --index;
      const double d = std::hypot(trajectory_[index].first - rx,
                                  trajectory_[index].second - ry);
      if (d > best_d) {
        best_d = d;
        best = index;
      }
      if (d >= want_euclid) break;
    }
    if (best_d < min_euclid) return false;
    const double goal_x = trajectory_[best].first;
    const double goal_y = trajectory_[best].second;
    Goal retreat;
    retreat.x = goal_x;
    retreat.y = goal_y;
    retreat.yaw = std::atan2(goal_y - ry, goal_x - rx);
    // Exit path out of a pinch may already be cooled from first-segment
    // stalls; clear so the driven trail can be planned again.
    cooled_signatures_.clear();
    retreat_attempted_ = true;
    retreat_cell_x_ = cell.first;
    retreat_cell_y_ = cell.second;
    paint_driven_corridor_ = true;
    paint_trail_budget_m_ = pinched ? 12.0 : 4.0;
    clear_start_pinch_ = true;
    retreat_goal_ = true;
    if (StartNavigation(retreat, false)) return true;
    retreat_goal_ = false;
    paint_driven_corridor_ = false;
    return false;
  }

  void SelectNextGoal() {
    if (phase_ == MissionPhase::kReturning || returning_) {
      BeginReturn(partial_return_);
      return;
    }
    // Locked mouth owns the robot until crossed or hard-failed. Do not rebuild
    // a living-room tour while the kitchen threshold is still open.
    if (locked_entry_mouth_ && !pending_entry_goal_)
      pending_entry_goal_ = locked_entry_mouth_;
    if (pending_entry_goal_) {
      if (EntryProbeSatisfied()) {
        explorer_.RecordAttempt(*pending_entry_goal_, true, false);
        ForgetEntryRoom(*pending_entry_goal_);
        ClearEntryMouthLock();
        QueueActiveRegionFinish();
        next_selection_ = now();
        PublishStatus("SELECTING",
                      "room entered; finishing active region first");
        return;
      }
      const int before_attempts = entry_probe_attempts_;
      if (before_attempts < EntryAttemptBudget() && StartEntryProbe()) return;
      if (EasyLockedMouth() && entry_probe_attempts_ < EntryAttemptBudget()) {
        // Keep the lock on a wide mouth; open-loop crawl mid-gap instead of
        // abandoning into a tour loop (live: "entry blocked" spam).
        if (entry_open_loop_ticks_ == 0) entry_open_loop_ticks_ = 1;
        next_selection_ = now() + rclcpp::Duration::from_seconds(0.4);
        PublishStatus("COMPLETING",
                      "wide mouth blocked; crawling mid-gap instead of abandon");
        return;
      }
      if (entry_probe_attempts_ > before_attempts &&
          entry_probe_attempts_ < EntryAttemptBudget()) {
        next_selection_ = now() + rclcpp::Duration::from_seconds(0.4);
        PublishStatus("COMPLETING", "entry plan rejected; budget remains");
        return;
      }
      // Plan failed at the mouth: leave it and continue the tour from HERE.
      // Trail retreat "then retry" was the post-脱困 折返 (drive back, then
      // re-approach the same doorway).
      AbandonLockedEntryMouth(
          "entry blocked; leave mouth and continue tour (no reverse)");
    }
    // Drop mouths the lidar already cleared; keep the remaining committed
    // order. Rebuilding the tour on every SelectNextGoal reordered T1..Tn and
    // made the robot U-turn between alcoves (triangular stutter in open space).
    while (!committed_tour_.empty() &&
           !explorer_.GoalRegionStillFrontier(committed_tour_.front())) {
      committed_tour_.pop_front();
    }
    if (committed_tour_.empty()) BuildTour();
    // A frontier extractor can emit many viewpoints around one wall.  Do not
    // synchronously exhaust that list when the local controller rejects a
    // viewpoint: that turns one transient costmap disagreement into hundreds
    // of failures before the mapper has published its next observation.  This
    // is the same bounded retry principle used by Nav2 recovery trees: try a
    // small, diverse budget, then yield to new sensor/map evidence.
    constexpr int kCandidatePlanBudget = 3;
    int attempted = 0;
    int rotated_reverse = 0;
    const double yaw = Yaw(pose_->pose.orientation);
    while (!committed_tour_.empty() && attempted < kCandidatePlanBudget) {
      const Goal goal = committed_tour_.front();
      if (!explorer_.GoalRegionStillFrontier(goal)) {
        RCLCPP_INFO(get_logger(), "Skipping resolved tour viewpoint (%.2f, %.2f)", goal.x, goal.y);
        committed_tour_.pop_front();
        continue;
      }
      if (RecentlyDemoted(goal)) {
        RCLCPP_INFO(get_logger(),
                    "Skipping recently demoted viewpoint (%.2f, %.2f)", goal.x,
                    goal.y);
        committed_tour_.pop_front();
        // Rebuild inside the open room. Do not pull a global goal while a
        // door in this room is still holding the tour.
        if (committed_tour_.empty() && pose_) BuildTour();
        continue;
      }
      // Defer reverse goals only when they are far. A tip inside the local
      // pocket must be finished even if it sits behind the robot — otherwise
      // we flee the alcove with a red frontier arc still visible.
      const double bearing = std::atan2(goal.y - pose_->pose.position.y,
                                        goal.x - pose_->pose.position.x);
      const double turn = std::abs(NormalizeAngle(bearing - yaw));
      const double goal_range = std::hypot(goal.x - pose_->pose.position.x,
                                           goal.y - pose_->pose.position.y);
      if (turn > 1.75 && goal_range > 2.5 &&
          rotated_reverse < static_cast<int>(committed_tour_.size())) {
        committed_tour_.pop_front();
        committed_tour_.push_back(goal);
        ++rotated_reverse;
        continue;
      }
      if (StartNavigation(goal, false)) {
        empty_frontier_cycles_ = 0;
        frontier_probe_until_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        frontier_probe_count_ = 0;
        reject_streak_ = 0;
        phase_ = MissionPhase::kExploring;
        completing_probe_rounds_ = 0;
        return;
      }
      committed_tour_.pop_front();
      ++attempted;
      // Start pinched in wall-corner inflation: every other goal fails the same
      // start check. Do not entry-probe / rebuild-tour — leave the pinch first.
      const bool start_pinched =
          last_planning_failure_.find("start occupied") != std::string::npos;
      if (start_pinched) {
        ++reject_streak_;
        clear_start_pinch_ = true;
        paint_driven_corridor_ = true;
        RCLCPP_WARN(get_logger(),
                    "Start pinched (%s); peel/short-retreat instead of reject-looping frontiers",
                    last_planning_failure_.c_str());
        if (EscapeOrShortRetreat(
                "start pinched; peel out of inflation",
                "start pinched; short trail retreat")) {
          cooled_signatures_.clear();
          return;
        }
        break;
      }
      // Entry only for real corridor mouths. Open-wall / residual frontiers
      // (user: 没有门缝) must not LockEntryMouth — that burned attempts on
      // gap=1.65 open spans and looked like "stuck at a door".
      if (PassableMouthGoal(goal)) {
        if (locked_entry_mouth_ && !SameEntryMouth(*locked_entry_mouth_, goal)) {
          explorer_.RecordAttempt(goal, false);
          continue;
        }
        LockEntryMouth(goal);
        if (StartEntryProbe()) return;
        if (entry_probe_attempts_ >= EntryAttemptBudget()) {
          AbandonLockedEntryMouth(
              "hallway reject; entry probe exhausted, leaving mouth");
          continue;
        }
        pending_entry_goal_.reset();  // lock retained
        next_selection_ = now() + rclcpp::Duration::from_seconds(0.4);
        return;
      }
      // Soft scrape rejects: rotate to tour tail without RecordAttempt so
      // failed_goals does not explode (live: 2 completed / 17 failed).
      const bool scrape_reject =
          last_planning_failure_.find("wall-scrape") != std::string::npos ||
          last_planning_failure_.find("tip-scrape") != std::string::npos;
      if (scrape_reject) {
        if (last_path_signature_ != 0)
          cooled_signatures_.insert(last_path_signature_);
        const bool same_vp =
            scrape_reject_goal_ &&
            (std::hypot(goal.x - scrape_reject_goal_->x,
                        goal.y - scrape_reject_goal_->y) < 1.20 ||
             std::hypot(goal.frontier_x - scrape_reject_goal_->frontier_x,
                        goal.frontier_y - scrape_reject_goal_->frontier_y) <
                 1.50);
        if (!same_vp) {
          scrape_reject_goal_ = goal;
          scrape_reject_count_ = 1;
        } else {
          ++scrape_reject_count_;
        }
        if (scrape_reject_count_ < 2) {
          committed_tour_.push_back(goal);
          RCLCPP_WARN(get_logger(),
                      "Open-wall scrape reject (%s); rotate tour (soft %d/2) "
                      "gap=%.2f",
                      last_planning_failure_.c_str(), scrape_reject_count_,
                      explorer_.MouthGapWidth(goal.frontier_x, goal.frontier_y));
          continue;
        }
      }
      // Hard demote: occupied / no path / scrape soft budget exhausted.
      explorer_.RecordAttempt(goal, false, true, false);
      recently_demoted_ = goal;
      recently_demoted_until_ = now() + rclcpp::Duration::from_seconds(10.0);
      scrape_reject_goal_.reset();
      scrape_reject_count_ = 0;
      RCLCPP_WARN(get_logger(),
                  "Open-wall goal rejected (%s); demoting, not entry-locking "
                  "gap=%.2f corridor=%s",
                  last_planning_failure_.c_str(),
                  explorer_.MouthGapWidth(goal.frontier_x, goal.frontier_y),
                  explorer_.IsCorridorMouth(goal.frontier_x, goal.frontier_y)
                      ? "yes"
                      : "no");
      continue;
    }
    // Same pose keeps rejecting every queued frontier (often cooled first-
    // segment signatures while pinched in an alcove). Retreat along the
    // driven trail before spinning on identical rejections until PARTIAL.
    if (reject_streak_ >= 2 &&
        EscapeOrShortRetreat("reject streak; peel before reselect",
                             "reject streak; short trail retreat")) {
      cooled_signatures_.clear();
      return;
    }
    geometry_msgs::msg::Twist stop;
    command_publisher_->publish(stop);
    if (!committed_tour_.empty() || attempted > 0 || explorer_.raw_frontier_cells() > 0) {
      const std::size_t known_now = KnownCells();
      // One newly marked cell is scan noise, not map progress.  Resetting the
      // no-executable budget on +1 kept warehouse leftovers spinning until
      // the mission timeout without ever starting the home path.
      constexpr std::size_t kMeaningfulKnownGrowth = 500;
      if (known_now >= last_no_executable_known_cells_ + kMeaningfulKnownGrowth) {
        no_executable_frontier_cycles_ = 0;
        last_no_executable_known_cells_ = known_now;
        frontier_probe_count_ = 0;
        // New map evidence may open a tip the previous leftover missed.
        completing_probe_rounds_ = 0;
      }
      NoteKnownProgress(known_now);
      ++no_executable_frontier_cycles_;
      const std::size_t reachable_unknown = explorer_.ReachableUnknownCells(
          pose_->pose.position.x, pose_->pose.position.y);
      // Nominated mouths that all fail StartNavigation are not executable
      // either. Requiring candidates==0 skipped leftover completion and
      // returned PARTIAL while the same 3 unplannable T-labels looped.
      // Empty tour with candidates>0 is the BLOCKED-DFS trap: mouths exist but
      // Order wiped them — treat as no-executable so completing/entry runs.
      const bool tour_unplannable =
          committed_tour_.empty() && attempted > 0;
      const bool tour_empty_with_candidates =
          committed_tour_.empty() && attempted == 0 &&
          explorer_.candidate_frontier_goals() > 0;
      const bool no_executable_mouths =
          explorer_.candidate_frontier_goals() == 0 || tour_unplannable ||
          tour_empty_with_candidates;
      // Stay for a door only when this room still has a goal to run.
      // Zero candidates means the room is finished; re-entering it here
      // put the stack back and skipped coverage.
      if (explorer_.candidate_frontier_goals() > 0 &&
          explorer_.ActiveHoldsPassage()) {
        QueueActiveRegionFinish();
        if (!committed_tour_.empty()) {
          next_selection_ = now() + rclcpp::Duration::from_seconds(0.4);
          PublishStatus("SELECTING",
                        "door still open; staying in the active room");
          return;
        }
      }
      if (no_executable_mouths && explorer_.ShouldCoverResiduals()) {
        if (completing_probe_rounds_ < 3) {
          StartCompletingProbe();
          return;
        }
        // Completing probe can fail while sealed mouths still open into
        // unknown rooms on later map updates. Returning here left rooms
        // unexplored with PARTIAL and unresolved frontiers.
        if (TipPinchedNow() &&
            TryRescueDistantMouths("no-executable while tip-pinched"))
          return;
        if (ShouldKeepWaitingForFrontiers()) {
          PublishStatus("SELECTING",
                        "openable unknown remains; waiting for map update");
          return;
        }
        BeginReturnOrRescue(reachable_unknown > 0 ||
                            explorer_.raw_frontier_cells() > 0);
        return;
      }
      // Failed mouths were already RecordAttempt'd. Wait for the next map
      // tick instead of a busy reject/rebuild loop (that was the hesitation).
      ClearDisplayedGoal();
      ClearPublishedNavPaths();
      PublishStatus("SELECTING",
                    "frontiers locally blocked; waiting for map update");
      // Tip / open aisle: do not sit through a long empty budget — peel and
      // rescue distant mouths (kitchen) before PARTIAL.
      const int wait_budget =
          TipPinchedNow() ? 2
                          : ((attempted > 0 &&
              navigation2d::AxisSwingClear(ReadBodyRanges().front))
                                 ? 3
                                 : 6);
      if (no_executable_frontier_cycles_ >= wait_budget) {
        if (TipPinchedNow() &&
            TryRescueDistantMouths("wait-budget tip; rescue distant"))
          return;
        if (completing_probe_rounds_ < 3 && explorer_.ShouldCoverResiduals()) {
          StartCompletingProbe();
          return;
        }
        if (ShouldKeepWaitingForFrontiers()) return;
        BeginReturnOrRescue(true);
      }
      return;
    }
    ++empty_frontier_cycles_;
    no_executable_frontier_cycles_ = 0;
    // A mapper can briefly publish a small, fully-known crop before the next
    // scan expands it.  Never report a zero-goal mission as complete from
    // that bootstrap artifact: wait for enough observed support unless an
    // actual frontier visit has already occurred.
    constexpr std::size_t kMinimumInitialCoverageCells = 25000;
    const bool map_is_mature = KnownCells() >= kMinimumInitialCoverageCells ||
        explorer_.completed_goals() > 0;
    PublishStatus("SELECTING", map_is_mature ?
        "no reachable frontier; confirming completion" :
        "initial map still growing; waiting for frontiers");
    if (map_is_mature && empty_frontier_cycles_ >= 12) BeginReturn();
  }

  bool HandoffToNextTourGoal() {
    if (phase_ != MissionPhase::kExploring || returning_ || !navigation_ ||
        committed_tour_.size() < 2) return false;
    const double distance = std::hypot(active_goal_.x - pose_->pose.position.x,
                                       active_goal_.y - pose_->pose.position.y);
    // A frontier viewpoint is a 360-degree observation pose, not a docking
    // point. Once inside this small observation envelope, the lidar has the
    // intended view and the next already-ranked task can be adopted without
    // braking to zero at every frontier boundary.
    if (distance > .24) return false;
    const Goal completed = active_goal_;
    std::size_t next_index = 1;
    // Match BuildTour's ~5 m pocket: cross-hall handoff must stop at a decision
    // boundary so newly opened junction mouths are not skipped.
    constexpr double kHandoffPocketM = 5.0;
    while (next_index < committed_tour_.size()) {
      const Goal next = committed_tour_[next_index];
      if (!explorer_.GoalRegionStillFrontier(next)) {
        ++next_index;
        continue;
      }
      if (!explorer_.InSameReachableRegion(
              pose_->pose.position.x, pose_->pose.position.y, next.x, next.y,
              kHandoffPocketM) &&
          !explorer_.InSameReachableRegion(
              active_goal_.x, active_goal_.y, next.x, next.y, kHandoffPocketM))
        return false;
      const double next_bearing = std::atan2(
          next.y - pose_->pose.position.y, next.x - pose_->pose.position.x);
      const double direction_change = std::abs(NormalizeAngle(
          next_bearing - Yaw(pose_->pose.orientation)));
      // Continuous handoff is valid only when the next task continues the
      // current motion direction. Switching a moving differential drive to
      // a goal behind it creates an artificial emergency stop and lets
      // inertia carry the base outside the newly planned corridor.
      if (direction_change > .60) return false;
      auto previous = std::move(navigation_);
      if (StartNavigation(next, false)) {
        explorer_.RecordAttempt(completed, true);
        // Drop the completed viewpoint and any resolved entries before the
        // newly active goal; leave that goal at the front for normal result
        // bookkeeping after it is reached.
        for (std::size_t index = 0; index < next_index; ++index)
          committed_tour_.pop_front();
        PublishStatus("NAVIGATING", "continuous handoff to next frontier viewpoint");
        return true;
      }
      navigation_ = std::move(previous);
      explorer_.RecordAttempt(next, false);
      ++next_index;
    }
    return false;
  }

  bool MaybePreemptForNearerMouth(double /*path_remaining_m*/) {
    // Mid-drive preempt yanked Navigation2D onto another mouth and caused
    // goal flicker / crossed paths — wasted meters. Region-ordered tours
    // already finish the active room; keep the committed transit.
    return false;
  }

  Goal MakeHomeGoal() const {
    Goal home{start_pose_->pose.position.x, start_pose_->pose.position.y,
              Yaw(start_pose_->pose.orientation)};
    if (!map_ || map_->info.resolution <= 0.) return home;
    const double resolution = map_->info.resolution;
    const double origin_x = map_->info.origin.position.x;
    const double origin_y = map_->info.origin.position.y;
    const int width = static_cast<int>(map_->info.width);
    const int height = static_cast<int>(map_->info.height);
    auto clear_at = [&](double wx, double wy, double clearance) {
      const int col = static_cast<int>(std::floor((wx - origin_x) / resolution));
      const int row = static_cast<int>(std::floor((wy - origin_y) / resolution));
      const int reach = static_cast<int>(std::ceil(clearance / resolution));
      for (int dy = -reach; dy <= reach; ++dy) {
        for (int dx = -reach; dx <= reach; ++dx) {
          if (dx * dx + dy * dy > reach * reach) continue;
          const int x = col + dx, y = row + dy;
          if (x < 0 || y < 0 || x >= width || y >= height) return false;
          if (map_->data[static_cast<std::size_t>(y) * map_->info.width + x] >= 50)
            return false;
        }
      }
      return true;
    };
    constexpr double kHomeClearance = .30;
    if (clear_at(home.x, home.y, kHomeClearance)) return home;
    // Spawn often sits beside a mapped wall. Snap to the nearest clear cell
    // inside 1 m so return planning is not rejected as goal-occupied.
    double best_x = home.x, best_y = home.y;
    double best_d = std::numeric_limits<double>::infinity();
    const int search = static_cast<int>(std::ceil(1.0 / resolution));
    const int home_col = static_cast<int>(std::floor((home.x - origin_x) / resolution));
    const int home_row = static_cast<int>(std::floor((home.y - origin_y) / resolution));
    for (int dy = -search; dy <= search; ++dy) {
      for (int dx = -search; dx <= search; ++dx) {
        const int x = home_col + dx, y = home_row + dy;
        if (x < 0 || y < 0 || x >= width || y >= height) continue;
        const double wx = origin_x + (x + .5) * resolution;
        const double wy = origin_y + (y + .5) * resolution;
        if (!clear_at(wx, wy, kHomeClearance)) continue;
        const double d = std::hypot(wx - home.x, wy - home.y);
        if (d < best_d) {
          best_d = d;
          best_x = wx;
          best_y = wy;
        }
      }
    }
    if (best_d < std::numeric_limits<double>::infinity()) {
      home.x = best_x;
      home.y = best_y;
    }
    return home;
  }

  bool StartBreadcrumbTowardHome() {
    if (!start_pose_ || !pose_ || trajectory_.size() < 4) return false;
    const Goal home = MakeHomeGoal();
    const double rx = pose_->pose.position.x;
    const double ry = pose_->pose.position.y;
    const double home_dist = std::hypot(rx - home.x, ry - home.y);
    if (home_dist < .40) {
      breadcrumb_returning_ = false;
      paint_driven_corridor_ = true;
      paint_trail_budget_m_ = 80.0;
      clear_start_pinch_ = true;
      return StartNavigation(home, true);
    }
    // Nearest driven sample, then one short step back along that trail.
    // A long chord from a doorway aims through the jambs: the controller spins
    // and the same hop is replanned every second.
    std::size_t nearest = trajectory_.size() - 1;
    double nearest_d = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < trajectory_.size(); ++i) {
      const double d = std::hypot(trajectory_[i].first - rx,
                                  trajectory_[i].second - ry);
      if (d < nearest_d) {
        nearest_d = d;
        nearest = i;
      }
    }
    const double min_hop = navigation2d::kSwingClearance;
    const double max_hop = navigation2d::kOpenRetreat;
    Goal step = home;
    bool found = false;
    double best_robot = std::numeric_limits<double>::infinity();
    for (std::size_t i = nearest; i > 0; --i) {
      const auto& sample = trajectory_[i - 1];
      const double d_robot = std::hypot(sample.first - rx, sample.second - ry);
      if (d_robot < min_hop || d_robot > max_hop) continue;
      const double d_home = std::hypot(sample.first - home.x, sample.second - home.y);
      if (d_home >= home_dist) continue;
      if (d_robot < best_robot) {
        found = true;
        best_robot = d_robot;
        step.x = sample.first;
        step.y = sample.second;
        step.yaw = std::atan2(step.y - ry, step.x - rx);
      }
    }
    // The previous hop did not move the chassis. Repeating it is the spin loop.
    if (found && breadcrumb_returning_ && std::isfinite(active_goal_.x) &&
        std::hypot(step.x - active_goal_.x, step.y - active_goal_.y) <
            navigation2d::kPlanningRadius)
      return false;
    if (!found) {
      // No useful trail hop — paint the corridor and drive home directly.
      breadcrumb_returning_ = false;
      paint_driven_corridor_ = true;
      paint_trail_budget_m_ = 80.0;
      clear_start_pinch_ = true;
      return StartNavigation(home, true);
    }
    phase_ = MissionPhase::kReturning;
    returning_ = true;
    breadcrumb_returning_ = true;
    cooled_signatures_.clear();
    paint_driven_corridor_ = true;
    paint_trail_budget_m_ = 80.0;
    clear_start_pinch_ = true;
    if (!StartNavigation(step, true)) return false;
    RCLCPP_INFO(get_logger(),
                "Breadcrumb return hop: pose=(%.2f,%.2f) step=(%.2f,%.2f) home_dist=%.2f",
                rx, ry, step.x, step.y, home_dist);
    PublishStatus("RETURNING", "following driven trail breadcrumbs home");
    return true;
  }

  void RememberEntryRoom(const Goal& frontier, const Goal& probe) {
    for (const auto& room : unentered_rooms_) {
      if (std::hypot(room.frontier_x - frontier.frontier_x,
                     room.frontier_y - frontier.frontier_y) < navigation2d::kSwingClearance)
        return;
    }
    unentered_rooms_.push_back({frontier.frontier_x, frontier.frontier_y, probe.x, probe.y});
  }

  void ForgetEntryRoom(const Goal& frontier) {
    unentered_rooms_.erase(
        std::remove_if(unentered_rooms_.begin(), unentered_rooms_.end(),
                       [&](const UnenteredRoom& room) {
                         return std::hypot(room.frontier_x - frontier.frontier_x,
                                           room.frontier_y - frontier.frontier_y) <
                                navigation2d::kSwingClearance;
                       }),
        unentered_rooms_.end());
  }

  // Frontiers only prove a room was seen. A door the explorer tried to cross
  // but never did leaves a room with no frontier, so before going home drive
  // into the known space behind that door. Only doors with an entry attempt
  // count: far-but-mapped pockets of rooms already entered are not revisited.
  // Each spot gets two tries; a reached spot drops out because the trail now
  // passes it.
  bool TryVisitUnenteredSpace() {
    if (!pose_ || returning_ || phase_ == MissionPhase::kReturning) return false;
    if (unentered_rooms_.empty()) return false;
    constexpr int kVisitAttempts = 2;
    const double rx = pose_->pose.position.x;
    const double ry = pose_->pose.position.y;
    std::vector<std::pair<double, double>> room_seeds;
    for (const auto& room : unentered_rooms_) room_seeds.emplace_back(room.probe_x, room.probe_y);
    auto goals = explorer_.UnvisitedSpaceGoals(
        trajectory_, navigation2d::kOpenRetreat,
        navigation2d::kRoomPocketDepthM * navigation2d::kRoomPocketDepthM,
        room_seeds, navigation2d::kOpenRetreat);
    std::sort(goals.begin(), goals.end(), [&](const Goal& a, const Goal& b) {
      return std::hypot(a.x - rx, a.y - ry) < std::hypot(b.x - rx, b.y - ry);
    });
    for (auto& goal : goals) {
      auto attempt = std::find_if(
          room_visit_attempts_.begin(), room_visit_attempts_.end(),
          [&](const RoomVisitAttempt& a) {
            return std::hypot(a.x - goal.x, a.y - goal.y) < navigation2d::kSwingClearance;
          });
      if (attempt == room_visit_attempts_.end()) {
        room_visit_attempts_.push_back({goal.x, goal.y, 0});
        attempt = std::prev(room_visit_attempts_.end());
      }
      if (attempt->count >= kVisitAttempts) continue;
      ++attempt->count;
      goal.yaw = std::atan2(goal.y - ry, goal.x - rx);
      phase_ = MissionPhase::kExploring;
      committed_tour_.clear();
      if (StartNavigation(goal, false)) {
        RCLCPP_INFO(get_logger(),
                    "Visiting unentered space before return: goal=(%.2f, %.2f) area=%.2f m2 try=%d/%d",
                    goal.x, goal.y, goal.information_gain, attempt->count, kVisitAttempts);
        PublishStatus("NAVIGATING", "visiting a room not yet entered");
        return true;
      }
      RCLCPP_WARN(get_logger(), "Unentered space goal unplannable at (%.2f, %.2f)",
                  goal.x, goal.y);
    }
    return false;
  }

  void BeginReturn(bool force_return = false) {
    if (!start_pose_) return;
    if (phase_ == MissionPhase::kReturning && navigation_) return;
    if (TryVisitUnenteredSpace()) return;
    phase_ = MissionPhase::kReturning;
    returning_ = true;
    committed_tour_.clear();
    ClearEntryMouthLock();
    if (force_return) {
      partial_return_ = true;
      RCLCPP_WARN(get_logger(),
                  "No executable frontier after bounded retries; returning safely with current map");
    } else if (!explorer_.CompletionEligible(initial_known_cells_))
      RCLCPP_WARN(get_logger(),
                  "No frontier survived a stable completion window; returning with reachable map coverage");
    // Return on the final online map, with the FULL driven trail painted free
    // so SLAM door seals / speckles on the outbound path cannot block home.
    const Goal home = MakeHomeGoal();
    cooled_signatures_.clear();
    paint_driven_corridor_ = true;
    paint_trail_budget_m_ = 80.0;
    breadcrumb_returning_ = false;
    if (StartNavigation(home, true)) {
      return_escape_attempts_ = 0;
      RCLCPP_INFO(get_logger(), "Final online-map return path accepted; tracking with RPP");
      PublishStatus("RETURNING", "tracking shortest route home on final observed map");
      return;
    }
    // Direct home blocked: hop along the outbound trail. This recovers the
    // ~3 m short docks where theta* accepted a chord the controller cannot run.
    if (StartBreadcrumbTowardHome()) {
      return_escape_attempts_ = 0;
      return;
    }
    // Cap escape→return retries. An uncapped loop (escape 1.8s → BeginReturn
    // fails → escape again) thrashed for hundreds of cycles, disagreed scan
    // vs odometry, and blew localization by >1 m / 180°.
    constexpr int kMaxReturnEscapes = 5;
    if (return_escape_attempts_ < kMaxReturnEscapes && TryStartEscape()) {
      ++return_escape_attempts_;
      escape_then_return_ = true;
      return;
    }
    // Last attempt: trail paint again after escapes may have moved the pose.
    paint_driven_corridor_ = true;
    paint_trail_budget_m_ = 80.0;
    if (StartNavigation(MakeHomeGoal(), true)) {
      return_escape_attempts_ = 0;
      RCLCPP_INFO(get_logger(), "Final online-map return path accepted after escape; tracking with RPP");
      PublishStatus("RETURNING", "tracking shortest route home on final observed map");
      return;
    }
    if (StartBreadcrumbTowardHome()) return;
    Finish(false, "return path unavailable; stopping to protect localization");
  }

  bool ReplanActiveGoalOnLatestMap(const char* reason) {
    // Each attempt rebuilds NavigationSystem from the newest mapper output
    // and the robot's current true pose. It cannot replay or bias toward the
    // historical trajectory. Bound the loop so a genuinely disconnected
    // frontier/return reports failure instead of churning forever.
    constexpr int kMaximumLatestMapReplans = 2;
    if (latest_map_replans_ >= kMaximumLatestMapReplans) return false;
    ++latest_map_replans_;
    navigation_.reset();
    RCLCPP_WARN(get_logger(),
                "%s; replanning active %s goal on latest online map (%d/%d)",
                reason, returning_ ? "return" : "frontier", latest_map_replans_,
                kMaximumLatestMapReplans);
    return StartNavigation(active_goal_, returning_, true);
  }

  void ClearDisplayedGoal() {
    active_goal_ = Goal{};
    active_goal_.x = std::numeric_limits<double>::quiet_NaN();
    active_goal_.y = std::numeric_limits<double>::quiet_NaN();
    active_goal_.frontier_x = std::numeric_limits<double>::quiet_NaN();
    active_goal_.frontier_y = std::numeric_limits<double>::quiet_NaN();
  }

  void DemoteActiveGoal(const char* reason) {
    if (phase_ != MissionPhase::kReturning && !returning_) {
      explorer_.RecordAttempt(pending_entry_goal_.value_or(active_goal_), false);
      recently_demoted_ = pending_entry_goal_.value_or(active_goal_);
      recently_demoted_until_ = now() + rclcpp::Duration::from_seconds(8.0);
    }
    ClearEntryMouthLock();
    navigation_.reset();
    ClearPublishedNavPaths();
    ClearDisplayedGoal();
    recovery_.Clear();
    if (!committed_tour_.empty()) committed_tour_.pop_front();
    // Dwell before hopping mouths — immediate BuildTour caused junction thrash.
    next_selection_ = now() + rclcpp::Duration::from_seconds(1.5);
    PublishStatus("SELECTING", reason);
  }

  bool AbandonWallTipAfterPathCut(const char* reason) {
    // Log showed: tip (-4.79,-2.22) path-cut → CoolRegion(robot, 1.5 m) froze
    // every local mouth → BuildTour hopped to (-0.33,4.13) across the map.
    // Cool only the tip itself; keep nearby unfinished siblings on the tour.
    RCLCPP_WARN(get_logger(), "%s", reason);
    const Goal tip = active_goal_;
    explorer_.RecordAttempt(tip, false);
    recently_demoted_ = tip;
    recently_demoted_until_ = now() + rclcpp::Duration::from_seconds(10.0);
    explorer_.CoolRegion(tip.x, tip.y, 0.80, 1800u);
    explorer_.CoolRegion(tip.frontier_x, tip.frontier_y, 0.80, 1800u);
    ClearEntryMouthLock();
    navigation_.reset();
    recovery_.Clear();
    constexpr double kLocalHopM = 4.0;
    const double rx = pose_ ? pose_->pose.position.x : tip.x;
    const double ry = pose_ ? pose_->pose.position.y : tip.y;
    const auto is_local = [&](const Goal& goal) {
      return std::hypot(goal.x - rx, goal.y - ry) <= kLocalHopM ||
             std::hypot(goal.frontier_x - tip.frontier_x,
                        goal.frontier_y - tip.frontier_y) <= kLocalHopM;
    };
    std::deque<Goal> local;
    std::deque<Goal> far;
    for (const auto& goal : committed_tour_) {
      if (SameEntryMouth(goal, tip) || RecentlyDemoted(goal)) continue;
      if (is_local(goal))
        local.push_back(goal);
      else
        far.push_back(goal);
    }
    if (pose_) {
      auto mouths = explorer_.SelectGoals(rx, ry);
      mouths = explorer_.OrderByRegion(std::move(mouths), rx, ry,
                                       Yaw(pose_->pose.orientation));
      for (const auto& candidate : mouths) {
        if (SameEntryMouth(candidate, tip) || RecentlyDemoted(candidate))
          continue;
        if (!is_local(candidate)) continue;
        const bool duplicate = std::any_of(
            local.begin(), local.end(),
            [&](const Goal& prior) { return SameEntryMouth(prior, candidate); });
        if (!duplicate) local.push_back(candidate);
      }
      if (local.empty()) {
        for (const auto& candidate : mouths) {
          if (SameEntryMouth(candidate, tip) || RecentlyDemoted(candidate))
            continue;
          far.push_back(candidate);
          break;
        }
      }
    }
    committed_tour_.clear();
    for (auto& goal : local) committed_tour_.push_back(std::move(goal));
    for (auto& goal : far) committed_tour_.push_back(std::move(goal));
    cooled_signatures_.clear();
    latest_map_replans_ = 0;
    mapped_obstacle_rejects_ = 0;
    retreat_attempted_ = false;
    next_selection_ = now() + rclcpp::Duration::from_seconds(1.0);
    PublishStatus("SELECTING", reason);
    if (EscapeOrShortRetreat("leaving wall tip after path-cut thrash; peel",
                             "leaving wall tip after path-cut thrash; short retreat"))
      return true;
    return true;
  }

  bool HandlePathCutsWhileExploring(const navigation2d::NavigationState& state) {
    NoteMappedObstacleReject(active_goal_);
    navigation_.reset();
    // Locked mouth owns the junction. Live lidar often thickens jamb speckles
    // after the plan was accepted; cooling the throat signature then burning
    // deeper probes produced start→cut→start in <200 ms and room_entered=0.
    // Replan the SAME probe on the latest opened grid first; only deepen when
    // that fails. Do not cool entry chords — the doorway path is the mission.
    if (locked_entry_mouth_ || pending_entry_goal_) {
      if (!pending_entry_goal_) pending_entry_goal_ = locked_entry_mouth_;
      latest_map_replans_ = 0;
      if (ReplanActiveGoalOnLatestMap("path cut at locked mouth; replan entry")) {
        PublishStatus("COMPLETING",
                      "path cut; replanning locked mouth on latest map");
        return true;
      }
      if (entry_probe_attempts_ < EntryAttemptBudget()) {
        const int before_attempts = entry_probe_attempts_;
        if (StartEntryProbe()) {
          PublishStatus("COMPLETING",
                        "path cut; retrying locked mouth at deeper probe");
          return true;
        }
        // One rejected plan is not the whole mouth budget. Abandoning here
        // left the room after attempt 2 of 6.
        if (entry_probe_attempts_ > before_attempts &&
            entry_probe_attempts_ < EntryAttemptBudget()) {
          next_selection_ = now() + rclcpp::Duration::from_seconds(0.4);
          PublishStatus("COMPLETING",
                        "path cut; entry budget remains");
          return true;
        }
      }
      AbandonLockedEntryMouth("locked mouth path cut exhausted; leaving mouth");
      return true;
    }
    if (state.path_signature != 0) cooled_signatures_.insert(state.path_signature);
    const double clearance =
        ActivePathMargin(state.global_planning.path_min_clearance_m);
    // Wall-tip chords: replan-once just rebuilds another illegal tip graze and
    // the robot pause→drop→replan→pause in place (red-box tip thrash).
    if (navigation2d::PreferTipAbandonAfterPathCut(mapped_obstacle_rejects_,
                                                   clearance)) {
      return AbandonWallTipAfterPathCut(
          "path cut at low-clearance wall tip; leaving tip pocket");
    }
    if (ReplanActiveGoalOnLatestMap("path cut mapped obstacle; replan once"))
      return true;
    DemoteActiveGoal("path intersected mapped obstacles");
    if (BreakMappedObstacleThrash()) return true;
    return true;
  }

  void NotePoseMotion() {
    if (!pose_) return;
    if (!pose_motion_initialized_ ||
        std::hypot(pose_->pose.position.x - last_motion_x_,
                   pose_->pose.position.y - last_motion_y_) > .12) {
      last_motion_x_ = pose_->pose.position.x;
      last_motion_y_ = pose_->pose.position.y;
      pose_motion_stamp_ = now();
      pose_motion_initialized_ = true;
    }
  }

  bool PoseImmobilityWatchdog(const navigation2d::NavigationState& state) {
    NotePoseMotion();
    if (!navigation_ || !pose_ || !pose_motion_initialized_) return false;
    const bool entry_drive = pending_entry_goal_ || locked_entry_mouth_;
    const BodyRanges body = ReadBodyRanges();
    const double path_clear =
        ActivePathMargin(state.global_planning.path_min_clearance_m);
    const bool tip_wedged =
        !entry_drive &&
        IsPinchGeometryStall(path_clear, body.front, body.side);
    // Open-front slow crawl is not a tip. Longer stall, never peel.
    const bool open_slow =
        !entry_drive && !tip_wedged &&
        navigation2d::AxisSwingClear(body.front);
    const double min_goal_s =
        entry_drive ? 14.0 : (open_slow ? 8.0 : 4.0);
    if ((now() - goal_started_).seconds() < min_goal_s) return false;
    const bool pose_stuck = (now() - pose_motion_stamp_).seconds() >=
                            (entry_drive ? 10.0 : (open_slow ? 7.5 : 3.5));
    const double path_stall_s =
        entry_drive ? 12.0 : (open_slow ? 7.5 : 3.5);
    const bool path_stuck =
        state.path_progress_m < .05 ||
        (last_goal_path_progress_time_.nanoseconds() > 0 &&
         (now() - last_goal_path_progress_time_).seconds() > path_stall_s);
    // Still creeping into a mouth — do not abort and hop to another tour goal.
    if (entry_drive && state.path_progress_m >= .15 &&
        last_goal_path_progress_time_.nanoseconds() > 0 &&
        (now() - last_goal_path_progress_time_).seconds() < 10.0)
      return false;
    if (!pose_stuck || !path_stuck) return false;
    ++immobility_recoveries_;
    RCLCPP_WARN(get_logger(),
                "Pose immobility watchdog: stuck=(%.2f,%.2f) recoveries=%d "
                "returning=%s entry=%s progress=%.2f tip=%s front=%.2f",
                pose_->pose.position.x, pose_->pose.position.y,
                immobility_recoveries_, returning_ ? "yes" : "no",
                pending_entry_goal_ ? "yes" : "no", state.path_progress_m,
                tip_wedged ? "yes" : "no", body.front);
    geometry_msgs::msg::Twist stop;
    command_publisher_->publish(stop);
    navigation_.reset();
    pose_motion_stamp_ = now();  // debounce
    if (phase_ == MissionPhase::kReturning || returning_) {
      cooled_signatures_.clear();
      if (StartBreadcrumbTowardHome()) return true;
      if (return_escape_attempts_ < 5 && TryStartEscape()) {
        ++return_escape_attempts_;
        escape_then_return_ = true;
        return true;
      }
      partial_return_ = true;
      Finish(true, "return stalled; stopping on driven trail near home");
      return true;
    }
    if (entry_drive) {
      if (!pending_entry_goal_) pending_entry_goal_ = locked_entry_mouth_;
      // progress=0 far from the mouth: one peel, then leave the mouth. Do not
      // trail-retreat — that drives back along the outbound path (折返).
      const double mouth_range = std::hypot(
          pose_->pose.position.x - pending_entry_goal_->frontier_x,
          pose_->pose.position.y - pending_entry_goal_->frontier_y);
      if (state.path_progress_m < .08 && mouth_range > 1.5) {
        clear_start_pinch_ = true;
        retreat_attempted_ = false;
        if (EscapeOrShortRetreat(
                "immobility far from locked mouth; peel then retry",
                "unused"))
          return true;
        AbandonLockedEntryMouth(
            "immobility far from mouth; leave and continue tour (no reverse)");
        next_selection_ = now();
        return true;
      }
      retreat_attempted_ = false;
      if (EscapeOrShortRetreat("immobility at locked mouth; peel then retry",
                               "unused"))
        return true;
      if (mouth_range <= 1.5 && entry_probe_attempts_ < EntryAttemptBudget() && StartEntryProbe())
        return true;
      AbandonLockedEntryMouth(
          "immobility exhausted locked mouth; continue tour (no reverse)");
      next_selection_ = now();
      return true;
    }
    // Open / non-tip stall: same-goal replan only — never peel (live: front
    // 1.96 still started body-frame escape and burned the tour).
    if (!tip_wedged) {
      if (last_path_signature_ != 0)
        cooled_signatures_.insert(last_path_signature_);
      if (immobility_open_replans_ < 1 &&
          StartNavigation(active_goal_, false, true)) {
        ++immobility_open_replans_;
        PublishStatus("NAVIGATING", "immobility open-front; replanned goal");
        return true;
      }
      immobility_open_replans_ = 0;
      next_selection_ = now() + rclcpp::Duration::from_seconds(0.4);
      PublishStatus("SELECTING", "immobility open-front; reselect without peel");
      return true;
    }
    // True tip wedge: peel + demote after the first confirmed tip stall.
    explorer_.RecordAttempt(active_goal_, false);
    recently_demoted_ = active_goal_;
    recently_demoted_until_ = now() + rclcpp::Duration::from_seconds(6.0);
    explorer_.CoolRegion(active_goal_.x, active_goal_.y, 0.80, 1800u);
    explorer_.CoolRegion(active_goal_.frontier_x, active_goal_.frontier_y, 0.80,
                         1800u);
    ClearEntryMouthLock();
    retreat_attempted_ = false;
    clear_start_pinch_ = true;
    next_selection_ = now() + rclcpp::Duration::from_seconds(0.4);
    if (EscapeOrShortRetreat(
            "immobility at wall tip; peel out of inflation",
            "immobility at wall tip; short trail retreat"))
      return true;
    if (TryRescueDistantMouths("immobility tip; rescue distant mouths"))
      return true;
    DemoteActiveGoal("immobility tip thrash; demoting cooled viewpoint");
    return true;
  }

  bool RecentlyDemoted(const Goal& goal) {
    if (!recently_demoted_ || recently_demoted_until_.nanoseconds() == 0)
      return false;
    if (now() >= recently_demoted_until_) return false;
    // Mouth match must cover the whole doorway cluster, not just one cell —
    // 0.5 m let adjacent hall standoffs re-queue the same shuttle.
    return std::hypot(goal.x - recently_demoted_->x,
                      goal.y - recently_demoted_->y) < 1.20 ||
           std::hypot(goal.frontier_x - recently_demoted_->frontier_x,
                      goal.frontier_y - recently_demoted_->frontier_y) < 1.50;
  }

  void Finish(bool success, const std::string& message) {
    if (finished_) return;
    finished_ = true;
    ClearEntryMouthLock();
    breadcrumb_returning_ = false;
    geometry_msgs::msg::Twist stop;
    command_publisher_->publish(stop);
    std::string final_message = message;
    double return_error = std::numeric_limits<double>::infinity();
    if (pose_ && start_pose_) {
      return_error = std::hypot(pose_->pose.position.x - start_pose_->pose.position.x,
                                pose_->pose.position.y - start_pose_->pose.position.y);
      // Docking may use a cleared snap of spawn when the recorded start sits
      // inside mapped inflation; accept either the raw start or that snap.
      const Goal dock = MakeHomeGoal();
      const double dock_error =
          std::hypot(pose_->pose.position.x - dock.x, pose_->pose.position.y - dock.y);
      return_error = std::min(return_error, dock_error);
    }
    if (success && return_error > .35) {
      success = false;
      final_message = "return ended outside the home tolerance";
    }
    std::filesystem::create_directories(std::filesystem::path(result_path_).parent_path());
    if (map_) {
      // Persist the exact online occupancy grid used by exploration, together
      // with the trajectory from the selected pose source.  The host-side renderer turns this
      // into the reviewable PNG artifact after the mapper exits.
      std::ofstream snapshot(snapshot_path_);
      snapshot << "{\"width\":" << map_->info.width << ",\"height\":" << map_->info.height
               << ",\"resolution\":" << map_->info.resolution
               << ",\"origin_x\":" << map_->info.origin.position.x
               << ",\"origin_y\":" << map_->info.origin.position.y << ",\"cells\":[";
      for (std::size_t index = 0; index < map_->data.size(); ++index) {
        if (index) snapshot << ',';
        snapshot << static_cast<int>(map_->data[index]);
      }
      snapshot << "],\"trajectory\":[";
      for (std::size_t index = 0; index < trajectory_.size(); ++index) {
        if (index) snapshot << ',';
        snapshot << '[' << trajectory_[index].first << ',' << trajectory_[index].second << ']';
      }
      snapshot << "]}";
    }
    std::ofstream output(result_path_);
    const char* result_status = !success ? "FAILED" : (partial_return_ ? "PARTIAL" : "COMPLETE");
    output << "{\n  \"status\": \"" << result_status << "\",\n"
           << "  \"message\": \"" << final_message << "\",\n"
           << "  \"pose_source\": \"" << pose_source_ << "\",\n"
           << "  \"map_source\": \""
           << (pose_source_ == "ground_truth" ? "ground_truth_scan_insertion"
                                                : "localization2d_scan_matching")
           << "\",\n"
           << "  \"planning\": \"navigation2d/theta_star+rpp\",\n"
           << "  \"last_controller_backend\": \"" << last_backend_ << "\",\n"
           << "  \"mppi_commands\": " << mppi_commands_ << ",\n"
           << "  \"rpp_commands\": " << rpp_commands_ << ",\n"
           << "  \"travel_distance_m\": " << total_distance_m_ << ",\n"
           << "  \"known_cells\": " << KnownCells() << ",\n"
           << "  \"completed_goals\": " << explorer_.completed_goals() << ",\n"
           << "  \"failed_goals\": " << explorer_.failed_goals() << ",\n"
           << "  \"return_error_m\": " << return_error << "\n}\n";
    PublishStatus(!success ? "FAILED" : (partial_return_ ? "PARTIAL" : "COMPLETE"), final_message);
    // Leave the process so the debug UI can Start a new run without a stuck
    // finished_ node continuing to republish the old latched status.
    rclcpp::shutdown();
  }

  static std::string EscapeJson(const std::string& value) {
    std::string out;
    out.reserve(value.size() + 8);
    for (const char ch : value) {
      if (ch == '"' || ch == '\\') out.push_back('\\');
      out.push_back(ch);
    }
    return out;
  }

  void PublishStatus(const std::string& state, const std::string& message) {
    last_state_ = state;
    last_message_ = message;
    std::string tour_json = "[";
    bool first_tour = true;
    auto append_tour_goal = [&](const Goal& goal) {
      if (!first_tour) tour_json += ",";
      first_tour = false;
      tour_json +=
          "{\"x\":" + std::to_string(goal.x) +
          ",\"y\":" + std::to_string(goal.y) +
          ",\"yaw\":" + std::to_string(goal.yaw) +
          ",\"frontier_x\":" + std::to_string(goal.frontier_x) +
          ",\"frontier_y\":" + std::to_string(goal.frontier_y) +
          ",\"frontier_cells\":" + std::to_string(goal.frontier_cells) +
          ",\"information_gain\":" + std::to_string(goal.information_gain) + "}";
    };
    bool status_published_active = false;
    if (std::isfinite(active_goal_.x) && std::isfinite(active_goal_.y)) {
      append_tour_goal(active_goal_);
      status_published_active = true;
    }
    for (const Goal& goal : committed_tour_) {
      if (status_published_active &&
          std::hypot(goal.x - active_goal_.x, goal.y - active_goal_.y) < .05 &&
          std::hypot(goal.frontier_x - active_goal_.frontier_x,
                     goal.frontier_y - active_goal_.frontier_y) < .05) {
        status_published_active = false;
        continue;
      }
      append_tour_goal(goal);
    }

    std_msgs::msg::String status;
    status.data =
        "{\"state\":\"" + EscapeJson(state) +
        "\",\"message\":\"" + EscapeJson(message) +
        "\",\"mission_phase\":\"" + EscapeJson(
            phase_ == MissionPhase::kCompleting ? "completing" :
            phase_ == MissionPhase::kReturning ? "returning" : "exploring") +
        "\",\"known_cells\":" + std::to_string(KnownCells()) +
        ",\"completed_goals\":" + std::to_string(explorer_.completed_goals()) +
        ",\"failed_goals\":" + std::to_string(explorer_.failed_goals()) +
        ",\"raw_frontier_cells\":" + std::to_string(explorer_.raw_frontier_cells()) +
        ",\"unresolved_frontier_cells\":" +
        std::to_string(explorer_.unresolved_frontier_cells()) +
        ",\"candidates\":" + std::to_string(explorer_.candidate_frontier_goals()) +
        ",\"tour_remaining\":" + std::to_string(committed_tour_.size()) +
        ",\"tour\":" + tour_json + "]" +
        ",\"current_goal\":" +
        (std::isfinite(active_goal_.x) && std::isfinite(active_goal_.y)
             ? ("{\"x\":" + std::to_string(active_goal_.x) +
                ",\"y\":" + std::to_string(active_goal_.y) +
                ",\"yaw\":" + std::to_string(active_goal_.yaw) +
                ",\"frontier_x\":" + std::to_string(active_goal_.frontier_x) +
                ",\"frontier_y\":" + std::to_string(active_goal_.frontier_y) +
                ",\"frontier_cells\":" +
                std::to_string(active_goal_.frontier_cells) +
                ",\"information_gain\":" +
                std::to_string(active_goal_.information_gain) + "}")
             : "null") +
        ",\"carrot\":{\"x\":" + std::to_string(last_carrot_x_) +
        ",\"y\":" + std::to_string(last_carrot_y_) +
        ",\"yaw\":" + std::to_string(last_carrot_yaw_) +
        ",\"valid\":" + (last_carrot_valid_ ? "true" : "false") +
        "},\"path_progress_m\":" + std::to_string(last_path_progress_m_) +
        ",\"path_length_m\":" + std::to_string(last_path_length_m_) +
        ",\"path_min_clearance_m\":" + std::to_string(last_path_min_clearance_m_) +
        ",\"path_max_curvature\":" + std::to_string(last_path_max_curvature_) +
        ",\"replans\":" + std::to_string(last_replans_) +
        ",\"incremental_replans\":" + std::to_string(last_incremental_replans_) +
        ",\"recoveries\":" + std::to_string(last_recoveries_) +
        ",\"emergency_stops\":" + std::to_string(last_emergency_stops_) +
        ",\"controller_backend\":\"" + EscapeJson(last_backend_) +
        "\",\"nav_status\":\"" + EscapeJson(last_nav_status_) +
        "\",\"nav_phase\":\"" + EscapeJson(last_nav_phase_) +
        "\",\"maneuver\":\"" + EscapeJson(last_maneuver_) +
        "\",\"monitor_action\":\"" + EscapeJson(last_monitor_action_) +
        "\",\"planning_failure_reason\":\"" + EscapeJson(last_planning_failure_) +
        "\",\"controller_commanded_motion\":" +
        (last_controller_commanded_ ? "true" : "false") +
        ",\"safety_stopped_motion\":" + (last_safety_stopped_ ? "true" : "false") +
        ",\"intentional_stop\":" + (last_intentional_stop_ ? "true" : "false") +
        ",\"fallback_level\":" + std::to_string(last_fallback_level_) +
        ",\"controller_solve_us\":" + std::to_string(last_controller_solve_us_) +
        ",\"minimum_ttc_s\":" + std::to_string(last_minimum_ttc_s_) +
        ",\"planner_expansions\":" + std::to_string(last_planner_expansions_) +
        ",\"planner_elapsed_s\":" + std::to_string(last_planner_elapsed_s_) +
        ",\"robot_radius_m\":" + std::to_string(navigation2d::kPlanningRadius) +
        ",\"cmd_vel\":{\"linear\":" + std::to_string(last_cmd_linear_) +
        ",\"angular\":" + std::to_string(last_cmd_angular_) +
        "},\"requested_cmd\":{\"linear\":" + std::to_string(last_requested_linear_) +
        ",\"angular\":" + std::to_string(last_requested_angular_) +
        "},\"published_cmd\":{\"linear\":" + std::to_string(last_published_linear_) +
        ",\"angular\":" + std::to_string(last_published_angular_) +
        "},\"measured_vel\":{\"linear\":" + std::to_string(last_measured_linear_) +
        ",\"angular\":" + std::to_string(last_measured_angular_) +
        "},\"pose_source\":\"" + EscapeJson(pose_source_) + "\"}";
    status_publisher_->publish(status);
  }

  void CaptureNavTelemetry(const navigation2d::NavigationState& state,
                           const navigation2d::Twist2d& measured_velocity) {
    last_backend_ = BackendName(state.controller_diagnostics.backend);
    last_nav_status_ = NavStatusName(state.status);
    last_nav_phase_ = NavPhaseName(state.phase);
    last_maneuver_ = ManeuverName(state.controller_diagnostics.maneuver);
    last_monitor_action_ = MonitorActionName(state.collision_monitor_action);
    last_planning_failure_ = state.planning_failure_reason;
    last_path_progress_m_ = state.path_progress_m;
    last_path_length_m_ = state.global_path_length_m;
    last_path_min_clearance_m_ = state.global_planning.path_min_clearance_m;
    if (state.path_signature != 0) last_path_signature_ = state.path_signature;
    last_path_max_curvature_ = state.global_planning.path_max_curvature;
    last_replans_ = state.replans;
    last_incremental_replans_ = state.incremental_replans;
    last_recoveries_ = state.recoveries;
    last_emergency_stops_ = state.emergency_stops;
    last_cmd_linear_ = state.command.linear;
    last_cmd_angular_ = state.command.angular;
    last_requested_linear_ = state.requested_command.linear;
    last_requested_angular_ = state.requested_command.angular;
    last_published_linear_ = state.published_command.linear;
    last_published_angular_ = state.published_command.angular;
    last_measured_linear_ = measured_velocity.linear;
    last_measured_angular_ = measured_velocity.angular;
    last_controller_commanded_ = state.controller_commanded_motion;
    last_safety_stopped_ = state.safety_stopped_motion;
    last_intentional_stop_ = state.controller_diagnostics.intentional_stop;
    last_fallback_level_ = state.controller_diagnostics.fallback_level;
    last_controller_solve_us_ = state.controller_diagnostics.solve_us > 0.
                                    ? state.controller_diagnostics.solve_us
                                    : state.controller_solve_us;
    last_minimum_ttc_s_ = state.minimum_ttc_s;
    last_planner_expansions_ = static_cast<int>(state.global_planning.expansions);
    last_planner_elapsed_s_ = state.global_planning.elapsed_s;
  }

  void NoteMappedObstacleReject(const Goal& goal) {
    ++mapped_obstacle_rejects_;
    // Cool only the failed viewpoint briefly. Cooling the whole mouth/room
    // (1.8 m / 4000 cells) hid kitchen and bedroom frontiers while the robot
    // kept reworking living-room crumbs.
    explorer_.CoolRegion(goal.x, goal.y, 0.45, 600u);
  }

  bool BreakMappedObstacleThrash() {
    if (mapped_obstacle_rejects_ < 2) return false;
    mapped_obstacle_rejects_ = 0;
    navigation_.reset();
    recovery_.Clear();
    cooled_signatures_.clear();
    const std::size_t local_unknown =
        pose_ ? explorer_.ReachableUnknownCells(pose_->pose.position.x,
                                                pose_->pose.position.y)
              : 0;
    // Unfinished pockets: try room-side entry before cooling/fleeing.
    if (local_unknown > 80 && pose_) {
      RCLCPP_WARN(get_logger(),
                  "mapped-obstacle reject thrash with local unknown=%zu; "
                  "trying entry before retreat",
                  local_unknown);
      LockEntryMouth(active_goal_);
      paint_driven_corridor_ = true;
      paint_trail_budget_m_ = 8.0;
      if (StartEntryProbe()) return true;
      explorer_.RecordAttempt(active_goal_, false);
      ClearEntryMouthLock();
      if (!committed_tour_.empty()) committed_tour_.pop_front();
      retreat_attempted_ = false;
      if (EscapeOrShortRetreat(
              "mapped-obstacle thrash; peel then retry local pocket",
              "mapped-obstacle thrash; short retreat then retry local pocket"))
        return true;
      next_selection_ = now();
      PublishStatus("SELECTING", "mapped-obstacle thrash; local pocket kept");
      return true;
    }
    RCLCPP_WARN(get_logger(),
                "mapped-obstacle reject thrash; cool tip pocket and retreat");
    ClearEntryMouthLock();
    if (pose_) {
      explorer_.CoolRegion(active_goal_.x, active_goal_.y, 1.80, 3000u);
      explorer_.CoolRegion(active_goal_.frontier_x, active_goal_.frontier_y, 1.80,
                           3000u);
      explorer_.CoolRegion(pose_->pose.position.x, pose_->pose.position.y, 1.80,
                           3000u);
    }
    retreat_attempted_ = false;
    committed_tour_.clear();
    PublishStatus("SELECTING", "leaving pinch after mapped-obstacle rejects");
    if (EscapeOrShortRetreat(
            "leaving pinch after mapped-obstacle rejects; peel",
            "leaving pinch after mapped-obstacle rejects; short retreat"))
      return true;
    next_selection_ = now() + rclcpp::Duration::from_seconds(1.0);
    return true;
  }

  bool PathCutsMappedObstacle(bool execution_tick = false) const {
    (void)execution_tick;
    // Live-map check always runs (including entry). SLAM occupied is lethal;
    // unknown is not. No planning-side carve or flood before PathCuts.
    if (!navigation_ || !map_ || map_->info.resolution <= 0.) return false;
    const auto& route = navigation_->path();
    if (route.size() < 2) return false;
    navigation2d::ExplorationGrid live;
    live.width = static_cast<int>(map_->info.width);
    live.height = static_cast<int>(map_->info.height);
    live.resolution = map_->info.resolution;
    live.origin_x = map_->info.origin.position.x;
    live.origin_y = map_->info.origin.position.y;
    live.cells.resize(static_cast<std::size_t>(live.width) * live.height);
    for (int row = 0; row < live.height; ++row) {
      for (int col = 0; col < live.width; ++col) {
        const auto cell =
            map_->data[static_cast<std::size_t>(row) * map_->info.width + col];
        std::int8_t value = 0;
        if (cell < 0) value = -1;
        else if (cell >= 50) value = 100;
        live.cells[static_cast<std::size_t>(row) * live.width + col] = value;
      }
    }
    std::vector<std::pair<double, double>> path_xy;
    path_xy.reserve(route.size());
    for (const auto& pose : route) {
      path_xy.emplace_back(navigation2d::X(pose) + navigation_origin_x_,
                           navigation2d::Y(pose) + navigation_origin_y_);
    }
    const double robot_x = pose_ ? pose_->pose.position.x : path_xy.front().first;
    const double robot_y = pose_ ? pose_->pose.position.y : path_xy.front().second;
    // The planner already treats the body disk, lidar interiors, and (on the
    // way home) the driven trail as free. Checking the raw grid afterwards
    // rejected that same path: a thickened kitchen jamb sat inside the
    // planning circle while the lidar still saw the throat open, so every
    // return chord was "path intersects mapped obstacle".
    const auto force_free = [&](int col, int row) {
      if (col < 0 || row < 0 || col >= live.width || row >= live.height) return;
      auto& cell = live.cells[static_cast<std::size_t>(row) * live.width + col];
      if (cell > 0) cell = 0;
    };
    const auto free_disk = [&](double x, double y, double radius_m) {
      const int reach = static_cast<int>(std::ceil(radius_m / live.resolution));
      const int col = static_cast<int>(std::floor((x - live.origin_x) / live.resolution));
      const int row = static_cast<int>(std::floor((y - live.origin_y) / live.resolution));
      for (int dy = -reach; dy <= reach; ++dy) {
        for (int dx = -reach; dx <= reach; ++dx) {
          if (dx * dx + dy * dy > reach * reach) continue;
          force_free(col + dx, row + dy);
        }
      }
    };
    free_disk(robot_x, robot_y, navigation2d::kPlanningRadius);
    if (scan_ && pose_) {
      const double yaw = Yaw(pose_->pose.orientation);
      const double sensor_x = robot_x + .18 * std::cos(yaw);
      const double sensor_y = robot_y + .18 * std::sin(yaw);
      const double evidence_range = std::min(3.0, static_cast<double>(scan_->range_max));
      const double step = std::max(.5 * live.resolution, .02);
      for (std::size_t beam = 0; beam < scan_->ranges.size(); ++beam) {
        const double range = scan_->ranges[beam];
        if (!std::isfinite(range) || range < scan_->range_min || range > evidence_range)
          continue;
        const double angle = yaw + scan_->angle_min +
                             static_cast<double>(beam) * scan_->angle_increment;
        const double usable = std::max(0., range - std::max(.06, .5 * live.resolution));
        for (double distance = 0.; distance < usable; distance += step) {
          const double x = sensor_x + distance * std::cos(angle);
          const double y = sensor_y + distance * std::sin(angle);
          const int col = static_cast<int>(std::floor((x - live.origin_x) / live.resolution));
          const int row = static_cast<int>(std::floor((y - live.origin_y) / live.resolution));
          force_free(col, row);
        }
      }
    }
    if (returning_ && trajectory_.size() >= 2) {
      // Same corridor StartNavigation paints for the home plan (80 m at
      // 0.28 m). Exempting only the last kPocketRetreat (1.76 m) dropped the
      // direct home path at the first doorway the planner had already
      // reopened, and the return then retraced the outbound trail in
      // 0.44 m hops (live: accepted, dropped 11 s later, goal=(0.01,0.01)).
      double walked = 0.;
      for (std::size_t i = trajectory_.size(); i-- > 0;) {
        if (i + 1 < trajectory_.size())
          walked += std::hypot(trajectory_[i].first - trajectory_[i + 1].first,
                                trajectory_[i].second - trajectory_[i + 1].second);
        if (walked > 80.) break;
        free_disk(trajectory_[i].first, trajectory_[i].second, .28);
      }
    }
    // Same body as the planner radius — entry no longer uses a 0.14 m
    // exemption that let GLOBAL squeeze tip gaps PathCuts would otherwise cut.
    return navigation2d::PathCutsLiveOccupied(
        live, path_xy, robot_x, robot_y, navigation2d::kPlanningRadius);
  }

  bool TourSegmentCutsWall(double x0, double y0, double x1, double y1) const {
    if (!map_ || map_->info.resolution <= 0.) return false;
    const double resolution = map_->info.resolution;
    const double origin_x = map_->info.origin.position.x;
    const double origin_y = map_->info.origin.position.y;
    const int width = static_cast<int>(map_->info.width);
    const int height = static_cast<int>(map_->info.height);
    const double length = std::hypot(x1 - x0, y1 - y0);
    const int samples = std::max(2, static_cast<int>(std::ceil(length / std::max(.02, .5 * resolution))));
    for (int sample = 0; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / samples;
      const double wx = x0 + ratio * (x1 - x0);
      const double wy = y0 + ratio * (y1 - y0);
      const int col = static_cast<int>(std::floor((wx - origin_x) / resolution));
      const int row = static_cast<int>(std::floor((wy - origin_y) / resolution));
      if (col < 0 || row < 0 || col >= width || row >= height) return true;
      if (map_->data[static_cast<std::size_t>(row) * map_->info.width + col] >= 50)
        return true;
    }
    return false;
  }

  void ClearPublishedNavPaths() {
    nav_msgs::msg::Path empty;
    empty.header.stamp = now();
    empty.header.frame_id = "map";
    path_publisher_->publish(empty);
    local_trajectory_publisher_->publish(empty);
    tour_publisher_->publish(empty);
    last_carrot_valid_ = false;
  }

  void PublishDebugPaths(const navigation2d::NavigationState& state) {
    if (!navigation_ || !pose_) {
      ClearPublishedNavPaths();
      return;
    }
    const auto& route = navigation_->path();
    nav_msgs::msg::Path global;
    global.header.stamp = now();
    global.header.frame_id = "map";
    global.poses.reserve(route.size());
    for (const auto& pose : route) {
      geometry_msgs::msg::PoseStamped stamped;
      stamped.header = global.header;
      stamped.pose = PoseFromXyYaw(
          navigation2d::X(pose) + navigation_origin_x_,
          navigation2d::Y(pose) + navigation_origin_y_,
          navigation2d::Yaw(pose));
      global.poses.push_back(stamped);
    }
    path_publisher_->publish(global);

    nav_msgs::msg::Path local;
    local.header = global.header;
    // Lookahead band from current progress (Nav2-style local plan window).
    double traveled = 0.;
    bool started = false;
    last_carrot_valid_ = false;
    for (std::size_t i = 1; i < global.poses.size(); ++i) {
      const auto& a = global.poses[i - 1].pose.position;
      const auto& b = global.poses[i].pose.position;
      const double seg = std::hypot(b.x - a.x, b.y - a.y);
      const double prev = traveled;
      traveled += seg;
      if (!started && traveled + 1e-6 >= state.path_progress_m) {
        started = true;
        local.poses.push_back(global.poses[i - 1]);
        const double remain = state.path_progress_m - prev;
        const double t = seg > 1e-6 ? std::clamp(remain / seg, 0., 1.) : 0.;
        last_carrot_x_ = a.x + t * (b.x - a.x);
        last_carrot_y_ = a.y + t * (b.y - a.y);
        last_carrot_yaw_ = Yaw(global.poses[i].pose.orientation);
        last_carrot_valid_ = true;
      }
      if (started) {
        local.poses.push_back(global.poses[i]);
        if (traveled - state.path_progress_m > 2.5) break;
      }
    }
    if (local.poses.empty() && !global.poses.empty()) {
      local.poses.push_back(global.poses.front());
      last_carrot_x_ = global.poses.front().pose.position.x;
      last_carrot_y_ = global.poses.front().pose.position.y;
      last_carrot_yaw_ = Yaw(global.poses.front().pose.orientation);
      last_carrot_valid_ = true;
    }
    local_trajectory_publisher_->publish(local);

    geometry_msgs::msg::PoseStamped carrot;
    carrot.header = global.header;
    if (last_carrot_valid_) {
      carrot.pose = PoseFromXyYaw(last_carrot_x_, last_carrot_y_, last_carrot_yaw_);
    }
    carrot_publisher_->publish(carrot);

    nav_msgs::msg::Path tour;
    tour.header = global.header;
    geometry_msgs::msg::PoseArray frontiers;
    frontiers.header = global.header;
    // Tour is a Path message, so the UI draws straight edges between poses.
    // Never chain goals that cut a live mapped wall — that looked like a
    // through-wall global plan even when Navigation2D routed around.
    bool published_active = false;
    double prev_x = pose_->pose.position.x;
    double prev_y = pose_->pose.position.y;
    if (std::isfinite(active_goal_.x) && std::isfinite(active_goal_.y)) {
      frontiers.poses.push_back(
          PoseFromXyYaw(active_goal_.frontier_x, active_goal_.frontier_y,
                        active_goal_.yaw));
      // Skip the straight robot→goal chord when it pierces a mapped wall; the
      // real route is on /navigation/path (GLOBAL). Publishing the chord made
      // the UI look like 穿墙.
      if (!TourSegmentCutsWall(prev_x, prev_y, active_goal_.x, active_goal_.y)) {
        geometry_msgs::msg::PoseStamped stamped;
        stamped.header = tour.header;
        stamped.pose =
            PoseFromXyYaw(active_goal_.x, active_goal_.y, active_goal_.yaw);
        tour.poses.push_back(stamped);
        prev_x = active_goal_.x;
        prev_y = active_goal_.y;
        published_active = true;
      }
    }
    for (const Goal& goal : committed_tour_) {
      if (published_active &&
          std::hypot(goal.x - active_goal_.x, goal.y - active_goal_.y) < .05 &&
          std::hypot(goal.frontier_x - active_goal_.frontier_x,
                     goal.frontier_y - active_goal_.frontier_y) < .05) {
        published_active = false;  // skip only the matching front entry once
        continue;
      }
      if (TourSegmentCutsWall(prev_x, prev_y, goal.x, goal.y)) break;
      geometry_msgs::msg::PoseStamped stamped;
      stamped.header = tour.header;
      stamped.pose = PoseFromXyYaw(goal.x, goal.y, goal.yaw);
      tour.poses.push_back(stamped);
      prev_x = goal.x;
      prev_y = goal.y;
    }
    tour_publisher_->publish(tour);
    frontiers_publisher_->publish(frontiers);
    PublishFrontierDebug();
  }

  void Tick() {
    if (finished_) return;
    if (!map_ || !pose_ || !scan_ || KnownCells() < 100) return;
    if (!start_pose_) {
      start_pose_ = pose_;
      // Simulation can advance rapidly while the mapper receives its
      // first scans. Exploration time is a planning budget, so start it only
      // once pose, scan and a usable map are simultaneously available.
      started_ = now();
      mission_started_ = true;
      return_escape_attempts_ = 0;
      escape_then_return_ = false;
      initial_known_cells_ = KnownCells();
      last_no_executable_known_cells_ = initial_known_cells_;
      last_progress_known_cells_ = initial_known_cells_;
      last_progress_time_ = now();
      bootstrap_until_ = now() + rclcpp::Duration::from_seconds(8.0);
      next_selection_ = bootstrap_until_;
      PublishStatus("WAITING", pose_source_ + " pose ready; waiting for a stable online map");
    }
    if (mission_started_ && (now() - started_).seconds() > max_duration_s_) {
      Finish(false, "exploration timed out");
      return;
    }
    NoteKnownProgress(KnownCells());
    NotePoseMotion();
    if (TickEscape()) return;
    if (now() < bootstrap_until_) {
      geometry_msgs::msg::Twist stop;
      command_publisher_->publish(stop);
      return;
    }
    if (phase_ == MissionPhase::kReturning || returning_) {
      if (!navigation_) {
        // Do not re-enter BeginReturn every tick while an escape is queued —
        // that was the thrash. TickEscape already resumes return once.
        if (escape_until_.nanoseconds() != 0 || escape_then_return_) return;
        if (breadcrumb_returning_ && StartBreadcrumbTowardHome()) return;
        // Wall-tip return: allow more reverse peels before giving up 4 m from
        // home (result.json: return_error=4.1 with first-segment replay fails).
        constexpr int kMaxReturnEscapes = 5;
        if (return_escape_attempts_ < kMaxReturnEscapes && TryStartEscape()) {
          ++return_escape_attempts_;
          escape_then_return_ = true;
          clear_start_pinch_ = true;
          return;
        }
        if (StartBreadcrumbTowardHome()) return;
        if (return_escape_attempts_ >= kMaxReturnEscapes) {
          paint_driven_corridor_ = true;
          paint_trail_budget_m_ = 80.0;
          clear_start_pinch_ = true;
          if (StartNavigation(MakeHomeGoal(), true)) return;
          Finish(true, "return path unavailable; stopping to protect localization");
          return;
        }
        BeginReturn(partial_return_);
        return;
      }
    } else if (!navigation_) {
      // Continue open-loop doorway crawl across ticks (navigation_ was reset
      // when CommandStalled fired; without this the crawl lasted one frame).
      if (pending_entry_goal_ && pose_ && entry_open_loop_ticks_ > 0 &&
          entry_open_loop_ticks_ <= 40) {
        if (EntryProbeSatisfied()) {
          recently_demoted_ = *pending_entry_goal_;
          recently_demoted_until_ = now() + rclcpp::Duration::from_seconds(12.0);
          explorer_.RecordAttempt(*pending_entry_goal_, true, false);
          ForgetEntryRoom(*pending_entry_goal_);
          ClearEntryMouthLock();
          QueueActiveRegionFinish();
          next_selection_ = now();
          PublishStatus("SELECTING",
                        "room entered; finishing active region first");
          RCLCPP_INFO(get_logger(),
                      "room entered at pose=(%.2f,%.2f) (open-loop crawl)",
                      pose_->pose.position.x, pose_->pose.position.y);
          return;
        }
        ++entry_open_loop_ticks_;
        const double rx = pose_->pose.position.x;
        const double ry = pose_->pose.position.y;
        geometry_msgs::msg::Twist crawl;
        {
          // Stop at the planned goal. A point 0.58 m past the frontier is
          // beyond that contract, and the centerline test does not cover the
          // planning circle, so the crawl walked into the wall.
          const double tx = active_goal_.x;
          const double ty = active_goal_.y;
          if (std::hypot(tx - rx, ty - ry) <= navigation2d::kPlanningRadius ||
              TourSegmentCutsWall(rx, ry, tx, ty)) {
            entry_open_loop_ticks_ = 0;
            AbandonLockedEntryMouth(
                "open-loop crawl blocked by wall; leaving mouth");
            return;
          }
          const double bearing = std::atan2(ty - ry, tx - rx);
          const double err =
              NormalizeAngle(bearing - Yaw(pose_->pose.orientation));
          if (std::abs(err) < .90) {
            crawl.linear.x = .18;
            crawl.angular.z = std::clamp(1.6 * err, -.70, .70);
          } else {
            crawl.angular.z = std::clamp(1.9 * err, -.90, .90);
          }
        }
        command_publisher_->publish(crawl);
        PublishStatus("COMPLETING",
                      "entry no-motion; open-loop crawl into mouth");
        if (entry_open_loop_ticks_ > 40) {
          entry_open_loop_ticks_ = 0;
          if (entry_probe_attempts_ < EntryAttemptBudget() && StartEntryProbe()) return;
          AbandonLockedEntryMouth(
              "open-loop crawl exhausted; leaving mouth");
        }
        return;
      }
      // No route and no crawl: drop the last approach command. Leaving it
      // running while the next goal is planned rolls the chassis from the
      // viewpoint into the planning circle (live: arrived, then
      // "start occupied, nearest obstacle 0.19 m").
      geometry_msgs::msg::Twist stop;
      command_publisher_->publish(stop);
      if (now() >= next_selection_) {
        next_selection_ = now() + rclcpp::Duration::from_seconds(1.0);
        SelectNextGoal();
      }
      return;
    }
    if (HandoffToNextTourGoal()) return;
    // Do not cancel a running path merely because its frontier has been
    // observed. That feedback loop was the source of the visible zig-zags:
    // every new scan invalidated the target, stopped the robot, then selected
    // another frontier. Keep tracking this safe standoff pose until the
    // controller reports success or an actual navigation failure.
    const auto local_pose = LocalPose(pose_->pose);
    const auto tick_time = now();
    const auto measured_velocity = FreshVelocity(tick_time);
    const auto state = navigation_->ComputeCommand(local_pose, measured_velocity,
                                                    tick_time.seconds());
    if (PoseImmobilityWatchdog(state)) return;
    // Mouth crossing is geometric — acknowledge once past the jambs, but keep
    // tracking the room-side probe so the chassis finishes into the pocket.
    if (pending_entry_goal_ && !returning_ && !retreat_goal_ && pose_ &&
        EntryProbeSatisfied()) {
      recently_demoted_ = *pending_entry_goal_;
      recently_demoted_until_ = now() + rclcpp::Duration::from_seconds(12.0);
      explorer_.RecordAttempt(*pending_entry_goal_, true, false);
      ForgetEntryRoom(*pending_entry_goal_);
      ClearEntryMouthLock();
      // Queue in-room leftovers now (while still driving the probe) so a later
      // stall/abandon cannot soft-unlock out of the kitchen with white left.
      QueueActiveRegionFinish();
      pending_region_finish_ = true;
      PublishStatus("NAVIGATING",
                    "room entered; finishing probe inside active region");
      RCLCPP_INFO(get_logger(),
                  "room entered at pose=(%.2f,%.2f) (mid-drive mouth cross)",
                  pose_->pose.position.x, pose_->pose.position.y);
      return;
    }
    // Goal already in the observation disk — finish it instead of sitting on
    // CARROT/GOAL with zero command (junction hesitation).
    if (phase_ == MissionPhase::kExploring && !returning_ && pose_ &&
        !retreat_goal_) {
      const double goal_dist = std::hypot(active_goal_.x - pose_->pose.position.x,
                                          active_goal_.y - pose_->pose.position.y);
      if (goal_dist < .28 && state.path_progress_m >= .05) {
        navigation_.reset();
        geometry_msgs::msg::Twist stop;
        command_publisher_->publish(stop);
        // A doorway peek moves the frontier past the 0.35 m association disk.
        // The viewpoint is not done while a room of unknown is still beyond it.
        const bool room_beyond = explorer_.UnenteredRoomBeyond(active_goal_);
        if (pending_entry_goal_ ||
            explorer_.GoalRegionStillFrontier(active_goal_) || room_beyond) {
          if (!pending_entry_goal_) {
            if (room_beyond &&
                !explorer_.GoalRegionStillFrontier(active_goal_)) {
              RCLCPP_INFO(get_logger(),
                          "At viewpoint with room still beyond "
                          "frontier=(%.2f,%.2f); entering",
                          active_goal_.frontier_x, active_goal_.frontier_y);
            }
            LockEntryMouth(active_goal_);
          }
          if (EntryProbeSatisfied()) {
            explorer_.RecordAttempt(*pending_entry_goal_, true, false);
            ForgetEntryRoom(*pending_entry_goal_);
            ClearEntryMouthLock();
            QueueActiveRegionFinish();
            next_selection_ = now();
            PublishStatus("SELECTING", "room entered; finishing active region first");
            RCLCPP_INFO(get_logger(),
                        "room entered at pose=(%.2f,%.2f) (arrived near probe)",
                        pose_->pose.position.x, pose_->pose.position.y);
            return;
          }
          if (entry_probe_attempts_ < EntryAttemptBudget() && StartEntryProbe()) return;
          // One rejected probe is not the mouth. Abandoning here marked the
          // doorway visited and left the room white.
          if (entry_probe_attempts_ < EntryAttemptBudget()) {
            next_selection_ = now() + rclcpp::Duration::from_seconds(0.35);
            PublishStatus("SELECTING",
                          "at doorway; retrying centered entry probe");
            return;
          }
          AbandonLockedEntryMouth(
              "at doorway; entry incomplete, leaving mouth");
          return;
        }
        explorer_.RecordAttempt(active_goal_, true);
        ClearEntryMouthLock();
        if (!committed_tour_.empty()) committed_tour_.pop_front();
        next_selection_ = now();
        PublishStatus("SELECTING", "arrived at viewpoint; continuing tour");
        return;
      }
    }
    if (PathCutsMappedObstacle(/*execution_tick=*/true)) {
      RCLCPP_WARN(get_logger(),
                  "Dropping path that intersects mapped obstacles goal=(%.2f, %.2f)",
                  active_goal_.x, active_goal_.y);
      geometry_msgs::msg::Twist stop;
      command_publisher_->publish(stop);
      ClearPublishedNavPaths();
      if (phase_ == MissionPhase::kReturning || returning_) {
        navigation_.reset();
        cooled_signatures_.clear();
        clear_start_pinch_ = true;
        if (StartBreadcrumbTowardHome()) return;
        if (return_escape_attempts_ < 5 && TryStartEscape()) {
          ++return_escape_attempts_;
          escape_then_return_ = true;
          return;
        }
        paint_driven_corridor_ = true;
        paint_trail_budget_m_ = 80.0;
        if (StartNavigation(MakeHomeGoal(), true)) return;
        partial_return_ = true;
        Finish(true, "return path intersected mapped obstacles");
      } else if (HandlePathCutsWhileExploring(state)) {
        return;
      }
      return;
    }
    CaptureNavTelemetry(state, measured_velocity);
    if (state.path_progress_m >= best_goal_path_progress_m_ + .08) {
      best_goal_path_progress_m_ = state.path_progress_m;
      last_goal_path_progress_time_ = now();
    }
    // Driving a scrape chord into a wall pocket: do not wait for monitor STOP
    // after the nose is already in the corner (live: clear=0.13, cmd still
    // 0.27 m/s into south tip).
    if (phase_ == MissionPhase::kExploring && !returning_ && !retreat_goal_ &&
        !pending_entry_goal_ && !locked_entry_mouth_ && pose_) {
      const double path_clear =
          ActivePathMargin(state.global_planning.path_min_clearance_m);
      const double goal_dist = std::hypot(active_goal_.x - pose_->pose.position.x,
                                          active_goal_.y - pose_->pose.position.y);
      const BodyRanges body = ReadBodyRanges();
      if (goal_dist < 4. * navigation2d::kSwingClearance &&
          state.path_progress_m >= navigation2d::kPlanningRadius &&
          IsCornerTipStall(path_clear, body.front, body.side)) {
        geometry_msgs::msg::Twist stop;
        command_publisher_->publish(stop);
        BreakWallScrapeThrash("driving scrape path into corner tip");
        return;
      }
    }
    if (phase_ == MissionPhase::kExploring && !returning_) {
      const double path_remaining =
          std::max(0., state.global_path_length_m - state.path_progress_m);
      if (MaybePreemptForNearerMouth(path_remaining)) return;
    }
    if (state.controller_diagnostics.backend == navigation2d::ControllerBackend::kAcados)
      ++acados_commands_;
    else if (state.controller_diagnostics.backend == navigation2d::ControllerBackend::kMppi)
      ++mppi_commands_;
    else if (state.controller_diagnostics.backend == navigation2d::ControllerBackend::kRpp)
      ++rpp_commands_;
    geometry_msgs::msg::Twist command;
    command.linear.x = state.command.linear;
    command.angular.z = state.command.angular;
    const bool safety_held =
        state.safety_stopped_motion ||
        state.collision_monitor_action ==
            navigation2d::CollisionMonitorAction::kStop ||
        state.collision_monitor_action ==
            navigation2d::CollisionMonitorAction::kBlindZoneStop;
    // Near a locked mouth, RPP often accel-limits below 0.12 m/s. A short
    // crawl may finish the remaining distance to the planned goal. It must
    // not run after arrival: the success command is already zero, and the
    // old target 0.58 m past the frontier then drove into the wall.
    if (!safety_held && (pending_entry_goal_ || locked_entry_mouth_) && pose_ &&
        !retreat_goal_ &&
        state.status != navigation2d::NavigationStatus::kSucceeded) {
      const Goal& mouth = pending_entry_goal_.value_or(*locked_entry_mouth_);
      const double rx = pose_->pose.position.x;
      const double ry = pose_->pose.position.y;
      const double mouth_range =
          std::hypot(rx - mouth.frontier_x, ry - mouth.frontier_y);
      const double goal_dist =
          std::hypot(rx - active_goal_.x, ry - active_goal_.y);
      const double front_clear = ReadBodyRanges().front;
      if (mouth_range < 1.20 &&
          goal_dist > navigation2d::kPlanningRadius &&
          std::abs(command.linear.x) < .12 &&
          (front_clear <= 0. ||
           front_clear >= navigation2d::kPlanningRadius) &&
          !TourSegmentCutsWall(rx, ry, active_goal_.x, active_goal_.y)) {
        const double bearing =
            std::atan2(active_goal_.y - ry, active_goal_.x - rx);
        const double err =
            NormalizeAngle(bearing - Yaw(pose_->pose.orientation));
        if (std::abs(err) < .90) {
          command.linear.x = .16;
          command.angular.z = std::clamp(1.6 * err, -.70, .70);
        } else {
          command.linear.x = 0.;
          command.angular.z = std::clamp(1.9 * err, -.90, .90);
        }
      }
    }
    // Last hard gate: never publish a forward shove into a near lidar wall,
    // even if a crawl/escape path above set linear > 0.
    if (command.linear.x > 0.) {
      const double front = ReadBodyRanges().front;
      if (navigation2d::HitInsideBody(front)) {
        command.linear.x = 0.;
        if (std::abs(command.angular.z) < .15)
          command.angular.z = 0.;
      }
    }
    command_publisher_->publish(command);
    // Throttle debug topics; control still runs every tick.
    if ((now() - last_debug_publish_).seconds() >= 0.2) {
      last_debug_publish_ = now();
      PublishDebugPaths(state);
      PublishStatus(last_state_.empty() ? "NAVIGATING" : last_state_,
                    last_message_.empty() ? "navigating" : last_message_);
    }
    if (state.recoveries > reported_recoveries_) {
      reported_recoveries_ = state.recoveries;
      RCLCPP_WARN(get_logger(),
                  "Route progress recovery: phase=%d arc=%.2f/%.2f requested=(%.3f,%.3f) published=(%.3f,%.3f) controller_motion=%s safety_stop=%s rpp_stage=%d controller_maneuver=%d intentional_stop=%s monitor_action=%d ttc=%.3f velocity=(%.3f,%.3f) path_signature=%llu planner_error=%s",
                  static_cast<int>(state.phase),
                  state.path_progress_m, state.global_path_length_m,
                  state.requested_command.linear, state.requested_command.angular,
                  state.published_command.linear, state.published_command.angular,
                  state.controller_commanded_motion ? "yes" : "no",
                  state.safety_stopped_motion ? "yes" : "no",
                  state.controller_diagnostics.fallback_level,
                  static_cast<int>(state.controller_diagnostics.maneuver),
                  state.controller_diagnostics.intentional_stop ? "yes" : "no",
                  static_cast<int>(state.collision_monitor_action), state.minimum_ttc_s,
                  measured_velocity.linear, measured_velocity.angular,
                  static_cast<unsigned long long>(state.path_signature),
                  state.planning_failure_reason.empty() ? "none" :
                      state.planning_failure_reason.c_str());
    }
    // Dead-end tip crawl: GOAL sits in a U-alcove against the wall. The exit
    // is open sideways/behind, but RPP keeps probing into the wall. Drop the
    // terminal cell (lidar already saw the pocket), then turn/brief-reverse
    // out along the driven trail instead of kissing the wall until timeout.
    const double goal_distance_now = pose_
        ? std::hypot(active_goal_.x - pose_->pose.position.x,
                     active_goal_.y - pose_->pose.position.y)
        : std::numeric_limits<double>::infinity();
    const BodyRanges body = ReadBodyRanges();
    const double front_range = body.front;
    const double side_range = body.side;
    // Parallel room wall (front open, one flank close) is normal driving, not a
    // U-tip. Pinch requires both axes inside the planning circle.
    const bool forward_clearly_open = navigation2d::AxisSwingClear(front_range);
    const bool corridor_pinch = navigation2d::HitBlocksSwing(front_range) &&
                                 navigation2d::HitBlocksSwing(side_range);
    // Include retreat_goal_: tip thrash used frontier_cells=0 retreat poses
    // that were exempt from this abort, so the robot spun forever in the U.
    const bool near_goal_alcove_crawl =
        phase_ == MissionPhase::kExploring && !returning_ &&
        (goal_distance_now < navigation2d::kSwingClearance ||
         (state.global_path_length_m > .20 &&
          state.path_progress_m >= .80 * state.global_path_length_m)) &&
        last_goal_path_progress_time_.nanoseconds() > 0 &&
        (now() - last_goal_path_progress_time_).seconds() > 2.5 &&
        state.requested_command.linear < .06 &&
        corridor_pinch;
    if (near_goal_alcove_crawl) {
      // Unfinished local unknown (kitchen corner / red-box pocket) must not be
      // CoolRegion'd away. The old 1.8 m pocket cool + RecordAttempt(true) made
      // leftovers vanish (raw_cells>>0, completing found none) while the robot
      // fled down the hall.
      const bool local_frontier_open =
          !retreat_goal_ && explorer_.GoalRegionStillFrontier(active_goal_);
      const std::size_t local_unknown = explorer_.ReachableUnknownCells(
          pose_->pose.position.x, pose_->pose.position.y);
      if (local_frontier_open || local_unknown > 80) {
        RCLCPP_WARN(get_logger(),
                    "Corner crawl stalled with local unknown=%zu frontier=%s; "
                    "retry entry instead of cooling pocket",
                    local_unknown, local_frontier_open ? "yes" : "no");
        navigation_.reset();
        cooled_signatures_.clear();
        if (local_frontier_open && PassableMouthGoal(active_goal_)) {
          LockEntryMouth(active_goal_);
          paint_driven_corridor_ = true;
          paint_trail_budget_m_ = 8.0;
          if (StartEntryProbe()) return;
        }
        // Drop only this viewpoint; keep the pocket nominatable.
        if (!retreat_goal_) explorer_.RecordAttempt(active_goal_, false);
        if (!committed_tour_.empty()) committed_tour_.pop_front();
        retreat_goal_ = false;
        next_selection_ = now();
        PublishStatus("SELECTING",
                      "local pocket unfinished; retrying without pocket cool");
        return;
      }
      RCLCPP_WARN(get_logger(),
                  "Leaving near-goal alcove tip: dist=%.2f front=%.2f side=%.2f "
                  "progress=%.2f/%.2f elapsed=%.1f retreat=%s",
                  goal_distance_now, front_range, side_range,
                  state.path_progress_m, state.global_path_length_m,
                  (now() - goal_started_).seconds(),
                  retreat_goal_ ? "yes" : "no");
      navigation_.reset();
      if (!retreat_goal_) explorer_.RecordAttempt(active_goal_, true);
      // Cool only when the tip is truly done (no local unknown left).
      explorer_.CoolRegion(active_goal_.x, active_goal_.y, 1.80, 3000u);
      explorer_.CoolRegion(active_goal_.frontier_x, active_goal_.frontier_y, 1.80, 3000u);
      explorer_.CoolRegion(pose_->pose.position.x, pose_->pose.position.y, 1.80, 3000u);
      ClearEntryMouthLock();
      retreat_goal_ = false;
      retreat_attempted_ = false;  // allow a short trail exit from this cell
      committed_tour_.clear();
      cooled_signatures_.clear();
      if (EscapeOrShortRetreat("leaving alcove tip; peel",
                               "leaving alcove tip; short trail retreat"))
        return;
      next_selection_ = now();
      PublishStatus("SELECTING", "alcove tip left; pocket cooled, picking a farther goal");
      return;
    }
    if (state.status == navigation2d::NavigationStatus::kSucceeded) {
      navigation_.reset();
      if (phase_ == MissionPhase::kReturning || returning_) {
        const Goal home = MakeHomeGoal();
        const double home_dist =
            std::hypot(pose_->pose.position.x - home.x, pose_->pose.position.y - home.y);
        if (breadcrumb_returning_ && home_dist > .40) {
          if (StartBreadcrumbTowardHome()) return;
          paint_driven_corridor_ = true;
          paint_trail_budget_m_ = 80.0;
          if (StartNavigation(home, true)) return;
        }
        breadcrumb_returning_ = false;
        Finish(true, partial_return_
                         ? "returned home early; some frontiers left unreachable"
                         : "frontiers exhausted and robot returned through final-map global path");
      } else if (phase_ == MissionPhase::kCompleting) {
        explorer_.RecordAttempt(active_goal_, true);
        FinishCompleting();
      } else if (retreat_goal_) {
        retreat_goal_ = false;
        reject_streak_ = 0;
        // After a trail retreat while a mouth is locked, resume that mouth.
        if (locked_entry_mouth_ && entry_probe_attempts_ < EntryAttemptBudget()) {
          pending_entry_goal_ = locked_entry_mouth_;
          if (StartEntryProbe()) return;
        }
        // Keep the committed tour; retreat is not a license to rebuild reverse.
        next_selection_ = now();
        PublishStatus("SELECTING", "left the pinched corridor");
      } else if (pending_entry_goal_) {
        if (EntryProbeSatisfied()) {
          explorer_.RecordAttempt(*pending_entry_goal_, true, false);
          ForgetEntryRoom(*pending_entry_goal_);
          ClearEntryMouthLock();
          QueueActiveRegionFinish();
          next_selection_ = now();
          PublishStatus("SELECTING", "room entered; finishing active region first");
        } else if (entry_probe_attempts_ < EntryAttemptBudget() && StartEntryProbe()) {
          return;
        } else {
          RCLCPP_WARN(get_logger(),
                      "Frontier entry incomplete: frontier=(%.2f,%.2f) attempts=%d "
                      "depth=%.2f; keeping unresolved",
                      pending_entry_goal_->frontier_x, pending_entry_goal_->frontier_y,
                      entry_probe_attempts_, EntryProbeDepth());
          AbandonLockedEntryMouth("frontier entry blocked; unresolved boundary retained");
        }
      } else if (explorer_.UnenteredRoomBeyond(active_goal_) ||
                 (explorer_.GoalRegionStillFrontier(active_goal_) &&
                  PassableMouthGoal(active_goal_))) {
        // Real corridor mouth still open — cross into the room. The frontier
        // may already have stepped past the association disk.
        if (!explorer_.GoalRegionStillFrontier(active_goal_)) {
          RCLCPP_INFO(get_logger(),
                      "Goal reached with room still beyond frontier=(%.2f,%.2f); "
                      "entering",
                      active_goal_.frontier_x, active_goal_.frontier_y);
        }
        LockEntryMouth(active_goal_);
        if (StartEntryProbe()) return;
        if (entry_probe_attempts_ < EntryAttemptBudget()) {
          next_selection_ = now() + rclcpp::Duration::from_seconds(0.35);
          PublishStatus("SELECTING",
                        "at doorway; retrying centered entry probe");
          return;
        }
        AbandonLockedEntryMouth(
            "frontier reached but no safe entry probe; leaving mouth");
      } else if (explorer_.GoalRegionStillFrontier(active_goal_)) {
        // Open-wall frontier (no door throat): observe done, do not entry-lock.
        explorer_.RecordAttempt(active_goal_, true);
        if (!committed_tour_.empty()) committed_tour_.pop_front();
        next_selection_ = now();
        PublishStatus("SELECTING",
                      "open-wall frontier observed; continuing tour");
      } else {
        explorer_.RecordAttempt(active_goal_, true);
        // Goal dispatch already CommitRegion'd; do not re-Enter the robot cell
        // (often still the hallway parent) or the room stack collapses.
        if (!committed_tour_.empty()) committed_tour_.pop_front();
        // After a mid-drive mouth cross, or while white remains in the active
        // room: cover local leftovers before soft-unlock hops out.
        const bool need_finish =
            pending_region_finish_ ||
            (explorer_.GoalInActiveRegion(active_goal_) &&
             explorer_.ReachableUnknownCells(pose_->pose.position.x,
                                             pose_->pose.position.y) > 20);
        pending_region_finish_ = false;
        if (need_finish) {
          QueueActiveRegionFinish();
          next_selection_ = now();
          PublishStatus("SELECTING",
                        "active room unfinished; covering local leftovers");
          return;
        }
        next_selection_ = now();
        PublishStatus("SELECTING", "frontier reached; continuing active region tour");
      }
    } else if (state.status == navigation2d::NavigationStatus::kBlocked ||
               // Explore goals used to skip the wall-clock timeout while the
               // controller sat in kDockToGoal / terminal crawl. Keep the dock
               // exemption only for the home return pose.
               (((phase_ == MissionPhase::kReturning || returning_)
                     ? state.phase != navigation2d::NavigationPhase::kDockToGoal
                     : true) &&
                (now() - goal_started_).seconds() > goal_timeout_s_) ||
               // NavigationSystem's internal recovery can keep returning a
               // nominal command while the robot has made no progress on the
               // first path segment.  Do not wait for the full path-length
               // timeout (130-160 s in the maze) before trying the escape.
               // Three recovery cycles are enough to distinguish a transient
               // controller reset from a genuinely stuck target.
               (state.path_progress_m < .05 && state.recoveries >= 3 &&
                (now() - goal_started_).seconds() > 20.0) ||
               // A frontier can pass the static-map clearance test but still
               // terminate against a wall after the live scan reveals the
               // final approach is too tight.  RPP then commands pure
               // rotation (linear ~= 0) indefinitely.  Treat that as a local
               // approach failure after a short grace period so the recovery
               // can back out and select a different viewpoint.
               (state.path_progress_m < .05 && state.requested_command.linear < .01 &&
                navigation2d::HitBlocksSwing(front_range) &&
                (now() - goal_started_).seconds() > 8.0) ||
               // Mid-route pinch crawl: progress already left the first 5 cm,
               // but the robot sits in a doorway spinning/creeping without
               // meaningful advance. First-segment watches never fire here and
               // the full goal timeout left the robot stuck for minutes.
               // Tight front (or side while forward is also constrained)
               // shortens the grace to 5 s. A parallel room wall with open
               // forward must not count as pinch — that froze robots mid-room.
               (state.path_progress_m >= .05 &&
                last_goal_path_progress_time_.nanoseconds() > 0 &&
                (now() - last_goal_path_progress_time_).seconds() >
                    (corridor_pinch ? 5.0 : 10.0) &&
                (navigation2d::HitBlocksSwing(front_range) ||
                 (state.recoveries >= 2 && !forward_clearly_open) ||
                 (state.requested_command.linear < .05 &&
                  std::abs(state.requested_command.angular) > .15 &&
                  !forward_clearly_open))) ||
               // Mid-route safety latch: only when the forward corridor is
               // actually blocked. Side-jamb STOP during a door entry is not a
               // failed mouth — abandoning it caused hesitate/replan loops.
               (state.path_progress_m >= .05 &&
                state.collision_monitor_action ==
                    navigation2d::CollisionMonitorAction::kStop &&
                state.requested_command.linear > .01 &&
                state.minimum_ttc_s < .15 &&
                navigation2d::HitInsideBody(front_range) &&
                last_goal_path_progress_time_.nanoseconds() > 0 &&
                (now() - last_goal_path_progress_time_).seconds() > 2.0)) {
      const bool monitor_stop =
          state.status == navigation2d::NavigationStatus::kBlocked &&
          state.collision_monitor_action ==
              navigation2d::CollisionMonitorAction::kStop;
      const bool forward_blocked_close =
          navigation2d::HitInsideBody(front_range);
      const bool exploring_drive =
          phase_ == MissionPhase::kExploring && !returning_;

      // Corner-tip: both axes inside the planning circle (IsCornerTipStall).
      if (exploring_drive && monitor_stop) {
        const double path_clear =
            ActivePathMargin(state.global_planning.path_min_clearance_m);
        if (IsCornerTipStall(path_clear, front_range, side_range) &&
            last_goal_path_progress_time_.nanoseconds() > 0 &&
            (now() - last_goal_path_progress_time_).seconds() > 2.0) {
          BreakWallScrapeThrash("corner-tip CARROT stall");
          return;
        }
      }

      // SideSpeckStall: open forward with a lateral speck STOP.
      // Forward outside the planning circle and a close flank is a normal
      // room wall — peel/resume of a wall-foot path is the red-box sawtooth.
      if (exploring_drive && monitor_stop && forward_clearly_open) {
        // Locked doorway: jamb STOP with open forward is the entry crawl, not
        // a mid-room wall. Retreating here left CARROT inside the room.
        if (locked_entry_mouth_ || pending_entry_goal_) {
          if (!pending_entry_goal_) pending_entry_goal_ = locked_entry_mouth_;
          if (last_goal_path_progress_time_.nanoseconds() > 0 &&
              (now() - last_goal_path_progress_time_).seconds() > 4.0) {
            navigation_.reset();
            // Keep cooled_signatures_: clearing re-enabled the banned chord
            // and restarted the probe↔retreat shuttle.
            paint_driven_corridor_ = true;
            paint_trail_budget_m_ = 8.0;
            if (entry_probe_attempts_ < EntryAttemptBudget() && StartEntryProbe()) {
              PublishStatus("COMPLETING",
                            "doorjamb STOP; retrying locked mouth entry");
              return;
            }
            AbandonLockedEntryMouth(
                "doorjamb STOP; leave mouth and continue tour (no reverse)");
            next_selection_ = now();
            return;
          }
          return;
        }
        const double path_clear =
            ActivePathMargin(state.global_planning.path_min_clearance_m);
        if (IsOpenWallScrape(path_clear, side_range, true)) {
          BreakWallScrapeThrash("open-forward wall-scrape STOP");
          return;
        }
        // Open forward + side STOP: replan once, never peel (wide-room false
        // scrape thrash). Progress stall alone is not a wall pocket.
        if (last_goal_path_progress_time_.nanoseconds() > 0 &&
            (now() - last_goal_path_progress_time_).seconds() > 6.0) {
          if (recovery_.open_replans < 1 &&
              ReplanActiveGoalOnLatestMap(
                  "open-forward side stall; replan without peel")) {
            ++recovery_.open_replans;
            return;
          }
          next_selection_ = now() + rclcpp::Duration::from_seconds(0.35);
          PublishStatus("SELECTING",
                        "open-forward side stall; reselect without peel");
          navigation_.reset();
          return;
        }
        return;
      }

      // Nose hit is inside the planning circle. Do not resume the same path.
      if (exploring_drive && monitor_stop && forward_blocked_close) {
        // Locked mouth: one peel, else leave and continue tour from here.
        if (locked_entry_mouth_ || pending_entry_goal_) {
          if (!pending_entry_goal_) pending_entry_goal_ = locked_entry_mouth_;
          retreat_attempted_ = false;
          navigation_.reset();
          if (EscapeOrShortRetreat("forward blocked at locked mouth; peel",
                                   "unused"))
            return;
          AbandonLockedEntryMouth(
              "forward blocked at mouth; leave and continue tour (no reverse)");
          next_selection_ = now();
          return;
        }
        explorer_.RecordAttempt(pending_entry_goal_.value_or(active_goal_),
                                false);
        ClearEntryMouthLock();
        recently_demoted_ = active_goal_;
        recently_demoted_until_ = now() + rclcpp::Duration::from_seconds(8.0);
        if (!committed_tour_.empty()) committed_tour_.pop_front();
        recovery_.Clear();
        if (TryStartEscape()) {
          navigation_.reset();
          PublishStatus("RECOVERING",
                        "forward blocked; escape then advance tour");
          return;
        }
        navigation_.reset();
        next_selection_ = now();
        PublishStatus("SELECTING", "forward blocked; skipping viewpoint");
        return;
      }

      RCLCPP_WARN(get_logger(),
                  "Navigation2D frontier failed: status=%s phase=%d elapsed=%.1f replans=%d path=%.2f requested=(%.3f,%.3f) published=(%.3f,%.3f) controller_maneuver=%d intentional_stop=%s monitor_action=%d ttc=%.3f path_signature=%llu planner_error=%s",
                  state.status == navigation2d::NavigationStatus::kBlocked ? "blocked" : "timeout",
                  static_cast<int>(state.phase), (now() - goal_started_).seconds(), state.replans,
                  state.global_path_length_m,
                  state.requested_command.linear, state.requested_command.angular,
                  state.published_command.linear, state.published_command.angular,
                  static_cast<int>(state.controller_diagnostics.maneuver),
                  state.controller_diagnostics.intentional_stop ? "yes" : "no",
                  static_cast<int>(state.collision_monitor_action), state.minimum_ttc_s,
                  static_cast<unsigned long long>(state.path_signature),
                  state.planning_failure_reason.empty() ? "none" :
                      state.planning_failure_reason.c_str());
      const bool first_segment_stall = state.path_progress_m < .05;
      const bool mid_path_pinch =
          state.path_progress_m >= .05 &&
          last_goal_path_progress_time_.nanoseconds() > 0 &&
          (now() - last_goal_path_progress_time_).seconds() >
              (corridor_pinch ? 5.0 : 10.0);
      const bool timed_out =
          (now() - goal_started_).seconds() > goal_timeout_s_;

      // Timeout: demote without peeling back onto the same goal.
      if (exploring_drive && timed_out && !first_segment_stall &&
          !mid_path_pinch) {
        DemoteActiveGoal("goal timeout; demoting viewpoint");
        return;
      }

      if (first_segment_stall || mid_path_pinch) {
        const bool force_return =
            first_segment_stall ? NoteFirstSegmentFailure(state) : false;
        if (phase_ == MissionPhase::kReturning || returning_) {
          // Direct home chords often fail the first segment ~3 m short.
          // Follow the outbound trail instead of finishing away from spawn.
          navigation_.reset();
          cooled_signatures_.clear();
          if (StartBreadcrumbTowardHome()) return;
          if (return_escape_attempts_ < 5 && TryStartEscape()) {
            ++return_escape_attempts_;
            escape_then_return_ = true;
            return;
          }
        } else {
          // Locked entry: retry within budget; otherwise abandon the mouth so
          // the tour cannot shuttle hall→threshold again.
          if (locked_entry_mouth_ || pending_entry_goal_) {
            if (!pending_entry_goal_) pending_entry_goal_ = locked_entry_mouth_;
            retreat_attempted_ = false;
            navigation_.reset();
            // Accel-limited 0.05 m/s from rest never moves the base; replan
            // storms (live: 190× no-motion at one pose) made it worse. Open-
            // loop crawl toward the probe, then one bounded replan.
            if (state.planning_failure_reason.find("no measured motion") !=
                    std::string::npos &&
                pose_) {
              cooled_signatures_.clear();
              last_planning_failure_ = state.planning_failure_reason;
              // Hand off to the !navigation_ open-loop crawler for ~40 ticks.
              if (entry_open_loop_ticks_ == 0) entry_open_loop_ticks_ = 1;
              return;
            }
            if (entry_probe_attempts_ < EntryAttemptBudget() && StartEntryProbe()) return;
            AbandonLockedEntryMouth(
                "entry stall; leave mouth and continue tour (no reverse)");
            recovery_.Clear();
            next_selection_ = now();
            return;
          }
          // Do NOT recovery_.Clear() here — that zeroed wall_scrapes every
          // tick so room-side stall forever saw scrapes=1 and never left the
          // doorway jamb (live thrash at threshold corner).
          explorer_.RecordAttempt(active_goal_, false);
          recently_demoted_ = active_goal_;
          recently_demoted_until_ =
              now() + rclcpp::Duration::from_seconds(25.0);
        }
        // Unified pinch exit: doorway jamb / corner tip / open wall foot.
        if (phase_ != MissionPhase::kReturning && !returning_ &&
            IsPinchGeometryStall(
                ActivePathMargin(state.global_planning.path_min_clearance_m),
                front_range, side_range)) {
          BreakWallScrapeThrash("room-side / doorway pinch stall");
          return;
        }
        // Not a pinch: the viewpoint was reached and the next route simply
        // did not start (live: nose 0.90 m open, "start occupied, nearest
        // obstacle 0.19 m" after rolling into the wall). Reversing here
        // drives the tail into the wall the goal was placed against. A real
        // pinch already returned through BreakWallScrapeThrash above.
        // Fall through and skip this viewpoint.
        if (force_return && !returning_) {
          navigation_.reset();
          committed_tour_.clear();
          BeginReturn(true);
          return;
        }
      }
      if (!mid_path_pinch &&
          state.planning_failure_reason.find("no measured motion") == std::string::npos &&
          ReplanActiveGoalOnLatestMap(
              state.status == navigation2d::NavigationStatus::kBlocked ?
                  "controller could not execute validated route" : "goal execution timed out"))
        return;
      if (state.planning_failure_reason.find("no measured motion") != std::string::npos &&
          phase_ != MissionPhase::kReturning && !returning_) {
        // Replanning the same goal drives the chassis back into the jamb.
        navigation_.reset();
        explorer_.RecordAttempt(active_goal_, false);
        recently_demoted_ = active_goal_;
        recently_demoted_until_ = now() + rclcpp::Duration::from_seconds(25.0);
        if (!committed_tour_.empty()) committed_tour_.pop_front();
        if (TryStartEscape()) return;
        next_selection_ = now();
        PublishStatus("SELECTING", "no motion against the jamb; leaving this route");
        return;
      }
      navigation_.reset();
      recovery_.Clear();
      if (phase_ == MissionPhase::kReturning || returning_) {
        if (StartBreadcrumbTowardHome()) return;
        // Exploration itself may have covered useful space; a blocked home
        // path is a partial mission, not a hard algorithm failure for debug UI.
        partial_return_ = true;
        Finish(true, "exploration ended; return path blocked on online map");
      } else if (phase_ == MissionPhase::kCompleting) {
        if (!first_segment_stall && !mid_path_pinch)
          explorer_.RecordAttempt(active_goal_, false);
        FinishCompleting();
      } else {
        if (!committed_tour_.empty()) committed_tour_.pop_front();
        next_selection_ = now();
        PublishStatus("SELECTING", first_segment_stall ?
            "first-segment failed; skipping cooled viewpoint" :
            (mid_path_pinch ? "mid-path pinch stall; skipping viewpoint" :
                              "frontier failed; continuing committed tour"));
      }
    }
  }

  std::string result_path_;
  std::string snapshot_path_;
  std::string map_snapshot_path_;
  double max_duration_s_ = 600.;
  double goal_timeout_s_ = 55.;
  double simulation_speed_ = 1.;
  double navigation_origin_x_ = 0., navigation_origin_y_ = 0.;
  std::uint64_t map_revision_ = 0;
  std::uint64_t tour_revision_ = 0;
  std::size_t initial_known_cells_ = 0;
  int empty_frontier_cycles_ = 0;
  int no_executable_frontier_cycles_ = 0;
  std::size_t last_no_executable_known_cells_ = 0;
  bool partial_return_ = false;
  int latest_map_replans_ = 0;
  int entry_open_loop_ticks_ = 0;
  bool locked_entry_easy_ = false;
  bool pending_region_finish_ = false;
  int first_segment_failures_at_pose_ = 0;
  int execution_stall_cycles_ = 0;
  int stall_pose_cell_x_ = 0;
  int stall_pose_cell_y_ = 0;
  bool stall_pose_initialized_ = false;
  std::size_t last_stall_known_cells_ = 0;
  std::size_t last_progress_known_cells_ = 0;
  rclcpp::Time last_progress_time_{0, 0, RCL_ROS_TIME};
  enum class MissionPhase { kExploring, kCompleting, kReturning };
  MissionPhase phase_ = MissionPhase::kExploring;
  int completing_probe_rounds_ = 0;
  std::unordered_set<std::uint64_t> cooled_signatures_;
  int reject_streak_ = 0;
  int mapped_obstacle_rejects_ = 0;
  bool mapped_obstacle_retry_ = false;
  bool narrow_passage_retry_ = false;
  bool open_scrape_open_retry_ = false;
  bool reject_pose_valid_ = false;
  int reject_cell_x_ = 0;
  int reject_cell_y_ = 0;
  bool retreat_goal_ = false;
  bool retreat_attempted_ = false;
  int retreat_cell_x_ = 0;
  int retreat_cell_y_ = 0;
  bool paint_driven_corridor_ = false;
  double paint_trail_budget_m_ = 0.;
  bool clear_start_pinch_ = false;
  std::unordered_set<std::size_t> driven_corridor_;
  bool escape_then_return_ = false;
  bool breadcrumb_returning_ = false;
  int return_escape_attempts_ = 0;
  bool pose_motion_initialized_ = false;
  double last_motion_x_ = 0., last_motion_y_ = 0.;
  rclcpp::Time pose_motion_stamp_{0, 0, RCL_ROS_TIME};
  int immobility_recoveries_ = 0;
  int immobility_open_replans_ = 0;
  // Soft wall-scrape rejects: rotate tour twice before RecordAttempt demote.
  std::optional<Goal> scrape_reject_goal_;
  int scrape_reject_count_ = 0;
  bool tip_rescue_attempted_ = false;
  struct RoomVisitAttempt { double x, y; int count; };
  std::vector<RoomVisitAttempt> room_visit_attempts_;
  struct UnenteredRoom { double frontier_x, frontier_y, probe_x, probe_y; };
  std::vector<UnenteredRoom> unentered_rooms_;
  geometry_msgs::msg::Twist escape_command_;
  rclcpp::Time escape_until_{0, 0, RCL_ROS_TIME};
  int reported_recoveries_ = 0;
  std::uint64_t acados_commands_ = 0;
  std::uint64_t mppi_commands_ = 0, rpp_commands_ = 0;
  double total_distance_m_ = 0.;
  double truth_origin_x_ = 0., truth_origin_y_ = 0., truth_origin_yaw_ = 0.;
  std::string last_backend_ = "not-run";
  std::string last_state_;
  std::string last_message_;
  std::string last_nav_status_ = "idle";
  std::string last_nav_phase_ = "align_to_path";
  std::string last_maneuver_ = "tracking";
  std::string last_monitor_action_ = "none";
  std::string last_planning_failure_;
  double last_path_progress_m_ = 0.;
  double last_path_length_m_ = 0.;
  double last_path_min_clearance_m_ = 0.;
  std::uint64_t last_path_signature_ = 0;
  double last_path_max_curvature_ = 0.;
  double last_cmd_linear_ = 0.;
  double last_cmd_angular_ = 0.;
  double last_requested_linear_ = 0.;
  double last_requested_angular_ = 0.;
  double last_published_linear_ = 0.;
  double last_published_angular_ = 0.;
  double last_measured_linear_ = 0.;
  double last_measured_angular_ = 0.;
  double last_controller_solve_us_ = 0.;
  double last_minimum_ttc_s_ = 0.;
  double last_planner_elapsed_s_ = 0.;
  double last_carrot_x_ = 0.;
  double last_carrot_y_ = 0.;
  double last_carrot_yaw_ = 0.;
  bool last_carrot_valid_ = false;
  bool last_controller_commanded_ = false;
  bool last_safety_stopped_ = false;
  bool last_intentional_stop_ = false;
  int last_replans_ = 0;
  int last_incremental_replans_ = 0;
  int last_recoveries_ = 0;
  int last_emergency_stops_ = 0;
  int last_fallback_level_ = 0;
  int last_planner_expansions_ = 0;
  bool returning_ = false, finished_ = false, mission_started_ = false;
  bool truth_origin_ = false;
  std::string pose_source_;
  Goal active_goal_;
  std::optional<Goal> pending_entry_goal_;
  std::optional<Goal> locked_entry_mouth_;
  int entry_probe_attempts_ = 0;
  std::size_t entry_probe_initial_known_cells_ = 0;
  std::uint64_t entry_probe_initial_map_revision_ = 0;
  GoalRecoverySession recovery_;
  std::optional<Goal> recently_demoted_;
  rclcpp::Time recently_demoted_until_{0, 0, RCL_ROS_TIME};
  double best_goal_path_progress_m_ = 0.;
  rclcpp::Time last_goal_path_progress_time_{0, 0, RCL_ROS_TIME};
  std::deque<Goal> committed_tour_;
  rclcpp::Time started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time goal_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_selection_{0, 0, RCL_ROS_TIME};
  rclcpp::Time bootstrap_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time frontier_probe_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time velocity_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_debug_publish_{0, 0, RCL_ROS_TIME};
  int frontier_probe_count_ = 0;
  std::optional<nav_msgs::msg::OccupancyGrid> map_;
  std::optional<geometry_msgs::msg::PoseStamped> pose_, start_pose_;
  std::optional<sensor_msgs::msg::LaserScan> scan_;
  std::vector<std::pair<double, double>> trajectory_;
  navigation2d::Twist2d velocity_;
  navigation2d::FrontierExplorer explorer_;
  std::unique_ptr<navigation2d::NavigationSystem> navigation_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_trajectory_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tour_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr frontiers_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr frontier_debug_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr planning_input_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_costmap_publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr localization_pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<AutonomousExplorer>());
  } catch (const std::exception& error) {
    std::fprintf(stderr, "autonomous explorer failed: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
