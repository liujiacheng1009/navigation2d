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
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

namespace {

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
  // ~0.207 m; planning uses 0.22 m + soft inflation 0.26 m (same as production).
  // Open-space standoff stays well off walls — lidar covers the rim without
  // driving there. Narrow doorways still use kCloseStandoff via SafeViewpoint.
  config.footprint_clearance = .38;
  config.minimum_standoff = .45;
  config.maximum_standoff = 1.20;
  config.blacklist_radius = 1.20;
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
    resume_after_escape = false;
    active = true;
  }

  void Clear() {
    active = false;
    side_peels = 0;
    open_replans = 0;
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

  bool StartNavigation(const Goal& goal, bool returning, bool latest_map_retry = false) {
    if (!latest_map_retry) latest_map_replans_ = 0;
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
    // the effective planner radius (robot radius + raster half-diagonal), so
    // a valid current pose is not rejected as "start occupied" on a replan
    // after a tight-corner approach.  This only clears the measured current
    // footprint; all cells ahead remain lethal unless observed free.
    // Match production: clear under the measured pose past inflation so a
    // wall-corner approach is not rejected as "start occupied" every tick.
    const double snapshot_clearance = .40;
    const int robot_clearance = static_cast<int>(
        std::ceil(snapshot_clearance / map_->info.resolution));
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
    if (paint_driven_corridor_ && map_->info.resolution > 0. && trajectory_.size() >= 2) {
      // The chassis already drove this corridor. A later scan can pinch the
      // costmap below the planning radius and trap the robot; reopen the
      // driven trail for this one plan. Return / entry use a long budget so
      // door seals along the whole outbound path stay plannable.
      paint_driven_corridor_ = false;
      const double trail_budget_m =
          paint_trail_budget_m_ > 0. ? paint_trail_budget_m_ : 2.0;
      paint_trail_budget_m_ = 0.;
      const double paint_radius_m =
          (returning || trail_budget_m > 4.) ? .28 : .16;
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
            driven_corridor_.insert(static_cast<std::size_t>(y) * map_->info.width + x);
          }
        }
      }
    }
    // Same opening the explorer uses for the viewpoint. Without this, a
    // doorway goal that ClearAt(0.22) accepted sits next to unknown jambs on
    // the raw SLAM grid and Navigation2D rejects it as "goal occupied".
    navigation2d::ExplorationGrid planning_grid;
    planning_grid.width = static_cast<int>(map_->info.width);
    planning_grid.height = static_cast<int>(map_->info.height);
    planning_grid.resolution = map_->info.resolution;
    planning_grid.origin_x = map_->info.origin.position.x;
    planning_grid.origin_y = map_->info.origin.position.y;
    planning_grid.cells = map_->data;
    navigation2d::OpenPassableOpenings(&planning_grid);
    // Locked-mouth entry: paint a short unknown corridor past the frontier so
    // theta* can step into a still-white room after OpenPassable opened the
    // seal. Occupied cells stay lethal — only unknown is temporarily free.
    if (pending_entry_goal_ && map_->info.resolution > 0.) {
      const double ax =
          pending_entry_goal_->frontier_x - pending_entry_goal_->x;
      const double ay =
          pending_entry_goal_->frontier_y - pending_entry_goal_->y;
      const double alen = std::hypot(ax, ay);
      if (alen > .05) {
        const double ux = ax / alen, uy = ay / alen;
        const double lx = -uy, ly = ux;
        const double res = map_->info.resolution;
        const int width = static_cast<int>(map_->info.width);
        const int height = static_cast<int>(map_->info.height);
        for (double depth = -.15; depth <= 1.35; depth += res) {
          for (double lat = -.28; lat <= .28 + 1e-9; lat += res) {
            const double wx = pending_entry_goal_->frontier_x + depth * ux + lat * lx;
            const double wy = pending_entry_goal_->frontier_y + depth * uy + lat * ly;
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
      // The robot's current footprint is physically known to be free. SLAM's
      // confidence threshold can otherwise leave isolated unknown cells below
      // the base and make every first plan fail its start-collision check.
      const auto cell = planning_grid.cells[index];
      const bool lidar_evidence = cell < 0 && near_field_free.count(index) > 0;
      const bool driven = driven_corridor_.count(index) > 0;
      const bool forced_free = cell == 0 || under_robot || lidar_evidence || driven;
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
    config.planner = "theta_star";
    // Regulated Pure Pursuit is the Nav2 controller used for robust service
    // robot path following. Its velocity regulation around curvature and
    // obstacles avoids MPPI fallback oscillation on partially observed maps.
    config.controller = "rpp";
    // Prefer forward tracking, but allow a short reverse so a U-alcove tip
    // can back out a few tens of centimetres while turning toward the exit.
    config.max_reverse_velocity = .10;
    // CAD: chassis 290×230 mm, wheel envelope half-width 0.147 m, max
    // footprint radius ~0.207 m. Match production navigation2d_node (+margin).
    config.robot_radius = .22;
    config.footprint = {{-.145, -.147}, {.115, -.147}, {.145, -.132},
                        {.145, .132}, {.115, .147}, {-.145, .147}};
    // Soft preference band outside the inscribed robot_radius. Keep the
    // production footprint contract; doorway progress is handled by mouth
    // geometry / entry locking, not by shrinking safety distances.
    config.inflation_radius = .30;
    // The global plan already enforces the footprint buffer.  A long local
    // collision projection into sparse unknown cells caused false stops in
    // visibly clear aisles; retain near-field collision checking only.
    config.collision_horizon = .50;
    // Doorway crawl: jamb returns sit at ~body half-width and latched STOP
    // with the open-space travel corridor. Passage mode narrows that band.
    if (pending_entry_goal_) {
      config.collision_monitor_passage_mode = true;
      config.collision_monitor_slowdown_distance = .18;
      config.collision_monitor_approach_horizon = .55;
      config.collision_monitor_stop_distance = .06;
    }
    config.desired_linear_velocity = .28;
    config.regulated_min_speed = .10;
    // Soft target in open space: pull paths off speckles/walls when the room
    // is wide. Doorways cannot reach 0.50 m clearance; the footprint check
    // keeps those segments on the throat centreline.
    config.smoother_min_clearance = .50;
    config.smoother_obstacle_weight = 1.4;
    config.smoother_data_weight = .05;
    config.smoother_max_deviation = .55;
    config.smoother_max_step = .05;
    // Frontier goals are observation viewpoints, but home is a docking pose.
    // Keep their completion contracts separate: return must converge to the
    // recorded ground-truth start instead of stopping at the edge of the
    // ordinary navigation tolerance disk.
    config.goal_xy_tolerance = returning ? .04 : .18;
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
      active_goal_ = goal;
      returning_ = returning;
      // Resume / latest-map replan of the same viewpoint must keep peel and
      // open-replan counters; only a materially new goal opens a session.
      recovery_.Begin(goal.x, goal.y);
      goal_started_ = now();
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
        return false;
      }
      // The planner snapshot opens thin SLAM seals. A chord that is legal on
      // that copy can still put the 0.22 m body on cells the live map draws
      // as occupied. Reject it before it is published or tracked.
      if (PathCutsMappedObstacle()) {
        last_planning_failure_ = "path intersects mapped obstacle";
        RCLCPP_WARN(get_logger(),
                    "Navigation2D rejected goal before execution: %s",
                    last_planning_failure_.c_str());
        if (initial_state.path_signature != 0)
          cooled_signatures_.insert(initial_state.path_signature);
        navigation_.reset();
        // Ban the illegal chord and replan once before cooling the mouth.
        if (!mapped_obstacle_retry_) {
          mapped_obstacle_retry_ = true;
          const bool ok = StartNavigation(goal, returning, true);
          mapped_obstacle_retry_ = false;
          if (ok) return true;
        }
        NoteMappedObstacleReject(goal);
        return false;
      }
      mapped_obstacle_rejects_ = 0;
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
      best_goal_path_progress_m_ = 0.;
      last_goal_path_progress_time_ = now();
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
    navigation2d::OpenPassableOpenings(&planning_grid);

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
    constexpr double kRobotRadius = .22;
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
    RCLCPP_INFO(get_logger(),
                "Frontier graph tour %zu: raw_cells=%zu raw_clusters=%zu candidates=%zu "
                "stable=%zu %s",
                tour_revision_, explorer_.raw_frontier_cells(),
                explorer_.raw_frontier_clusters(), explorer_.candidate_frontier_goals(),
                committed_tour_.size(), explorer_.RegionAudit().c_str());
  }

  void LockEntryMouth(const Goal& mouth) {
    locked_entry_mouth_ = mouth;
    pending_entry_goal_ = mouth;
    entry_probe_attempts_ = 0;
    entry_probe_initial_known_cells_ = KnownCells();
    entry_probe_initial_map_revision_ = map_revision_;
  }

  void ClearEntryMouthLock() {
    locked_entry_mouth_.reset();
    pending_entry_goal_.reset();
    entry_probe_attempts_ = 0;
  }

  bool SameEntryMouth(const Goal& a, const Goal& b) const {
    return std::hypot(a.frontier_x - b.frontier_x, a.frontier_y - b.frontier_y) <
               .75 ||
           std::hypot(a.x - b.x, a.y - b.y) < .55;
  }

  bool StartEntryProbe() {
    if (!pending_entry_goal_ || !pose_) return false;
    // Commit to one mouth: hopping across kitchen / alcove frontiers while the
    // robot sits on the threshold produces the scribble thrash in the red box.
    if (!locked_entry_mouth_ || !SameEntryMouth(*locked_entry_mouth_, *pending_entry_goal_))
      locked_entry_mouth_ = *pending_entry_goal_;
    else
      pending_entry_goal_ = *locked_entry_mouth_;
    // First attempt past the jambs; deeper retries into the OpenPassable nose.
    // Kitchen doors need ~0.30 m past the frontier or the robot stops in the
    // hallway and never observes the room. Widen max depth on retry so a
    // corner pocket past an L-jamb remains reachable.
    const double minimum_depth = .20 + .16 * entry_probe_attempts_;
    const double maximum_depth = entry_probe_attempts_ == 0 ? 1.00 : 1.40;
    const auto probe = explorer_.SelectEntryProbe(
        *pending_entry_goal_, pose_->pose.position.x, pose_->pose.position.y,
        minimum_depth, maximum_depth);
    if (!probe) {
      ++entry_probe_attempts_;
      RCLCPP_WARN(get_logger(),
                  "Entry probe unavailable: frontier=(%.2f,%.2f) attempts=%d",
                  pending_entry_goal_->frontier_x, pending_entry_goal_->frontier_y,
                  entry_probe_attempts_);
      return false;
    }
    ++entry_probe_attempts_;
    entry_probe_initial_known_cells_ = KnownCells();
    entry_probe_initial_map_revision_ = map_revision_;
    // Paint a long outbound corridor so SLAM speckles / door seals near the
    // mouth cannot reject the only path into the pocket.
    paint_driven_corridor_ = true;
    paint_trail_budget_m_ = 8.0;
    cooled_signatures_.clear();
    if (!StartNavigation(*probe, false)) {
      RCLCPP_WARN(get_logger(),
                  "Entry probe rejected: target=(%.2f,%.2f) depth=%.2f attempt=%d "
                  "reason=%s",
                  probe->x, probe->y, minimum_depth, entry_probe_attempts_,
                  last_planning_failure_.empty() ? "unknown" :
                      last_planning_failure_.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(),
                "Entry probe started: frontier=(%.2f,%.2f) target=(%.2f,%.2f) "
                "depth=%.2f attempt=%d",
                pending_entry_goal_->frontier_x, pending_entry_goal_->frontier_y,
                probe->x, probe->y, minimum_depth, entry_probe_attempts_);
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
    if (!pending_entry_goal_ || !pose_) return false;
    // Geometric mouth crossing: hallway viewpoint is on the approach side of
    // the frontier plane; the robot has entered when it is on the opposite
    // side. Depth thresholds falsely fired when lidar peeked from the hall.
    const double ax =
        pending_entry_goal_->frontier_x - pending_entry_goal_->x;
    const double ay =
        pending_entry_goal_->frontier_y - pending_entry_goal_->y;
    const double length = std::hypot(ax, ay);
    if (length < .05) return false;
    const double ux = ax / length, uy = ay / length;
    const double hall_side =
        (pending_entry_goal_->x - pending_entry_goal_->frontier_x) * ux +
        (pending_entry_goal_->y - pending_entry_goal_->frontier_y) * uy;
    const double robot_side =
        (pose_->pose.position.x - pending_entry_goal_->frontier_x) * ux +
        (pose_->pose.position.y - pending_entry_goal_->frontier_y) * uy;
    const double cell = map_ ? map_->info.resolution : .05;
    if (hall_side < 0. && robot_side > cell) return true;
    // Tip already deep in a pocket with the local boundary gone.
    return robot_side > cell &&
           !explorer_.GoalRegionStillFrontier(*pending_entry_goal_);
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

  bool TryStartEscape() {
    if (!scan_ || !pose_ || now() < escape_until_) return false;
    const double front = SectorMinRange(0., .45);
    const double back = SectorMinRange(std::acos(-1.), .45);
    const double left = SectorMinRange(+std::acos(-1.) / 2., .45);
    const double right = SectorMinRange(-std::acos(-1.) / 2., .45);
    geometry_msgs::msg::Twist command;
    // Open room with one tight flank: peel into free space instead of
    // crawling forward along the speck that stalled the monitor.
    const bool pinched_side =
        std::min(left, right) < .30 && std::max(left, right) >= .70;
    if (front >= .45) {
      command.linear.x = pinched_side ? .22 : .18;
      if (left + .25 < right) command.angular.z = -.55;
      else if (right + .25 < left) command.angular.z = .55;
    } else if (back >= .35) {
      // Brief reverse out of a wall tip; bias yaw toward the more open side.
      command.linear.x = -.16;
      if (left > right + .05) command.angular.z = .65;
      else if (right > left + .05) command.angular.z = -.65;
    } else if (std::max(left, right) >= .35) {
      // Too tight to translate — turn toward the open flank first.
      command.angular.z = left >= right ? .70 : -.70;
    } else {
      return false;
    }
    escape_command_ = command;
    escape_until_ = now() + rclcpp::Duration::from_seconds(
        std::abs(command.linear.x) > .01 ? 1.4 : 1.0);
    RCLCPP_WARN(get_logger(),
                "Starting clearance escape: front=%.2f back=%.2f left=%.2f "
                "right=%.2f v=%.2f w=%.2f",
                front, back, left, right, command.linear.x, command.angular.z);
    return true;
  }

  bool TickEscape() {
    if (escape_until_.nanoseconds() == 0) return false;
    if (now() < escape_until_) {
      command_publisher_->publish(escape_command_);
      PublishStatus("RECOVERING", "short clearance translation after first-segment stall");
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
          if (entry_probe_attempts_ < 4 && StartEntryProbe()) {
            PublishStatus("COMPLETING", "resumed locked mouth after peel");
            return true;
          }
        }
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
      if (ShouldKeepWaitingForFrontiers()) {
        PublishStatus("SELECTING",
                      "unresolved boundary; waiting for map before return");
        return;
      }
      BeginReturn(explorer_.raw_frontier_cells() > 0);
      return;
    }
    for (const auto& leftover : leftovers) {
      // Leftover goals inside the same doorway cell just bounce the robot on
      // the threshold. Skip without cooling the mouth.
      if (std::hypot(leftover.x - pose_->pose.position.x,
                      leftover.y - pose_->pose.position.y) < .60) {
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
    if (ShouldKeepWaitingForFrontiers()) {
      PublishStatus("SELECTING",
                    "leftover blocked; waiting for map update before return");
      return;
    }
    BeginReturn(explorer_.raw_frontier_cells() > 0 ||
                explorer_.ReachableUnknownCells(pose_->pose.position.x,
                                                pose_->pose.position.y) > 0);
  }

  bool ShouldKeepWaitingForFrontiers() const {
    if (explorer_.raw_frontier_cells() == 0) return false;
    // Allow several completing rounds on distinct mouths before giving up.
    if (completing_probe_rounds_ < 5) return true;
    return no_executable_frontier_cycles_ < 36;
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
    if (explorer_.raw_frontier_cells() > 0 && completing_probe_rounds_ < 5) {
      StartCompletingProbe();
      return;
    }
    if (ShouldKeepWaitingForFrontiers()) {
      PublishStatus("SELECTING",
                    "completing emptied tour; waiting for unresolved frontiers");
      return;
    }
    BeginReturn(explorer_.raw_frontier_cells() > 0);
  }

  bool StartRetreat() {
    if (trajectory_.size() < 4 || !pose_) return false;
    const auto cell = PoseCell();
    if (retreat_attempted_ && retreat_cell_x_ == cell.first && retreat_cell_y_ == cell.second)
      return false;
    // Wall-tip thrash: a 1.2 m trail walk still lands inside the same U-alcove.
    // Back farther into open space when lidar already shows a pinch.
    const double front = SectorMinRange(0., .45);
    const double left = SectorMinRange(+std::acos(-1.) / 2., .45);
    const double right = SectorMinRange(-std::acos(-1.) / 2., .45);
    const double side = std::min(left > 0. ? left : 10., right > 0. ? right : 10.);
    const bool pinched = (front > 0. && front < .55) || (side < .50);
    const double want_arc = pinched ? 3.2 : 1.2;
    const double min_arc = pinched ? 1.6 : .6;
    double arc = 0.;
    std::size_t index = trajectory_.size() - 1;
    while (index > 0 && arc < want_arc) {
      arc += std::hypot(trajectory_[index].first - trajectory_[index - 1].first,
                         trajectory_[index].second - trajectory_[index - 1].second);
      --index;
    }
    if (arc < min_arc) return false;
    const double goal_x = trajectory_[index].first;
    const double goal_y = trajectory_[index].second;
    if (std::hypot(goal_x - pose_->pose.position.x, goal_y - pose_->pose.position.y) < .55)
      return false;
    Goal retreat;
    retreat.x = goal_x;
    retreat.y = goal_y;
    retreat.yaw = std::atan2(goal_y - pose_->pose.position.y, goal_x - pose_->pose.position.x);
    // Exit path out of a pinch may already be cooled from first-segment
    // stalls; clear so the driven trail can be planned again.
    cooled_signatures_.clear();
    retreat_attempted_ = true;
    retreat_cell_x_ = cell.first;
    retreat_cell_y_ = cell.second;
    paint_driven_corridor_ = true;
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
        ClearEntryMouthLock();
      } else if (entry_probe_attempts_ < 4 && StartEntryProbe()) {
        return;
      } else if (entry_probe_attempts_ < 4) {
        // Plan failed but mouth still open: leave the threshold along the
        // trail, then retry the SAME mouth instead of hopping.
        retreat_attempted_ = false;
        if (StartRetreat()) {
          PublishStatus("NAVIGATING",
                        "entry blocked; trail retreat then retry locked mouth");
          return;
        }
        if (TryStartEscape()) return;
        explorer_.RecordAttempt(*pending_entry_goal_, false);
        ClearEntryMouthLock();
      } else {
        explorer_.RecordAttempt(*pending_entry_goal_, false);
        ClearEntryMouthLock();
      }
    }
    // Drop mouths the lidar already cleared; keep the remaining committed
    // order. Rebuilding the tour on every SelectNextGoal reordered T1..Tn and
    // made the robot U-turn between alcoves (triangular stutter in open space).
    while (!committed_tour_.empty() &&
           !explorer_.GoalRegionStillFrontier(committed_tour_.front())) {
      committed_tour_.pop_front();
    }
    if (committed_tour_.empty()) {
      BuildTour();
      // BuildTour can be empty while mouths still exist (pinched kitchen graph).
      // Take only the nearest forward mouth — never a reverse U-turn crumb.
      if (committed_tour_.empty() && explorer_.candidate_frontier_goals() > 0) {
        auto mouths = explorer_.SelectGoals(
            pose_->pose.position.x, pose_->pose.position.y);
        mouths = explorer_.OrderByRegion(
            std::move(mouths), pose_->pose.position.x, pose_->pose.position.y,
            Yaw(pose_->pose.orientation));
        if (!mouths.empty()) committed_tour_.push_back(mouths.front());
      }
    }
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
                    "Start pinched (%s); retreating instead of reject-looping frontiers",
                    last_planning_failure_.c_str());
        if (StartRetreat()) return;
        if (TryStartEscape()) {
          cooled_signatures_.clear();
          return;
        }
        break;
      }
      // Pre-execution rejection at a doorway: try the bounded room-side entry
      // probe instead of stuffing more hallway standoffs into the tour.
      // Skip if another mouth is already locked.
      if (locked_entry_mouth_ && !SameEntryMouth(*locked_entry_mouth_, goal)) {
        explorer_.RecordAttempt(goal, false);
        continue;
      }
      LockEntryMouth(goal);
      if (StartEntryProbe()) return;
      // Hallway reject + failed entry: keep the lock for a few SelectNextGoal
      // retries; only cool after the attempt budget is exhausted.
      if (entry_probe_attempts_ >= 4) {
        explorer_.RecordAttempt(goal, false);
        ClearEntryMouthLock();
      } else {
        pending_entry_goal_.reset();  // lock retained
        next_selection_ = now() + rclcpp::Duration::from_seconds(0.4);
        return;
      }
      const auto cell = PoseCell();
      if (!reject_pose_valid_ || cell.first != reject_cell_x_ || cell.second != reject_cell_y_) {
        reject_pose_valid_ = true;
        reject_cell_x_ = cell.first;
        reject_cell_y_ = cell.second;
        reject_streak_ = 0;
      }
      ++reject_streak_;
    }
    // Same pose keeps rejecting every queued frontier (often cooled first-
    // segment signatures while pinched in an alcove). Retreat along the
    // driven trail before spinning on identical rejections until PARTIAL.
    if (reject_streak_ >= 2 && StartRetreat()) return;
    if (reject_streak_ >= 2 && TryStartEscape()) {
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
      if (explorer_.raw_frontier_cells() > 0 && no_executable_mouths) {
        if (completing_probe_rounds_ < 5) {
          StartCompletingProbe();
          return;
        }
        // Completing probe can fail while OpenPassable still opens sealed
        // mouths into unknown rooms. Returning here left kitchen/side rooms
        // unexplored with PARTIAL and unresolved frontiers.
        if (ShouldKeepWaitingForFrontiers()) {
          PublishStatus("SELECTING",
                        "openable unknown remains; waiting for map update");
          return;
        }
        BeginReturn(reachable_unknown > 0 || explorer_.raw_frontier_cells() > 0);
        return;
      }
      // Failed mouths were already RecordAttempt'd. Wait for the next map
      // tick instead of a busy reject/rebuild loop (that was the hesitation).
      PublishStatus("SELECTING",
                    "frontiers locally blocked; waiting for map update");
      if (no_executable_frontier_cycles_ >= 12) {
        if (completing_probe_rounds_ < 5 && explorer_.raw_frontier_cells() > 0) {
          StartCompletingProbe();
          return;
        }
        if (ShouldKeepWaitingForFrontiers()) return;
        BeginReturn(true);
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
    const double home_dist =
        std::hypot(pose_->pose.position.x - home.x, pose_->pose.position.y - home.y);
    if (home_dist < .40) {
      breadcrumb_returning_ = false;
      paint_driven_corridor_ = true;
      paint_trail_budget_m_ = 80.0;
      return StartNavigation(home, true);
    }
    // Nearest driven sample, then walk toward the start of the trail (home).
    std::size_t nearest = trajectory_.size() - 1;
    double nearest_d = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < trajectory_.size(); ++i) {
      const double d = std::hypot(trajectory_[i].first - pose_->pose.position.x,
                                  trajectory_[i].second - pose_->pose.position.y);
      if (d < nearest_d) {
        nearest_d = d;
        nearest = i;
      }
    }
    constexpr double kHopM = 2.8;
    constexpr double kMinHopM = 0.90;
    double arc = 0.;
    std::size_t index = nearest;
    while (index > 0 && arc < kHopM) {
      arc += std::hypot(trajectory_[index].first - trajectory_[index - 1].first,
                        trajectory_[index].second - trajectory_[index - 1].second);
      --index;
    }
    Goal step = home;
    if (arc >= kMinHopM) {
      step.x = trajectory_[index].first;
      step.y = trajectory_[index].second;
      step.yaw = std::atan2(step.y - pose_->pose.position.y,
                            step.x - pose_->pose.position.x);
      // Prefer a trail cell that is strictly closer to home than the robot.
      const double step_home = std::hypot(step.x - home.x, step.y - home.y);
      if (step_home + .25 >= home_dist && index > 0) {
        // Keep walking toward trail origin until closer or out of samples.
        while (index > 0 &&
               std::hypot(trajectory_[index].first - home.x,
                          trajectory_[index].second - home.y) + .25 >= home_dist) {
          --index;
        }
        step.x = trajectory_[index].first;
        step.y = trajectory_[index].second;
        step.yaw = std::atan2(step.y - pose_->pose.position.y,
                              step.x - pose_->pose.position.x);
      }
    }
    phase_ = MissionPhase::kReturning;
    returning_ = true;
    breadcrumb_returning_ = true;
    cooled_signatures_.clear();
    paint_driven_corridor_ = true;
    paint_trail_budget_m_ = 80.0;
    if (!StartNavigation(step, true)) return false;
    RCLCPP_INFO(get_logger(),
                "Breadcrumb return hop: pose=(%.2f,%.2f) step=(%.2f,%.2f) home_dist=%.2f",
                pose_->pose.position.x, pose_->pose.position.y, step.x, step.y,
                home_dist);
    PublishStatus("RETURNING", "following driven trail breadcrumbs home");
    return true;
  }

  void BeginReturn(bool force_return = false) {
    if (!start_pose_) return;
    if (phase_ == MissionPhase::kReturning && navigation_) return;
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

  void DemoteActiveGoal(const char* reason) {
    if (phase_ != MissionPhase::kReturning && !returning_) {
      explorer_.RecordAttempt(pending_entry_goal_.value_or(active_goal_), false);
      recently_demoted_ = pending_entry_goal_.value_or(active_goal_);
      recently_demoted_until_ = now() + rclcpp::Duration::from_seconds(8.0);
    }
    ClearEntryMouthLock();
    navigation_.reset();
    recovery_.Clear();
    if (!committed_tour_.empty()) committed_tour_.pop_front();
    // Dwell before hopping mouths — immediate BuildTour caused junction thrash.
    next_selection_ = now() + rclcpp::Duration::from_seconds(1.5);
    PublishStatus("SELECTING", reason);
  }

  bool HandlePathCutsWhileExploring(const navigation2d::NavigationState& state) {
    NoteMappedObstacleReject(active_goal_);
    if (state.path_signature != 0) cooled_signatures_.insert(state.path_signature);
    navigation_.reset();
    // Locked mouth owns the junction: ban the illegal chord and retry the SAME
    // entry. Demoting here was the hesitation loop (probe A → cut → mouth B → …).
    if (locked_entry_mouth_ || pending_entry_goal_) {
      if (!pending_entry_goal_) pending_entry_goal_ = locked_entry_mouth_;
      if (entry_probe_attempts_ < 4 && StartEntryProbe()) {
        PublishStatus("COMPLETING", "path cut; retrying locked mouth with banned chord");
        return true;
      }
      retreat_attempted_ = false;
      if (StartRetreat()) {
        PublishStatus("NAVIGATING", "path cut at locked mouth; retreat then retry");
        return true;
      }
      explorer_.RecordAttempt(*pending_entry_goal_, false);
      recently_demoted_ = *pending_entry_goal_;
      recently_demoted_until_ = now() + rclcpp::Duration::from_seconds(8.0);
      ClearEntryMouthLock();
      next_selection_ = now() + rclcpp::Duration::from_seconds(2.0);
      PublishStatus("SELECTING", "locked mouth path cut exhausted");
      return true;
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
    // Entry crawls past jambs are slow; the old 8 s / 6 s path-stall fired while
    // progress was still advancing and yanked the robot back to the junction.
    const double min_goal_s = entry_drive ? 14.0 : 8.0;
    if ((now() - goal_started_).seconds() < min_goal_s) return false;
    const bool pose_stuck = (now() - pose_motion_stamp_).seconds() >=
                            (entry_drive ? 10.0 : 7.0);
    const double path_stall_s = entry_drive ? 12.0 : 6.0;
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
                "returning=%s entry=%s progress=%.2f",
                pose_->pose.position.x, pose_->pose.position.y,
                immobility_recoveries_, returning_ ? "yes" : "no",
                pending_entry_goal_ ? "yes" : "no", state.path_progress_m);
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
      // Prefer a short peel, then the SAME locked mouth — never BuildTour hop.
      if (TryStartEscape()) {
        recovery_.resume_after_escape = true;
        PublishStatus("RECOVERING", "immobility at locked mouth; peel then retry");
        return true;
      }
      retreat_attempted_ = false;
      if (StartRetreat()) {
        PublishStatus("NAVIGATING",
                      "immobility at locked mouth; trail retreat then retry");
        return true;
      }
      if (entry_probe_attempts_ < 4 && StartEntryProbe()) return true;
      explorer_.RecordAttempt(*pending_entry_goal_, false);
      ClearEntryMouthLock();
      next_selection_ = now() + rclcpp::Duration::from_seconds(2.0);
      PublishStatus("SELECTING", "immobility exhausted locked mouth");
      return true;
    }
    retreat_attempted_ = false;
    if (StartRetreat()) {
      PublishStatus("NAVIGATING", "immobility; leaving along driven trail");
      return true;
    }
    if (TryStartEscape()) return true;
    DemoteActiveGoal("immobility watchdog demoting viewpoint");
    return true;
  }

  bool RecentlyDemoted(const Goal& goal) {
    if (!recently_demoted_ || recently_demoted_until_.nanoseconds() == 0)
      return false;
    if (now() >= recently_demoted_until_) return false;
    return std::hypot(goal.x - recently_demoted_->x,
                      goal.y - recently_demoted_->y) < 0.5 ||
           std::hypot(goal.frontier_x - recently_demoted_->frontier_x,
                      goal.frontier_y - recently_demoted_->frontier_y) < 0.5;
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
        ",\"current_goal\":{\"x\":" + std::to_string(active_goal_.x) +
        ",\"y\":" + std::to_string(active_goal_.y) +
        ",\"yaw\":" + std::to_string(active_goal_.yaw) +
        ",\"frontier_x\":" + std::to_string(active_goal_.frontier_x) +
        ",\"frontier_y\":" + std::to_string(active_goal_.frontier_y) +
        ",\"frontier_cells\":" + std::to_string(active_goal_.frontier_cells) +
        ",\"information_gain\":" + std::to_string(active_goal_.information_gain) +
        "},\"carrot\":{\"x\":" + std::to_string(last_carrot_x_) +
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
        ",\"robot_radius_m\":0.22" +
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
    if (mapped_obstacle_rejects_ < 3) return false;
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
      if (StartRetreat()) {
        PublishStatus("NAVIGATING",
                      "brief retreat then retry unfinished local pocket");
        return true;
      }
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
    if (StartRetreat()) return true;
    if (TryStartEscape()) return true;
    next_selection_ = now() + rclcpp::Duration::from_seconds(1.0);
    return true;
  }

  bool PathCutsMappedObstacle() const {
    // Validate against the LIVE occupancy grid. Full OpenPassable carve here
    // previously let falsely opened structural walls look free, so theta*
    // chords through black walls were accepted. Thin door bars remain
    // crossable via IsOpenableDoorSealCell — do NOT skip this check during
    // entry/return: that published GOAL↔CARROT chords through real walls.
    if (!navigation_ || !map_ || map_->info.resolution <= 0.) return false;
    const auto& route = navigation_->path();
    if (route.size() < 2) return false;
    const double resolution = map_->info.resolution;
    const double origin_x = map_->info.origin.position.x;
    const double origin_y = map_->info.origin.position.y;
    const int width = static_cast<int>(map_->info.width);
    const int height = static_cast<int>(map_->info.height);
    navigation2d::ExplorationGrid seal_grid;
    seal_grid.width = width;
    seal_grid.height = height;
    seal_grid.resolution = resolution;
    seal_grid.origin_x = origin_x;
    seal_grid.origin_y = origin_y;
    seal_grid.cells.resize(static_cast<std::size_t>(width * height));
    for (int row = 0; row < height; ++row) {
      for (int col = 0; col < width; ++col) {
        const auto cell = map_->data[static_cast<std::size_t>(row) * map_->info.width + col];
        std::int8_t value = 0;
        if (cell < 0) value = -1;
        else if (cell >= 50) value = 100;
        seal_grid.cells[static_cast<std::size_t>(row) * width + col] = value;
      }
    }
    constexpr double kBodyRadius = .22;
    // Only the chassis footprint may ignore occupied cells (SLAM bleed under
    // the base). The old 0.35 m disk hid short through-wall chords at the
    // doorway when GOAL sat just past a black cluster.
    constexpr double kUnderRobotM = .22;
    const double robot_x = pose_ ? pose_->pose.position.x : 0.;
    const double robot_y = pose_ ? pose_->pose.position.y : 0.;
    const auto hits = [&](double world_x, double world_y) {
      if (pose_ && std::hypot(world_x - robot_x, world_y - robot_y) < kUnderRobotM)
        return false;
      const int col = static_cast<int>(std::floor((world_x - origin_x) / resolution));
      const int row = static_cast<int>(std::floor((world_y - origin_y) / resolution));
      const int reach = static_cast<int>(std::ceil(
          (kBodyRadius + std::sqrt(.5) * resolution) / resolution));
      for (int dy = -reach; dy <= reach; ++dy) {
        for (int dx = -reach; dx <= reach; ++dx) {
          const int x = col + dx, y = row + dy;
          if (x < 0 || y < 0 || x >= width || y >= height) continue;
          const auto cell = map_->data[static_cast<std::size_t>(y) * map_->info.width + x];
          if (cell < 50) continue;
          // Thin door bars OpenPassable would open remain legal to cross.
          if (navigation2d::IsOpenableDoorSealCell(seal_grid, x, y)) continue;
          const double center_x = origin_x + (x + .5) * resolution;
          const double center_y = origin_y + (y + .5) * resolution;
          const double half = .5 * resolution;
          const double nearest_x = std::max(std::abs(world_x - center_x) - half, 0.);
          const double nearest_y = std::max(std::abs(world_y - center_y) - half, 0.);
          if (std::hypot(nearest_x, nearest_y) <= kBodyRadius) return true;
        }
      }
      return false;
    };
    // Also reject when the densified centerline itself steps on a non-seal
    // occupied cell (catches thin wall clips the body-disk sampling misses).
    const auto centerline_hits = [&](double world_x, double world_y) {
      if (pose_ && std::hypot(world_x - robot_x, world_y - robot_y) < kUnderRobotM)
        return false;
      const int col = static_cast<int>(std::floor((world_x - origin_x) / resolution));
      const int row = static_cast<int>(std::floor((world_y - origin_y) / resolution));
      if (col < 0 || row < 0 || col >= width || row >= height) return true;
      const auto cell = map_->data[static_cast<std::size_t>(row) * map_->info.width + col];
      if (cell < 50) return false;
      return !navigation2d::IsOpenableDoorSealCell(seal_grid, col, row);
    };
    for (std::size_t index = 0; index < route.size(); ++index) {
      const double x = navigation2d::X(route[index]) + navigation_origin_x_;
      const double y = navigation2d::Y(route[index]) + navigation_origin_y_;
      if (hits(x, y) || centerline_hits(x, y)) return true;
      if (index == 0) continue;
      const double x0 = navigation2d::X(route[index - 1]) + navigation_origin_x_;
      const double y0 = navigation2d::Y(route[index - 1]) + navigation_origin_y_;
      const double length = std::hypot(x - x0, y - y0);
      const int samples = std::max(1, static_cast<int>(std::ceil(length / std::max(.02, .5 * resolution))));
      for (int sample = 1; sample < samples; ++sample) {
        const double ratio = static_cast<double>(sample) / samples;
        const double wx = x0 + ratio * (x - x0);
        const double wy = y0 + ratio * (y - y0);
        if (hits(wx, wy) || centerline_hits(wx, wy)) return true;
      }
    }
    return false;
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

  void PublishDebugPaths(const navigation2d::NavigationState& state) {
    if (!navigation_ || !pose_) return;
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
      geometry_msgs::msg::PoseStamped stamped;
      stamped.header = tour.header;
      stamped.pose = PoseFromXyYaw(active_goal_.x, active_goal_.y, active_goal_.yaw);
      tour.poses.push_back(stamped);
      prev_x = active_goal_.x;
      prev_y = active_goal_.y;
      frontiers.poses.push_back(
          PoseFromXyYaw(active_goal_.frontier_x, active_goal_.frontier_y, active_goal_.yaw));
      published_active = true;
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
        if (return_escape_attempts_ >= 3) {
          if (StartBreadcrumbTowardHome()) return;
          Finish(true, "return path unavailable; stopping to protect localization");
          return;
        }
        BeginReturn(partial_return_);
        return;
      }
    } else if (!navigation_) {
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
    // Goal already in the observation disk — finish it instead of sitting on
    // CARROT/GOAL with zero command (junction hesitation).
    if (phase_ == MissionPhase::kExploring && !returning_ && pose_ &&
        !retreat_goal_) {
      const double goal_dist = std::hypot(active_goal_.x - pose_->pose.position.x,
                                          active_goal_.y - pose_->pose.position.y);
      if (goal_dist < .28 && state.path_progress_m >= .05) {
        navigation_.reset();
        if (pending_entry_goal_ || explorer_.GoalRegionStillFrontier(active_goal_)) {
          if (!pending_entry_goal_) LockEntryMouth(active_goal_);
          if (EntryProbeSatisfied()) {
            explorer_.RecordAttempt(*pending_entry_goal_, true, false);
            explorer_.CommitRegion(pose_->pose.position.x, pose_->pose.position.y);
            ClearEntryMouthLock();
            committed_tour_.clear();
            next_selection_ = now();
            PublishStatus("SELECTING", "arrived at mouth; finishing active region");
            return;
          }
          if (entry_probe_attempts_ < 4 && StartEntryProbe()) return;
          // Still on the threshold with the room open — never mark success or
          // hop mouths; that left CARROT inside while the robot walked away.
          explorer_.RecordAttempt(*pending_entry_goal_, false);
          recently_demoted_ = *pending_entry_goal_;
          recently_demoted_until_ = now() + rclcpp::Duration::from_seconds(4.0);
          next_selection_ = now() + rclcpp::Duration::from_seconds(1.0);
          PublishStatus("SELECTING",
                        "at doorway; entry incomplete, keeping locked mouth");
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
    if (PathCutsMappedObstacle()) {
      RCLCPP_WARN(get_logger(),
                  "Dropping path that intersects mapped obstacles goal=(%.2f, %.2f)",
                  active_goal_.x, active_goal_.y);
      geometry_msgs::msg::Twist stop;
      command_publisher_->publish(stop);
      nav_msgs::msg::Path empty;
      empty.header.stamp = now();
      empty.header.frame_id = "map";
      path_publisher_->publish(empty);
      if (phase_ == MissionPhase::kReturning || returning_) {
        navigation_.reset();
        if (StartBreadcrumbTowardHome()) return;
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
    const double front_range = SectorMinRange(0., .40);
    const double left_range = SectorMinRange(+std::acos(-1.) / 2., .40);
    const double right_range = SectorMinRange(-std::acos(-1.) / 2., .40);
    const double side_range = std::min(left_range > 0. ? left_range : 10.,
                                       right_range > 0. ? right_range : 10.);
    // Parallel room wall (front open, one flank close) is normal driving, not a
    // U-tip. Side-only tightness must not trigger tip-crawl / pinch escapes.
    const bool forward_clearly_open =
        SectorMinRange(0., .50) >= .70 || front_range >= .70;
    const bool corridor_pinch =
        (front_range > 0. && front_range < .45) ||
        (side_range < .40 && !forward_clearly_open);
    // Include retreat_goal_: tip thrash used frontier_cells=0 retreat poses
    // that were exempt from this abort, so the robot spun forever in the U.
    const bool near_goal_alcove_crawl =
        phase_ == MissionPhase::kExploring && !returning_ &&
        (goal_distance_now < .45 ||
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
        if (local_frontier_open) {
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
      retreat_attempted_ = false;  // allow a long trail exit from this cell
      committed_tour_.clear();
      cooled_signatures_.clear();
      if (StartRetreat()) {
        PublishStatus("NAVIGATING", "leaving alcove tip along driven trail");
        return;
      }
      if (TryStartEscape()) return;
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
        if (locked_entry_mouth_ && entry_probe_attempts_ < 4) {
          pending_entry_goal_ = locked_entry_mouth_;
          if (StartEntryProbe()) return;
        }
        // Keep the committed tour; retreat is not a license to rebuild reverse.
        next_selection_ = now();
        PublishStatus("SELECTING", "left the pinched corridor");
      } else if (pending_entry_goal_) {
        if (EntryProbeSatisfied()) {
          explorer_.RecordAttempt(*pending_entry_goal_, true, false);
          // Only a real doorway crossing may push the entered room onto the DFS
          // stack. CommitRegion(living-room pose) used to pop kitchen/bedroom.
          explorer_.CommitRegion(pose_->pose.position.x, pose_->pose.position.y);
          ClearEntryMouthLock();
          committed_tour_.clear();
          next_selection_ = now();
          PublishStatus("SELECTING", "room entered; finishing active region first");
        } else if (entry_probe_attempts_ < 4 && StartEntryProbe()) {
          return;
        } else {
          RCLCPP_WARN(get_logger(),
                      "Frontier entry incomplete: frontier=(%.2f,%.2f) attempts=%d "
                      "depth=%.2f; keeping unresolved",
                      pending_entry_goal_->frontier_x, pending_entry_goal_->frontier_y,
                      entry_probe_attempts_, EntryProbeDepth());
          explorer_.RecordAttempt(*pending_entry_goal_, false);
          ClearEntryMouthLock();
          next_selection_ = now();
          PublishStatus("SELECTING", "frontier entry blocked; unresolved boundary retained");
        }
      } else if (explorer_.GoalRegionStillFrontier(active_goal_)) {
        // Hallway arrival with the mouth still open — cross into the room.
        // Keep the tour; clearing it forced a reverse rebuild every doorway.
        LockEntryMouth(active_goal_);
        if (StartEntryProbe()) return;
        RCLCPP_WARN(get_logger(),
                    "Frontier reached but no safe entry probe: frontier=(%.2f,%.2f)",
                    pending_entry_goal_->frontier_x, pending_entry_goal_->frontier_y);
        explorer_.RecordAttempt(*pending_entry_goal_, false);
        ClearEntryMouthLock();
        if (!committed_tour_.empty()) committed_tour_.pop_front();
        next_selection_ = now();
        PublishStatus("SELECTING", "frontier entry unavailable; unresolved boundary retained");
      } else {
        explorer_.RecordAttempt(active_goal_, true);
        // Goal dispatch already CommitRegion'd; do not re-Enter the robot cell
        // (often still the hallway parent) or the room stack collapses.
        if (!committed_tour_.empty()) committed_tour_.pop_front();
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
                SectorMinRange(0., .45) > 0. && SectorMinRange(0., .45) < .30 &&
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
                (SectorMinRange(0., .50) < .45 ||
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
                SectorMinRange(0., .50) < .28 &&
                last_goal_path_progress_time_.nanoseconds() > 0 &&
                (now() - last_goal_path_progress_time_).seconds() > 2.0)) {
      const bool monitor_stop =
          state.status == navigation2d::NavigationStatus::kBlocked &&
          state.collision_monitor_action ==
              navigation2d::CollisionMonitorAction::kStop;
      const double forward_clearance = SectorMinRange(0., .50);
      const bool forward_open = forward_clearance >= .45;
      const bool forward_blocked_close =
          forward_clearance > 0. && forward_clearance < .28;
      const bool exploring_drive =
          phase_ == MissionPhase::kExploring && !returning_;

      // SideSpeckStall: moderately open forward with a lateral speck STOP.
      // When forward is clearly open (>=0.70 m), a close flank is a normal
      // room wall — peel/replan here parked the robot mid-room forever
      // (front=1.0 left=0.18 escape loop).
      if (exploring_drive && monitor_stop && forward_clearly_open) {
        // Locked doorway: jamb STOP with open forward is the entry crawl, not
        // a mid-room wall. Retreating here left CARROT inside the room.
        if (locked_entry_mouth_ || pending_entry_goal_) {
          if (!pending_entry_goal_) pending_entry_goal_ = locked_entry_mouth_;
          if (last_goal_path_progress_time_.nanoseconds() > 0 &&
              (now() - last_goal_path_progress_time_).seconds() > 4.0) {
            navigation_.reset();
            cooled_signatures_.clear();
            paint_driven_corridor_ = true;
            paint_trail_budget_m_ = 8.0;
            if (entry_probe_attempts_ < 4 && StartEntryProbe()) {
              PublishStatus("COMPLETING",
                            "doorjamb STOP; retrying locked mouth entry");
              return;
            }
            if (TryStartEscape()) {
              recovery_.resume_after_escape = true;
              PublishStatus("RECOVERING",
                            "doorjamb STOP; peel then resume locked mouth");
              return;
            }
          }
          return;
        }
        // Keep the active route; only trail-retreat if progress is dead.
        if (last_goal_path_progress_time_.nanoseconds() > 0 &&
            (now() - last_goal_path_progress_time_).seconds() > 6.0) {
          retreat_attempted_ = false;
          navigation_.reset();
          if (!committed_tour_.empty()) committed_tour_.pop_front();
          if (StartRetreat()) {
            PublishStatus("NAVIGATING",
                          "open-forward side-wall STOP; leaving along trail");
            return;
          }
        }
        return;
      }
      if (exploring_drive && monitor_stop && forward_open && side_range < .35) {
        if (locked_entry_mouth_ || pending_entry_goal_) {
          if (!pending_entry_goal_) pending_entry_goal_ = locked_entry_mouth_;
          navigation_.reset();
          cooled_signatures_.clear();
          paint_driven_corridor_ = true;
          paint_trail_budget_m_ = 8.0;
          if (entry_probe_attempts_ < 4 && StartEntryProbe()) {
            PublishStatus("COMPLETING",
                          "side clearance at mouth; deeper entry probe");
            return;
          }
          if (TryStartEscape()) {
            recovery_.resume_after_escape = true;
            PublishStatus("RECOVERING",
                          "side clearance at mouth; peel then resume entry");
            return;
          }
          return;
        }
        if (recovery_.side_peels < 2 && TryStartEscape()) {
          ++recovery_.side_peels;
          navigation_.reset();
          recovery_.resume_after_escape = true;
          PublishStatus("RECOVERING",
                        "open-space peel after side clearance stop");
          return;
        }
        if (recovery_.open_replans < 1) {
          ++recovery_.open_replans;
          if (ReplanActiveGoalOnLatestMap(
                  "side speck stall; open path replan"))
            return;
        }
        // Leave the cell before demoting — demote alone re-queued goals from
        // the same pose and restarted the speck loop.
        retreat_attempted_ = false;
        navigation_.reset();
        if (StartRetreat()) {
          PublishStatus("NAVIGATING",
                        "side speck exhausted; leaving along driven trail");
          return;
        }
        DemoteActiveGoal("side speck stall; demoting viewpoint");
        return;
      }

      // ForwardBlocked: do not resume the same path signature after peel.
      if (exploring_drive && monitor_stop && forward_blocked_close) {
        // Locked mouth: retreat and retry instead of abandoning the kitchen.
        if (locked_entry_mouth_ || pending_entry_goal_) {
          if (!pending_entry_goal_) pending_entry_goal_ = locked_entry_mouth_;
          retreat_attempted_ = false;
          navigation_.reset();
          if (StartRetreat()) {
            PublishStatus("NAVIGATING",
                          "forward blocked at locked mouth; trail retreat");
            return;
          }
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
          // Locked entry: do not demote the kitchen mouth on one stall.
          if (locked_entry_mouth_ || pending_entry_goal_) {
            if (!pending_entry_goal_) pending_entry_goal_ = locked_entry_mouth_;
            retreat_attempted_ = false;
            navigation_.reset();
            if (StartRetreat()) {
              PublishStatus("NAVIGATING",
                            "entry stall; trail retreat then retry locked mouth");
              return;
            }
            if (entry_probe_attempts_ < 4 && StartEntryProbe()) return;
          }
          // One failed execution cools that mouth via the explorer policy —
          // do not soft-cool and immediately reselect the same doorway.
          explorer_.RecordAttempt(
              pending_entry_goal_.value_or(active_goal_), false);
          recently_demoted_ = pending_entry_goal_.value_or(active_goal_);
          recently_demoted_until_ =
              now() + rclcpp::Duration::from_seconds(8.0);
          ClearEntryMouthLock();
          recovery_.Clear();
        }
        // Open-forward + side wall: micro-peels spin in place. Trail retreat
        // is the only recovery that actually leaves the room cell.
        if (phase_ != MissionPhase::kReturning && !returning_ &&
            forward_clearly_open && side_range < .40) {
          retreat_attempted_ = false;
          navigation_.reset();
          if (!committed_tour_.empty()) committed_tour_.pop_front();
          if (StartRetreat()) {
            PublishStatus("NAVIGATING",
                          "room-side wall stall; leaving along driven trail");
            return;
          }
        }
        // Returning already owns the home path.  A short clearance jiggle
        // here only changes the start cell, so the next plan looks new and
        // the same first-segment stall repeats until the mission times out.
        if (phase_ != MissionPhase::kReturning && !returning_ && TryStartEscape()) {
          navigation_.reset();
          if (force_return) {
            committed_tour_.clear();
            escape_then_return_ = true;
          } else if (!committed_tour_.empty()) {
            committed_tour_.pop_front();
          }
          return;
        }
        if (force_return && !returning_) {
          navigation_.reset();
          committed_tour_.clear();
          BeginReturn(true);
          return;
        }
      }
      if (!mid_path_pinch && ReplanActiveGoalOnLatestMap(
              state.status == navigation2d::NavigationStatus::kBlocked ?
                  "controller could not execute validated route" : "goal execution timed out"))
        return;
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
