#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
  config.minimum_frontier_cells = 12;
  // Keep frontier generation consistent with the navigation costmap: a goal
  // must leave room for the physical footprint and its obstacle buffer, not
  // merely be free in the raw occupancy image.
  // 0.50 m rejected every boundary viewpoint at valid corner spawns.  The
  // physical radius is 0.28 m; retain a 0.12 m planning margin here and let
  // Navigation2D's footprint, global path check, and live laser perform the
  // authoritative second-stage feasibility test.
  config.footprint_clearance = .40;
  config.minimum_standoff = .55;
  config.maximum_standoff = 1.10;
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

class AutonomousExplorer final : public rclcpp::Node {
 public:
  AutonomousExplorer() : Node("sweepnav_autonomous_explorer"), explorer_(ExplorerConfig()) {
    result_path_ = std::getenv("SWEEPNAV_EXPLORATION_RESULT") != nullptr
        ? std::getenv("SWEEPNAV_EXPLORATION_RESULT") : "/tmp/exploration-result.json";
    snapshot_path_ = (std::filesystem::path(result_path_).parent_path() /
                      "exploration_snapshot.json").string();
    max_duration_s_ = RequiredPositiveEnvironment("SWEEPNAV_EXPLORATION_TIMEOUT", 600.);
    simulation_speed_ = RequiredPositiveEnvironment("SWEEPNAV_SIMULATION_SPEED", 1.);
    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_publisher_ = create_publisher<std_msgs::msg::String>(
        "/exploration/status", rclcpp::QoS(1).transient_local().reliable());
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
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr value) {
          velocity_ = {value->twist.twist.linear.x, value->twist.twist.angular.z};
        });
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr value) {
          scan_ = *value;
          if (navigation_ && pose_) FeedScan();
        });
    timer_ = create_wall_timer(
        std::chrono::duration<double>(.06 / simulation_speed_), [this]() { Tick(); });
    PublishStatus("WAITING", "waiting for ground-truth pose and online map");
  }

 private:
  std::size_t KnownCells() const {
    return explorer_.KnownCells();
  }

  navigation2d::Pose2d LocalPose(const geometry_msgs::msg::Pose& pose) const {
    return navigation2d::MakePose2d(pose.position.x - navigation_origin_x_,
                                    pose.position.y - navigation_origin_y_, Yaw(pose.orientation));
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
    const double snapshot_clearance = .34;
    const int robot_clearance = static_cast<int>(
        std::ceil(snapshot_clearance / map_->info.resolution));
    for (std::size_t index = 0; index < map_->data.size(); ++index) {
      if (index) output << ',';
      // Navigation2D must never plan through unknown space. Frontier targets
      // are selected in observed free space with a full footprint margin.
      const int row = static_cast<int>(index / map_->info.width);
      const int col = static_cast<int>(index % map_->info.width);
      const bool under_robot = (col - robot_col) * (col - robot_col) +
          (row - robot_row) * (row - robot_row) <= robot_clearance * robot_clearance;
      // The robot's current footprint is physically known to be free. SLAM's
      // confidence threshold can otherwise leave isolated unknown cells below
      // the base and make every first plan fail its start-collision check.
      output << ((map_->data[index] == 0 || under_robot) ? 0 : 255);
    }
    output << "]}";
    output.close();
    PublishPlanningDebugMap(returning);
    navigation2d::NavigationConfig config;
    config.map_resolution = map_->info.resolution;
    // Theta* removes the axis-aligned staircase from an A* grid path before
    // it reaches the controller, so the robot can follow long smooth chords.
    config.planner = "theta_star";
    // Regulated Pure Pursuit is the Nav2 controller used for robust service
    // robot path following. Its velocity regulation around curvature and
    // obstacles avoids MPPI fallback oscillation on partially observed maps.
    config.controller = "rpp";
    // Exploration tracking is forward-only. Recovery invalidates the route
    // and replans from the measured pose; it never replays a breadcrumb path
    // or issues a scripted reverse manoeuvre.
    config.max_reverse_velocity = 0.;
    config.robot_radius = .28;
    config.inflation_radius = .32;
    // The global plan already enforces the footprint buffer.  A long local
    // collision projection into sparse unknown cells caused false stops in
    // visibly clear aisles; retain near-field collision checking only.
    config.collision_horizon = .35;
    // Hard footprint collision remains robot_radius=.28. Requiring .36 m in
    // the optional smoother rejected otherwise valid narrow-door paths and
    // contradicted the exploration viewpoint clearance.
    config.smoother_min_clearance = .28;
    // Frontier goals are observation viewpoints, but home is a docking pose.
    // Keep their completion contracts separate: return must converge to the
    // recorded ground-truth start instead of stopping at the edge of the
    // ordinary navigation tolerance disk.
    config.goal_xy_tolerance = returning ? .03 : .18;
    if (returning) config.min_approach_velocity = .025;
    // LD14 observes 360 degrees, so frontier visits need no stop-and-turn
    // terminal orientation. Preserve precise heading only when docking home.
    // Backtrack edges are positional checkpoints; requiring a docking yaw at
    // every 0.35 m checkpoint creates needless stop-turn-stop motion.
    config.goal_yaw_tolerance = returning ? .08 : 3.13;
    try {
      navigation_ = std::make_unique<navigation2d::NavigationSystem>(config, map_snapshot_path_);
      reported_recoveries_ = 0;
      navigation_->SetGoal(navigation2d::MakePose2d(
          goal.x - navigation_origin_x_, goal.y - navigation_origin_y_, goal.yaw));
      active_goal_ = goal;
      returning_ = returning;
      goal_started_ = now();
      if (scan_) FeedScan();
      const auto initial_state = navigation_->ComputeCommand(
          LocalPose(pose_->pose), velocity_, now().seconds());
      const double goal_distance = std::hypot(
          goal.x - pose_->pose.position.x, goal.y - pose_->pose.position.y);
      if (goal_distance > config.goal_xy_tolerance &&
          initial_state.global_path_length_m <= 0.) {
        RCLCPP_WARN(get_logger(), "Navigation2D rejected unreachable frontier before execution");
        navigation_.reset();
        return false;
      }
      // Do not classify a frontier from the controller's first sample. At a
      // continuous goal handoff the measured velocity still belongs to the
      // previous route, so acceleration and collision constraints may
      // correctly emit one zero command before tracking starts. Executability
      // is a temporal property owned by NavigationSystem's progress and
      // bounded-rotation state machines, not a one-frame threshold.
      // RPP deliberately slows in curves and near occupied space. A fixed
      // 55 s watchdog therefore turns otherwise healthy long paths into fake
      // "blocked" failures. Budget travel from the planned path at a
      // conservative 0.10 m/s, plus time for initial alignment and final
      // approach; NavigationSystem still owns genuine progress recovery.
      goal_timeout_s_ = std::max(55., initial_state.global_path_length_m / .10 + 20.);
      PublishStatus(returning ? "RETURNING" : "NAVIGATING",
                    returning ? "returning to exploration start" : "navigating to frontier");
      RCLCPP_INFO(get_logger(),
                  "Navigation2D goal: pose=(%.2f, %.2f) goal=(%.2f, %.2f) frontier_cells=%d",
                  pose_->pose.position.x, pose_->pose.position.y, goal.x, goal.y,
                  goal.frontier_cells);
      return true;
    } catch (const std::exception& error) {
      RCLCPP_WARN(get_logger(), "Navigation2D rejected goal: %s", error.what());
      navigation_.reset();
      return false;
    }
  }

  void PublishPlanningDebugMap(bool returning) {
    if (!map_ || !pose_) return;
    nav_msgs::msg::OccupancyGrid input = *map_;
    input.header.stamp = now();
    input.header.frame_id = "map";
    const int robot_col = static_cast<int>(std::floor(
        (pose_->pose.position.x - navigation_origin_x_) / map_->info.resolution));
    const int robot_row = static_cast<int>(std::floor(
        (pose_->pose.position.y - navigation_origin_y_) / map_->info.resolution));
    const double evidence_clearance = .34;
    const int evidence_cells = static_cast<int>(std::ceil(evidence_clearance / map_->info.resolution));
    input.data.assign(map_->data.size(), 100);
    for (std::size_t index = 0; index < map_->data.size(); ++index) {
      const int row = static_cast<int>(index / map_->info.width);
      const int col = static_cast<int>(index % map_->info.width);
      const bool under_robot = (col - robot_col) * (col - robot_col) +
          (row - robot_row) * (row - robot_row) <= evidence_cells * evidence_cells;
      if (map_->data[index] == 0 || under_robot) input.data[index] = 0;
    }
    planning_input_publisher_->publish(input);

    nav_msgs::msg::OccupancyGrid inflated = input;
    const int inflation_cells = static_cast<int>(std::ceil(.32 / map_->info.resolution));
    for (int row = 0; row < static_cast<int>(map_->info.height); ++row) {
      for (int col = 0; col < static_cast<int>(map_->info.width); ++col) {
        if (input.data[static_cast<std::size_t>(row) * map_->info.width + col] == 0) continue;
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            const int x = col + dx, y = row + dy;
            if (x < 0 || y < 0 || x >= static_cast<int>(map_->info.width) ||
                y >= static_cast<int>(map_->info.height) ||
                std::hypot(dx, dy) * map_->info.resolution > .32)
              continue;
            inflated.data[static_cast<std::size_t>(y) * map_->info.width + x] = 100;
          }
        }
      }
    }
    inflated_costmap_publisher_->publish(inflated);
  }

  void BuildTour() {
    committed_tour_.clear();
    const auto goals = explorer_.BuildTour(pose_->pose.position.x, pose_->pose.position.y);
    for (const auto& goal : goals) committed_tour_.push_back(goal);
    ++tour_revision_;
    RCLCPP_INFO(get_logger(), "Frontier graph tour %zu contains %zu reachable viewpoints",
                tour_revision_, committed_tour_.size());
  }

  void SelectNextGoal() {
    // Keep a topology-level tour commitment. Re-ranking every scan makes a
    // robot chase freshly split fragments of the same local frontier and is
    // the root cause of the visible knot/zig-zag trajectory. A queued goal is
    // still discarded immediately if its exact frontier has resolved; only an
    // exhausted or invalidated tour is rebuilt from the latest online map.
    if (committed_tour_.empty()) BuildTour();
    // A frontier extractor can emit many viewpoints around one wall.  Do not
    // synchronously exhaust that list when the local controller rejects a
    // viewpoint: that turns one transient costmap disagreement into hundreds
    // of failures before the mapper has published its next observation.  This
    // is the same bounded retry principle used by Nav2 recovery trees: try a
    // small, diverse budget, then yield to new sensor/map evidence.
    constexpr int kCandidatePlanBudget = 3;
    int attempted = 0;
    while (!committed_tour_.empty() && attempted < kCandidatePlanBudget) {
      const Goal goal = committed_tour_.front();
      if (!explorer_.GoalRegionStillFrontier(goal)) {
        RCLCPP_INFO(get_logger(), "Skipping resolved tour viewpoint (%.2f, %.2f)", goal.x, goal.y);
        committed_tour_.pop_front();
        continue;
      }
      if (StartNavigation(goal, false)) {
        empty_frontier_cycles_ = 0;
        no_executable_frontier_cycles_ = 0;
        return;
      }
      explorer_.RecordAttempt(goal, false);
      committed_tour_.pop_front();
      ++attempted;
    }
    geometry_msgs::msg::Twist stop;
    command_publisher_->publish(stop);
    if (!committed_tour_.empty() || attempted > 0) {
      ++no_executable_frontier_cycles_;
      PublishStatus("SELECTING", "frontiers locally blocked; waiting for map update");
      // Three independent, freshly-ranked candidate batches without one
      // executable plan means continuing to churn cannot improve coverage.
      // Preserve the actual map and return over the observed corridor instead
      // of clearing blacklist state and blindly retrying the same walls.
      if (no_executable_frontier_cycles_ >= 3) BeginReturn(true);
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
    if (returning_ || !navigation_ || committed_tour_.size() < 2) return false;
    const double distance = std::hypot(active_goal_.x - pose_->pose.position.x,
                                       active_goal_.y - pose_->pose.position.y);
    // A frontier viewpoint is a 360-degree observation pose, not a docking
    // point. Once inside this small observation envelope, the lidar has the
    // intended view and the next already-ranked task can be adopted without
    // braking to zero at every frontier boundary.
    if (distance > .24) return false;
    const Goal completed = active_goal_;
    std::size_t next_index = 1;
    while (next_index < committed_tour_.size()) {
      const Goal next = committed_tour_[next_index];
      if (!explorer_.GoalRegionStillFrontier(next)) {
        ++next_index;
        continue;
      }
      const double next_bearing = std::atan2(
          next.y - pose_->pose.position.y, next.x - pose_->pose.position.x);
      const double direction_change = std::abs(NormalizeAngle(
          next_bearing - Yaw(pose_->pose.orientation)));
      // Continuous handoff is valid only when the next task continues the
      // current motion direction. Switching a moving differential drive to
      // a goal behind it creates an artificial emergency stop and lets
      // inertia carry the base outside the newly planned corridor.
      if (direction_change > .60) return false;
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
      explorer_.RecordAttempt(next, false);
      ++next_index;
    }
    return false;
  }

  void BeginReturn(bool force_return = false) {
    if (!start_pose_) return;
    if (force_return)
      RCLCPP_WARN(get_logger(),
                  "No executable frontier after bounded retries; returning safely with current map");
    else if (!explorer_.CompletionEligible(initial_known_cells_))
      RCLCPP_WARN(get_logger(),
                  "No frontier survived a stable completion window; returning with reachable map coverage");
    // Return is a fresh global query on the final online map. Unknown remains
    // lethal and historical robot positions are never replayed as a route.
    const Goal home{start_pose_->pose.position.x, start_pose_->pose.position.y,
                    Yaw(start_pose_->pose.orientation)};
    if (StartNavigation(home, true)) {
      RCLCPP_INFO(get_logger(), "Final online-map return path accepted; tracking with RPP");
      PublishStatus("RETURNING", "tracking shortest route home on final observed map");
      return;
    }
    Finish(false, "final online-map return path is unavailable");
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

  void Finish(bool success, const std::string& message) {
    if (finished_) return;
    finished_ = true;
    geometry_msgs::msg::Twist stop;
    command_publisher_->publish(stop);
    std::string final_message = message;
    double return_error = std::numeric_limits<double>::infinity();
    if (pose_ && start_pose_)
      return_error = std::hypot(pose_->pose.position.x - start_pose_->pose.position.x,
                                pose_->pose.position.y - start_pose_->pose.position.y);
    if (success && return_error > .35) {
      success = false;
      final_message = "return ended outside the home tolerance";
    }
    std::filesystem::create_directories(std::filesystem::path(result_path_).parent_path());
    if (map_) {
      // Persist the exact online occupancy grid used by exploration, together
      // with the true-pose trajectory.  The host-side renderer turns this
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
    output << "{\n  \"status\": \"" << (success ? "COMPLETE" : "FAILED") << "\",\n"
           << "  \"message\": \"" << final_message << "\",\n"
           << "  \"pose_source\": \"ground_truth\",\n"
           << "  \"map_source\": \"ground_truth_scan_insertion\",\n"
           << "  \"planning\": \"navigation2d/theta_star+rpp\",\n"
           << "  \"last_controller_backend\": \"" << last_backend_ << "\",\n"
           << "  \"mppi_commands\": " << mppi_commands_ << ",\n"
           << "  \"rpp_commands\": " << rpp_commands_ << ",\n"
           << "  \"travel_distance_m\": " << total_distance_m_ << ",\n"
           << "  \"known_cells\": " << KnownCells() << ",\n"
           << "  \"completed_goals\": " << explorer_.completed_goals() << ",\n"
           << "  \"failed_goals\": " << explorer_.failed_goals() << ",\n"
           << "  \"return_error_m\": " << return_error << "\n}\n";
    PublishStatus(success ? "COMPLETE" : "FAILED", final_message);
  }

  void PublishStatus(const std::string& state, const std::string& message) {
    std_msgs::msg::String status;
    status.data = "{\"state\":\"" + state + "\",\"message\":\"" + message +
        "\",\"known_cells\":" + std::to_string(KnownCells()) +
        ",\"completed_goals\":" + std::to_string(explorer_.completed_goals()) +
        ",\"failed_goals\":" + std::to_string(explorer_.failed_goals()) + "}";
    status_publisher_->publish(status);
  }

  void Tick() {
    if (finished_) return;
    if (!map_ || !pose_ || !scan_ || KnownCells() < 100) return;
    if (!start_pose_) {
      start_pose_ = pose_;
      // Simulation can advance rapidly while the truth mapper receives its
      // first scans. Exploration time is a planning budget, so start it only
      // once pose, scan and a usable map are simultaneously available.
      started_ = now();
      mission_started_ = true;
      initial_known_cells_ = KnownCells();
      bootstrap_until_ = now() + rclcpp::Duration::from_seconds(8.0);
      next_selection_ = bootstrap_until_;
      PublishStatus("WAITING", "ground-truth pose ready; waiting for a stable online map");
    }
    if (mission_started_ && (now() - started_).seconds() > max_duration_s_) {
      Finish(false, "exploration timed out");
      return;
    }
    if (now() < bootstrap_until_) {
      geometry_msgs::msg::Twist stop;
      command_publisher_->publish(stop);
      return;
    }
    if (!navigation_) {
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
    const auto state = navigation_->ComputeCommand(local_pose, velocity_, now().seconds());
    last_backend_ = BackendName(state.controller_diagnostics.backend);
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
    if (state.recoveries > reported_recoveries_) {
      reported_recoveries_ = state.recoveries;
      RCLCPP_WARN(get_logger(),
                  "Route progress recovery: arc=%.2f/%.2f controller_motion=%s safety_stop=%s rpp_stage=%d velocity=(%.3f,%.3f)",
                  state.path_progress_m, state.global_path_length_m,
                  state.controller_commanded_motion ? "yes" : "no",
                  state.safety_stopped_motion ? "yes" : "no",
                  state.controller_diagnostics.fallback_level,
                  velocity_.linear, velocity_.angular);
    }
    if (state.status == navigation2d::NavigationStatus::kSucceeded) {
      navigation_.reset();
      if (returning_) {
        Finish(true, "frontiers exhausted and robot returned through final-map global path");
      } else {
        explorer_.RecordAttempt(active_goal_, true);
        if (!committed_tour_.empty()) committed_tour_.pop_front();
        next_selection_ = now();
        PublishStatus("SELECTING", "frontier reached; continuing committed tour");
      }
    } else if (state.status == navigation2d::NavigationStatus::kBlocked ||
               (now() - goal_started_).seconds() > goal_timeout_s_) {
      RCLCPP_WARN(get_logger(),
                  "Navigation2D frontier failed: status=%s elapsed=%.1f replans=%d path=%.2f",
                  state.status == navigation2d::NavigationStatus::kBlocked ? "blocked" : "timeout",
                  (now() - goal_started_).seconds(), state.replans,
                  state.global_path_length_m);
      if (ReplanActiveGoalOnLatestMap(
              state.status == navigation2d::NavigationStatus::kBlocked ?
                  "controller could not execute validated route" : "goal execution timed out"))
        return;
      navigation_.reset();
      if (returning_) {
        Finish(false, "final online-map return path execution failed");
      } else {
        explorer_.RecordAttempt(active_goal_, false);
        if (!committed_tour_.empty()) committed_tour_.pop_front();
        next_selection_ = now();
        PublishStatus("SELECTING", "frontier failed; continuing committed tour");
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
  int latest_map_replans_ = 0;
  int reported_recoveries_ = 0;
  std::uint64_t acados_commands_ = 0;
  std::uint64_t mppi_commands_ = 0, rpp_commands_ = 0;
  double total_distance_m_ = 0.;
  double truth_origin_x_ = 0., truth_origin_y_ = 0., truth_origin_yaw_ = 0.;
  std::string last_backend_ = "not-run";
  bool returning_ = false, finished_ = false, mission_started_ = false;
  bool truth_origin_ = false;
  Goal active_goal_;
  std::deque<Goal> committed_tour_;
  rclcpp::Time started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time goal_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_selection_{0, 0, RCL_ROS_TIME};
  rclcpp::Time bootstrap_until_{0, 0, RCL_ROS_TIME};
  std::optional<nav_msgs::msg::OccupancyGrid> map_;
  std::optional<geometry_msgs::msg::PoseStamped> pose_, start_pose_;
  std::optional<sensor_msgs::msg::LaserScan> scan_;
  std::vector<std::pair<double, double>> trajectory_;
  navigation2d::Twist2d velocity_;
  navigation2d::FrontierExplorer explorer_;
  std::unique_ptr<navigation2d::NavigationSystem> navigation_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr planning_input_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_costmap_publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_subscription_;
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
