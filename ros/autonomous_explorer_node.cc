#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "navigation2d/ros/exploration/motion_output.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigation2d/application/exploration/navigation_session.h"
#include "navigation2d/application/exploration/debug_overlay.h"
#include "navigation2d/ros/exploration/debug_paths.h"
#include "navigation2d/ros/exploration/pose_feedback.h"
#include "navigation2d/application/exploration/pose_observation.h"
#include "navigation2d/ros/exploration/explorer_parameters.h"
#include "navigation2d/ros/exploration/map_observation.h"
#include "navigation2d/ros/exploration/artifact_writer.h"
#include "navigation2d/ros/exploration/planning_map_adapter.h"
#include "navigation2d/application/exploration/return_coordinator.h"
#include "navigation2d/application/exploration/recovery_policy.h"
#include "navigation2d/application/exploration/escape_execution.h"
#include "navigation2d/application/exploration/pose_motion_watch.h"
#include "navigation2d/application/exploration/dispatch_policy.h"
#include "navigation2d/application/exploration/mission_lifecycle.h"
#include "navigation2d/application/exploration/recovery_session.h"
#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/entry_probe_executor.h"
#include "navigation2d/application/exploration/entry_selection.h"
#include "navigation2d/application/exploration/entry_crawl_executor.h"
#include "navigation2d/application/exploration/execution_command.h"
#include "navigation2d/application/exploration/viewpoint_arrival.h"
#include "navigation2d/application/exploration/retreat_execution.h"
#include "navigation2d/application/exploration/goal_execution_watch.h"
#include "navigation2d/application/exploration/open_forward_stop.h"
#include "navigation2d/application/exploration/route_failure_history.h"
#include "navigation2d/application/exploration/selection_exhaustion.h"
#include "navigation2d/application/exploration/viewpoint_rejection_executor.h"
#include "navigation2d/application/exploration/viewpoint_rejections.h"
#include "navigation2d/application/exploration/entry_completion.h"
#include "navigation2d/application/exploration/tour_execution.h"
#include "navigation2d/ros/exploration/entry_reporter.h"
#include "navigation2d/application/exploration/status_serializer.h"
#include "navigation2d/application/exploration/room_visit_ledger.h"
#include "navigation2d/application/exploration/navigation_telemetry.h"
#include "navigation2d/application/exploration/route_validator.h"
#include "navigation2d/application/exploration/planning_snapshot.h"
#include "navigation2d/application/exploration/planning_evidence_budget.h"
#include "navigation2d/application/algorithm_registry.h"
#include "navigation2d/application/exploration/navigation_profile.h"
#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/application/exploration/completion_execution.h"
#include "navigation2d/ros/exploration/completion_reporter.h"
#include "navigation2d/geometry/robot_geometry.h"
#include "navigation2d/configuration/vehicle_config.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

namespace {

constexpr double kExploreGoalTolerance = .18;

double NormalizeAngle(double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

using Goal = navigation2d::ExplorationGoal;

class AutonomousExplorer final : public rclcpp::Node,
                                 private navigation2d::exploration::ReturnActions,
                                 private navigation2d::exploration::EntryActions,
                                 private navigation2d::exploration::EntrySelectionActions,
                                 private navigation2d::exploration::EntryCrawlActions,
                                 private navigation2d::exploration::ExecutionCommandEvidence,
                                 private navigation2d::exploration::ViewpointArrivalActions,
                                 private navigation2d::exploration::RetreatActions,
                                 private navigation2d::exploration::GoalExecutionClock,
                                 private navigation2d::exploration::OpenForwardStopActions,
                                 private navigation2d::exploration::SelectionExhaustionActions,
                                 private navigation2d::exploration::ViewpointRejectionActions,
                                 private navigation2d::exploration::TourConstraints,
                                 private navigation2d::exploration::DispatchActions,
                                 private navigation2d::exploration::CompletionActions,
                                 private navigation2d::exploration_ros::EntryStatusSink {
 public:
  AutonomousExplorer()
      : Node("sweepnav_autonomous_explorer"),
        explorer_(navigation2d::exploration_ros::ReadExplorerConfig(*this)),
        parameters_(navigation2d::exploration_ros::ReadExplorerParameters(
            *this, *navigation2d::AlgorithmRegistry::Builtins())) {
    motion_output_ = navigation2d::exploration_ros::MakeRosMotionOutput(*this);
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
    if (parameters_.pose_source == "ground_truth") {
      // Planning-only evaluation deliberately bypasses Localization2D's pose.
      // The online mapper still supplies its live occupancy grid, while the
      // controller and explorer receive the simulator's exact pose expressed in
      // the same initial-odometry/map coordinate system used by that grid.
      truth_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
          "/ground_truth", rclcpp::SensorDataQoS(),
          [this](nav_msgs::msg::Odometry::ConstSharedPtr value) {
            pose_feedback_.ObserveGroundTruth(*value);
          });
    } else {
      // In the joint localization/navigation evaluation the map and every
      // planning/control decision use Localization2D's scan-matched map pose.
      // Do not subscribe to /ground_truth here: it remains an offline oracle.
      localization_pose_subscription_ =
          create_subscription<geometry_msgs::msg::PoseStamped>(
              "/localization2d/pose", rclcpp::SensorDataQoS(),
              [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr value) {
                pose_feedback_.ObserveLocalization(*value);
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
          if (navigation_.has_session() && Pose()) FeedScan();
        });
    timer_ = create_wall_timer(
        std::chrono::duration<double>(.06 / parameters_.simulation_speed), [this]() { Tick(); });
    PublishStatus("WAITING", "waiting for " + parameters_.pose_source + " pose and online map");
  }

 private:
  const std::optional<navigation2d::exploration::PoseObservation>& Pose() const {
    return pose_feedback_.observation();
  }

  const navigation2d::ExplorationGrid& ObservationGrid() const {
    return explorer_.observation_map();
  }

  std::optional<navigation2d::exploration::ScanEvidenceView> ScanEvidence() const {
    if (!scan_) { return std::nullopt; }
    return navigation2d::exploration::ScanEvidenceView{
        scan_->angle_min, scan_->angle_increment, scan_->range_min, scan_->range_max,
        scan_->ranges};
  }

  std::size_t KnownCells() const {
    return explorer_.KnownCells();
  }

  navigation2d::Pose2d LocalPose(const navigation2d::exploration::PoseObservation& pose) const {
    return navigation2d::MakePose2d(pose.x - planning_map_.origin_x(),
                                    pose.y - planning_map_.origin_y(), pose.yaw);
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
    auto robot = LocalPose(*Pose());
    const double yaw = navigation2d::Yaw(robot);
    const double lidar_forward = navigation2d::kLidarForwardOffset();
    const auto sensor = navigation2d::MakePose2d(
        navigation2d::X(robot) + lidar_forward * std::cos(yaw),
        navigation2d::Y(robot) + lidar_forward * std::sin(yaw), yaw);
    // The online mapper has already fused this scan into the static map.
    // Do not let instantaneous endpoint noise disconnect its global route;
    // retain the scan for near-field collision monitoring during execution.
    navigation_.UpdateCollisionScan(sensor, ConvertScan());
  }

  void RejectPreparation(navigation2d::exploration::DispatchFailure failure) override {
    last_planning_failure_ = std::move(failure.reason);
    start_occupied_ = failure.start_occupied;
    using navigation2d::exploration::DispatchRejection;
    if (failure.kind == DispatchRejection::kOpenScrape ||
        failure.kind == DispatchRejection::kPinchScrape) {
      RCLCPP_WARN(get_logger(), "Navigation2D rejected goal before execution: %s (margin=%.3f)",
                  last_planning_failure_.c_str(), failure.margin_m);
    } else {
      RCLCPP_WARN(get_logger(), "Navigation2D rejected goal before execution: %s",
                  last_planning_failure_.c_str());
    }
    navigation_.Cancel();
  }
  bool PreparedRouteCutsWall() const override { return PathCutsMappedObstacle(); }
  void CoolRouteSignature(std::uint64_t signature) override { route_failures_.Cool(signature); }
  void ObservePreparedRoute(std::uint64_t signature, double margin_m) override {
    mapped_obstacle_rejects_ = 0;
    last_path_signature_ = signature;
    active_path_min_clearance_m_ = margin_m;
  }
  bool RetryNavigation(const Goal& goal,
                       navigation2d::exploration::DispatchRetryRequest request) override {
    return StartNavigation(goal, request.returning, request.latest_map);
  }
  void ReportDispatchRetry(navigation2d::exploration::DispatchRetryNotice notice) override {
    using Kind = navigation2d::exploration::DispatchRetryNotice::Kind;
    if (notice.kind == Kind::kNoRoutePinch) {
      RCLCPP_INFO(get_logger(), "%s no route (%s); retrying pinch clearance",
                  notice.entry ? "Entry" : "Open", last_planning_failure_.c_str());
    } else {
      // A nested open-scrape retry may replace the last observed margin.
      RCLCPP_INFO(get_logger(), "Open scrape on corridor mouth (margin=%.3f); pinch",
                  active_path_min_clearance_m_);
    }
  }

  bool StartNavigation(const Goal& requested_goal, bool returning,
                       bool latest_map_retry = false) {
    if (!latest_map_retry) {
      latest_map_replans_ = 0;
      cut_replans_ = 0;
    }
    start_occupied_ = false;
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
    const double trail_budget = evidence_budget_.Consume(
        map_->info.resolution, pose_feedback_.trail().size());
    const auto entry = entry_.pending() ? entry_.pending() : entry_.locked();
    // Preserve the loader's existing diagnosis for stream failures.
    static_cast<void>(planning_map_.Prepare(
        ObservationGrid(),
        {.robot = navigation2d::MakePose2d(Pose()->x, Pose()->y,
                                          Pose()->yaw),
         .goal = goal,
         .entry = entry,
         .scan = ScanEvidence(),
         .trail = pose_feedback_.trail(),
         .trail_budget_m = trail_budget,
         .returning = returning,
         .map_revision = map_revision_}));
    PublishPlanningDebugMap(returning);
    bool home_goal = false;
    if (returning && mission_.home()) {
      const Goal home = MakeHomeGoal();
      home_goal = std::hypot(goal.x - home.x, goal.y - home.y) <= map_->info.resolution;
    }
    const bool entry_drive = entry_.pending() || entry_.locked();
    using navigation2d::exploration::NavigationPurpose;
    const auto purpose = entry_drive ? NavigationPurpose::kEntry :
        (returning ? NavigationPurpose::kReturn : NavigationPurpose::kExplore);
    using navigation2d::exploration::DispatchPolicy;
    using RetryKind = DispatchPolicy::RetryKind;
    const bool pinch_retry = dispatch_.active(RetryKind::kPinch);
    const bool pinch_plan = pinch_retry && !entry_drive && !returning;
    const auto config = navigation2d::exploration::BuildNavigationProfile(
        {.resolution = map_->info.resolution,
         .entry = entry_drive,
         .pinch_retry = pinch_retry,
         .returning = returning,
         .home_goal = home_goal},
        parameters_.navigation_profiles.For(purpose));
    try {
      navigation_.Begin(config, planning_map_.path(), navigation2d::MakePose2d(
          goal.x - planning_map_.origin_x(), goal.y - planning_map_.origin_y(), goal.yaw),
          route_failures_.CooledSignatures());
      reported_recoveries_ = 0;
      // Do not commit active_goal_/returning_/recovery_ until the plan is
      // accepted. Early assign left ghost GOALs outside painted free and kept
      // recovery sessions alive after open-area plan rejects (stuck mid-room).
      if (scan_) FeedScan();
      const auto initial_time = now();
      const auto probe = navigation_.Probe(
          LocalPose(*Pose()), FreshVelocity(initial_time), initial_time.seconds());
      const auto& initial_state = probe.state;
      const auto verdict = dispatch_.Evaluate(
          goal, {.entry = entry_drive,
                 .returning = returning,
                 .retreat = retreat_.active(),
                 .pinch = pinch_plan},
          latest_map_retry, probe, *this);
      using navigation2d::exploration::DispatchVerdict;
      if (verdict != DispatchVerdict::kAcceptPrepared) {
        return verdict == DispatchVerdict::kAcceptedRetry;
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
      goal_execution_.SetTimeout(DispatchPolicy::GoalTimeout(returning, initial_state.global_path_length_m));
      navigation_.Accept();
      active_goal_ = goal;
      selected_algorithms_ = parameters_.navigation_profiles.For(purpose);
      returning_ = returning;
      // Resume / latest-map replan of the same viewpoint must keep peel and
      // open-replan counters; only a materially new goal opens a session.
      recovery_.Begin(goal.x, goal.y);
      const auto goal_started = now();
      const auto goal_progress = now();
      goal_execution_.Start(goal_started.nanoseconds(), goal_progress.nanoseconds());
      immobility_open_replans_ = 0;
      viewpoint_rejections_.ResetScrapes();
      PublishStatus(returning ? "RETURNING" :
                        (goal.leftover_approach ? "COMPLETING" : "NAVIGATING"),
                    returning ? "returning to exploration start" :
                        (retreat_.active() ? "leaving a pinched corridor along the driven trail" :
                        (goal.leftover_approach
                             ? "one leftover approach after exploring tour emptied"
                             : "navigating to frontier")));
      // Accepted dispatch commits the DFS room context. Inspecting candidates
      // must not Enter — only a path that will actually be tracked.
      if (!returning && !retreat_.active()) explorer_.CommitRegion(goal);
      RCLCPP_INFO(get_logger(),
                  "Navigation2D goal: pose=(%.2f, %.2f) goal=(%.2f, %.2f) "
                  "frontier_cells=%d region=%llu path_signature=%llu %s",
                  Pose()->x, Pose()->y, goal.x, goal.y,
                  goal.frontier_cells,
                  static_cast<unsigned long long>(goal.region_id),
                  static_cast<unsigned long long>(initial_state.path_signature),
                  explorer_.RegionAudit().c_str());
      return true;
    } catch (const std::exception& error) {
      last_planning_failure_ = error.what();
      RCLCPP_WARN(get_logger(), "Navigation2D rejected goal: %s", error.what());
      navigation_.Cancel();
      return false;
    }
  }

  void PublishPlanningDebugMap(bool returning) {
    if (!map_ || !Pose()) return;
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
    navigation2d::exploration::PaintPlanningDebugInput(
        planning_grid, LocalPose(*Pose()), input.data);
    planning_input_publisher_->publish(input);

    nav_msgs::msg::OccupancyGrid inflated = input;
    navigation2d::exploration::PaintInflatedDebugMap(planning_grid, input.data, inflated.data);
    inflated_costmap_publisher_->publish(inflated);
  }

  std::optional<navigation2d::exploration::TourPose> TourRobot() const {
    if (!Pose()) { return std::nullopt; }
    return navigation2d::exploration::TourPose{
        Pose()->x, Pose()->y, Pose()->yaw};
  }
  void BuildTour() override {
    tour_.Build(explorer_, *TourRobot());
    PublishFrontierDebug();
    RCLCPP_INFO(get_logger(),
                "Frontier graph tour %zu: raw_cells=%zu raw_clusters=%zu candidates=%zu "
                "stable=%zu %s",
                tour_.revision(), explorer_.raw_frontier_cells(),
                explorer_.raw_frontier_clusters(), explorer_.candidate_frontier_goals(),
                tour_.goals().size(), explorer_.RegionAudit().c_str());
  }

  void PublishFrontierDebug() {
    // Rebuild FR cells from the live map every publish. SelectGoals snapshots
    // go stale while navigating (unknown→free), which left the overlay empty
    // after prune-only. CG/SG markers still come from the last SelectGoals.
    explorer_.RefreshDebugFrontierCells();
    const auto& snap = explorer_.debug_snapshot();
    std::string json = navigation2d::exploration::SerializeFrontierDebug(snap);
    std_msgs::msg::String msg;
    msg.data = std::move(json);
    frontier_debug_publisher_->publish(msg);
  }

  void LockEntryMouth(const Goal& mouth) override {
    entry_.LockObserved(mouth, explorer_, {KnownCells(), map_revision_});
  }

  void ClearEntryMouthLock() override { entry_.Clear(); }

  // Entry budget exhausted or mouth unplannable: short demote so the tour
  // cannot immediately re-drive the same hall→threshold shuttle. Keep the
  // remaining tour — clearing it forced BuildTour to hop across the map to a
  // far sibling while the kitchen mouth was only briefly demoted (冤枉路).
  // Far mouths that were already queued stay, but local unfinished mouths
  // are rotated to the front so we do not leave the pocket for a detour.
  void AbandonLockedEntryMouth(const std::string& reason) override {
    Goal mouth = entry_.pending().value_or(
        entry_.locked().value_or(active_goal_));
    // Do not 1.1 m / 4000-cell blacklist the frontier: that hid the sealed
    // room for the rest of the run after a first-segment miss. Cool only the
    // viewpoint briefly so the tour can re-approach after open-loop / peel.
    explorer_.RecordAttempt(mouth, false, true, false);
    explorer_.CoolRegion(mouth.x, mouth.y, 0.55, 1200u);
    DemoteViewpointForSeconds(mouth, 8.0);
    ClearEntryMouthLock();
    tour_.AfterMouthFailure(explorer_, mouth, TourRobot(), *this);
    next_selection_ = now();
    PublishStatus("SELECTING", reason);
  }

  bool SameEntryMouth(const Goal& a, const Goal& b) const {
    return navigation2d::exploration::EntryCoordinator::SameMouth(a, b);
  }

  bool EasyLockedMouth() const { return entry_.EasyNow(explorer_); }

  bool PassableMouthGoal(const Goal& goal) const override {
    return navigation2d::exploration::EntryCoordinator::Passable(goal, explorer_);
  }

  int EntryAttemptBudget() const { return entry_.AttemptBudget(explorer_); }

  void DeferEntrySelection(navigation2d::exploration::EntrySelectionDelay delay) override {
    next_selection_ = now() + rclcpp::Duration::from_seconds(0.4);
    PublishStatus("COMPLETING",
        delay == navigation2d::exploration::EntrySelectionDelay::kWideMouthCrawl
            ? "wide mouth blocked; crawling mid-gap instead of abandon"
            : "entry plan rejected; budget remains");
  }

  // Promote only true corridor mouths into locked entry.
  bool PromotePassableMouthEntry(const char* reason) {
    if (returning_ || retreat_.active() || mission_.phase() == MissionPhase::kReturning)
      return false;
    if (!std::isfinite(active_goal_.x) || !PassableMouthGoal(active_goal_))
      return false;
    if (entry_.locked() && !SameEntryMouth(*entry_.locked(), active_goal_))
      return false;
    RCLCPP_WARN(get_logger(), "Passable-mouth promote: %s gap=%.2f", reason,
                explorer_.MouthGapWidth(active_goal_.frontier_x,
                                        active_goal_.frontier_y));
    LockEntryMouth(active_goal_);
    evidence_budget_.RequestTrailReplay(8.0);
    route_failures_.ClearSignatures();
    if (StartEntryProbe()) {
      PublishStatus("COMPLETING", reason);
      return true;
    }
    if (EasyLockedMouth() && entry_.crawl_ticks() == 0) {
      entry_.StartCrawl();
      PublishStatus("COMPLETING", "passable mouth; mid-gap crawl");
      return true;
    }
    return false;
  }

  bool EntryMouthCooling(const Goal& mouth) override { return RecentlyDemoted(mouth); }
  void PrepareEntryPlan(double trail_budget_m) override {
    evidence_budget_.RequestTrailReplay(trail_budget_m);
    route_failures_.ClearSignatures();
  }
  bool NavigateEntry(const Goal& goal) override { return StartNavigation(goal, false); }
  void CoolEnteredMouth(const Goal& mouth, double seconds) override {
    DemoteViewpointForSeconds(mouth, seconds);
  }
  void MarkRegionFinishPending() override { viewpoint_arrival_.MarkRegionFinishPending(); }
  void AcknowledgeEntry(navigation2d::exploration::EntryArrival arrival) override {
    navigation2d::exploration::CompleteEntryArrival(entry_, explorer_, room_visits_, *this,
        arrival, Pose()->x, Pose()->y);
  }
  void SelectAfterEntry() override { next_selection_ = now(); }
  void EntryStatus(const char* phase, const char* reason) override { PublishStatus(phase, reason); }
  void ReportEntry(const navigation2d::exploration::EntryNotice& notice) override {
    navigation2d::exploration_ros::ReportEntry(get_logger(), notice, last_planning_failure_, *this);
  }
  bool StartEntryProbe() override {
    if (!Pose()) { return false; }
    const auto result = navigation2d::exploration::ExecuteEntryProbe(
        entry_, explorer_, room_visits_, *this,
        {Pose()->x, Pose()->y, map_revision_, kExploreGoalTolerance});
    return result == navigation2d::exploration::EntryProbeResult::kDriving;
  }

  bool EntryProbeSatisfied() const {
    if (!Pose()) { return false; }
    return entry_.Satisfied(Pose()->x, Pose()->y, active_goal_, explorer_);
  }

  // After crossing a mouth: queue in-room leftovers before any hallway tour so
  // the kitchen white pocket is observed before soft-unlock hops out.
  void QueueActiveRegionFinish() override {
    if (!Pose()) return;
    const double rx = Pose()->x;
    const double ry = Pose()->y;
    tour_.QueueActiveRoom(explorer_, *TourRobot(), *this);
    RCLCPP_INFO(get_logger(),
                "active room finish queued: local_goals=%zu unknown=%zu %s",
                tour_.goals().size(),
                explorer_.ReachableUnknownCells(rx, ry),
                explorer_.RegionAudit().c_str());
  }

  std::pair<int, int> PoseCell() const {
    const double resolution = map_ ? map_->info.resolution : .05;
    return {static_cast<int>(std::floor(Pose()->x / resolution)),
            static_cast<int>(std::floor(Pose()->y / resolution))};
  }

  void RefreshStallContext() {
    if (!Pose()) return;
    route_failures_.ObserveCell(PoseCell());
    const std::size_t known = KnownCells();
    route_failures_.ObserveMapGrowth(known);
    NoteKnownProgress(known);
  }

  void NoteKnownProgress(std::size_t known) override {
    if (last_progress_known_cells_ == 0 || known >= last_progress_known_cells_ + 500) {
      last_progress_known_cells_ = known;
      last_progress_time_ = now();
    }
  }

  bool NoteFirstSegmentFailure(const navigation2d::NavigationState& state) {
    RefreshStallContext();
    const int failures = route_failures_.NoteFailure(state.path_signature);
    RCLCPP_WARN(get_logger(),
                "First-segment execution failed: signature=%llu failures_at_pose=%d/%d cooled=%zu",
                static_cast<unsigned long long>(state.path_signature),
                failures, 3, route_failures_.size());
    return failures >= 3;
  }

  using BodyRanges = navigation2d::exploration::BodyRanges;
  double SectorMinRange(double center_angle, double half_width) const {
    return navigation2d::exploration::SectorMinRange(ScanEvidence(), center_angle, half_width);
  }
  BodyRanges ReadBodyRanges() const {
    return navigation2d::exploration::ReadBodyRanges(ScanEvidence());
  }

  // Prefer the live route metric. A stored 0 after navigation_.Cancel() is not
  // a measurement — callers that require an active route check navigation_.
  double ActivePathMargin(double sample) const {
    if (navigation_.has_session() && std::isfinite(sample)) return sample;
    return active_path_min_clearance_m_;
  }

  // Body-frame lidar only — no map pose / yaw. LD14 distorts hard while
  // spinning and localization coasts on odom, so tip escape must prefer
  // reverse translation over in-place rotate.
  // Blind body-frame motion is only for a robot the planner cannot move:
  // boxed in on both axes, or its latest plan failed because the start sits
  // in an obstacle. Anywhere else a plan is the safer way out; escaping in
  // open space drove the tail into walls it could not see.
  bool TryStartEscape() override {
    if (!scan_ || !Pose() || escape_.Busy(now().nanoseconds())) return false;
    const BodyRanges body = ReadBodyRanges();
    const bool boxed_in = navigation2d::HitBlocksSwing(body.front) &&
                          navigation2d::HitBlocksSwing(body.side);
    if (!boxed_in && !start_occupied_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                  "Escape skipped: not boxed in and start is plannable "
                  "(front=%.2f side=%.2f)", body.front, body.side);
      return false;
    }
    const auto decision = navigation2d::exploration::SelectEscape(body, returning_);
    if (!decision) { return false; }
    const auto [velocity, hold_s] = *decision;
    const double front = body.front, back = body.back, left = body.left, right = body.right;
    escape_.Begin(velocity, (now() + rclcpp::Duration::from_seconds(hold_s)).nanoseconds());
    RCLCPP_WARN(get_logger(),
                "Starting body-frame escape: front=%.2f back=%.2f left=%.2f "
                "right=%.2f v=%.2f w=%.2f hold=%.1f",
                front, back, left, right, velocity.linear, velocity.angular,
                hold_s);
    return true;
  }

  bool TickEscape() {
    if (!escape_.scheduled()) return false;
    if (escape_.Busy(now().nanoseconds())) {
      // Re-read body-frame lidar every tick. Do not trust the frozen command's
      // yaw bias after localization has coasted on odom.
      const BodyRanges body = ReadBodyRanges();
      const auto adjusted = escape_.Command(body);
      motion_output_->Send(adjusted);
      PublishStatus("RECOVERING", "body-frame tip escape (prefer reverse)");
      return true;
    }
    escape_.Unschedule();
    motion_output_->Send({});
    const bool resume_return = escape_.ConsumeReturnIntent();
    if (resume_return) {
      BeginReturn(true);
      return true;
    }
    // Session-owned peel exit: resume without clearing side_peels/open_replans.
    if (recovery_.resume_after_escape) {
      recovery_.resume_after_escape = false;
      if (mission_.phase() == MissionPhase::kExploring && !returning_) {
        if (entry_.locked() || entry_.pending()) {
          navigation2d::exploration::ResumeLockedEntryAfterPeel(entry_, explorer_, *this);
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
    if (mission_.phase() == MissionPhase::kCompleting) {
      FinishCompleting();
      return true;
    }
    next_selection_ = now();
    return true;
  }

  void ClearCompletionRouteCooling() override { route_failures_.ClearSignatures(); }
  bool NavigateCompletionGoal(const Goal& goal) override { return StartNavigation(goal, false); }
  void CompletionStatus(const char* message) override { PublishStatus("SELECTING", message); }
  void ReportCompletion(const navigation2d::exploration::CompletionNotice& notice) override {
    navigation2d::exploration_ros::ReportCompletion(get_logger(), notice);
  }
  void StartCompletingProbe() override {
    completion_.Start(explorer_, mission_, tour_, *TourRobot(), *this);
  }

  bool ShouldKeepWaitingForFrontiers() const override {
    return completion_.ShouldWait(explorer_.HasExploringWork(), selection_exhaustion_.blocked_cycles());
  }

  bool TipPinchedNow() const override {
    if (!scan_ || !Pose()) return false;
    const BodyRanges body = ReadBodyRanges();
    if (navigation_.has_session())
      return IsPinchGeometryStall(active_path_min_clearance_m_, body.front,
                                  body.side);
    // No route: both axes inside the planning circle. A near flank alone is
    // not a pocket.
    return navigation2d::HitBlocksSwing(body.front) &&
           navigation2d::HitBlocksSwing(body.side);
  }

  // Leave a tip pocket and re-queue far corridor mouths (kitchen) instead of
  // PARTIAL while raw frontiers remain. One shot per mission.
  bool TryRescueDistantMouths(const char* why) override {
    if (tip_rescue_attempted_ || returning_ ||
        mission_.phase() == MissionPhase::kReturning || !Pose())
      return false;
    if (!explorer_.HasExploringWork()) return false;
    tip_rescue_attempted_ = true;
    RCLCPP_WARN(get_logger(),
                "Tip/tour rescue (%s): peel + retry distant mouths; raw=%zu",
                why, explorer_.raw_frontier_cells());
    viewpoint_rejections_.ClearDemotion();
    route_failures_.ClearSignatures();
    explorer_.ClearFailedBlacklist();
    completion_.ResetBudget();
    selection_exhaustion_.ResetBlocked();
    reject_streak_ = 0;
    ClearEntryMouthLock();
    navigation_.Cancel();
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
    BuildTour();
    if (!tour_.PrioritizeDistant(explorer_, *TourRobot(), *this)) {
      RCLCPP_WARN(get_logger(), "Tip rescue found no distant mouth to retry");
      return false;
    }
    mission_.ResumeExploring();
    SelectNextGoal();
    return navigation_.has_session() || entry_.pending() || entry_.locked();
  }

  void BeginReturnOrRescue(bool force_partial) override {
    if (force_partial && TryRescueDistantMouths("before PARTIAL return")) return;
    BeginReturn(force_partial);
  }

  void FinishCompleting() {
    completion_.Finish(explorer_, mission_, tour_, *this);
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
    if (entry_.pending() || entry_.locked()) return false;
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
    if (entry_.pending() || entry_.locked() || returning_ || retreat_.active())
      return false;
    if (!navigation_.has_session() || navigation2d::PathClearsOpen(path_clearance_m))
      return false;
    return navigation2d::HitBlocksSwing(front_range) &&
           navigation2d::HitBlocksSwing(side_range);
  }

  void CancelRecoveryNavigation() override { navigation_.Cancel(); }
  void PrepareDoorjambRetry() override { evidence_budget_.RequestTrailReplay(8.0); }
  bool OpenForwardWallScrape(double path_clearance_m, double side_range_m) const override {
    return IsOpenWallScrape(ActivePathMargin(path_clearance_m), side_range_m, true);
  }
  bool IsOpenWallScrape(double path_clearance_m, double side_range,
                        bool forward_clearly_open) const {
    // Open nose, but the flank hit is inside the planning circle and the
    // path is not an open-tier (mid-aisle) route.
    if (!forward_clearly_open || !navigation_.has_session()) return false;
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
      route_failures_.Cool(last_path_signature_);
    RCLCPP_WARN(get_logger(),
                "Pinch unstick: %s clearance=%.3f scrapes=%d pose=(%.2f,%.2f)",
                reason, active_path_min_clearance_m_, recovery_.wall_scrapes,
                Pose() ? Pose()->x : 0.,
                Pose() ? Pose()->y : 0.);
    motion_output_->Send({});
    if (mission_.phase() != MissionPhase::kReturning && !returning_ &&
        std::isfinite(active_goal_.x) && std::isfinite(active_goal_.y)) {
      explorer_.RecordAttempt(entry_.pending().value_or(active_goal_), false);
      // Short cool — 20 s kept re-queuing the same tip while kitchen waited.
      DemoteViewpointForSeconds(entry_.pending().value_or(active_goal_), 6.0);
      explorer_.CoolRegion(active_goal_.x, active_goal_.y, 0.70, 1500u);
      explorer_.CoolRegion(active_goal_.frontier_x, active_goal_.frontier_y, 0.70,
                           1500u);
    }
    ClearEntryMouthLock();
    navigation_.Cancel();
    ClearPublishedNavPaths();
    ClearDisplayedGoal();
    if (!tour_.goals().empty()) tour_.Advance();
    recovery_.Clear();
    retreat_.AllowRetry();
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

  bool BreakWallScrapeThrash(const char* reason) override {
    if (last_path_signature_ != 0)
      route_failures_.Cool(last_path_signature_);
    ++recovery_.wall_scrapes;
    navigation_.Cancel();
    RCLCPP_WARN(get_logger(),
                "%s; wall_scrapes=%d clearance=%.3f signature=%llu", reason,
                recovery_.wall_scrapes, active_path_min_clearance_m_,
                static_cast<unsigned long long>(last_path_signature_));
    // First scrape: same-goal latest-map replan. Second scrape / prior peel /
    // lethal tip → demote. Do not unstick on first open-front scrape (that
    // burned tour goals via "driving scrape path into corner tip").
    if (recovery_.wall_scrapes >= 2 || recovery_.side_peels > 0 ||
        !navigation2d::PathClearsPinch(active_path_min_clearance_m_)) {
      return UnstickFromPinch(reason);
    }
    latest_map_replans_ = 0;
    const int scrapes = recovery_.wall_scrapes;
    if (StartNavigation(active_goal_, false, true)) {
      recovery_.wall_scrapes = scrapes;  // Begin must not erase the budget
      if (!navigation2d::PathClearsPinch(active_path_min_clearance_m_)) {
        if (last_path_signature_ != 0)
          route_failures_.Cool(last_path_signature_);
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
  bool GoalIsBehindTrail(const Goal& goal) const override {
    return Pose() && navigation2d::exploration::GoalBehindDrivenTrail(
        pose_feedback_.trail(), *TourRobot(), goal);
  }
  bool RetreatPinched() const override {
    const auto body = ReadBodyRanges();
    return navigation2d::HitBlocksSwing(body.front) && navigation2d::HitBlocksSwing(body.side);
  }
  void ClearRetreatCooling() override { route_failures_.ClearSignatures(); }
  void PrepareRetreatEvidence(bool pinched) override {
    evidence_budget_.RequestTrailReplay(pinched ? 12.0 : 4.0);
    evidence_budget_.RequestStartReplay();
  }
  bool NavigateRetreat(const Goal& goal) override { return StartNavigation(goal, false); }
  void CancelRetreatEvidence() override { evidence_budget_.CancelTrailReplay(); }
  bool StartRetreat() {
    if (pose_feedback_.trail().size() < 4 || !Pose()) return false;
    return retreat_.Start(pose_feedback_.trail(), *TourRobot(), PoseCell(),
                          entry_.pending() || entry_.locked(), *this);
  }

  void SelectNextGoal() override {
    if (mission_.phase() == MissionPhase::kReturning || returning_) {
      BeginReturn(return_.partial());
      return;
    }
    if ((entry_.pending() || entry_.locked()) &&
        navigation2d::exploration::SelectLockedEntry(
            entry_, explorer_, TourRobot(), active_goal_, *this) ==
            navigation2d::exploration::EntrySelectionResult::kHandled) {
      return;
    }
    // Drop mouths the lidar already cleared; keep the remaining committed
    // order. Rebuilding the tour on every SelectNextGoal reordered T1..Tn and
    // made the robot U-turn between alcoves (triangular stutter in open space).
    tour_.SkipResolved(explorer_);
    if (tour_.goals().empty()) BuildTour();
    // A frontier extractor can emit many viewpoints around one wall.  Do not
    // synchronously exhaust that list when the local controller rejects a
    // viewpoint: that turns one transient costmap disagreement into hundreds
    // of failures before the mapper has published its next observation.  This
    // is the same bounded retry principle used by Nav2 recovery trees: try a
    // small, diverse budget, then yield to new sensor/map evidence.
    constexpr int kCandidatePlanBudget = 3;
    int attempted = 0;
    int rotated_reverse = 0;
    const double yaw = Pose()->yaw;
    while (!tour_.goals().empty() && attempted < kCandidatePlanBudget) {
      const Goal goal = tour_.goals().front();
      if (!explorer_.GoalRegionStillFrontier(goal)) {
        RCLCPP_INFO(get_logger(), "Skipping resolved tour viewpoint (%.2f, %.2f)", goal.x, goal.y);
        tour_.Advance();
        continue;
      }
      if (RecentlyDemoted(goal)) {
        RCLCPP_INFO(get_logger(),
                    "Skipping recently demoted viewpoint (%.2f, %.2f)", goal.x,
                    goal.y);
        tour_.Advance();
        // Rebuild inside the open room. Do not pull a global goal while a
        // door in this room is still holding the tour.
        if (tour_.goals().empty() && Pose()) BuildTour();
        continue;
      }
      // Defer reverse goals only when they are far. A tip inside the local
      // pocket must be finished even if it sits behind the robot — otherwise
      // we flee the alcove with a red frontier arc still visible.
      const double bearing = std::atan2(goal.y - Pose()->y,
                                        goal.x - Pose()->x);
      const double turn = std::abs(NormalizeAngle(bearing - yaw));
      const double goal_range = std::hypot(goal.x - Pose()->x,
                                           goal.y - Pose()->y);
      if (turn > 1.75 && goal_range > 2.5 &&
          rotated_reverse < static_cast<int>(tour_.goals().size())) {
        tour_.DeferCurrent(goal);
        ++rotated_reverse;
        continue;
      }
      if (StartNavigation(goal, false)) {
        selection_exhaustion_.GoalAccepted();
        reject_streak_ = 0;
        mission_.ResumeExploring();
        completion_.ResetBudget();
        return;
      }
      tour_.Advance();
      ++attempted;
      // Start pinched in wall-corner inflation: every other goal fails the same
      // start check. Do not entry-probe / rebuild-tour — leave the pinch first.
      const bool start_pinched = start_occupied_;
      if (start_pinched) {
        ++reject_streak_;
        evidence_budget_.RequestStartReplay();
        evidence_budget_.ResumeTrailReplay();
        RCLCPP_WARN(get_logger(),
                    "Start pinched (%s); peel/short-retreat instead of reject-looping frontiers",
                    last_planning_failure_.c_str());
        if (EscapeOrShortRetreat(
                "start pinched; peel out of inflation",
                "start pinched; short trail retreat")) {
          route_failures_.ClearSignatures();
          return;
        }
        break;
      }
      // Entry only for real corridor mouths. Open-wall / residual frontiers
      // (user: 没有门缝) must not LockEntryMouth — that burned attempts on
      // gap=1.65 open spans and looked like "stuck at a door".
      if (PassableMouthGoal(goal)) {
        if (entry_.locked() && !SameEntryMouth(*entry_.locked(), goal)) {
          explorer_.RecordAttempt(goal, false);
          continue;
        }
        LockEntryMouth(goal);
        if (StartEntryProbe()) return;
        if (entry_.attempts() >= EntryAttemptBudget()) {
          AbandonLockedEntryMouth(
              "hallway reject; entry probe exhausted, leaving mouth");
          continue;
        }
        entry_.ClearPending();  // lock retained
        next_selection_ = now() + rclcpp::Duration::from_seconds(0.4);
        return;
      }
      navigation2d::exploration::RejectOpenWallViewpoint(
          explorer_, tour_, viewpoint_rejections_, {goal, last_planning_failure_, last_path_signature_},
          *this);
      continue;
    }
    // Same pose keeps rejecting every queued frontier (often cooled first-
    // segment signatures while pinched in an alcove). Retreat along the
    // driven trail before spinning on identical rejections until PARTIAL.
    if (reject_streak_ >= 2 &&
        EscapeOrShortRetreat("reject streak; peel before reselect",
                             "reject streak; short trail retreat")) {
      route_failures_.ClearSignatures();
      return;
    }
    selection_exhaustion_.Resolve(explorer_, completion_, tour_, {*TourRobot(), attempted}, *this);
  }

  void StopSelectionMotion() override {
    motion_output_->Send({});
  }
  void DeferBlockedSelection(const char* message) override {
    next_selection_ = now() + rclcpp::Duration::from_seconds(0.4);
    SelectionStatus(message);
  }
  void SelectionStatus(const char* message) override { PublishStatus("SELECTING", message); }
  void ClearBlockedSelection() override {
    ClearDisplayedGoal();
    ClearPublishedNavPaths();
  }
  bool SelectionFrontSwingClear() const override {
    return navigation2d::AxisSwingClear(ReadBodyRanges().front);
  }

  bool HandoffToNextTourGoal() {
    if (mission_.phase() != MissionPhase::kExploring || returning_ || !navigation_.has_session() ||
        tour_.goals().size() < 2) return false;
    const double distance = std::hypot(active_goal_.x - Pose()->x,
                                       active_goal_.y - Pose()->y);
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
    while (next_index < tour_.goals().size()) {
      const Goal next = tour_.goals()[next_index];
      if (!explorer_.GoalRegionStillFrontier(next)) {
        ++next_index;
        continue;
      }
      if (!explorer_.InSameReachableRegion(
              Pose()->x, Pose()->y, next.x, next.y,
              kHandoffPocketM) &&
          !explorer_.InSameReachableRegion(
              active_goal_.x, active_goal_.y, next.x, next.y, kHandoffPocketM))
        return false;
      const double next_bearing = std::atan2(
          next.y - Pose()->y, next.x - Pose()->x);
      const double direction_change = std::abs(NormalizeAngle(
          next_bearing - Pose()->yaw));
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
        tour_.HandoffThrough(next_index);
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
    return return_.Home(map_ ? &ObservationGrid() : nullptr,
        {mission_.home()->x, mission_.home()->y, mission_.home()->yaw});
  }

  bool StartBreadcrumbTowardHome() override {
    if (!mission_.home() || !Pose() || pose_feedback_.trail().size() < 4) return false;
    const Goal home = MakeHomeGoal();
    const double rx = Pose()->x;
    const double ry = Pose()->y;
    const double home_dist = std::hypot(rx - home.x, ry - home.y);
    const auto next = return_.NextStep(pose_feedback_.trail(), home, rx, ry, active_goal_);
    using navigation2d::exploration::ReturnStepKind;
    if (next.kind == ReturnStepKind::kRepeated) { return false; }
    if (next.kind == ReturnStepKind::kDirect) {
      return_.UseBreadcrumbs(false);
      evidence_budget_.RequestTrailReplay(80.0);
      evidence_budget_.RequestStartReplay();
      return StartNavigation(home, true);
    }
    const Goal& step = next.goal;
    mission_.BeginReturning();
    returning_ = true;
    return_.UseBreadcrumbs(true);
    route_failures_.ClearSignatures();
    evidence_budget_.RequestTrailReplay(80.0);
    evidence_budget_.RequestStartReplay();
    if (!StartNavigation(step, true)) return false;
    RCLCPP_INFO(get_logger(),
                "Breadcrumb return hop: pose=(%.2f,%.2f) step=(%.2f,%.2f) home_dist=%.2f",
                rx, ry, step.x, step.y, home_dist);
    PublishStatus("RETURNING", "following driven trail breadcrumbs home");
    return true;
  }

  // Frontiers only prove a room was seen. A door the explorer tried to cross
  // but never did leaves a room with no frontier, so before going home drive
  // into the known space behind that door. Only doors with an entry attempt
  // count: far-but-mapped pockets of rooms already entered are not revisited.
  // Each spot gets two tries; a reached spot drops out because the trail now
  // passes it.
  bool TryVisitUnenteredSpace() {
    if (!Pose() || returning_ || mission_.phase() == MissionPhase::kReturning) return false;
    if (room_visits_.empty()) return false;
    constexpr int kVisitAttempts = navigation2d::exploration::RoomVisitLedger::kVisitAttempts;
    const double rx = Pose()->x;
    const double ry = Pose()->y;
    const auto room_seeds = room_visits_.Seeds();
    auto goals = explorer_.UnvisitedSpaceGoals(
        pose_feedback_.trail(), navigation2d::kOpenRetreat(),
        navigation2d::kRoomPocketDepthM() * navigation2d::kRoomPocketDepthM(),
        room_seeds, navigation2d::kOpenRetreat());
    std::sort(goals.begin(), goals.end(), [&](const Goal& a, const Goal& b) {
      return std::hypot(a.x - rx, a.y - ry) < std::hypot(b.x - rx, b.y - ry);
    });
    for (auto& goal : goals) {
      const int attempt = room_visits_.TryVisit(goal);
      if (attempt == 0) { continue; }
      goal.yaw = std::atan2(goal.y - ry, goal.x - rx);
      mission_.ResumeExploring();
      tour_.Suspend();
      if (StartNavigation(goal, false)) {
        RCLCPP_INFO(get_logger(),
                    "Visiting unentered space before return: goal=(%.2f, %.2f) area=%.2f m2 try=%d/%d",
                    goal.x, goal.y, goal.information_gain, attempt, kVisitAttempts);
        PublishStatus("NAVIGATING", "visiting a room not yet entered");
        return true;
      }
      RCLCPP_WARN(get_logger(), "Unentered space goal unplannable at (%.2f, %.2f)",
                  goal.x, goal.y);
    }
    return false;
  }

  void HandleReturnRecovery(navigation2d::exploration::ReturnRecoveryMode mode,
                            const char* unavailable_message) {
    const auto result = return_.Recover(mode, *this);
    if (result == navigation2d::exploration::ReturnExecutionResult::kEscaping) {
      escape_.ResumeReturn();
    } else if (result == navigation2d::exploration::ReturnExecutionResult::kUnavailable) {
      Finish(true, unavailable_message);
    }
  }

  void BeginReturn(bool force_return = false) override {
    if (!mission_.home()) return;
    if (mission_.phase() == MissionPhase::kReturning && navigation_.has_session()) return;
    if (TryVisitUnenteredSpace()) return;
    mission_.BeginReturning();
    returning_ = true;
    tour_.Suspend();
    ClearEntryMouthLock();
    if (force_return) {
      return_.MarkPartial();
      RCLCPP_WARN(get_logger(),
                  "No executable frontier after bounded retries; returning safely with current map");
    } else if (!explorer_.CompletionEligible(mission_.initial_known_cells()))
      RCLCPP_WARN(get_logger(),
                  "No frontier survived a stable completion window; returning with reachable map coverage");
    route_failures_.ClearSignatures();
    return_.UseBreadcrumbs(false);
    const auto result = return_.Execute(*this);
    if (result == navigation2d::exploration::ReturnExecutionResult::kEscaping) {
      escape_.ResumeReturn();
    } else if (result == navigation2d::exploration::ReturnExecutionResult::kUnavailable) {
      Finish(false, "return path unavailable; stopping to protect localization");
    }
  }

  bool PlanHome(navigation2d::exploration::ReturnPlanStage stage) override {
    evidence_budget_.RequestTrailReplay(80.0);
    if (stage == navigation2d::exploration::ReturnPlanStage::kIdleFinal) {
      evidence_budget_.RequestStartReplay();
    }
    return StartNavigation(MakeHomeGoal(), true);
  }

  void HomeAccepted(navigation2d::exploration::ReturnPlanStage stage) override {
    RCLCPP_INFO(get_logger(), "%s",
                stage == navigation2d::exploration::ReturnPlanStage::kInitial
                    ? "Final online-map return path accepted; tracking with RPP"
                    : "Final online-map return path accepted after escape; tracking with RPP");
    PublishStatus("RETURNING", "tracking shortest route home on final observed map");
  }

  bool ReplanActiveGoalOnLatestMap(const char* reason) override {
    // Each attempt rebuilds NavigationSystem from the newest mapper output
    // and the robot's current true pose. It cannot replay or bias toward the
    // historical trajectory. Bound the loop so a genuinely disconnected
    // frontier/return reports failure instead of churning forever.
    constexpr int kMaximumLatestMapReplans = 2;
    if (latest_map_replans_ >= kMaximumLatestMapReplans) return false;
    ++latest_map_replans_;
    navigation_.Cancel();
    RCLCPP_WARN(get_logger(),
                "%s; replanning active %s goal on latest online map (%d/%d)",
                reason, returning_ ? "return" : "frontier", latest_map_replans_,
                kMaximumLatestMapReplans);
    return StartNavigation(active_goal_, returning_, true);
  }

  // A route cut while driving means the map changed, not that the goal is
  // bad: plan the same goal again on the newest map, painted the way the cut
  // route was, before any failure bookkeeping (reject counts, signature
  // cooling, breadcrumbs). The new snapshot contains the new obstacle, so the
  // planner routes around it or reports no route.
  bool ReplanAfterRouteCut() {
    constexpr int kMaxCutReplans = 3;
    if (cut_replans_ >= kMaxCutReplans) return false;
    ++cut_replans_;
    navigation_.Cancel();
    if (planning_map_.snapshot().trail_budget_m > 0.) {
      evidence_budget_.ResumeTrailReplay();
    }
    RCLCPP_WARN(get_logger(),
                "Route cut by newly mapped obstacle; replanning same %s goal on latest map (%d/%d)",
                returning_ ? "return" : "frontier", cut_replans_, kMaxCutReplans);
    const int cut_replans = cut_replans_;
    const bool accepted = StartNavigation(active_goal_, returning_, true);
    cut_replans_ = cut_replans;
    return accepted;
  }

  void ClearDisplayedGoal() {
    active_goal_ = Goal{};
    active_goal_.x = std::numeric_limits<double>::quiet_NaN();
    active_goal_.y = std::numeric_limits<double>::quiet_NaN();
    active_goal_.frontier_x = std::numeric_limits<double>::quiet_NaN();
    active_goal_.frontier_y = std::numeric_limits<double>::quiet_NaN();
  }

  void DemoteActiveGoal(const char* reason) {
    if (mission_.phase() != MissionPhase::kReturning && !returning_) {
      explorer_.RecordAttempt(entry_.pending().value_or(active_goal_), false);
      DemoteViewpointForSeconds(entry_.pending().value_or(active_goal_), 8.0);
    }
    ClearEntryMouthLock();
    navigation_.Cancel();
    ClearPublishedNavPaths();
    ClearDisplayedGoal();
    recovery_.Clear();
    if (!tour_.goals().empty()) tour_.Advance();
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
    DemoteViewpointForSeconds(tip, 10.0);
    explorer_.CoolRegion(tip.x, tip.y, 0.80, 1800u);
    explorer_.CoolRegion(tip.frontier_x, tip.frontier_y, 0.80, 1800u);
    ClearEntryMouthLock();
    navigation_.Cancel();
    recovery_.Clear();
    tour_.AfterWallTip(explorer_, tip, TourRobot(), *this);
    route_failures_.ClearSignatures();
    latest_map_replans_ = 0;
    mapped_obstacle_rejects_ = 0;
    retreat_.AllowRetry();
    next_selection_ = now() + rclcpp::Duration::from_seconds(1.0);
    PublishStatus("SELECTING", reason);
    if (EscapeOrShortRetreat("leaving wall tip after path-cut thrash; peel",
                             "leaving wall tip after path-cut thrash; short retreat"))
      return true;
    return true;
  }

  bool HandlePathCutsWhileExploring(const navigation2d::NavigationState& state) {
    NoteMappedObstacleReject(active_goal_);
    navigation_.Cancel();
    // Locked mouth owns the junction. Live lidar often thickens jamb speckles
    // after the plan was accepted; cooling the throat signature then burning
    // deeper probes produced start→cut→start in <200 ms and room_entered=0.
    // Replan the SAME probe on the latest opened grid first; only deepen when
    // that fails. Do not cool entry chords — the doorway path is the mission.
    if (entry_.locked() || entry_.pending()) {
      if (!entry_.pending()) entry_.SetPending(entry_.locked());
      latest_map_replans_ = 0;
      if (cut_replans_ == 0 &&
          ReplanActiveGoalOnLatestMap("path cut at locked mouth; replan entry")) {
        PublishStatus("COMPLETING",
                      "path cut; replanning locked mouth on latest map");
        return true;
      }
      if (entry_.attempts() < EntryAttemptBudget()) {
        const int before_attempts = entry_.attempts();
        if (StartEntryProbe()) {
          PublishStatus("COMPLETING",
                        "path cut; retrying locked mouth at deeper probe");
          return true;
        }
        // One rejected plan is not the whole mouth budget. Abandoning here
        // left the room after attempt 2 of 6.
        if (entry_.attempts() > before_attempts &&
            entry_.attempts() < EntryAttemptBudget()) {
          next_selection_ = now() + rclcpp::Duration::from_seconds(0.4);
          PublishStatus("COMPLETING",
                        "path cut; entry budget remains");
          return true;
        }
      }
      AbandonLockedEntryMouth("locked mouth path cut exhausted; leaving mouth");
      return true;
    }
    if (state.path_signature != 0) route_failures_.Cool(state.path_signature);
    const double clearance =
        ActivePathMargin(state.global_planning.path_min_clearance_m);
    // Wall-tip chords: replan-once just rebuilds another illegal tip graze and
    // the robot pause→drop→replan→pause in place (red-box tip thrash).
    if (navigation2d::PreferTipAbandonAfterPathCut(mapped_obstacle_rejects_,
                                                   clearance)) {
      return AbandonWallTipAfterPathCut(
          "path cut at low-clearance wall tip; leaving tip pocket");
    }
    if (cut_replans_ == 0 &&
        ReplanActiveGoalOnLatestMap("path cut mapped obstacle; replan once"))
      return true;
    DemoteActiveGoal("path intersected mapped obstacles");
    if (BreakMappedObstacleThrash()) return true;
    return true;
  }

  void NotePoseMotion() {
    if (!Pose()) {
      return;
    }
    pose_motion_.Observe(Pose()->x, Pose()->y, now().nanoseconds());
  }

  bool PoseImmobilityWatchdog(const navigation2d::NavigationState& state) {
    NotePoseMotion();
    if (!navigation_.has_session() || !Pose()) return false;
    const bool entry_drive = entry_.pending() || entry_.locked();
    const BodyRanges body = ReadBodyRanges();
    const double path_clear =
        ActivePathMargin(state.global_planning.path_min_clearance_m);
    const bool tip_wedged =
        !entry_drive &&
        IsPinchGeometryStall(path_clear, body.front, body.side);
    if (!pose_motion_.Stalled({now().nanoseconds(), goal_execution_.started_ns(),
                              goal_execution_.progress_ns(), state.path_progress_m,
                              entry_drive, tip_wedged, navigation2d::AxisSwingClear(body.front)})) {
      return false;
    }
    ++immobility_recoveries_;
    RCLCPP_WARN(get_logger(),
                "Pose immobility watchdog: stuck=(%.2f,%.2f) recoveries=%d "
                "returning=%s entry=%s progress=%.2f tip=%s front=%.2f",
                Pose()->x, Pose()->y,
                immobility_recoveries_, returning_ ? "yes" : "no",
                entry_.pending() ? "yes" : "no", state.path_progress_m,
                tip_wedged ? "yes" : "no", body.front);
    motion_output_->Send({});
    navigation_.Cancel();
    pose_motion_.Debounce(now().nanoseconds());
    if (mission_.phase() == MissionPhase::kReturning || returning_) {
      route_failures_.ClearSignatures();
      HandleReturnRecovery(navigation2d::exploration::ReturnRecoveryMode::kImmobility,
                           "return stalled; stopping on driven trail near home");
      return true;
    }
    if (entry_drive) {
      if (!entry_.pending()) entry_.SetPending(entry_.locked());
      // progress=0 far from the mouth: one peel, then leave the mouth. Do not
      // trail-retreat — that drives back along the outbound path (折返).
      const double mouth_range = std::hypot(
          Pose()->x - entry_.pending()->frontier_x,
          Pose()->y - entry_.pending()->frontier_y);
      if (state.path_progress_m < .08 && mouth_range > 1.5) {
        evidence_budget_.RequestStartReplay();
        retreat_.AllowRetry();
        if (EscapeOrShortRetreat(
                "immobility far from locked mouth; peel then retry",
                "unused"))
          return true;
        AbandonLockedEntryMouth(
            "immobility far from mouth; leave and continue tour (no reverse)");
        next_selection_ = now();
        return true;
      }
      retreat_.AllowRetry();
      if (EscapeOrShortRetreat("immobility at locked mouth; peel then retry",
                               "unused"))
        return true;
      if (mouth_range <= 1.5 && entry_.attempts() < EntryAttemptBudget() && StartEntryProbe())
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
        route_failures_.Cool(last_path_signature_);
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
    DemoteViewpointForSeconds(active_goal_, 6.0);
    explorer_.CoolRegion(active_goal_.x, active_goal_.y, 0.80, 1800u);
    explorer_.CoolRegion(active_goal_.frontier_x, active_goal_.frontier_y, 0.80,
                         1800u);
    ClearEntryMouthLock();
    retreat_.AllowRetry();
    evidence_budget_.RequestStartReplay();
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

  void DeferViewpointSelection(double seconds, const char* message) override {
    next_selection_ = seconds == 0. ? now() : now() + rclcpp::Duration::from_seconds(seconds);
    PublishStatus("SELECTING", message);
  }
  void ReportViewpointArrival(navigation2d::exploration::ViewpointArrivalNotice kind,
                              const Goal& goal, int attempts, double depth) override {
    using navigation2d::exploration::ViewpointArrivalNotice;
    if (kind == ViewpointArrivalNotice::kEntryIncomplete) {
      RCLCPP_WARN(get_logger(),
                  "Frontier entry incomplete: frontier=(%.2f,%.2f) attempts=%d "
                  "depth=%.2f; keeping unresolved", goal.frontier_x, goal.frontier_y, attempts, depth);
    } else if (kind == ViewpointArrivalNotice::kNearRoomBeyond) {
      RCLCPP_INFO(get_logger(),
                  "At viewpoint with room still beyond frontier=(%.2f,%.2f); entering",
                  goal.frontier_x, goal.frontier_y);
    } else {
      RCLCPP_INFO(get_logger(),
                  "Goal reached with room still beyond frontier=(%.2f,%.2f); entering",
                  goal.frontier_x, goal.frontier_y);
    }
  }

  std::int64_t ExecutionNowNs() const override { return now().nanoseconds(); }
  bool RecentlyDemoted(const Goal& goal) override {
    // Preserve the missing-cooldown short circuit before reading the ROS clock.
    return viewpoint_rejections_.HasDemotion() &&
        viewpoint_rejections_.RecentlyDemoted(goal, now().nanoseconds());
  }

  void DemoteViewpointForSeconds(const Goal& goal, double seconds) {
    viewpoint_rejections_.Demote(goal, RejectionDeadlineNs(seconds));
  }
  std::int64_t RejectionDeadlineNs(double seconds) const override {
    return (now() + rclcpp::Duration::from_seconds(seconds)).nanoseconds();
  }
  void CoolRejectedSignature(std::uint64_t signature) override {
    route_failures_.Cool(signature);
  }
  void ReportViewpointRejection(
      const navigation2d::exploration::ViewpointRejectionNotice& notice) override {
    const auto& goal = notice.goal;
    if (notice.kind == navigation2d::exploration::ViewpointRejectionKind::kSoftScrape) {
      RCLCPP_WARN(get_logger(),
                  "Open-wall scrape reject (%s); rotate tour (soft %d/2) gap=%.2f",
                  notice.reason.c_str(), notice.scrape_count,
                  explorer_.MouthGapWidth(goal.frontier_x, goal.frontier_y));
      return;
    }
    RCLCPP_WARN(get_logger(),
                "Open-wall goal rejected (%s); demoting, not entry-locking gap=%.2f corridor=%s",
                notice.reason.c_str(), explorer_.MouthGapWidth(goal.frontier_x, goal.frontier_y),
                explorer_.IsCorridorMouth(goal.frontier_x, goal.frontier_y) ? "yes" : "no");
  }

  void Finish(bool success, const std::string& message) {
    if (!mission_.Finish()) {
      return;
    }
    ClearEntryMouthLock();
    return_.UseBreadcrumbs(false);
    motion_output_->Send({});
    using navigation2d::exploration::MissionPosition;
    std::optional<MissionPosition> current, dock;
    if (Pose() && mission_.home()) {
      current = MissionPosition{Pose()->x,
                                Pose()->y};
      const Goal home = MakeHomeGoal();
      dock = MissionPosition{home.x, home.y};
    }
    const auto completion = mission_.EvaluateCompletion(success, message, current, dock);
    success = completion.success;
    const auto& final_message = completion.message;
    const double return_error =
        completion.home_error_m.value_or(std::numeric_limits<double>::infinity());
    navigation2d::exploration_io::WriteArtifacts(
        {parameters_.result_path, parameters_.snapshot_path},
        {.status = !success ? "FAILED" : (return_.partial() ? "PARTIAL" : "COMPLETE"),
         .message = final_message,
         .pose_source = parameters_.pose_source,
         .backend = telemetry_.state().backend,
         .global_planner = selected_algorithms_.global,
         .local_controller = selected_algorithms_.local,
         .mppi_commands = mppi_commands_,
         .rpp_commands = rpp_commands_,
         .total_distance_m = pose_feedback_.distance_m(),
         .known_cells = KnownCells(),
         .completed_goals = explorer_.completed_goals(),
         .failed_goals = explorer_.failed_goals(),
         .return_error = return_error},
        map_ ? &ObservationGrid() : nullptr, pose_feedback_.trail());
    PublishStatus(!success ? "FAILED" : (return_.partial() ? "PARTIAL" : "COMPLETE"), final_message);
    // Leave the process so the debug UI can Start a new run without a stuck
    // finished node continuing to republish the old latched status.
    rclcpp::shutdown();
  }

  void PublishStatus(const std::string& state, const std::string& message) override {
    last_state_ = state;
    last_message_ = message;
    std_msgs::msg::String status;
    status.data = navigation2d::exploration::SerializeExplorationStatus({
        .state = state,
        .message = message,
        .phase = mission_.phase() == MissionPhase::kCompleting ? "completing" :
            mission_.phase() == MissionPhase::kReturning ? "returning" : "exploring",
        .known_cells = KnownCells(),
        .completed_goals = explorer_.completed_goals(),
        .failed_goals = explorer_.failed_goals(),
        .raw_frontier_cells = explorer_.raw_frontier_cells(),
        .unresolved_frontier_cells = explorer_.unresolved_frontier_cells(),
        .candidates = explorer_.candidate_frontier_goals(),
        .tour = tour_.goals(),
        .active_goal = active_goal_,
        .navigation = telemetry_.state(),
        .planning_failure = last_planning_failure_,
        .path_min_clearance_m = active_path_min_clearance_m_,
        .carrot_x = debug_paths_.x(),
        .carrot_y = debug_paths_.y(),
        .carrot_yaw = debug_paths_.yaw(),
        .carrot_valid = debug_paths_.valid(),
        .pose_source = parameters_.pose_source});
    status_publisher_->publish(status);
  }

  void CaptureNavTelemetry(const navigation2d::NavigationState& state,
                           const navigation2d::Twist2d& measured_velocity) {
    telemetry_.Capture(state, measured_velocity);
    active_path_min_clearance_m_ = state.global_planning.path_min_clearance_m;
    last_planning_failure_ = state.planning_failure_reason;
    if (state.path_signature != 0) { last_path_signature_ = state.path_signature; }
  }

  void NoteMappedObstacleReject(const Goal& goal) override {
    ++mapped_obstacle_rejects_;
    // Cool only the failed viewpoint briefly. Cooling the whole mouth/room
    // (1.8 m / 4000 cells) hid kitchen and bedroom frontiers while the robot
    // kept reworking living-room crumbs.
    explorer_.CoolRegion(goal.x, goal.y, 0.45, 600u);
  }

  bool BreakMappedObstacleThrash() override {
    if (mapped_obstacle_rejects_ < 2) return false;
    mapped_obstacle_rejects_ = 0;
    navigation_.Cancel();
    recovery_.Clear();
    route_failures_.ClearSignatures();
    const std::size_t local_unknown =
        Pose() ? explorer_.ReachableUnknownCells(Pose()->x,
                                                Pose()->y)
              : 0;
    // Unfinished pockets: try room-side entry before cooling/fleeing.
    if (local_unknown > 80 && Pose()) {
      RCLCPP_WARN(get_logger(),
                  "mapped-obstacle reject thrash with local unknown=%zu; "
                  "trying entry before retreat",
                  local_unknown);
      LockEntryMouth(active_goal_);
      evidence_budget_.RequestTrailReplay(8.0);
      if (StartEntryProbe()) return true;
      explorer_.RecordAttempt(active_goal_, false);
      ClearEntryMouthLock();
      if (!tour_.goals().empty()) tour_.Advance();
      retreat_.AllowRetry();
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
    if (Pose()) {
      explorer_.CoolRegion(active_goal_.x, active_goal_.y, 1.80, 3000u);
      explorer_.CoolRegion(active_goal_.frontier_x, active_goal_.frontier_y, 1.80,
                           3000u);
      explorer_.CoolRegion(Pose()->x, Pose()->y, 1.80,
                           3000u);
    }
    retreat_.AllowRetry();
    tour_.Suspend();
    PublishStatus("SELECTING", "leaving pinch after mapped-obstacle rejects");
    if (EscapeOrShortRetreat(
            "leaving pinch after mapped-obstacle rejects; peel",
            "leaving pinch after mapped-obstacle rejects; short retreat"))
      return true;
    next_selection_ = now() + rclcpp::Duration::from_seconds(1.0);
    return true;
  }

  // Live-map check of the active route. SLAM occupied is lethal, unknown is
  // not. It must agree with the plan it checks: every cell the planner was
  // told is free (body disk, lidar interiors, painted trail) stays free here,
  // otherwise the checker vetoes routes the planner was right to take (live:
  // about one accepted route in four dropped mid-drive, return retraced in
  // 0.44 m hops). While driving, only the part ahead of the robot matters and
  // a cut needs a new cell touching the chassis, not the planning circle.
  bool PathCutsMappedObstacle(bool execution_tick = false) const {
    if (!navigation_.has_session() || !map_ || map_->info.resolution <= 0.) return false;
    const auto& route = navigation_.path();
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
      path_xy.emplace_back(navigation2d::X(pose) + planning_map_.origin_x(),
                           navigation2d::Y(pose) + planning_map_.origin_y());
    }
    const double robot_x = Pose() ? Pose()->x : path_xy.front().first;
    const double robot_y = Pose() ? Pose()->y : path_xy.front().second;
    return navigation2d::exploration::RouteCutsMappedObstacle(
        std::move(live), std::move(path_xy), planning_map_.snapshot(), robot_x, robot_y,
        Pose() ? Pose()->yaw : 0., Pose() ? ScanEvidence() : std::nullopt,
        execution_tick);
  }

  double CommandFrontRange() const override { return ReadBodyRanges().front; }
  bool CommandSegmentCutsWall(double x, double y, const Goal& target) const override {
    return TourSegmentCutsWall(x, y, target.x, target.y);
  }
  bool CrawlSegmentCutsWall(double x, double y, const Goal& target) const override {
    return TourSegmentCutsWall(x, y, target.x, target.y);
  }
  void PublishEntryCrawl(navigation2d::Twist2d velocity) override {
    motion_output_->Send(velocity);
    PublishStatus("COMPLETING", "entry no-motion; open-loop crawl into mouth");
  }

  bool TourSegmentCutsWall(double x0, double y0, double x1, double y1) const {
    return navigation2d::exploration_ros::SegmentCutsWall(
        map_ ? &*map_ : nullptr, x0, y0, x1, y1);
  }

  void ClearPublishedNavPaths() {
    nav_msgs::msg::Path empty;
    empty.header.stamp = now();
    empty.header.frame_id = "map";
    path_publisher_->publish(empty);
    local_trajectory_publisher_->publish(empty);
    tour_publisher_->publish(empty);
    debug_paths_.Clear();
  }

  void PublishDebugPaths(const navigation2d::NavigationState& state) {
    if (!navigation_.has_session() || !Pose()) {
      ClearPublishedNavPaths();
      return;
    }
    const auto global = debug_paths_.Global(navigation_.path(), planning_map_.origin_x(),
                                            planning_map_.origin_y(), now());
    path_publisher_->publish(global);
    const auto local = debug_paths_.Local(global, state.path_progress_m);
    local_trajectory_publisher_->publish(local);
    const auto carrot = debug_paths_.Carrot(global.header);
    carrot_publisher_->publish(carrot);
    const auto [tour, frontiers] = debug_paths_.Tour(
        global.header, map_ ? &*map_ : nullptr, Pose()->x,
        Pose()->y, active_goal_, tour_.goals());
    tour_publisher_->publish(tour);
    frontiers_publisher_->publish(frontiers);
    PublishFrontierDebug();
  }

  void Tick() {
    if (mission_.finished()) {
      return;
    }
    if (!map_ || !Pose() || !scan_ || KnownCells() < 100) return;
    if (!mission_.started()) {
      const auto& pose = *Pose();
      const navigation2d::exploration::MissionHome home{
          pose.x, pose.y, pose.yaw};
      const auto started = now();
      return_.ResetEscapes();
      escape_.ClearReturnIntent();
      const auto initial_known = KnownCells();
      selection_exhaustion_.InitializeKnown(initial_known);
      last_progress_known_cells_ = initial_known;
      last_progress_time_ = now();
      const auto bootstrap_deadline = now() + rclcpp::Duration::from_seconds(8.0);
      mission_.Start(home, initial_known, started.nanoseconds(), bootstrap_deadline.nanoseconds());
      next_selection_ = bootstrap_deadline;
      PublishStatus("WAITING", parameters_.pose_source + " pose ready; waiting for a stable online map");
    }
    if (mission_.TimedOut(now().nanoseconds(), parameters_.max_duration_s)) {
      Finish(false, "exploration timed out");
      return;
    }
    NoteKnownProgress(KnownCells());
    NotePoseMotion();
    if (TickEscape()) return;
    if (mission_.Bootstrapping(now().nanoseconds())) {
      motion_output_->Send({});
      return;
    }
    if (mission_.phase() == MissionPhase::kReturning || returning_) {
      if (!navigation_.has_session()) {
        using navigation2d::exploration::ReturnIdleResult;
        const auto result = return_.ContinueIdle(escape_.scheduled() || escape_.resume_return(), *this);
        if (result == ReturnIdleResult::kEscaping) {
          escape_.ResumeReturn();
          evidence_budget_.RequestStartReplay();
        } else if (result == ReturnIdleResult::kUnavailable) {
          Finish(true, "return path unavailable; stopping to protect localization");
        } else if (result == ReturnIdleResult::kRestart) {
          BeginReturn(return_.partial());
        }
        return;
      }
    } else if (!navigation_.has_session()) {
      // Continue open-loop doorway crawl across ticks (navigation_ was reset
      // when CommandStalled fired; without this the crawl lasted one frame).
      if (navigation2d::exploration::HasActiveEntryCrawl(entry_) && Pose()) {
        navigation2d::exploration::ExecuteEntryCrawl(entry_, explorer_, active_goal_, *TourRobot(), *this);
        return;
      }
      // No route and no crawl: drop the last approach command. Leaving it
      // running while the next goal is planned rolls the chassis from the
      // viewpoint into the planning circle (live: arrived, then
      // "start occupied, nearest obstacle 0.19 m").
      motion_output_->Send({});
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
    const auto local_pose = LocalPose(*Pose());
    const auto tick_time = now();
    const auto measured_velocity = FreshVelocity(tick_time);
    const auto state = navigation_.Step(local_pose, measured_velocity,
                                                    tick_time.seconds());
    if (PoseImmobilityWatchdog(state)) return;
    // Mouth crossing is geometric — acknowledge once past the jambs, but keep
    // tracking the room-side probe so the chassis finishes into the pocket.
    if (entry_.pending() && !returning_ && !retreat_.active() && Pose() &&
        EntryProbeSatisfied()) {
      AcknowledgeEntry(navigation2d::exploration::EntryArrival::kDriving);
      return;
    }
    // Goal already in the observation disk — finish it instead of sitting on
    // CARROT/GOAL with zero command (junction hesitation).
    if (mission_.phase() == MissionPhase::kExploring && !returning_ && Pose() &&
        !retreat_.active()) {
      const double goal_dist = std::hypot(active_goal_.x - Pose()->x,
                                          active_goal_.y - Pose()->y);
      if (goal_dist < .28 && state.path_progress_m >= .05) {
        navigation_.Cancel();
        motion_output_->Send({});
        viewpoint_arrival_.Near(explorer_, entry_, tour_, active_goal_, *TourRobot(), *this);
        return;
      }
    }
    if (PathCutsMappedObstacle(/*execution_tick=*/true)) {
      RCLCPP_WARN(get_logger(),
                  "Dropping path that intersects mapped obstacles goal=(%.2f, %.2f)",
                  active_goal_.x, active_goal_.y);
      motion_output_->Send({});
      ClearPublishedNavPaths();
      if (ReplanAfterRouteCut()) return;
      if (mission_.phase() == MissionPhase::kReturning || returning_) {
        navigation_.Cancel();
        route_failures_.ClearSignatures();
        evidence_budget_.RequestStartReplay();
        HandleReturnRecovery(navigation2d::exploration::ReturnRecoveryMode::kPathFailure,
                             "return path intersected mapped obstacles");
        return;
      }
      return;
    }
    CaptureNavTelemetry(state, measured_velocity);
    goal_execution_.ObserveProgress(state.path_progress_m, *this);
    // Driving a scrape chord into a wall pocket: do not wait for monitor STOP
    // after the nose is already in the corner (live: clear=0.13, cmd still
    // 0.27 m/s into south tip).
    if (mission_.phase() == MissionPhase::kExploring && !returning_ && !retreat_.active() &&
        !entry_.pending() && !entry_.locked() && Pose()) {
      const double path_clear =
          ActivePathMargin(state.global_planning.path_min_clearance_m);
      const double goal_dist = std::hypot(active_goal_.x - Pose()->x,
                                          active_goal_.y - Pose()->y);
      const BodyRanges body = ReadBodyRanges();
      if (goal_dist < 4. * navigation2d::kSwingClearance() &&
          state.path_progress_m >= navigation2d::kPlanningRadius() &&
          IsCornerTipStall(path_clear, body.front, body.side)) {
        motion_output_->Send({});
        BreakWallScrapeThrash("driving scrape path into corner tip");
        return;
      }
    }
    if (mission_.phase() == MissionPhase::kExploring && !returning_) {
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
    const bool safety_held =
        state.safety_stopped_motion ||
        state.collision_monitor_action == navigation2d::CollisionMonitorAction::kStop ||
        state.collision_monitor_action == navigation2d::CollisionMonitorAction::kBlindZoneStop;
    std::optional<navigation2d::exploration::EntryApproach> approach;
    if (!safety_held && (entry_.pending() || entry_.locked()) && Pose() &&
        !retreat_.active() && state.status != navigation2d::NavigationStatus::kSucceeded) {
      approach = navigation2d::exploration::EntryApproach{
          *entry_.ActiveMouth(), active_goal_, *TourRobot()};
    }
    const auto velocity = navigation2d::exploration::ExecutionCommand(
        state.command, safety_held, approach, *this);
    motion_output_->Send(velocity);
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
    const double goal_distance_now = Pose()
        ? std::hypot(active_goal_.x - Pose()->x,
                     active_goal_.y - Pose()->y)
        : std::numeric_limits<double>::infinity();
    const BodyRanges body = ReadBodyRanges();
    const double front_range = body.front;
    const double side_range = body.side;
    // Parallel room wall (front open, one flank close) is normal driving, not a
    // U-tip. Pinch requires both axes inside the planning circle.
    const bool forward_clearly_open = navigation2d::AxisSwingClear(front_range);
    const bool corridor_pinch = navigation2d::HitBlocksSwing(front_range) &&
                                 navigation2d::HitBlocksSwing(side_range);
    // Include retreat_.active(): tip thrash used frontier_cells=0 retreat poses
    // that were exempt from this abort, so the robot spun forever in the U.
    const bool near_goal_alcove_crawl =
        mission_.phase() == MissionPhase::kExploring && !returning_ &&
        (goal_distance_now < navigation2d::kSwingClearance() ||
         (state.global_path_length_m > .20 &&
          state.path_progress_m >= .80 * state.global_path_length_m)) &&
        goal_execution_.ProgressOlderThan(2.5, *this) &&
        state.requested_command.linear < .06 &&
        corridor_pinch;
    if (near_goal_alcove_crawl) {
      // Unfinished local unknown (kitchen corner / red-box pocket) must not be
      // CoolRegion'd away. The old 1.8 m pocket cool + RecordAttempt(true) made
      // leftovers vanish (raw_cells>>0, completing found none) while the robot
      // fled down the hall.
      const bool local_frontier_open =
          !retreat_.active() && explorer_.GoalRegionStillFrontier(active_goal_);
      const std::size_t local_unknown = explorer_.ReachableUnknownCells(
          Pose()->x, Pose()->y);
      if (local_frontier_open || local_unknown > 80) {
        RCLCPP_WARN(get_logger(),
                    "Corner crawl stalled with local unknown=%zu frontier=%s; "
                    "retry entry instead of cooling pocket",
                    local_unknown, local_frontier_open ? "yes" : "no");
        navigation_.Cancel();
        route_failures_.ClearSignatures();
        if (local_frontier_open && PassableMouthGoal(active_goal_)) {
          LockEntryMouth(active_goal_);
          evidence_budget_.RequestTrailReplay(8.0);
          if (StartEntryProbe()) return;
        }
        // Drop only this viewpoint; keep the pocket nominatable.
        if (!retreat_.active()) explorer_.RecordAttempt(active_goal_, false);
        if (!tour_.goals().empty()) tour_.Advance();
        retreat_.Finish();
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
                  goal_execution_.Elapsed(*this),
                  retreat_.active() ? "yes" : "no");
      navigation_.Cancel();
      if (!retreat_.active()) explorer_.RecordAttempt(active_goal_, true);
      // Cool only when the tip is truly done (no local unknown left).
      explorer_.CoolRegion(active_goal_.x, active_goal_.y, 1.80, 3000u);
      explorer_.CoolRegion(active_goal_.frontier_x, active_goal_.frontier_y, 1.80, 3000u);
      explorer_.CoolRegion(Pose()->x, Pose()->y, 1.80, 3000u);
      ClearEntryMouthLock();
      retreat_.Finish();
      retreat_.AllowRetry();  // allow a short trail exit from this cell
      tour_.Suspend();
      route_failures_.ClearSignatures();
      if (EscapeOrShortRetreat("leaving alcove tip; peel",
                               "leaving alcove tip; short trail retreat"))
        return;
      next_selection_ = now();
      PublishStatus("SELECTING", "alcove tip left; pocket cooled, picking a farther goal");
      return;
    }
    if (state.status == navigation2d::NavigationStatus::kSucceeded) {
      navigation_.Cancel();
      if (mission_.phase() == MissionPhase::kReturning || returning_) {
        const Goal home = MakeHomeGoal();
        const double home_dist =
            std::hypot(Pose()->x - home.x, Pose()->y - home.y);
        if (return_.breadcrumbs() && home_dist > .40) {
          // A hop moved the robot past whatever blocked the direct route;
          // go straight home from here when the map allows, and only keep
          // retracing the trail when it still does not.
          evidence_budget_.RequestTrailReplay(80.0);
          if (StartNavigation(home, true)) {
            return_.UseBreadcrumbs(false);
            PublishStatus("RETURNING", "direct route home available after breadcrumb hop");
            return;
          }
          if (StartBreadcrumbTowardHome()) return;
        }
        return_.UseBreadcrumbs(false);
        Finish(true, return_.partial()
                         ? "returned home early; some frontiers left unreachable"
                         : "frontiers exhausted and robot returned through final-map global path");
      } else if (mission_.phase() == MissionPhase::kCompleting) {
        explorer_.RecordAttempt(active_goal_, true);
        FinishCompleting();
      } else if (retreat_.active()) {
        retreat_.Finish();
        reject_streak_ = 0;
        // After a trail retreat while a mouth is locked, resume that mouth.
        if (navigation2d::exploration::ResumeEntryAfterRetreat(entry_, explorer_, *this)) return;
        // Keep the committed tour; retreat is not a license to rebuild reverse.
        next_selection_ = now();
        PublishStatus("SELECTING", "left the pinched corridor");
      } else {
        viewpoint_arrival_.Succeeded(explorer_, entry_, tour_, active_goal_, *TourRobot(), *this);
      }
    } else if (goal_execution_.FailureDue(
                   state, mission_.phase() == MissionPhase::kReturning || returning_,
                   front_range, corridor_pinch, forward_clearly_open, *this)) {
      const bool monitor_stop =
          state.status == navigation2d::NavigationStatus::kBlocked &&
          state.collision_monitor_action ==
              navigation2d::CollisionMonitorAction::kStop;
      const bool forward_blocked_close =
          navigation2d::HitInsideBody(front_range);
      const bool exploring_drive =
          mission_.phase() == MissionPhase::kExploring && !returning_;

      // Corner-tip: both axes inside the planning circle (IsCornerTipStall).
      if (exploring_drive && monitor_stop) {
        const double path_clear =
            ActivePathMargin(state.global_planning.path_min_clearance_m);
        if (IsCornerTipStall(path_clear, front_range, side_range) &&
            goal_execution_.ProgressOlderThan(2.0, *this)) {
          BreakWallScrapeThrash("corner-tip CARROT stall");
          return;
        }
      }

      // SideSpeckStall: open forward with a lateral speck STOP.
      // Forward outside the planning circle and a close flank is a normal
      // room wall — peel/resume of a wall-foot path is the red-box sawtooth.
      if (exploring_drive && monitor_stop && forward_clearly_open) {
        navigation2d::exploration::ExecuteOpenForwardStop(
            entry_, explorer_, recovery_, goal_execution_, *this,
            {state.global_planning.path_min_clearance_m, side_range}, *this);
        return;
      }

      // Nose hit is inside the planning circle. Do not resume the same path.
      if (exploring_drive && monitor_stop && forward_blocked_close) {
        // Locked mouth: one peel, else leave and continue tour from here.
        if (entry_.locked() || entry_.pending()) {
          if (!entry_.pending()) entry_.SetPending(entry_.locked());
          retreat_.AllowRetry();
          navigation_.Cancel();
          if (EscapeOrShortRetreat("forward blocked at locked mouth; peel",
                                   "unused"))
            return;
          AbandonLockedEntryMouth(
              "forward blocked at mouth; leave and continue tour (no reverse)");
          next_selection_ = now();
          return;
        }
        explorer_.RecordAttempt(entry_.pending().value_or(active_goal_),
                                false);
        ClearEntryMouthLock();
        DemoteViewpointForSeconds(active_goal_, 8.0);
        if (!tour_.goals().empty()) tour_.Advance();
        recovery_.Clear();
        if (TryStartEscape()) {
          navigation_.Cancel();
          PublishStatus("RECOVERING",
                        "forward blocked; escape then advance tour");
          return;
        }
        navigation_.Cancel();
        next_selection_ = now();
        PublishStatus("SELECTING", "forward blocked; skipping viewpoint");
        return;
      }

      RCLCPP_WARN(get_logger(),
                  "Navigation2D frontier failed: status=%s phase=%d elapsed=%.1f replans=%d path=%.2f requested=(%.3f,%.3f) published=(%.3f,%.3f) controller_maneuver=%d intentional_stop=%s monitor_action=%d ttc=%.3f path_signature=%llu planner_error=%s",
                  state.status == navigation2d::NavigationStatus::kBlocked ? "blocked" : "timeout",
                  static_cast<int>(state.phase), goal_execution_.Elapsed(*this), state.replans,
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
          goal_execution_.ProgressOlderThan(corridor_pinch ? 5.0 : 10.0, *this);
      const bool timed_out =
          goal_execution_.TimedOut(*this);

      // Timeout: demote without peeling back onto the same goal.
      if (exploring_drive && timed_out && !first_segment_stall &&
          !mid_path_pinch) {
        DemoteActiveGoal("goal timeout; demoting viewpoint");
        return;
      }

      if (first_segment_stall || mid_path_pinch) {
        const bool force_return =
            first_segment_stall ? NoteFirstSegmentFailure(state) : false;
        if (mission_.phase() == MissionPhase::kReturning || returning_) {
          // Direct home chords often fail the first segment ~3 m short.
          // Follow the outbound trail instead of finishing away from spawn.
          navigation_.Cancel();
          route_failures_.ClearSignatures();
          const auto fallback = return_.TryTrailOrEscape(*this);
          if (fallback != navigation2d::exploration::ReturnExecutionResult::kUnavailable) {
            if (fallback == navigation2d::exploration::ReturnExecutionResult::kEscaping) {
              escape_.ResumeReturn();
            }
            return;
          }
        } else {
          // Locked entry: retry within budget; otherwise abandon the mouth so
          // the tour cannot shuttle hall→threshold again.
          if (entry_.locked() || entry_.pending()) {
            if (!entry_.pending()) entry_.SetPending(entry_.locked());
            retreat_.AllowRetry();
            navigation_.Cancel();
            // Accel-limited 0.05 m/s from rest never moves the base; replan
            // storms (live: 190× no-motion at one pose) made it worse. Open-
            // loop crawl toward the probe, then one bounded replan.
            if (state.planning_failure_reason.find("no measured motion") !=
                    std::string::npos &&
                Pose()) {
              route_failures_.ClearSignatures();
              last_planning_failure_ = state.planning_failure_reason;
              // Hand off to the !navigation_ open-loop crawler for ~40 ticks.
              if (entry_.crawl_ticks() == 0) entry_.StartCrawl();
              return;
            }
            navigation2d::exploration::ResolveLockedEntryStall(entry_, explorer_, recovery_, *this);
            return;
          }
          // Do NOT recovery_.Clear() here — that zeroed wall_scrapes every
          // tick so room-side stall forever saw scrapes=1 and never left the
          // doorway jamb (live thrash at threshold corner).
          explorer_.RecordAttempt(active_goal_, false);
          DemoteViewpointForSeconds(active_goal_, 25.0);
        }
        // Unified pinch exit: doorway jamb / corner tip / open wall foot.
        if (mission_.phase() != MissionPhase::kReturning && !returning_ &&
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
          navigation_.Cancel();
          tour_.Suspend();
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
          mission_.phase() != MissionPhase::kReturning && !returning_) {
        // Replanning the same goal drives the chassis back into the jamb.
        navigation_.Cancel();
        explorer_.RecordAttempt(active_goal_, false);
        DemoteViewpointForSeconds(active_goal_, 25.0);
        if (!tour_.goals().empty()) tour_.Advance();
        if (TryStartEscape()) return;
        next_selection_ = now();
        PublishStatus("SELECTING", "no motion against the jamb; leaving this route");
        return;
      }
      navigation_.Cancel();
      recovery_.Clear();
      if (mission_.phase() == MissionPhase::kReturning || returning_) {
        if (StartBreadcrumbTowardHome()) return;
        // Exploration itself may have covered useful space; a blocked home
        // path is a partial mission, not a hard algorithm failure for debug UI.
        return_.MarkPartial();
        Finish(true, "exploration ended; return path blocked on online map");
      } else if (mission_.phase() == MissionPhase::kCompleting) {
        if (!first_segment_stall && !mid_path_pinch)
          explorer_.RecordAttempt(active_goal_, false);
        FinishCompleting();
      } else {
        if (!tour_.goals().empty()) tour_.Advance();
        next_selection_ = now();
        PublishStatus("SELECTING", first_segment_stall ?
            "first-segment failed; skipping cooled viewpoint" :
            (mid_path_pinch ? "mid-path pinch stall; skipping viewpoint" :
                              "frontier failed; continuing committed tour"));
      }
    }
  }

  navigation2d::exploration::GoalExecutionWatch goal_execution_;
  std::uint64_t map_revision_ = 0;
  navigation2d::exploration::SelectionExhaustion selection_exhaustion_;
  int latest_map_replans_ = 0;
  int cut_replans_ = 0;
  navigation2d::exploration::ViewpointArrival viewpoint_arrival_;
  std::size_t last_progress_known_cells_ = 0;
  rclcpp::Time last_progress_time_{0, 0, RCL_ROS_TIME};
  using MissionPhase = navigation2d::exploration::MissionPhase;
  navigation2d::exploration::MissionLifecycle mission_;
  navigation2d::exploration::CompletionExecution completion_;
  navigation2d::exploration::RouteFailureHistory route_failures_;
  int reject_streak_ = 0;
  int mapped_obstacle_rejects_ = 0;
  navigation2d::exploration::DispatchPolicy dispatch_;
  bool reject_pose_valid_ = false;
  int reject_cell_x_ = 0;
  int reject_cell_y_ = 0;
  navigation2d::exploration::RetreatExecution retreat_;
  navigation2d::exploration::PlanningEvidenceBudget evidence_budget_;
  // Trail budget the active plan was painted with (0 = not painted).
  // Per cell of the active plan's snapshot: 1 where the planner was told free.
  navigation2d::exploration_io::PlanningMapAdapter planning_map_{"/tmp/sweepnav-navigation-map-"};
  navigation2d::exploration::PoseMotionWatch pose_motion_;
  int immobility_recoveries_ = 0;
  int immobility_open_replans_ = 0;
  // Soft wall-scrape rejects: rotate tour twice before RecordAttempt demote.
  navigation2d::exploration::ViewpointRejections viewpoint_rejections_;
  bool tip_rescue_attempted_ = false;
  navigation2d::exploration::RoomVisitLedger room_visits_;
  navigation2d::exploration::EscapeExecution escape_;
  int reported_recoveries_ = 0;
  std::uint64_t acados_commands_ = 0;
  std::uint64_t mppi_commands_ = 0, rpp_commands_ = 0;
  navigation2d::exploration::NavigationTelemetry telemetry_;
  std::string last_state_;
  std::string last_message_;
  std::string last_planning_failure_;
  // Most recent StartNavigation was rejected with the start in an obstacle.
  bool start_occupied_ = false;
  std::uint64_t last_path_signature_ = 0;
  double active_path_min_clearance_m_ = 0.;
  navigation2d::exploration_ros::DebugPaths debug_paths_;
  bool returning_ = false;
  Goal active_goal_;
  navigation2d::exploration::ReturnCoordinator return_;
  navigation2d::exploration::EntryCoordinator entry_;
  navigation2d::exploration::GoalRecoverySession recovery_;
  navigation2d::exploration::TourExecution tour_;
  rclcpp::Time next_selection_{0, 0, RCL_ROS_TIME};
  rclcpp::Time velocity_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_debug_publish_{0, 0, RCL_ROS_TIME};
  std::optional<nav_msgs::msg::OccupancyGrid> map_;
  std::optional<sensor_msgs::msg::LaserScan> scan_;
  navigation2d::exploration_ros::PoseFeedback pose_feedback_;
  navigation2d::Twist2d velocity_;
  navigation2d::exploration::AlgorithmSelection selected_algorithms_;
  navigation2d::FrontierExplorer explorer_;
  navigation2d::exploration::NavigationSession navigation_;
  const navigation2d::exploration_ros::ExplorerParameters parameters_;
  std::unique_ptr<navigation2d::exploration::MotionOutput> motion_output_;
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
