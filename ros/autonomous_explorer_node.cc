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
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigation2d/application/algorithm_registry.h"
#include "navigation2d/application/exploration/alcove_exit.h"
#include "navigation2d/application/exploration/completion_execution.h"
#include "navigation2d/application/exploration/continuous_tour_handoff.h"
#include "navigation2d/application/exploration/debug_overlay.h"
#include "navigation2d/application/exploration/dispatch_policy.h"
#include "navigation2d/application/exploration/entry_completion.h"
#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/entry_crawl_executor.h"
#include "navigation2d/application/exploration/entry_probe_executor.h"
#include "navigation2d/application/exploration/entry_selection.h"
#include "navigation2d/application/exploration/escape_execution.h"
#include "navigation2d/application/exploration/execution_command.h"
#include "navigation2d/application/exploration/forward_blocked.h"
#include "navigation2d/application/exploration/goal_execution_watch.h"
#include "navigation2d/application/exploration/mission_lifecycle.h"
#include "navigation2d/application/exploration/navigation_profile.h"
#include "navigation2d/application/exploration/navigation_session.h"
#include "navigation2d/application/exploration/navigation_target_record.h"
#include "navigation2d/application/exploration/navigation_telemetry.h"
#include "navigation2d/application/exploration/open_forward_stop.h"
#include "navigation2d/application/exploration/planning_evidence_budget.h"
#include "navigation2d/application/exploration/planning_snapshot.h"
#include "navigation2d/application/exploration/pose_motion_watch.h"
#include "navigation2d/application/exploration/pose_observation.h"
#include "navigation2d/application/exploration/recovery_policy.h"
#include "navigation2d/application/exploration/recovery_session.h"
#include "navigation2d/application/exploration/retreat_execution.h"
#include "navigation2d/application/exploration/return_coordinator.h"
#include "navigation2d/application/exploration/room_visit_execution.h"
#include "navigation2d/application/exploration/room_visit_ledger.h"
#include "navigation2d/application/exploration/route_failure_history.h"
#include "navigation2d/application/exploration/route_validator.h"
#include "navigation2d/application/exploration/selection_exhaustion.h"
#include "navigation2d/application/exploration/status_serializer.h"
#include "navigation2d/application/exploration/tour_execution.h"
#include "navigation2d/application/exploration/viewpoint_arrival.h"
#include "navigation2d/application/exploration/viewpoint_rejection_executor.h"
#include "navigation2d/application/exploration/viewpoint_rejections.h"
#include "navigation2d/configuration/vehicle_config.h"
#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/geometry/robot_geometry.h"
#include "navigation2d/ros/exploration/artifact_writer.h"
#include "navigation2d/ros/exploration/completion_reporter.h"
#include "navigation2d/ros/exploration/debug_reporter.h"
#include "navigation2d/ros/exploration/entry_reporter.h"
#include "navigation2d/ros/exploration/explorer_parameters.h"
#include "navigation2d/ros/exploration/explorer_subscriptions.h"
#include "navigation2d/ros/exploration/map_observation.h"
#include "navigation2d/ros/exploration/motion_output.h"
#include "navigation2d/ros/exploration/navigation_reporter.h"
#include "navigation2d/ros/exploration/planning_map_adapter.h"
#include "navigation2d/ros/exploration/scan_observation.h"
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
                                 private navigation2d::exploration::ReturnArrivalActions,
                                 private navigation2d::exploration::ContinuousHandoffActions,
                                 private navigation2d::exploration::RoomVisitActions,
                                 private navigation2d::exploration::EntryActions,
                                 private navigation2d::exploration::EntrySelectionActions,
                                 private navigation2d::exploration::EntryCrawlActions,
                                 private navigation2d::exploration::ExecutionCommandEvidence,
                                 private navigation2d::exploration::ViewpointArrivalActions,
                                 private navigation2d::exploration::RetreatActions,
                                 private navigation2d::exploration::MissionRuntime,
                                 private navigation2d::exploration::OpenForwardStopActions,
                                 private navigation2d::exploration::ForwardBlockedActions,
                                 private navigation2d::exploration::AlcoveExitActions,
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
    debug_output_ = navigation2d::exploration_ros::MakeRosDebugOutput(*this);
    subscriptions_ = navigation2d::exploration_ros::MakeExplorerSubscriptions(
        *this, parameters_.pose_source, observations_,
        [this](const nav_msgs::msg::OccupancyGrid& value) {
          explorer_.UpdateMap(navigation2d::exploration_ros::ConvertOccupancyGrid(value));
          ++map_revision_;
        },
        [this] { if (navigation_.has_session() && Pose()) FeedScan(); });
    timer_ = create_wall_timer(
        std::chrono::duration<double>(.06 / parameters_.simulation_speed), [this]() { Tick(); });
    PublishStatus("WAITING", "waiting for " + parameters_.pose_source + " pose and online map");
  }

 private:
  const std::optional<nav_msgs::msg::OccupancyGrid>& Map() const {
    return observations_.map();
  }
  const std::optional<sensor_msgs::msg::LaserScan>& Scan() const {
    return observations_.scan();
  }
  const std::optional<navigation2d::exploration::PoseObservation>& Pose() const {
    return observations_.pose();
  }

  const navigation2d::ExplorationGrid& ObservationGrid() const {
    return explorer_.observation_map();
  }

  std::optional<navigation2d::exploration::ScanEvidenceView> ScanEvidence() const {
    return navigation2d::exploration_ros::ScanEvidence(Scan() ? &*Scan() : nullptr);
  }

  std::size_t KnownCells() const override {
    return explorer_.KnownCells();
  }

  navigation2d::Pose2d LocalPose(const navigation2d::exploration::PoseObservation& pose) const {
    return navigation2d::MakePose2d(pose.x - planning_map_.origin_x(),
                                    pose.y - planning_map_.origin_y(), pose.yaw);
  }

  navigation2d::Twist2d FreshVelocity(const rclcpp::Time& reference) const {
    return observations_.FreshVelocity(reference);
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
    navigation_.UpdateCollisionScan(sensor, navigation2d::exploration_ros::CopyScan(*Scan()));
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
    dispatch_.BeginNavigationAttempt(latest_map_retry);
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
        Map()->info.resolution, observations_.trail().size());
    const auto entry = entry_.pending() ? entry_.pending() : entry_.locked();
    // Preserve the loader's existing diagnosis for stream failures.
    static_cast<void>(planning_map_.Prepare(
        ObservationGrid(),
        {.robot = navigation2d::MakePose2d(Pose()->x, Pose()->y,
                                          Pose()->yaw),
         .goal = goal,
         .entry = entry,
         .scan = ScanEvidence(),
         .trail = observations_.trail(),
         .trail_budget_m = trail_budget,
         .returning = returning,
         .map_revision = map_revision_}));
    PublishPlanningDebugMap(returning);
    bool home_goal = false;
    if (returning && mission_.home()) {
      const Goal home = MakeHomeGoal();
      home_goal = std::hypot(goal.x - home.x, goal.y - home.y) <= Map()->info.resolution;
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
        {.resolution = Map()->info.resolution,
         .entry = entry_drive,
         .pinch_retry = pinch_retry,
         .returning = returning,
         .home_goal = home_goal},
        parameters_.navigation_profiles.For(purpose));
    try {
      navigation_.Begin(
          config, planning_map_.path(),
          navigation2d::MakePose2d(goal.x - planning_map_.origin_x(),
                                   goal.y - planning_map_.origin_y(), goal.yaw),
          route_failures_.CooledSignatures(),
          {.heading_policy = parameters_.navigation_profiles.For(purpose).heading_policy});
      telemetry_.BeginPreparation();
      // Do not commit target/returning/recovery until the plan is
      // accepted. Early assign left ghost GOALs outside painted free and kept
      // recovery sessions alive after open-area plan rejects (stuck mid-room).
      if (Scan()) FeedScan();
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
      navigation_target_.Accept(navigation_, goal, parameters_.navigation_profiles.For(purpose));
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
    if (!Map() || !Pose()) return;
    (void)returning;
    debug_reporter_.PlanningMap(*debug_output_, *Map(), LocalPose(*Pose()),
                                [this] { return now(); });
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

  void PublishFrontierDebug() { debug_reporter_.Frontiers(*debug_output_, explorer_); }

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
    Goal mouth = entry_.pending().value_or(entry_.locked().value_or(ActiveGoal()));
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
    if (!std::isfinite(ActiveGoal().x) || !PassableMouthGoal(ActiveGoal())) {
      return false;
    }
    if (entry_.locked() && !SameEntryMouth(*entry_.locked(), ActiveGoal())) {
      return false;
    }
    RCLCPP_WARN(get_logger(), "Passable-mouth promote: %s gap=%.2f", reason,
                explorer_.MouthGapWidth(ActiveGoal().frontier_x, ActiveGoal().frontier_y));
    LockEntryMouth(ActiveGoal());
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
    return entry_.Satisfied(Pose()->x, Pose()->y, ActiveGoal(), explorer_);
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
    const double resolution = Map() ? Map()->info.resolution : .05;
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
  void ReportUnfinishedPocket(std::size_t unknown, bool frontier) override {
    RCLCPP_WARN(get_logger(),
                "Corner crawl stalled with local unknown=%zu frontier=%s; "
                "retry entry instead of cooling pocket",
                unknown, frontier ? "yes" : "no");
  }
  void ReportExhaustedPocket(const navigation2d::exploration::AlcoveObservation& observation) override {
    RCLCPP_WARN(get_logger(),
                "Leaving near-goal alcove tip: dist=%.2f front=%.2f side=%.2f "
                "progress=%.2f/%.2f elapsed=%.1f retreat=%s",
                observation.goal_distance_m, observation.front_range_m, observation.side_range_m,
                observation.path_progress_m, observation.path_length_m,
                goal_execution_.Elapsed(*this), retreat_.active() ? "yes" : "no");
  }

  bool TryStartEscape() override {
    if (!Scan() || !Pose() || escape_.Busy(now().nanoseconds())) return false;
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

  bool TickEscape() override {
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
    using navigation2d::exploration::EscapeContinuation;
    const auto continuation = escape_.AfterStop(
        recovery_, {mission_.phase(), returning_, entry_.locked() || entry_.pending()});
    switch (continuation) {
      case EscapeContinuation::kReturn:
        BeginReturn(true);
        break;
      case EscapeContinuation::kEntry:
        navigation2d::exploration::ResumeLockedEntryAfterPeel(entry_, explorer_, *this);
        break;
      case EscapeContinuation::kResumeGoal:
        // A peeled wall scrape cannot resume its banned chord.
        if (BreakWallScrapeAfterPeel()) {
          break;
        }
        if (StartNavigation(ActiveGoal(), false)) {
          PublishStatus("NAVIGATING", "resumed goal after open-space peel");
          break;
        }
        [[fallthrough]];
      case EscapeContinuation::kDemoteGoal:
        DemoteActiveGoal("resume after peel failed; demoting viewpoint");
        break;
      case EscapeContinuation::kComplete:
        FinishCompleting();
        break;
      case EscapeContinuation::kSelect:
        next_selection_ = now();
        break;
    }
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
    if (!Scan() || !Pose()) return false;
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
  bool EscapeOrShortRetreat(const char* escape_msg, const char* retreat_msg) override {
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
        std::isfinite(ActiveGoal().x) && std::isfinite(ActiveGoal().y)) {
      explorer_.RecordAttempt(entry_.pending().value_or(ActiveGoal()), false);
      // Short cool — 20 s kept re-queuing the same tip while kitchen waited.
      DemoteViewpointForSeconds(entry_.pending().value_or(ActiveGoal()), 6.0);
      explorer_.CoolRegion(ActiveGoal().x, ActiveGoal().y, 0.70, 1500u);
      explorer_.CoolRegion(ActiveGoal().frontier_x, ActiveGoal().frontier_y, 0.70, 1500u);
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
    dispatch_.ResetLatestMapReplans();
    const int scrapes = recovery_.wall_scrapes;
    if (StartNavigation(ActiveGoal(), false, true)) {
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
        observations_.trail(), *TourRobot(), goal);
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
    if (observations_.trail().size() < 4 || !Pose()) return false;
    return retreat_.Start(observations_.trail(), *TourRobot(), PoseCell(),
                          entry_.pending() || entry_.locked(), *this);
  }

  void SelectNextGoal() override {
    if (mission_.phase() == MissionPhase::kReturning || returning_) {
      BeginReturn(return_.partial());
      return;
    }
    if ((entry_.pending() || entry_.locked()) &&
        navigation2d::exploration::SelectLockedEntry(entry_, explorer_, TourRobot(), ActiveGoal(),
                                                     *this) ==
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
    return navigation2d::exploration::TryContinuousTourHandoff(tour_, explorer_, navigation_,
                                                               ActiveGoal(), *TourRobot(), *this);
  }
  bool NavigateHandoff(const Goal& goal) override { return StartNavigation(goal, false); }
  void HandoffAccepted() override {
    PublishStatus("NAVIGATING", "continuous handoff to next frontier viewpoint");
  }

  bool MaybePreemptForNearerMouth(double /*path_remaining_m*/) {
    // Mid-drive preempt yanked Navigation2D onto another mouth and caused
    // goal flicker / crossed paths — wasted meters. Region-ordered tours
    // already finish the active room; keep the committed transit.
    return false;
  }

  Goal MakeHomeGoal() const {
    return return_.Home(Map() ? &ObservationGrid() : nullptr,
        {mission_.home()->x, mission_.home()->y, mission_.home()->yaw});
  }

  bool StartBreadcrumbTowardHome() override {
    if (!mission_.home() || !Pose() || observations_.trail().size() < 4) return false;
    const Goal home = MakeHomeGoal();
    const double rx = Pose()->x;
    const double ry = Pose()->y;
    const double home_dist = std::hypot(rx - home.x, ry - home.y);
    const auto next = return_.NextStep(observations_.trail(), home, rx, ry, ActiveGoal());
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
    return navigation2d::exploration::TryUnenteredRoomVisits(
        room_visits_, explorer_, observations_.trail(), Pose()->x, Pose()->y, *this);
  }
  void PrepareRoomVisit() override {
    mission_.ResumeExploring();
    tour_.Suspend();
  }
  bool NavigateRoomVisit(const Goal& goal) override { return StartNavigation(goal, false); }
  void ReportRoomVisit(const Goal& goal, int attempt, bool accepted) override {
    if (accepted) {
      RCLCPP_INFO(get_logger(),
                  "Visiting unentered space before return: goal=(%.2f, %.2f) area=%.2f m2 try=%d/%d",
                  goal.x, goal.y, goal.information_gain, attempt,
                  navigation2d::exploration::RoomVisitLedger::kVisitAttempts);
      PublishStatus("NAVIGATING", "visiting a room not yet entered");
    } else {
      RCLCPP_WARN(get_logger(), "Unentered space goal unplannable at (%.2f, %.2f)",
                  goal.x, goal.y);
    }
  }
  bool PlanDirectAfterHop(const Goal& home) override {
    evidence_budget_.RequestTrailReplay(80.0);
    return StartNavigation(home, true);
  }
  void DirectHomeAfterHopAccepted() override {
    PublishStatus("RETURNING", "direct route home available after breadcrumb hop");
  }
  void FinishReturn(bool partial) override {
    Finish(true, partial ? "returned home early; some frontiers left unreachable"
                         : "frontiers exhausted and robot returned through final-map global path");
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
    return dispatch_.ReplanOnLatestMap([&](auto attempt) {
      navigation_.Cancel();
      RCLCPP_WARN(get_logger(),
                  "%s; replanning active %s goal on latest online map (%d/%d)",
                  reason, returning_ ? "return" : "frontier", attempt.number, attempt.limit);
      return StartNavigation(ActiveGoal(), returning_, true);
    });
  }

  // A route cut while driving means the map changed, not that the goal is
  // bad: plan the same goal again on the newest map, painted the way the cut
  // route was, before any failure bookkeeping (reject counts, signature
  // cooling, breadcrumbs). The new snapshot contains the new obstacle, so the
  // planner routes around it or reports no route.
  bool ReplanAfterRouteCut() {
    return dispatch_.ReplanAfterRouteCut([&](auto attempt) {
      navigation_.Cancel();
      if (planning_map_.snapshot().trail_budget_m > 0.) {
        evidence_budget_.ResumeTrailReplay();
      }
      RCLCPP_WARN(get_logger(),
                  "Route cut by newly mapped obstacle; replanning same %s goal on latest map (%d/%d)",
                  returning_ ? "return" : "frontier", attempt.number, attempt.limit);
      return StartNavigation(ActiveGoal(), returning_, true);
    });
  }

  const Goal& ActiveGoal() const { return navigation_target_.legacy_goal(); }

  void ClearDisplayedGoal() { navigation_target_.Clear(); }

  void DemoteActiveGoal(const char* reason) {
    if (mission_.phase() != MissionPhase::kReturning && !returning_) {
      explorer_.RecordAttempt(entry_.pending().value_or(ActiveGoal()), false);
      DemoteViewpointForSeconds(entry_.pending().value_or(ActiveGoal()), 8.0);
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
    const Goal tip = ActiveGoal();
    explorer_.RecordAttempt(tip, false);
    DemoteViewpointForSeconds(tip, 10.0);
    explorer_.CoolRegion(tip.x, tip.y, 0.80, 1800u);
    explorer_.CoolRegion(tip.frontier_x, tip.frontier_y, 0.80, 1800u);
    ClearEntryMouthLock();
    navigation_.Cancel();
    recovery_.Clear();
    tour_.AfterWallTip(explorer_, tip, TourRobot(), *this);
    route_failures_.ClearSignatures();
    dispatch_.ResetLatestMapReplans();
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
    NoteMappedObstacleReject(ActiveGoal());
    navigation_.Cancel();
    // Locked mouth owns the junction. Live lidar often thickens jamb speckles
    // after the plan was accepted; cooling the throat signature then burning
    // deeper probes produced start→cut→start in <200 ms and room_entered=0.
    // Replan the SAME probe on the latest opened grid first; only deepen when
    // that fails. Do not cool entry chords — the doorway path is the mission.
    if (entry_.locked() || entry_.pending()) {
      if (!entry_.pending()) entry_.SetPending(entry_.locked());
      dispatch_.ResetLatestMapReplans();
      if (!dispatch_.HasRouteCutReplans() &&
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
    if (!dispatch_.HasRouteCutReplans() &&
        ReplanActiveGoalOnLatestMap("path cut mapped obstacle; replan once"))
      return true;
    DemoteActiveGoal("path intersected mapped obstacles");
    if (BreakMappedObstacleThrash()) return true;
    return true;
  }

  void NotePoseMotion() override {
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
      if (immobility_open_replans_ < 1 && StartNavigation(ActiveGoal(), false, true)) {
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
    explorer_.RecordAttempt(ActiveGoal(), false);
    DemoteViewpointForSeconds(ActiveGoal(), 6.0);
    explorer_.CoolRegion(ActiveGoal().x, ActiveGoal().y, 0.80, 1800u);
    explorer_.CoolRegion(ActiveGoal().frontier_x, ActiveGoal().frontier_y, 0.80, 1800u);
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

  void DemoteViewpointForSeconds(const Goal& goal, double seconds) override {
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

  void Finish(bool success, const std::string& message) override {
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
         .global_planner = navigation_target_.algorithms().global,
         .local_controller = navigation_target_.algorithms().local,
         .mppi_commands = telemetry_.command_counts().mppi,
         .rpp_commands = telemetry_.command_counts().rpp,
         .total_distance_m = observations_.distance_m(),
         .known_cells = KnownCells(),
         .completed_goals = explorer_.completed_goals(),
         .failed_goals = explorer_.failed_goals(),
         .return_error = return_error},
        Map() ? &ObservationGrid() : nullptr, observations_.trail());
    PublishStatus(!success ? "FAILED" : (return_.partial() ? "PARTIAL" : "COMPLETE"), final_message);
    // Leave the process so the debug UI can Start a new run without a stuck
    // finished node continuing to republish the old latched status.
    rclcpp::shutdown();
  }

  void PublishStatus(const std::string& state, const std::string& message) override {
    last_state_ = state;
    last_message_ = message;
    std_msgs::msg::String status;
    status.data = navigation2d::exploration::SerializeExplorationStatus(
        {.state = state,
         .message = message,
         .phase = mission_.phase() == MissionPhase::kCompleting  ? "completing"
                  : mission_.phase() == MissionPhase::kReturning ? "returning"
                                                                 : "exploring",
         .known_cells = KnownCells(),
         .completed_goals = explorer_.completed_goals(),
         .failed_goals = explorer_.failed_goals(),
         .raw_frontier_cells = explorer_.raw_frontier_cells(),
         .unresolved_frontier_cells = explorer_.unresolved_frontier_cells(),
         .candidates = explorer_.candidate_frontier_goals(),
         .tour = tour_.goals(),
         .active_goal = ActiveGoal(),
         .navigation = telemetry_.state(),
         .planning_failure = last_planning_failure_,
         .path_min_clearance_m = active_path_min_clearance_m_,
         .carrot_x = debug_reporter_.cursor().x(),
         .carrot_y = debug_reporter_.cursor().y(),
         .carrot_yaw = debug_reporter_.cursor().yaw(),
         .carrot_valid = debug_reporter_.cursor().valid(),
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
      LockEntryMouth(ActiveGoal());
      evidence_budget_.RequestTrailReplay(8.0);
      if (StartEntryProbe()) return true;
      explorer_.RecordAttempt(ActiveGoal(), false);
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
      explorer_.CoolRegion(ActiveGoal().x, ActiveGoal().y, 1.80, 3000u);
      explorer_.CoolRegion(ActiveGoal().frontier_x, ActiveGoal().frontier_y, 1.80, 3000u);
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
    if (!navigation_.has_session()) return false;
    return navigation2d::exploration_ros::ObservedRouteCutsWall(
        Map() ? &*Map() : nullptr, navigation_.path(), planning_map_.origin_x(), planning_map_.origin_y(),
        planning_map_.snapshot(), Pose(), ScanEvidence(), execution_tick);
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
        Map() ? &*Map() : nullptr, x0, y0, x1, y1);
  }

  void ClearPublishedNavPaths() {
    debug_reporter_.ClearPaths(*debug_output_, [this] { return now(); });
  }

  void PublishDebugPaths(const navigation2d::NavigationState& state) {
    if (!navigation_.has_session() || !Pose()) {
      ClearPublishedNavPaths();
      return;
    }
    debug_reporter_.Paths(
        *debug_output_,
        {navigation_.path(), planning_map_.origin_x(), planning_map_.origin_y(),
         state.path_progress_m},
        {Map() ? &*Map() : nullptr, Pose()->x, Pose()->y, ActiveGoal(), tour_.goals()},
        [this] { return now(); });
    PublishFrontierDebug();
  }

  void Tick() {
    mission_.Tick(parameters_.max_duration_s, *motion_output_, *this);
  }

  std::optional<navigation2d::exploration::MissionHome> ReadyMissionHome() const override {
    if (!Map() || !Pose() || !Scan() || KnownCells() < 100) {
      return std::nullopt;
    }
    const auto& pose = *Pose();
    return navigation2d::exploration::MissionHome{pose.x, pose.y, pose.yaw};
  }

  void ResetMissionRecovery() override {
    return_.ResetEscapes();
    escape_.ClearReturnIntent();
  }

  void InitializeMissionProgress(std::size_t known) override {
    selection_exhaustion_.InitializeKnown(known);
    last_progress_known_cells_ = known;
    last_progress_time_ = now();
  }

  void MissionBootstrapScheduled(std::int64_t deadline_ns) override {
    next_selection_ = rclcpp::Time(deadline_ns, RCL_ROS_TIME);
    PublishStatus("WAITING", parameters_.pose_source + " pose ready; waiting for a stable online map");
  }

  bool TickIdleNavigation() override {
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
        return true;
      }
    } else if (!navigation_.has_session()) {
      // Continue open-loop doorway crawl across ticks (navigation_ was reset
      // when CommandStalled fired; without this the crawl lasted one frame).
      if (navigation2d::exploration::HasActiveEntryCrawl(entry_) && Pose()) {
        navigation2d::exploration::ExecuteEntryCrawl(entry_, explorer_, ActiveGoal(), *TourRobot(),
                                                     *this);
        return true;
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
      return true;
    }
    return false;
  }

  void TickNavigation() override {
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
      const double goal_dist = std::hypot(ActiveGoal().x - Pose()->x, ActiveGoal().y - Pose()->y);
      if (goal_dist < .28 && state.path_progress_m >= .05) {
        navigation_.Cancel();
        motion_output_->Send({});
        viewpoint_arrival_.Near(explorer_, entry_, tour_, ActiveGoal(), *TourRobot(), *this);
        return;
      }
    }
    if (PathCutsMappedObstacle(/*execution_tick=*/true)) {
      RCLCPP_WARN(get_logger(), "Dropping path that intersects mapped obstacles goal=(%.2f, %.2f)",
                  ActiveGoal().x, ActiveGoal().y);
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
      const double goal_dist = std::hypot(ActiveGoal().x - Pose()->x, ActiveGoal().y - Pose()->y);
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
    telemetry_.NoteCommandBackend(state.controller_diagnostics.backend);
    const bool safety_held =
        state.safety_stopped_motion ||
        state.collision_monitor_action == navigation2d::CollisionMonitorAction::kStop ||
        state.collision_monitor_action == navigation2d::CollisionMonitorAction::kBlindZoneStop;
    std::optional<navigation2d::exploration::EntryApproach> approach;
    if (!safety_held && (entry_.pending() || entry_.locked()) && Pose() &&
        !retreat_.active() && state.status != navigation2d::NavigationStatus::kSucceeded) {
      approach = navigation2d::exploration::EntryApproach{*entry_.ActiveMouth(), ActiveGoal(),
                                                          *TourRobot()};
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
    if (telemetry_.TakeRecoveryNotice(state.recoveries)) {
      navigation2d::exploration_ros::ReportRouteProgressRecovery(get_logger(), state, measured_velocity);
    }
    // Dead-end tip crawl: GOAL sits in a U-alcove against the wall. The exit
    // is open sideways/behind, but RPP keeps probing into the wall. Drop the
    // terminal cell (lidar already saw the pocket), then turn/brief-reverse
    // out along the driven trail instead of kissing the wall until timeout.
    const double goal_distance_now =
        Pose() ? std::hypot(ActiveGoal().x - Pose()->x, ActiveGoal().y - Pose()->y)
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
      navigation2d::exploration::ExecuteAlcoveExit(
          entry_, explorer_, tour_, retreat_, route_failures_, ActiveGoal(),
          {*TourRobot(), goal_distance_now, front_range, side_range, state.path_progress_m,
           state.global_path_length_m},
          *this);
      return;
    }
    if (state.status == navigation2d::NavigationStatus::kSucceeded) {
      navigation_.Cancel();
      if (mission_.phase() == MissionPhase::kReturning || returning_) {
        return_.CompleteArrival(MakeHomeGoal(), Pose()->x, Pose()->y, *this);
        return;
      } else if (mission_.phase() == MissionPhase::kCompleting) {
        explorer_.RecordAttempt(ActiveGoal(), true);
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
        viewpoint_arrival_.Succeeded(explorer_, entry_, tour_, ActiveGoal(), *TourRobot(), *this);
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
        navigation2d::exploration::ExecuteForwardBlocked(entry_, explorer_, tour_, recovery_,
                                                         retreat_, ActiveGoal(), *this);
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
          explorer_.RecordAttempt(ActiveGoal(), false);
          DemoteViewpointForSeconds(ActiveGoal(), 25.0);
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
        explorer_.RecordAttempt(ActiveGoal(), false);
        DemoteViewpointForSeconds(ActiveGoal(), 25.0);
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
        if (!first_segment_stall && !mid_path_pinch) explorer_.RecordAttempt(ActiveGoal(), false);
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
  navigation2d::exploration::ViewpointArrival viewpoint_arrival_;
  std::size_t last_progress_known_cells_ = 0;
  rclcpp::Time last_progress_time_{0, 0, RCL_ROS_TIME};
  using MissionPhase = navigation2d::exploration::MissionPhase;
  navigation2d::exploration::ExplorationMission mission_;
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
  navigation2d::exploration::NavigationTelemetry telemetry_;
  std::string last_state_;
  std::string last_message_;
  std::string last_planning_failure_;
  // Most recent StartNavigation was rejected with the start in an obstacle.
  bool start_occupied_ = false;
  std::uint64_t last_path_signature_ = 0;
  double active_path_min_clearance_m_ = 0.;
  navigation2d::exploration_ros::DebugReporter debug_reporter_;
  bool returning_ = false;
  navigation2d::exploration::NavigationTargetRecord navigation_target_;
  navigation2d::exploration::ReturnCoordinator return_;
  navigation2d::exploration::EntryCoordinator entry_;
  navigation2d::exploration::GoalRecoverySession recovery_;
  navigation2d::exploration::TourExecution tour_;
  rclcpp::Time next_selection_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_debug_publish_{0, 0, RCL_ROS_TIME};
  navigation2d::exploration_ros::ExplorerObservations observations_;
  navigation2d::FrontierExplorer explorer_;
  navigation2d::exploration::NavigationSession navigation_;
  const navigation2d::exploration_ros::ExplorerParameters parameters_;
  std::unique_ptr<navigation2d::exploration::MotionOutput> motion_output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  std::unique_ptr<navigation2d::exploration_ros::DebugReportSink> debug_output_;
  std::unique_ptr<navigation2d::exploration_ros::ExplorerSubscriptions> subscriptions_;
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
