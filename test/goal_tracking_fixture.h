#pragma once
#include <utility>

#include "navigation2d/application/exploration/goal_tracking_actions.h"
#include "navigation2d/application/exploration/motion_output.h"
#include "navigation2d/application/exploration/navigation_session.h"
#include "navigation2d/application/exploration/navigation_telemetry.h"
#include "navigation2d/application/exploration/route_failure_history.h"
#include "navigation2d/test/goal_failure_fixture.h"
#include "navigation2d/test/navigation_plugin_fixture.h"
namespace navigation2d::test {
struct TrackingScene : FailureSceneBase<GoalTrackingActions>, MotionOutput {
  NavigationPluginFixture plugin;
  NavigationSession navigation{plugin.Registry()};
  NavigationTelemetry telemetry;
  ViewpointArrival arrival;
  RouteFailureHistory routes;
  Pose2d local_pose = MakePose2d(1., 1., 0.);
  bool handoff = false, immobile = false, cut = false, cut_replan = false;
  std::vector<Twist2d> commands;
  std::function<void(const ExplorationGoal&, const NavigationState&)> on_accepted_report;
  TrackingScene() {
    goal.x = goal.frontier_x = 2.;
    // Use the exploration tolerance for the ordinary path-progress stages.
    plugin.config.goal_xy_tolerance = .18;
    plugin.calls.control = [](const auto&) {
      ControllerDiagnostics diagnostics;
      diagnostics.backend = ControllerBackend::kRpp;
      return LocalControlResult{.status = LocalControlStatus::kSuccess,
                                .command = {.12, 0.},
                                .reason = {},
                                .diagnostics = diagnostics};
    };
    PrepareGoal(2.);
  }
  void PrepareGoal(double x) {
    goal.x = goal.frontier_x = x;
    navigation.Begin(plugin.config, plugin.map_path, MakePose2d(x, 1., 0.), {});
    auto initial = navigation.Probe(local_pose, {}, 1.);
    assert(initial.has_route());
    state = std::move(initial.state);
    navigation.Accept();
  }
  void ReportAcceptedNavigation(const ExplorationGoal& accepted,
                                const NavigationState& initial) override {
    events.emplace_back("accepted-report");
    if (on_accepted_report) {
      on_accepted_report(accepted, initial);
    }
  }
  void RunTrack() { execution.Track(navigation, telemetry, arrival, routes, *this, *this, *this); }
  void TickIdleReturn() override { events.emplace_back("return-idle"); }
  std::int64_t SelectionDeadlineNs(double seconds) const override {
    events.emplace_back("deadline");
    return ExecutionNowNs() + static_cast<std::int64_t>(seconds * 1e9);
  }
  void SelectNextGoal() override { events.emplace_back("select-next"); }
  bool CrawlSegmentCutsWall(double, double, const ExplorationGoal&) const override { return false; }
  void PublishEntryCrawl(Twist2d command) override {
    events.emplace_back("crawl-publish");
    Send(command);
  }
  bool HandoffToNextTourGoal() override {
    events.emplace_back("handoff");
    return handoff;
  }
  GoalStepInput ReadStepInput() const override {
    events.emplace_back("input");
    return {local_pose, {}, ExecutionNowNs() / 1e9};
  }
  bool PoseImmobilityWatchdog(const NavigationState&) override {
    events.emplace_back("immobility");
    return immobile;
  }
  bool PathCutsMappedObstacle(bool tick) const override {
    assert(tick);
    events.emplace_back("route-check");
    return cut;
  }
  void ReportCutNavigationRoute() override { events.emplace_back("cut-report"); }
  void ClearPublishedNavPaths() override { events.emplace_back("clear-paths"); }
  bool ReplanAfterRouteCut() override {
    events.emplace_back("cut-replan");
    return cut_replan;
  }
  void HandleReturnRecovery(ReturnRecoveryMode mode, const char* message) override {
    assert(mode == ReturnRecoveryMode::kPathFailure);
    assert(std::string(message) == "return path intersected mapped obstacles");
    events.emplace_back("return-recovery");
  }
  BodyRanges ReadBodyRanges() const override {
    events.emplace_back("body");
    return {1., 1., 1., 1., 1.};
  }
  void CaptureNavTelemetry(const NavigationState& value, const Twist2d& measured) override {
    events.emplace_back("capture");
    telemetry.Capture(value, measured);
  }
  void ReportExecutionCycle(const NavigationState&, Twist2d) override {
    events.emplace_back("report-cycle");
  }
  void CompleteReturnNavigation() override {
    assert(!navigation.has_session());
    events.emplace_back("return-complete");
  }
  void CompleteRetreatNavigation() override { events.emplace_back("retreat-complete"); }
  void AcknowledgeEntry(EntryArrival) override {
    events.emplace_back("ack");
    entry.Clear();
  }
  void ClearEntryMouthLock() override {
    events.emplace_back("clear-entry");
    entry.Clear();
  }
  void QueueActiveRegionFinish() override { events.emplace_back("queue-region"); }
  void ReportViewpointArrival(ViewpointArrivalNotice, const ExplorationGoal&, int,
                              double) override {
    events.emplace_back("arrival-report");
  }
  bool PassableMouthGoal(const ExplorationGoal&) const override { return false; }
  void ReportUnfinishedPocket(std::size_t, bool) override { events.emplace_back("unfinished"); }
  void ReportExhaustedPocket(const AlcoveObservation&) override {
    events.emplace_back("exhausted");
  }
  double CommandFrontRange() const override {
    events.emplace_back("command-front");
    return 1.;
  }
  bool CommandSegmentCutsWall(double, double, const ExplorationGoal&) const override {
    return false;
  }
  void Send(Twist2d command) override {
    events.emplace_back("motion");
    commands.push_back(command);
  }
};
}  // namespace navigation2d::test
