#pragma once
#include <cassert>
#include <cstdint>
#include <utility>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "navigation2d/application/exploration/navigation_dispatch_execution.h"
#include "navigation2d/application/exploration/navigation_session.h"
namespace navigation2d::test {
class NavigationDispatchRecorder final : public exploration::NavigationDispatchRuntime {
 public:
  exploration::NavigationSession& session;
  std::string path, diagnostic;
  mutable std::vector<std::string> events;
  bool throw_prepare = false, throw_commit = false;
  int latest = 0, cuts = 0;
  NavigationDispatchRecorder(exploration::NavigationSession& value, std::string file)
      : session(value), path(std::move(file)) {}
  void BeginNavigationPreparation() override { events.emplace_back("begin"); }
  void ReportSettledGoal(const ExplorationGoal&, const ExplorationGoal&) override {
    events.emplace_back("settled");
  }
  exploration::PreparedNavigationMap PrepareNavigationMap(const ExplorationGoal&, bool) override {
    events.emplace_back("map");
    if (throw_prepare) {
      throw std::runtime_error("snapshot stream");
    }
    return {path, 0., 0., .1, false, false};
  }
  std::vector<std::uint64_t> CooledNavigationSignatures() const override {
    events.emplace_back("signatures");
    return {};
  }
  void ObservePreparedNavigation() override { events.emplace_back("observed"); }
  exploration::NavigationProbeInput ReadNavigationProbe() const override {
    events.emplace_back("probe");
    return {MakePose2d(1., 1., 0.), {}, 0.};
  }
  bool DispatchRetreating() const override {
    events.emplace_back("retreat");
    return false;
  }
  void SetDispatchTimeout(double value) override {
    assert(value >= 40. && session.stage() == exploration::SessionStage::kReady);
    events.emplace_back("timeout");
  }
  void CommitAcceptedNavigation(const ExplorationGoal&, bool, const NavigationState&) override {
    assert(session.stage() == exploration::SessionStage::kActive);
    events.emplace_back("accepted");
    if (throw_commit) {
      throw std::runtime_error("accepted goal observer failed while reporting a transient publisher exception");
    }
  }
  void RecordNavigationException(std::string_view value) override {
    diagnostic = value;
    events.emplace_back("exception");
  }
  void ResumePreparedTrailReplay() override {
    assert(!session.has_session());
    events.emplace_back("trail");
  }
  void ReportDispatchReplan(exploration::DispatchReplanNotice notice) override {
    assert(!session.has_session());
    if (notice.kind == exploration::DispatchReplanNotice::Kind::kLatestMap) {
      assert(notice.number == ++latest && notice.limit == 2);
    } else {
      assert(notice.number == ++cuts && notice.limit == 3);
    }
    events.emplace_back("replan");
  }
  void RejectPreparation(exploration::DispatchFailure failure) override {
    diagnostic = failure.reason;
    events.emplace_back("rejected");
    static_cast<void>(session.Cancel());
  }
  bool PreparedRouteCutsWall() const override {
    events.emplace_back("wall");
    return false;
  }
  void CoolRouteSignature(std::uint64_t) override { events.emplace_back("cool"); }
  void NoteMappedObstacleReject(const ExplorationGoal&) override { assert(false); }
  void ObservePreparedRoute(std::uint64_t signature, double) override {
    assert(signature != 0);
    events.emplace_back("margin");
  }
  bool PassableMouthGoal(const ExplorationGoal&) const override { return false; }
  bool RetryNavigation(const ExplorationGoal&, exploration::DispatchRetryRequest) override {
    events.emplace_back("retry");
    return false;
  }
  void ReportDispatchRetry(exploration::DispatchRetryNotice) override {
    events.emplace_back("notice");
  }
};
}  // namespace navigation2d::test
