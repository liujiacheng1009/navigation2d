#include <cassert>
#include <functional>
#include <limits>
#include <stdexcept>

#include "navigation2d/planning/global_planner.h"
#include "navigation2d/test/algorithm_contract_fixture.h"
namespace {
using namespace navigation2d;
class LegacyPlanner final : public GlobalPlanner {
 public:
  mutable int calls = 0;
  std::function<Path()> run;
  Path Plan(const LayeredCostmap&, const Pose2d&, const Pose2d&) const override {
    ++calls;
    return run();
  }
  PathHeadingSemantics HeadingSemantics() const noexcept override {
    return PathHeadingSemantics::kForwardTangent;
  }
};
class TypedPlanner final : public GlobalPlanner {
 public:
  std::function<GlobalPlanResult(const GlobalPlanRequest&)> run;

 protected:
  GlobalPlanResult PlanRequest(const GlobalPlanRequest& request) const override {
    return run(request);
  }
};
}  // namespace
int main() {
  using namespace navigation2d;
  test::AlgorithmContractFixture fixture;
  auto map = fixture.Map();
  auto start = MakePose2d(1., 1., 0.);
  auto goal = MakePose2d(2., 1., 0.);
  GlobalPlanRequest request{map, start, goal, "fixture-map"};
  LegacyPlanner planner;
  planner.run = [&] { return Path{start, goal}; };
  auto result = planner.PlanWithContext(request);
  assert(result.succeeded() && result.path.size() == 2);
  assert(result.metadata.frame_id == "fixture-map" &&
         result.metadata.map_revision == map.revision());
  assert(result.metadata.headings == PathHeadingSemantics::kForwardTangent);
  planner.run = [] { return Path{}; };
  assert(planner.PlanWithContext(request).status == GlobalPlanStatus::kNoPath);
  planner.run = [&] {
    auto bad = goal;
    bad.translation().x() = std::numeric_limits<double>::quiet_NaN();
    return Path{start, bad};
  };
  result = planner.PlanWithContext(request);
  assert(result.status == GlobalPlanStatus::kInvalidOutput && result.path.empty());
  for (auto [text, status] : {std::pair{"start occupied", GlobalPlanStatus::kStartOccupied},
                              {"goal occupied", GlobalPlanStatus::kGoalOccupied},
                              {"no path", GlobalPlanStatus::kNoPath},
                              {"backend exploded", GlobalPlanStatus::kBackendError}}) {
    planner.run = [=]() -> Path { throw std::runtime_error(text); };
    result = planner.PlanWithContext(request);
    assert(result.status == status && result.reason == text && result.path.empty());
  }
  planner.run = []() -> Path { throw 17; };
  assert(planner.PlanWithContext(request).status == GlobalPlanStatus::kBackendError);
  const int calls = planner.calls;
  std::stop_source cancelled;
  cancelled.request_stop();
  request.budget.cancellation = cancelled.get_token();
  assert(planner.PlanWithContext(request).status == GlobalPlanStatus::kCancelled);
  request.budget = {};
  request.budget.deadline = std::chrono::steady_clock::now();
  assert(planner.PlanWithContext(request).status == GlobalPlanStatus::kBudgetExceeded);
  assert(planner.calls == calls);
  request.budget = {};
  start.translation().x() = std::numeric_limits<double>::quiet_NaN();
  assert(planner.PlanWithContext(request).status == GlobalPlanStatus::kInvalidInput);
  assert(planner.calls == calls);
  start = MakePose2d(1., 1., 0.);
  std::stop_source during;
  request.budget.cancellation = during.get_token();
  planner.run = [&] {
    during.request_stop();
    return Path{start, goal};
  };
  result = planner.PlanWithContext(request);
  assert(result.status == GlobalPlanStatus::kCancelled && result.path.empty());
  request.budget = {};
  TypedPlanner typed;
  typed.run = [&](const auto& r) {
    return GlobalPlanResult{
        GlobalPlanStatus::kSuccess,
        {start, goal},
        {std::string(r.frame_id), r.costmap.revision(), PathHeadingSemantics::kBodyHeading},
        {}};
  };
  assert(typed.PlanWithContext(request).succeeded());
  const auto valid = typed.run;
  typed.run = [&](const auto& r) {
    auto value = valid(r);
    value.metadata.frame_id = "wrong";
    return value;
  };
  assert(typed.PlanWithContext(request).status == GlobalPlanStatus::kInvalidOutput);
  typed.run = [&](const auto& r) {
    auto value = valid(r);
    ++value.metadata.map_revision;
    return value;
  };
  assert(typed.PlanWithContext(request).status == GlobalPlanStatus::kInvalidOutput);
  typed.run = [&](const auto& r) {
    auto value = valid(r);
    value.status = GlobalPlanStatus::kNoPath;
    return value;
  };
  assert(typed.PlanWithContext(request).path.empty());
}
