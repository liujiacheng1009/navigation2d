#include "navigation2d/application/exploration/retreat_execution.h"

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/geometry/robot_geometry.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
class Actions final : public RetreatActions {
 public:
  explicit Actions(const RetreatExecution& state) : state_(state) {}
  mutable std::vector<std::string> calls;
  bool pinched = false, accepts = true, throws = false;
  bool RetreatPinched() const override {
    calls.emplace_back("body");
    return pinched;
  }
  void ClearRetreatCooling() override { calls.emplace_back("cooling"); }
  void PrepareRetreatEvidence(bool value) override {
    assert(value == pinched && !state_.active());
    calls.emplace_back("evidence");
  }
  bool NavigateRetreat(const ExplorationGoal& goal) override {
    assert(state_.active() && goal.x < 0. && goal.y == 0.);
    assert(std::abs(goal.yaw) > 3.);
    calls.emplace_back("navigate");
    if (throws) {
      throw std::runtime_error("navigation failed");
    }
    return accepts;
  }
  void CancelRetreatEvidence() override {
    assert(!state_.active());
    calls.emplace_back("cancel");
  }

 private:
  const RetreatExecution& state_;
};
}  // namespace
int main() {
  RetreatExecution state;
  Actions actions(state);
  std::vector<std::pair<double, double>> trail{
      {-4., 0.}, {-3., 0.}, {-2., 0.}, {-1., 0.}, {0., 0.}};
  assert(!state.Start({}, {}, {1, 2}, false, actions) && actions.calls.empty());
  assert(state.Start(trail, {}, {1, 2}, false, actions));
  assert((actions.calls == std::vector<std::string>{"body", "cooling", "evidence", "navigate"}));
  assert(state.active());
  state.Finish();
  actions.calls.clear();
  assert(!state.Start(trail, {}, {1, 2}, false, actions) && actions.calls.empty());
  state.AllowRetry();
  actions.accepts = false;
  assert(!state.Start(trail, {}, {1, 2}, true, actions));
  assert(!state.active());
  assert((actions.calls ==
          std::vector<std::string>{"body", "cooling", "evidence", "navigate", "cancel"}));
  actions.calls.clear();
  assert(!state.Start(trail, {}, {1, 2}, true, actions) && actions.calls.empty());
  actions.accepts = true;
  actions.pinched = true;
  assert(state.Start(trail, {}, {2, 2}, false, actions));
  state.Finish();
  state.AllowRetry();
  actions.calls.clear();
  const std::vector<std::pair<double, double>> dense(100, {-.01, 0.});
  assert(!state.Start(dense, {}, {3, 2}, false, actions));
  assert(actions.calls == std::vector<std::string>{"body"});
  // Failure before navigation does not consume the cell latch.
  actions.calls.clear();
  assert(state.Start(trail, {}, {3, 2}, false, actions));
  state.Finish();
  state.AllowRetry();
  actions.throws = true;
  bool threw = false;
  try {
    state.Start(trail, {}, {3, 2}, false, actions);
  } catch (const std::runtime_error&) {
    threw = true;
  }
  assert(threw && state.active() && actions.calls.back() == "navigate");
  // Legacy exceptions propagate without inventing a rollback/cancel.
  const auto selected = SelectTrailRetreat(trail, {}, true, false);
  assert(selected && selected->x <= -.5 * kEntryRetreat());
  assert(!SelectTrailRetreat(dense, {}, true, true));
  std::vector<std::pair<double, double>> walked;
  for (int i = 0; i <= 16; ++i) {
    walked.emplace_back(-4. + .25 * i, 0.);
  }
  ExplorationGoal behind{};
  behind.x = -3.;
  assert(GoalBehindDrivenTrail(walked, {}, behind));
  behind.x = 3.;
  assert(!GoalBehindDrivenTrail(walked, {}, behind));
  behind.x = -.34;
  assert(!GoalBehindDrivenTrail(walked, {}, behind));
  assert(!GoalBehindDrivenTrail(dense, {}, behind));
}
