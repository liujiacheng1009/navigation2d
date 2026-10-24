#include "navigation2d/application/exploration/execution_command.h"

#include <cassert>
#include <cmath>
#include <vector>

#include "navigation2d/geometry/robot_geometry.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
class Evidence final : public ExecutionCommandEvidence {
 public:
  mutable std::vector<int> calls;
  double first_front = 1., last_front = 1.;
  bool blocked = false;
  double CommandFrontRange() const override {
    const bool first = calls.empty();
    calls.push_back(1);
    return first ? first_front : last_front;
  }
  bool CommandSegmentCutsWall(double, double, const ExplorationGoal&) const override {
    calls.push_back(2);
    return blocked;
  }
};
}  // namespace
int main() {
  EntryApproach approach{};
  approach.mouth.frontier_x = .5;
  approach.target.x = 1.;
  Evidence evidence;
  auto run = [&](Twist2d command, bool held = false) {
    evidence.calls.clear();
    return ExecutionCommand(command, held, approach, evidence);
  };
  auto command = run({.05, 0.});
  assert(command.linear == .16 && command.angular == 0.);
  assert((evidence.calls == std::vector<int>{1, 2, 1}));
  // A newly observed close hit still suppresses the compensated forward push.
  evidence.last_front = .1;
  command = run({.05, 0.});
  assert(command.linear == 0. && command.angular == 0.);
  evidence.last_front = 1.;
  approach.robot.yaw = 1.;
  command = run({.05, 0.});
  assert(command.linear == 0. && command.angular == -.9);
  assert((evidence.calls == std::vector<int>{1, 2}));
  approach.robot.yaw = .5;
  command = run({.05, 0.});
  assert(command.linear == .16 && command.angular == -.7);
  approach.robot.yaw = 0.;
  evidence.blocked = true;
  command = run({.05, .1});
  assert(command.linear == .05 && command.angular == .1);
  evidence.blocked = false;
  command = run({.05, .1}, true);
  assert(command.linear == .05 && command.angular == .1);
  assert(evidence.calls == std::vector<int>{1});
  command = run({0., .2}, true);
  assert(command.linear == 0. && evidence.calls.empty());
  command = run({.12, 0.});
  assert(command.linear == .12 && (evidence.calls == std::vector<int>{1, 1}));
  approach.mouth.frontier_x = 1.2;
  command = run({.05, 0.});
  assert(command.linear == .05 && (evidence.calls == std::vector<int>{1, 1}));
  approach.mouth.frontier_x = .5;
  approach.target.x = kPlanningRadius();
  command = run({.05, 0.});
  assert(command.linear == .05 && (evidence.calls == std::vector<int>{1, 1}));
  evidence.calls.clear();
  evidence.first_front = .1;
  command = ExecutionCommand({.2, .149}, false, std::nullopt, evidence);
  assert(command.linear == 0. && command.angular == 0.);
  evidence.calls.clear();
  command = ExecutionCommand({.2, .15}, false, std::nullopt, evidence);
  assert(command.linear == 0. && command.angular == .15);
  evidence.calls.clear();
  command = ExecutionCommand({-.2, 0.}, false, std::nullopt, evidence);
  assert(command.linear == -.2 && evidence.calls.empty());
}
