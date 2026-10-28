#include <cassert>
#include <string>
#include <vector>

#include "navigation2d/test/escape_runtime_fixture.h"
int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  using Events = std::vector<std::string>;
  EscapeExecution escape;
  GoalRecoverySession recovery;
  test::EscapeRecorder runtime(escape, recovery);
  test::EscapeMotion motion(escape, runtime.events);
  runtime.ready = false;
  assert(!escape.TryStart(runtime) && runtime.events == Events{"ready"});
  runtime.events.clear();
  assert(!escape.Tick(recovery, motion, runtime) && runtime.events.empty());
  runtime.ready = true;
  assert(!escape.TryStart(runtime));
  assert(runtime.events == Events({"ready", "now", "body", "occupied", "skipped"}));
  runtime.events.clear();
  runtime.body.front = runtime.body.side = .01;
  assert(escape.TryStart(runtime) && runtime.started.linear == -.20);
  assert(runtime.events == Events({"ready", "now", "body", "returning", "deadline", "started"}));
  runtime.events.clear();
  const auto deadline = runtime.deadline;
  assert(!escape.TryStart(runtime) && runtime.events == Events({"ready", "now"}));
  assert(runtime.deadline == deadline);
  runtime.events.clear();
  runtime.now_ns = deadline - 1;
  runtime.body.back = .01;
  assert(escape.Tick(recovery, motion, runtime));
  assert(runtime.events == Events({"now", "body", "motion", "driving"}));
  assert(motion.commands.back().linear == 0. && escape.scheduled());
  runtime.events.clear();
  runtime.now_ns = deadline;
  motion.expired = true;
  assert(escape.Tick(recovery, motion, runtime));
  assert(runtime.events == Events({"now", "motion", "context", "select"}));

  const std::vector<Events> endings{{"return"},
                                    {"entry"},
                                    {"break"},
                                    {"break", "navigate", "resumed"},
                                    {"break", "navigate", "demote"},
                                    {"demote"},
                                    {"complete"},
                                    {"select"},
                                    {"demote"}};
  for (std::size_t scene = 0; scene < endings.size(); ++scene) {
    EscapeExecution active;
    GoalRecoverySession budget;
    test::EscapeRecorder actions(active, budget);
    test::EscapeMotion output(active, actions.events);
    active.Begin({.1, .2}, 20);
    actions.now_ns = 20;
    output.expired = true;
    budget.resume_after_escape = scene <= 5 || scene == 8;
    if (scene == 0) {
      active.ResumeReturn();
    }
    actions.context.entry_active = scene == 1;
    actions.break_peel = scene == 2;
    actions.navigate = scene == 3;
    if (scene == 5) {
      actions.context.phase = MissionPhase::kReturning;
    }
    if (scene == 6) {
      actions.context.phase = MissionPhase::kCompleting;
    }
    actions.context.returning = scene == 8;
    assert(active.Tick(budget, output, actions));
    Events expected{"now", "motion", "context"};
    expected.insert(expected.end(), endings[scene].begin(), endings[scene].end());
    assert(actions.events == expected);
    assert(!active.scheduled() && !active.resume_return());
    assert(budget.resume_after_escape == (scene == 0));
    assert(output.commands.size() == 1 && output.commands.front().linear == 0.);
  }
}
