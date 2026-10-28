#include <stdexcept>

#include "navigation2d/application/exploration/pose_motion_watch.h"
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("pose motion watchdog contract failed");
  }
}
constexpr std::int64_t Second = 1000000000;
}  // namespace
int main() {
  using namespace navigation2d::exploration;
  PoseMotionWatch watch;
  PoseStallObservation tip{5 * Second, Second, Second, 0., false, true, false};
  Require(!watch.Stalled(tip));
  watch.Observe(0., 0., Second);
  Require(watch.Stalled(tip));
  watch.Observe(.10, 0., 5 * Second);  // Below 0.12 m is not accumulated motion yet.
  Require(watch.Stalled(tip));
  watch.Observe(.13, 0., 5 * Second);
  Require(!watch.Stalled(tip));
  watch.Observe(.20, 0., 8 * Second);
  tip.now_ns = 9 * Second;
  Require(watch.Stalled(tip));
  watch.Debounce(9 * Second);
  Require(!watch.Stalled(tip));
  PoseMotionWatch open;
  open.Observe(0., 0., Second);
  PoseStallObservation clear{8 * Second, Second, Second, 0., false, false, true};
  Require(!open.Stalled(clear));
  clear.now_ns = 9 * Second;
  Require(open.Stalled(clear));
  PoseStallObservation entry{15 * Second, Second, 10 * Second, .2, true, false, true};
  Require(!open.Stalled(entry));  // Recent entry progress protects slow traversal.
  entry.path_progress_stamp_ns = Second;
  Require(open.Stalled(entry));
  entry.now_ns = 0;  // A backwards clock does not manufacture elapsed time.
  Require(!open.Stalled(entry));
  // A nested accepted dispatch resets this budget before the caller commits
  // its reservation. Refusal/exception must not consume that one allowance.
  PoseMotionWatch budget;
  int calls = 0;
  Require(!budget.TryOpenReplan([&] {
    ++calls;
    return false;
  }));
  Require(budget.TryOpenReplan([&] {
    ++calls;
    budget.ResetOpenReplans();
    return true;
  }));
  Require(!budget.TryOpenReplan([&] {
    ++calls;
    return true;
  }));
  Require(calls == 2);
  budget.ResetOpenReplans();
  try {
    static_cast<void>(budget.TryOpenReplan([]() -> bool { throw 42; }));
    Require(false);
  } catch (int error) {
    Require(error == 42);
  }
  Require(budget.TryOpenReplan([] { return true; }));
}
