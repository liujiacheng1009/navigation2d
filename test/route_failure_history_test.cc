#include "navigation2d/application/exploration/route_failure_history.h"

#include <cassert>
#include <type_traits>
#include <utility>
int main() {
  using navigation2d::exploration::RouteFailureHistory;
  static_assert(
      std::is_same_v<decltype(std::declval<const RouteFailureHistory&>().CooledSignatures()),
                     std::vector<std::uint64_t>>);
  RouteFailureHistory history;
  history.Cool(9);
  assert(history.NoteFailure(9) == 1 && history.size() == 1);
  history.ObserveCell({0, 0});
  assert(history.failures_at_pose() == 0 && history.size() == 0);
  assert(history.NoteFailure(0) == 1 && history.size() == 0);
  assert(history.NoteFailure(7) == 2 && history.size() == 1);
  history.ClearSignatures();
  assert(history.failures_at_pose() == 2 && history.size() == 0);
  history.Cool(8);
  history.ObserveMapGrowth(499);
  assert(history.Contains(8));
  history.ObserveMapGrowth(500);
  assert(history.size() == 0 && history.failures_at_pose() == 2);
  history.Cool(8);
  history.ObserveMapGrowth(999);
  assert(history.size() == 1);
  history.ObserveMapGrowth(1000);
  assert(history.size() == 0 && history.failures_at_pose() == 2);
  history.Cool(9);
  history.ObserveCell({0, 0});
  assert(history.size() == 1 && history.failures_at_pose() == 2);
  history.ObserveCell({1, 0});
  assert(history.size() == 0 && history.failures_at_pose() == 0);
  // A new pose does not reset the map-growth anchor.
  history.Cool(4);
  history.ObserveMapGrowth(1499);
  assert(history.size() == 1);
  history.ObserveMapGrowth(1500);
  assert(history.size() == 0);
  history.Cool(0);  // Explicit cooling retains the former set insertion API.
  assert(history.Contains(0));
  assert(history.NoteFailure(0) == 1 && history.size() == 1);
  auto snapshot = history.CooledSignatures();
  snapshot.push_back(99);
  assert(!history.Contains(99) && history.size() == 1);
  history.ClearSignatures();
  assert(snapshot.size() == 2 && history.size() == 0);
}
