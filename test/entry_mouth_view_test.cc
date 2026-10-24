#include <cassert>
#include <type_traits>
#include <utility>

#include "navigation2d/application/exploration/entry_coordinator.h"
using namespace navigation2d;
using navigation2d::exploration::EntryCoordinator;
template <class T>
concept CanBorrowMouth = requires(T&& entry) { std::forward<T>(entry).ActiveMouth(); };
static_assert(CanBorrowMouth<const EntryCoordinator&>);
static_assert(!CanBorrowMouth<EntryCoordinator>);
static_assert(std::is_same_v<decltype(std::declval<const EntryCoordinator&>().ActiveMouth()),
                             const ExplorationGoal*>);
int main() {
  EntryCoordinator entry;
  assert(entry.ActiveMouth() == nullptr);
  ExplorationGoal first;
  first.x = 1.;
  entry.SetPending(first);
  assert(!entry.locked());
  assert(entry.ActiveMouth() == &*entry.pending());
  assert(entry.ActiveMouth()->x == 1.);
  entry.Clear();
  entry.Lock(first, true, {});
  entry.ClearPending();
  assert(entry.ActiveMouth() == &*entry.locked());
  auto second = first;
  second.x = 2.;
  entry.SetPending(second);
  assert(entry.ActiveMouth() == &*entry.pending());
  assert(entry.ActiveMouth()->x == 2.);
  entry.ClearPending();
  assert(entry.ActiveMouth()->x == 1.);
  entry.Clear();
  assert(entry.ActiveMouth() == nullptr);
}
