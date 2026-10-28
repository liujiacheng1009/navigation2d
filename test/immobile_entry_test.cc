#include <cassert>
#include <string>
#include <vector>

#include "navigation2d/application/exploration/entry_recovery_execution.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
using Events = std::vector<std::string>;
class Actions final : public EntryImmobilityActions {
 public:
  EntryCoordinator& entry;
  Events events;
  bool escape = false, probe = true;
  std::string abandoned;
  explicit Actions(EntryCoordinator& value) : entry(value) {}
  void PrepareImmobileEntry(bool replay) override {
    assert(entry.pending());
    events.emplace_back(replay ? "start-replay" : "retry");
  }
  bool EscapeOrShortRetreat(const char*, const char* retreat) override {
    assert(std::string(retreat) == "unused");
    events.emplace_back("escape");
    return escape;
  }
  bool StartEntryProbe() override {
    events.emplace_back("probe");
    return probe;
  }
  void AbandonLockedEntryMouth(const std::string& reason) override {
    abandoned = reason;
    events.emplace_back("abandon");
    entry.Clear();
  }
  void SelectAfterEntry() override { events.emplace_back("select"); }
};
ExplorationGoal Mouth(double x) {
  ExplorationGoal goal;
  goal.x = goal.frontier_x = x;
  return goal;
}
void Lock(EntryCoordinator& entry, double x) { entry.Lock(Mouth(x), false, {}); }
}  // namespace
int main() {
  FrontierExplorer explorer;
  EntryCoordinator entry;
  Actions actions(entry);
  Lock(entry, 1.501);
  ResolveImmobileEntry(entry, explorer, {0., 0., .079}, actions);
  assert(actions.events == Events({"start-replay", "escape", "abandon", "select"}));
  assert(actions.abandoned == "immobility far from mouth; leave and continue tour (no reverse)");
  assert(!entry.locked());

  actions.events.clear();
  Lock(entry, 1.501);
  actions.escape = true;
  ResolveImmobileEntry(entry, explorer, {0., 0., .079}, actions);
  assert(actions.events == Events({"start-replay", "escape"}) && entry.pending());

  actions.events.clear();
  Lock(entry, 1.5);
  actions.escape = false;
  ResolveImmobileEntry(entry, explorer, {0., 0., .079}, actions);
  assert(actions.events == Events({"retry", "escape", "probe"}));
  assert(entry.pending() && entry.locked());

  actions.events.clear();
  Lock(entry, 1.501);
  ResolveImmobileEntry(entry, explorer, {0., 0., .08}, actions);
  assert(actions.events == Events({"retry", "escape", "abandon", "select"}));
  assert(actions.abandoned == "immobility exhausted locked mouth; continue tour (no reverse)");

  actions.events.clear();
  Lock(entry, 3.);
  entry.SetPending(Mouth(.5));
  ResolveImmobileEntry(entry, explorer, {0., 0., 0.}, actions);
  assert(actions.events == Events({"retry", "escape", "probe"}));
  assert(entry.pending()->frontier_x == .5);  // Pending takes priority over lock.

  actions.events.clear();
  Lock(entry, 1.);
  const int limit = entry.AttemptBudget(explorer);
  for (int i = 0; i < limit; ++i) {
    entry.CountAttempt();
  }
  ResolveImmobileEntry(entry, explorer, {0., 0., 0.}, actions);
  assert(actions.events == Events({"retry", "escape", "abandon", "select"}));
}
