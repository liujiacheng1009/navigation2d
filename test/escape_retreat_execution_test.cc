#include <cassert>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/escape_retreat_actions.h"
#include "navigation2d/application/exploration/recovery_session.h"
namespace {
using namespace navigation2d::exploration;
struct Scene final : EscapeRetreatActions {
  GoalRecoverySession recovery;
  EntryCoordinator entry;
  std::vector<std::string> events;
  bool escape = false, retreat = true, clear_entry = false, add_entry = false;
  bool reset = false, throw_escape = false, throw_status = false;
  bool TryStartEscape() override {
    events.push_back("escape");
    if (throw_escape) {
      throw std::runtime_error("escape");
    }
    if (clear_entry) {
      entry.Clear();
    }
    if (add_entry) {
      entry.SetPending(navigation2d::ExplorationGoal{});
    }
    if (reset) {
      recovery.Clear();
    }
    return escape;
  }
  bool StartRetreat() override {
    assert(!entry.pending() && !entry.locked());
    events.push_back("retreat");
    return retreat;
  }
  void PublishStatus(const std::string& phase, const std::string& message) override {
    assert(phase == (escape ? "RECOVERING" : "NAVIGATING"));
    assert(message == (escape ? "peel" : "backtrack"));
    if (escape) {
      assert(recovery.resume_after_escape);
    }
    events.push_back("status");
    if (throw_status) {
      throw std::runtime_error("status");
    }
  }
  bool Run() { return recovery.TryEscapeOrRetreat(entry, "peel", "backtrack", *this); }
};
}  // namespace
int main() {
  for (int mouth = 0; mouth < 4; ++mouth) {
    for (bool escape : {false, true}) {
      Scene scene;
      scene.escape = escape;
      if (mouth == 1) {
        scene.entry.SetPending(navigation2d::ExplorationGoal{});
      } else if (mouth >= 2) {
        scene.entry.Lock({}, false, {});
        if (mouth == 2) {
          scene.entry.SetPending(std::nullopt);
        }
      }
      const bool started = scene.Run();
      assert(started == (escape || mouth == 0));
      assert(scene.recovery.resume_after_escape == escape);
      const auto expected = escape ? std::vector<std::string>{"escape", "status"}
                            : mouth ? std::vector<std::string>{"escape"}
                                    : std::vector<std::string>{"escape", "retreat", "status"};
      assert(scene.events == expected);
    }
  }
  Scene unavailable;
  unavailable.retreat = false;
  unavailable.recovery.resume_after_escape = true;
  assert(!unavailable.Run() && unavailable.recovery.resume_after_escape);
  assert((unavailable.events == std::vector<std::string>{"escape", "retreat"}));
  Scene cleared;
  cleared.entry.Lock({}, false, {});
  cleared.clear_entry = true;
  assert(cleared.Run());
  Scene added;
  added.add_entry = true;
  assert(!added.Run() && added.events == std::vector<std::string>{"escape"});
  Scene reset;
  reset.escape = reset.reset = true;
  assert(reset.Run() && reset.recovery.resume_after_escape);
  for (bool status : {false, true}) {
    Scene exception;
    exception.escape = true;
    exception.throw_status = status;
    exception.throw_escape = !status;
    bool threw = false;
    try {
      exception.Run();
    } catch (const std::runtime_error&) {
      threw = true;
    }
    assert(threw && exception.recovery.resume_after_escape == status);
    assert(exception.events.size() == (status ? 2u : 1u));
  }
}
