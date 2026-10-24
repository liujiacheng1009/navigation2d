#include <cmath>
#include <stdexcept>

#include "navigation2d/application/exploration/entry_target.h"
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("entry target contract failed");
  }
}
void Near(double a, double b) { Require(std::abs(a - b) < 1e-12); }
}  // namespace
int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  // A shallow wide doorway must try the short poke before deep entry.
  const auto shallow = EntryDepthWindows(true, .05, 0);
  Near(shallow[0].minimum_m, .10);
  Near(shallow[0].maximum_m, .30);
  Near(shallow[1].minimum_m, .22);
  Near(shallow[1].maximum_m, .85);
  const auto narrow = EntryDepthWindows(false, 0., 0);
  Near(narrow[0].minimum_m, .42);
  Near(narrow[0].maximum_m, 1.30);
  const auto retry = EntryDepthWindows(false, 0., 3);
  Near(retry[0].minimum_m, .72);
  Near(retry[0].maximum_m, 1.60);
  // Measured short free nose clips the search, while retaining a 0.10 m window.
  const auto clipped = EntryDepthWindows(false, .2, 3);
  Near(clipped[0].minimum_m, .15);
  Near(clipped[0].maximum_m, .28);
  const auto boundary = EntryDepthWindows(false, .05, 0);
  Near(boundary[0].minimum_m, .42);
  // No map/valid entry produces no target and must not invent a fallback goal.
  FrontierExplorer explorer;
  ExplorationGoal mouth;
  const auto none = SelectEntryTarget(explorer, mouth, 0., 0., 0);
  Require(!none.goal);
}
