#include "navigation2d/application/exploration/viewpoint_selection.h"

#include <cassert>
#include <cstdint>
#include <vector>

#include "navigation2d/application/exploration/completion_execution.h"
#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/tour_execution.h"
#include "navigation2d/application/exploration/viewpoint_rejections.h"
#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/test/viewpoint_selection_fixture.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
ExplorationGoal GoalAt(double x) {
  ExplorationGoal goal;
  goal.x = x;
  goal.y = goal.frontier_y = 1.5;
  goal.frontier_x = 4.95;
  goal.frontier_cells = 10;
  return goal;
}
struct Scene {
  FrontierExplorer explorer;
  EntryCoordinator entry;
  TourExecution tour;
  CompletionExecution completion;
  ViewpointRejections rejections;
  ViewpointSelection selection{explorer, entry, tour, completion, rejections};
  test::ViewpointSelectionRecorder actions;
  Scene() {
    ExplorationGrid map{200, 60, .05, 0., 0., std::vector<std::int8_t>(12000, -1)};
    for (int y = 0; y < 60; ++y) {
      for (int x = 0; x < 100; ++x) {
        map.cells[y * 200 + x] = 0;
      }
    }
    explorer.UpdateMap(map);
    selection.InitializeKnown(explorer.KnownCells());
    assert(explorer.GoalRegionStillFrontier(GoalAt(3.)));
    assert(!EntryCoordinator::Passable(GoalAt(3.), explorer));
  }
  void Seed() {
    for (int i = 0; i < 5; ++i) {
      tour.RetryLater(GoalAt(3. + .1 * i));
    }
  }
};
}  // namespace
int main() {
  Scene bounded;
  bounded.Seed();
  bounded.selection.Select({3., 1.5, 0.}, bounded.actions);
  assert(bounded.actions.planned.size() == 3 && bounded.actions.rejected == 3);
  assert(bounded.tour.goals().size() == 2 && bounded.selection.blocked_cycles() == 1);
  assert(bounded.actions.prepares == 0 && bounded.actions.escapes == 0);

  Scene accepted;
  accepted.Seed();
  accepted.actions.accept = true;
  accepted.selection.Select({3., 1.5, 0.}, accepted.actions);
  assert(accepted.actions.planned.size() == 1 && accepted.actions.accepted == 1);
  assert(accepted.tour.goals().size() == 5 && accepted.selection.blocked_cycles() == 0);
  assert(accepted.actions.exhausted.events.empty());

  Scene pinched;
  pinched.Seed();
  pinched.actions.occupied = true;
  pinched.selection.Select({3., 1.5, 0.}, pinched.actions);
  assert(pinched.actions.planned.size() == 1 && pinched.actions.escapes == 1);
  pinched.selection.Select({3., 1.5, 0.}, pinched.actions);
  assert(pinched.actions.planned.size() == 2 && pinched.actions.escapes == 3);
  pinched.selection.ResetRejected();
  pinched.selection.Select({3., 1.5, 0.}, pinched.actions);
  assert(pinched.actions.planned.size() == 3 && pinched.actions.escapes == 4);
  pinched.actions.escape = true;
  pinched.selection.Select({3., 1.5, 0.}, pinched.actions);
  assert(pinched.actions.cooled == 1 && pinched.actions.prepares == 4);

  Scene reverse;
  reverse.tour.RetryLater(GoalAt(.3));
  reverse.tour.RetryLater(GoalAt(3.5));
  reverse.actions.accept = true;
  reverse.selection.Select({3., 1.5, 0.}, reverse.actions);
  assert(reverse.actions.planned.size() == 1 && reverse.actions.planned.front().x == 3.5);
  assert(reverse.tour.goals().size() == 2 && reverse.tour.goals().back().x == .3);
}
