#include "navigation2d/exploration/exploration_policy.h"

#include <cassert>

int main() {
  using navigation2d::AdmitExploringGoal;
  using navigation2d::ExploringGoalFacts;
  using navigation2d::RoomStillOpen;
  using navigation2d::ShouldCoverResiduals;
  using navigation2d::TipLedger;

  ExploringGoalFacts speck;
  speck.frontier_cells = 1;
  speck.information_gain = 0.01;
  speck.cell_area = 0.01;
  assert(!AdmitExploringGoal(speck, 6));

  ExploringGoalFacts thin = speck;
  thin.frontier_cells = 9;
  thin.information_gain = 0.09;
  assert(!AdmitExploringGoal(thin, 6));

  ExploringGoalFacts room = thin;
  room.information_gain = 1.2;
  assert(AdmitExploringGoal(room, 6));

  ExploringGoalFacts door = speck;
  door.passable_mouth = true;
  assert(AdmitExploringGoal(door, 6));

  assert(RoomStillOpen(true, false));
  assert(RoomStillOpen(false, true));
  assert(!RoomStillOpen(false, false));

  assert(!ShouldCoverResiduals(true, true, true));
  assert(ShouldCoverResiduals(false, true, true));
  assert(!ShouldCoverResiduals(false, false, true));
  assert(!ShouldCoverResiduals(false, true, false));

  TipLedger open_tip;
  open_tip.NoteUnreached(0., 0., false);
  open_tip.NoteUnreached(0.2, 0.1, false);
  assert(!open_tip.Blocked(0., 0.));
  open_tip.NoteUnreached(0., 0., false);
  assert(open_tip.Blocked(0.1, 0.));

  TipLedger door_tip;
  for (int i = 0; i < 5; ++i) door_tip.NoteUnreached(2., 2., true);
  assert(!door_tip.Blocked(2., 2.));
  door_tip.NoteUnreached(2., 2., true);
  assert(door_tip.Blocked(2., 2.));

  TipLedger looked;
  looked.NoteNoNewArea(4., 4.);
  assert(looked.Blocked(4.2, 4.));
  looked.NoteProgress(4., 4.);
  assert(!looked.Blocked(4., 4.));
  return 0;
}
