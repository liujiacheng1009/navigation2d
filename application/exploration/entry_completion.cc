#include "navigation2d/application/exploration/entry_completion.h"
namespace navigation2d::exploration {
void CompleteEntryArrival(EntryCoordinator& entry, FrontierExplorer& explorer,
                          RoomVisitLedger& ledger, EntryActions& actions, EntryArrival arrival,
                          double robot_x, double robot_y) {
  const auto mouth = *entry.pending();
  if (arrival == EntryArrival::kCrawl || arrival == EntryArrival::kDriving) {
    actions.CoolEnteredMouth(mouth, 12.);
  }
  explorer.RecordAttempt(mouth, true, false);
  ledger.Forget(mouth);
  if (arrival == EntryArrival::kBeforeProbe) {
    actions.CoolEnteredMouth(mouth, 14.);
  }
  entry.Clear();
  actions.QueueActiveRegionFinish();
  if (arrival == EntryArrival::kDriving) {
    actions.MarkRegionFinishPending();
  } else {
    actions.SelectAfterEntry();
  }
  EntryNotice notice{EntryNoticeKind::kEntered, mouth, {}};
  notice.arrival = arrival;
  notice.robot_x = robot_x;
  notice.robot_y = robot_y;
  actions.ReportEntry(notice);
}
}  // namespace navigation2d::exploration
