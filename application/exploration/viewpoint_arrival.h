#pragma once
#include <string>

#include "navigation2d/application/exploration/entry_actions.h"
#include "navigation2d/application/exploration/tour_order.h"
namespace navigation2d::exploration {
class EntryCoordinator;
class TourExecution;
enum class ViewpointArrivalNotice { kNearRoomBeyond, kReachedRoomBeyond, kEntryIncomplete };
class ViewpointArrivalActions {
 public:
  virtual ~ViewpointArrivalActions() = default;
  virtual void LockEntryMouth(const ExplorationGoal&) = 0;
  virtual bool StartEntryProbe() = 0;
  virtual void AcknowledgeEntry(EntryArrival) = 0;
  virtual void AbandonLockedEntryMouth(const std::string&) = 0;
  virtual void ClearEntryMouthLock() = 0;
  virtual void QueueActiveRegionFinish() = 0;
  virtual void DeferViewpointSelection(double seconds, const char* message) = 0;
  virtual void ReportViewpointArrival(ViewpointArrivalNotice, const ExplorationGoal&, int attempts,
                                      double depth) = 0;
};
// Owns the pending region-finish obligation across doorway acknowledgement
// and normal navigation arrival. Domain state is borrowed for each event;
// this object never stores node, map, tour or entry references.
class ViewpointArrival {
 public:
  void MarkRegionFinishPending() { region_finish_pending_ = true; }
  // Caller has cancelled navigation and published stop at a near viewpoint.
  void Near(FrontierExplorer&, EntryCoordinator&, TourExecution&, const ExplorationGoal&, TourPose,
            ViewpointArrivalActions&);
  // Normal exploration success only: return/completion/retreat stay separate.
  void Succeeded(FrontierExplorer&, EntryCoordinator&, TourExecution&, const ExplorationGoal&,
                 TourPose, ViewpointArrivalActions&);

 private:
  bool region_finish_pending_ = false;
};
}  // namespace navigation2d::exploration
