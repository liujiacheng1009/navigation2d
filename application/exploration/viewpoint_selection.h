#pragma once
#include "navigation2d/application/exploration/viewpoint_selection_actions.h"
namespace navigation2d::exploration {
class EntryCoordinator;
// Owns selection rejection/empty-map budgets. Collaborators outlive this role
// and remain single-writer mission objects; no aggregate mutable context.
// Select consumes a stable world-frame pose snapshot after the caller has
// given returning and locked-entry execution priority.
class ViewpointSelection {
 public:
  ViewpointSelection(FrontierExplorer& explorer, EntryCoordinator& entry, TourExecution& tour,
                     CompletionExecution& completion, ViewpointRejections& rejections)
      : explorer_(explorer),
        entry_(entry),
        tour_(tour),
        completion_(completion),
        viewpoint_rejections_(rejections) {}
  void Select(TourPose robot, ViewpointSelectionActions&);
  void InitializeKnown(std::size_t known) { exhaustion_.InitializeKnown(known); }
  void ResetBlocked() { exhaustion_.ResetBlocked(); }
  int blocked_cycles() const { return exhaustion_.blocked_cycles(); }
  void ResetRejected() { reject_streak_ = 0; }

 private:
  FrontierExplorer& explorer_;
  EntryCoordinator& entry_;
  TourExecution& tour_;
  CompletionExecution& completion_;
  ViewpointRejections& viewpoint_rejections_;
  SelectionExhaustion exhaustion_;
  int reject_streak_ = 0;
};
}  // namespace navigation2d::exploration
