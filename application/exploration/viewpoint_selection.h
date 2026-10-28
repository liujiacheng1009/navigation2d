#pragma once
#include <cstdint>

#include "navigation2d/application/exploration/viewpoint_selection_actions.h"
namespace navigation2d::exploration {
class EntryCoordinator;
class ExplorationMission;
class EntryRecoveryActions;
// Owns selection scheduling and rejection/empty-map budgets. Collaborators outlive this role
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
  // A copy would alias collaborators while duplicating the coordinating role.
  ViewpointSelection(const ViewpointSelection&) = delete;
  ViewpointSelection& operator=(const ViewpointSelection&) = delete;
  // Deadlines and observations use nanoseconds in the same mission clock domain.
  // Scheduling does not read the clock; paused/backward clocks defer selection.
  // Select preserves the deadline, including when a goal is accepted.
  void ScheduleAt(std::int64_t deadline_ns) { next_selection_ns_ = deadline_ns; }
  bool Due(std::int64_t now_ns) const { return now_ns >= next_selection_ns_; }
  // One rescue per mission, consumed only after pose/work/return gates pass.
  // Pose and navigation acceptance remain lazy synchronous action observations.
  bool RescueDistant(ExplorationMission&, bool returning, const char* reason,
                     ViewpointSelectionActions&);
  void AfterRetreat(EntryRecoveryActions&);
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
  bool rescue_attempted_ = false;
  std::int64_t next_selection_ns_ = 0;
};
}  // namespace navigation2d::exploration
