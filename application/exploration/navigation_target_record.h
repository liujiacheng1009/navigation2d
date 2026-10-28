#pragma once

#include <string>

#include "navigation2d/exploration/frontier_types.h"

namespace navigation2d::exploration {
class NavigationSession;
struct AlgorithmSelection;
struct AlgorithmNames {
  std::string global;
  std::string local;
};

// Mission-local record of the accepted world-frame target (metres/radians).
// One writer on the mission thread. Session cancellation retains the target
// for failure bookkeeping; Clear explicitly removes it. Configuration tables
// belong to the profiles/session, not this reporting identity.
class NavigationTargetRecord {
 public:
  NavigationTargetRecord();
  // Requires a successfully probed session. Allocate reporting identity before
  // accepting; the session and target stay unchanged if preparation throws.
  void Accept(NavigationSession& session, const ExplorationGoal& world_goal,
              const AlgorithmSelection& configured_algorithms);
  void Clear() noexcept;
  const ExplorationGoal* target() const& { return has_target_ ? &goal_ : nullptr; }
  const ExplorationGoal* target() const&& = delete;
  const AlgorithmNames& algorithms() const& { return algorithms_; }
  const AlgorithmNames& algorithms() const&& = delete;
  // Transitional ROS/status schema: startup is Goal{}, explicit clear uses
  // NaN coordinates and retains the last configured names. New callers should
  // use target() to distinguish missing evidence from a real zero coordinate.
  const ExplorationGoal& legacy_goal() const& { return goal_; }
  const ExplorationGoal& legacy_goal() const&& = delete;

 private:
  ExplorationGoal goal_;
  AlgorithmNames algorithms_;
  bool has_target_ = false;
};
}  // namespace navigation2d::exploration
