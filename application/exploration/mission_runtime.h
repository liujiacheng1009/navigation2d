#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "navigation2d/application/exploration/goal_execution_watch.h"
#include "navigation2d/application/exploration/mission_types.h"
namespace navigation2d::exploration {
// Synchronous phase boundaries, serialized by the mission's single caller.
// Implementations retain their role's state; no mutable aggregate escapes.
// Clock reads are lazy and use one nanosecond time domain. Recovery and idle
// return true after handling the current tick, including deliberate waiting.
class MissionRuntime : public GoalExecutionClock {
 public:
  virtual std::optional<MissionHome> ReadyMissionHome() const = 0;
  virtual std::size_t KnownCells() const = 0;
  virtual void ResetMissionRecovery() = 0;
  virtual void InitializeMissionProgress(std::size_t initial_known) = 0;
  virtual void MissionBootstrapScheduled(std::int64_t deadline_ns) = 0;
  virtual void Finish(bool success, const std::string& message) = 0;
  virtual void NotePoseMotion() = 0;
  virtual bool TickEscape() = 0;
  virtual bool TickIdleNavigation() = 0;
  virtual void TickNavigation() = 0;
};
}  // namespace navigation2d::exploration
