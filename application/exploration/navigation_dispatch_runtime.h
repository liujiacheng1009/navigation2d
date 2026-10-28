#pragma once
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "navigation2d/application/exploration/navigation_dispatch.h"
#include "navigation2d/geometry/pose_2d.h"
#include "navigation2d/geometry/twist_2d.h"
namespace navigation2d {
struct NavigationState;
namespace exploration {
struct PreparedNavigationMap {
  const std::string& path;  // Synchronous borrow until next preparation.
  double origin_x, origin_y, resolution;
  bool entry, home_goal;
};
struct NavigationProbeInput {
  Pose2d pose;  // Prepared-map frame, metres/radians.
  Twist2d measured_velocity;
  double timestamp;  // Seconds in the caller's observation clock domain.
};
struct DispatchReplanNotice {
  enum class Kind { kLatestMap, kRouteCut };
  Kind kind;
  const char* reason;
  bool returning;
  int number, limit;
};
// Synchronous observation/storage and mission-commit boundary. Clock reads
// remain lazy in ReadNavigationProbe and CommitAcceptedNavigation. No
// navigation command may be published from a preparing session.
class NavigationDispatchRuntime : public DispatchActions {
 public:
  virtual void BeginNavigationPreparation() = 0;
  virtual void ReportSettledGoal(const ExplorationGoal& original,
                                 const ExplorationGoal& settled) = 0;
  virtual PreparedNavigationMap PrepareNavigationMap(const ExplorationGoal&, bool returning) = 0;
  virtual std::vector<std::uint64_t> CooledNavigationSignatures() const = 0;
  virtual void ObservePreparedNavigation() = 0;
  virtual NavigationProbeInput ReadNavigationProbe() const = 0;
  virtual bool DispatchRetreating() const = 0;
  virtual void SetDispatchTimeout(double seconds) = 0;
  virtual void CommitAcceptedNavigation(const ExplorationGoal&, bool returning,
                                        const NavigationState&) = 0;
  virtual void RecordNavigationException(std::string_view reason) = 0;
  virtual void ResumePreparedTrailReplay() = 0;
  virtual void ReportDispatchReplan(DispatchReplanNotice) = 0;
};
}  // namespace exploration
}  // namespace navigation2d
