#pragma once
#include <cstdint>
#include <string>

#include "navigation2d/exploration/frontier_types.h"
namespace navigation2d::exploration {
enum class DispatchVerdict { kAcceptPrepared, kAcceptedRetry, kRejected };
enum class DispatchRejection { kNoRoute, kMappedWall, kOpenScrape, kPinchScrape };
struct DispatchFailure {
  DispatchRejection kind;
  std::string reason;
  bool start_occupied = false;
  double margin_m = 0.;
};
struct DispatchRetryRequest {
  bool returning;
  bool latest_map;
};
struct DispatchRetryNotice {
  enum class Kind { kNoRoutePinch, kScrapePinch };
  Kind kind;
  bool entry;
};
// Immediate capabilities at the preparation/acceptance boundary. Reject must
// record the diagnostic and discard preparation before returning. No command
// can be published from a tentative session. No services are retained here.
class DispatchActions {
 public:
  virtual ~DispatchActions() = default;
  virtual void RejectPreparation(DispatchFailure) = 0;
  virtual bool PreparedRouteCutsWall() const = 0;
  virtual void CoolRouteSignature(std::uint64_t) = 0;
  virtual void NoteMappedObstacleReject(const ExplorationGoal&) = 0;
  virtual void ObservePreparedRoute(std::uint64_t signature, double margin_m) = 0;
  virtual bool PassableMouthGoal(const ExplorationGoal&) const = 0;
  virtual bool RetryNavigation(const ExplorationGoal&, DispatchRetryRequest) = 0;
  virtual void ReportDispatchRetry(DispatchRetryNotice) = 0;
};
}  // namespace navigation2d::exploration
