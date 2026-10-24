#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <utility>

#include "navigation2d/application/exploration/navigation_dispatch.h"
namespace navigation2d::exploration {
struct InitialPlanResult;
struct DispatchPurpose {
  bool entry, returning, retreat, pinch;
};
enum class RouteAcceptance { kAccept, kOpenScrape, kPinchScrape };
// Owns synchronous recursive retry scopes. Each kind may be active once;
// different kinds retain their original nesting order. No hot-path allocation.
class DispatchPolicy {
 public:
  enum class RetryKind : std::size_t { kMappedWall, kPinch, kOpenScrape };
  bool active(RetryKind kind) const { return active_[static_cast<std::size_t>(kind)]; }
  template <typename Attempt>
  bool Retry(RetryKind kind, Attempt&& attempt) {
    bool& active = active_[static_cast<std::size_t>(kind)];
    if (active) {
      return false;
    }
    struct Scope {
      bool& active;
      explicit Scope(bool& flag) : active(flag) { active = true; }
      Scope(const Scope&) = delete;
      Scope& operator=(const Scope&) = delete;
      ~Scope() { active = false; }
    } scope(active);
    return std::forward<Attempt>(attempt)();
  }
  [[nodiscard]] DispatchVerdict Evaluate(const ExplorationGoal&, DispatchPurpose,
                                         bool latest_map_retry, const InitialPlanResult&,
                                         DispatchActions&);
  static bool PinchWorthy(std::string_view failure, bool entry);
  static RouteAcceptance CheckClearance(DispatchPurpose purpose, double margin_m);
  static double GoalTimeout(bool returning, double path_length_m);

 private:
  DispatchVerdict RejectUnreachable(const ExplorationGoal&, DispatchPurpose, bool latest_map_retry,
                                    const InitialPlanResult&, DispatchActions&);
  DispatchVerdict RejectMappedWall(const ExplorationGoal&, DispatchPurpose, std::uint64_t signature,
                                   DispatchActions&);
  std::array<bool, 3> active_{};
};
}  // namespace navigation2d::exploration
