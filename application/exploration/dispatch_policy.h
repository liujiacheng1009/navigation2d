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
  struct ReplanAttempt {
    int number;
    int limit;
  };
  void BeginNavigationAttempt(bool latest_map_retry) {
    if (!latest_map_retry) {
      latest_map_replans_ = 0;
      cut_replans_ = 0;
    }
  }
  void ResetLatestMapReplans() { latest_map_replans_ = 0; }
  bool HasRouteCutReplans() const { return cut_replans_ != 0; }
  template <typename Operation>
  bool ReplanOnLatestMap(Operation&& operation) {
    constexpr int kLimit = 2;
    if (latest_map_replans_ >= kLimit) {
      return false;
    }
    const int attempt = ++latest_map_replans_;
    return std::forward<Operation>(operation)(ReplanAttempt{attempt, kLimit});
  }
  template <typename Operation>
  bool ReplanAfterRouteCut(Operation&& operation) {
    constexpr int kLimit = 3;
    if (cut_replans_ >= kLimit) {
      return false;
    }
    const int attempt = ++cut_replans_;
    const bool accepted = std::forward<Operation>(operation)(ReplanAttempt{attempt, kLimit});
    // A nested fresh dispatch can reset the budgets. Restore this route-cut
    // reservation on normal return, matching the original dispatch ordering.
    cut_replans_ = attempt;
    return accepted;
  }
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
  // Legacy diagnostic helper; live dispatch consumes InitialPlanFailure.
  static bool PinchWorthy(std::string_view failure, bool entry);
  static RouteAcceptance CheckClearance(DispatchPurpose purpose, double margin_m);
  static double GoalTimeout(bool returning, double path_length_m);

 private:
  DispatchVerdict RejectUnreachable(const ExplorationGoal&, DispatchPurpose, bool latest_map_retry,
                                    const InitialPlanResult&, DispatchActions&);
  DispatchVerdict RejectMappedWall(const ExplorationGoal&, DispatchPurpose, std::uint64_t signature,
                                   DispatchActions&);
  int latest_map_replans_ = 0;
  int cut_replans_ = 0;
  std::array<bool, 3> active_{};
};
}  // namespace navigation2d::exploration
