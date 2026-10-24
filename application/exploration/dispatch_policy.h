#pragma once
#include <array>
#include <cstddef>
#include <string_view>
#include <utility>
namespace navigation2d::exploration {
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
      ~Scope() { active = false; }
    } scope(active);
    return std::forward<Attempt>(attempt)();
  }
  static bool PinchWorthy(std::string_view failure, bool entry);
  static RouteAcceptance CheckClearance(DispatchPurpose purpose, double margin_m);
  static double GoalTimeout(bool returning, double path_length_m);

 private:
  std::array<bool, 3> active_{};
};
}  // namespace navigation2d::exploration
