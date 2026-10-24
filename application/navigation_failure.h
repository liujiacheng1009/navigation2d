#pragma once
#include <string_view>
namespace navigation2d {
enum class NavigationFailure {
  kNone,
  kDockingNoMotion,
  kControllerNoMotion,
  kAlignmentFullTurn,
  kCollisionHold,
  kTrackingFullTurn,
  kProgressRetry,
  kProgressExhausted,
  kControllerRetry,
  kControllerExhausted,
  kPublishedFullTurn,
  kDockingTimeout
};
// Legacy failure effects remain explicit: an exhausted progress budget keeps
// its route, while a docking timeout records no additional recovery event.
struct NavigationFailureEffects {
  bool clear_route = false, record_recovery = false, blocked = false, reset_replan_timer = false;
  std::string_view message;
};
NavigationFailureEffects EffectsFor(NavigationFailure failure);
}  // namespace navigation2d
