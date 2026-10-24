#include "navigation2d/application/navigation_failure.h"
namespace navigation2d {
NavigationFailureEffects EffectsFor(NavigationFailure failure) {
  switch (failure) {
    case NavigationFailure::kNone:
      return {false, false, false, false, {}};
    case NavigationFailure::kDockingNoMotion:
      return {true, true, true, false, "docking command produced no measured motion"};
    case NavigationFailure::kControllerNoMotion:
      return {true, true, true, false, "controller command produced no measured motion"};
    case NavigationFailure::kCollisionHold:
      return {true, true, true, false, "collision monitor held a stop"};
    case NavigationFailure::kTrackingFullTurn:
      return {true, true, true, false,
              "regulated pursuit rotated a full turn without path progress"};
    case NavigationFailure::kProgressRetry:
      return {true, true, false, true, {}};
    case NavigationFailure::kProgressExhausted:
      return {false, true, true, false, {}};
    case NavigationFailure::kControllerRetry:
      return {true, true, false, false, {}};
    case NavigationFailure::kDockingTimeout:
      return {false, false, true, false, {}};
    case NavigationFailure::kAlignmentFullTurn:
    case NavigationFailure::kControllerExhausted:
    case NavigationFailure::kPublishedFullTurn:
      return {true, true, true, false, {}};
  }
  return {true, false, true, false, "unknown navigation failure"};
}
}  // namespace navigation2d
