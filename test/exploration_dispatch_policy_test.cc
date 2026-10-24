#include <limits>
#include <stdexcept>

#include "navigation2d/application/exploration/dispatch_policy.h"
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("dispatch policy contract failed");
  }
}
}  // namespace
int main() {
  using namespace navigation2d::exploration;
  DispatchPolicy policy;
  using Kind = DispatchPolicy::RetryKind;
  int attempts = 0;
  Require(policy.Retry(Kind::kMappedWall, [&] {
    ++attempts;
    Require(policy.active(Kind::kMappedWall));
    Require(!policy.Retry(Kind::kMappedWall, [&] {
      ++attempts;
      return true;
    }));
    return policy.Retry(Kind::kPinch, [&] {
      ++attempts;
      return true;
    });
  }));
  Require(attempts == 2 && !policy.active(Kind::kMappedWall) && !policy.active(Kind::kPinch));
  try {
    policy.Retry(Kind::kOpenScrape, []() -> bool { throw std::runtime_error("backend"); });
  } catch (const std::runtime_error&) {
  }
  Require(!policy.active(Kind::kOpenScrape));
  Require(DispatchPolicy::PinchWorthy("no route", false));
  Require(!DispatchPolicy::PinchWorthy("goal occupied; no route", true));
  Require(!DispatchPolicy::PinchWorthy("first-segment replay", false));
  Require(DispatchPolicy::PinchWorthy("first-segment replay", true));
  Require(DispatchPolicy::CheckClearance({}, -1.) == RouteAcceptance::kOpenScrape);
  Require(DispatchPolicy::CheckClearance({false, false, false, true}, -1.) ==
          RouteAcceptance::kPinchScrape);
  Require(DispatchPolicy::CheckClearance({true, false, false, true}, -1.) ==
          RouteAcceptance::kAccept);
  Require(DispatchPolicy::CheckClearance({false, true, false, false}, -1.) ==
          RouteAcceptance::kAccept);
  Require(DispatchPolicy::CheckClearance({false, false, true, false}, -1.) ==
          RouteAcceptance::kAccept);
  Require(DispatchPolicy::CheckClearance({}, std::numeric_limits<double>::quiet_NaN()) ==
          RouteAcceptance::kAccept);
  Require(DispatchPolicy::GoalTimeout(false, 0.) == 40.);
  Require(DispatchPolicy::GoalTimeout(false, 100.) == 75.);
  Require(DispatchPolicy::GoalTimeout(true, 0.) == 55.);
  Require(DispatchPolicy::GoalTimeout(true, 10.) == 120.);
}
