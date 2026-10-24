#include <cmath>
#include <stdexcept>

#include "navigation2d/application/exploration/mission_lifecycle.h"
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("mission lifecycle contract failed");
  }
}
constexpr std::int64_t Second = 1000000000;
}  // namespace
int main() {
  using namespace navigation2d::exploration;
  MissionLifecycle mission;
  Require(!mission.started() && !mission.finished() && !mission.home());
  Require(!mission.TimedOut(1000 * Second, 600.));
  mission.Start({0., 0., .3}, 120, Second, 9 * Second);
  Require(mission.started() && mission.initial_known_cells() == 120);
  Require(mission.home()->yaw == .3);
  Require(mission.Bootstrapping(9 * Second - 1));
  Require(!mission.Bootstrapping(9 * Second));
  Require(!mission.TimedOut(601 * Second, 600.));
  Require(mission.TimedOut(601 * Second + 1, 600.));
  Require(!mission.TimedOut(0, 600.));
  bool repeated_start_rejected = false;
  try {
    mission.Start({1., 1., 0.}, 200, 2 * Second, 10 * Second);
  } catch (const std::logic_error&) {
    repeated_start_rejected = true;
  }
  Require(repeated_start_rejected && mission.home()->x == 0.);
  mission.BeginCompleting();
  Require(mission.phase() == MissionPhase::kCompleting);
  mission.ResumeExploring();
  Require(mission.phase() == MissionPhase::kExploring);
  mission.BeginReturning();
  const auto raw =
      mission.EvaluateCompletion(true, "done", MissionPosition{.3, 0.}, MissionPosition{2., 0.});
  Require(raw.success && raw.home_error_m && *raw.home_error_m == .3);
  const auto snapped =
      mission.EvaluateCompletion(true, "done", MissionPosition{1., 0.}, MissionPosition{1., 0.});
  Require(snapped.success && *snapped.home_error_m == 0.);
  const auto outside =
      mission.EvaluateCompletion(true, "done", MissionPosition{1., 0.}, MissionPosition{2., 0.});
  Require(!outside.success && *outside.home_error_m == 1.);
  Require(outside.message == "return ended outside the home tolerance");
  const auto missing = mission.EvaluateCompletion(true, "done", {}, {});
  Require(!missing.success && !missing.home_error_m);
  const auto failed = mission.EvaluateCompletion(false, "backend failed", {}, {});
  Require(!failed.success && failed.message == "backend failed");
  Require(mission.Finish() && !mission.Finish());
  Require(mission.finished() && mission.phase() == MissionPhase::kReturning);
}
