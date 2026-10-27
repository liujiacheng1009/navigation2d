#include "navigation2d/application/exploration/exploration_mission.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
namespace navigation2d::exploration {
void ExplorationMission::Tick(double max_duration_s, MotionOutput& motion,
                              MissionRuntime& runtime) {
  if (finished()) {
    return;
  }
  const auto ready_home = runtime.ReadyMissionHome();
  if (!ready_home) {
    return;
  }
  if (!started()) {
    const auto started_ns = runtime.ExecutionNowNs();
    runtime.ResetMissionRecovery();
    const auto initial_known = runtime.KnownCells();
    runtime.InitializeMissionProgress(initial_known);
    constexpr std::int64_t kBootstrapDurationNs = 8000000000LL;
    const auto bootstrap_ns = runtime.ExecutionNowNs();
    if (bootstrap_ns > std::numeric_limits<std::int64_t>::max() - kBootstrapDurationNs) {
      throw std::overflow_error("mission bootstrap deadline overflow");
    }
    const auto deadline_ns = bootstrap_ns + kBootstrapDurationNs;
    Start(*ready_home, initial_known, started_ns, deadline_ns);
    runtime.MissionBootstrapScheduled(deadline_ns);
  }
  if (TimedOut(runtime.ExecutionNowNs(), max_duration_s)) {
    runtime.Finish(false, "exploration timed out");
    return;
  }
  runtime.NoteKnownProgress(runtime.KnownCells());
  runtime.NotePoseMotion();
  if (runtime.TickEscape()) {
    return;
  }
  if (Bootstrapping(runtime.ExecutionNowNs())) {
    motion.Send({});
    return;
  }
  if (!runtime.TickIdleNavigation()) {
    runtime.TickNavigation();
  }
}
void ExplorationMission::Start(MissionHome home, std::size_t initial_known_cells,
                               std::int64_t started_ns, std::int64_t bootstrap_deadline_ns) {
  if (stage_ != Stage::kWaiting) {
    throw std::logic_error("mission already started or finished");
  }
  home_ = home;
  initial_known_cells_ = initial_known_cells;
  started_ns_ = started_ns;
  bootstrap_deadline_ns_ = bootstrap_deadline_ns;
  stage_ = Stage::kRunning;
}
bool ExplorationMission::Finish() {
  if (finished()) {
    return false;
  }
  stage_ = Stage::kFinished;
  return true;
}
bool ExplorationMission::TimedOut(std::int64_t now_ns, double budget_s) const {
  return started() && (now_ns - started_ns_) / 1e9 > budget_s;
}
MissionCompletion ExplorationMission::EvaluateCompletion(
    bool success, std::string message, std::optional<MissionPosition> current,
    std::optional<MissionPosition> dock) const {
  std::optional<double> error;
  if (current && home_) {
    error = std::hypot(current->x - home_->x, current->y - home_->y);
    if (dock) {
      *error = std::min(*error, std::hypot(current->x - dock->x, current->y - dock->y));
    }
  }
  if (success && (!error || *error > .35)) {
    success = false;
    message = "return ended outside the home tolerance";
  }
  return {success, error, std::move(message)};
}
}  // namespace navigation2d::exploration
