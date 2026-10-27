#pragma once
#include "navigation2d/application/exploration/mission_runtime.h"
#include "navigation2d/application/exploration/mission_types.h"
#include "navigation2d/application/exploration/motion_output.h"
namespace navigation2d::exploration {
// One mission per instance, one writer. Nanosecond stamps share the mission
// clock. Tick owns phase ordering; each runtime role executes synchronously.
class ExplorationMission {
 public:
  void Tick(double max_duration_s, MotionOutput&, MissionRuntime&);
  void Start(MissionHome home, std::size_t initial_known_cells, std::int64_t started_ns,
             std::int64_t bootstrap_deadline_ns);
  bool started() const { return home_.has_value(); }
  bool finished() const { return stage_ == Stage::kFinished; }
  bool Finish();
  bool TimedOut(std::int64_t now_ns, double budget_s) const;
  bool Bootstrapping(std::int64_t now_ns) const { return now_ns < bootstrap_deadline_ns_; }
  const std::optional<MissionHome>& home() const { return home_; }
  std::size_t initial_known_cells() const { return initial_known_cells_; }
  MissionPhase phase() const { return phase_; }
  void ResumeExploring() { phase_ = MissionPhase::kExploring; }
  void BeginCompleting() { phase_ = MissionPhase::kCompleting; }
  void BeginReturning() { phase_ = MissionPhase::kReturning; }
  // Missing pose/home remains missing; the legacy artifact adapter maps that
  // to its historical infinity representation. A successful return must be
  // within 0.35 m of either the raw start or its observed-map docking snap.
  MissionCompletion EvaluateCompletion(bool requested_success, std::string message,
                                       std::optional<MissionPosition> current,
                                       std::optional<MissionPosition> dock) const;

 private:
  enum class Stage { kWaiting, kRunning, kFinished };
  Stage stage_ = Stage::kWaiting;
  MissionPhase phase_ = MissionPhase::kExploring;
  std::optional<MissionHome> home_;
  std::size_t initial_known_cells_ = 0;
  std::int64_t started_ns_ = 0, bootstrap_deadline_ns_ = 0;
};
}  // namespace navigation2d::exploration
