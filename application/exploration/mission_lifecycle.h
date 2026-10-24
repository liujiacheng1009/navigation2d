#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
namespace navigation2d::exploration {
enum class MissionPhase { kExploring, kCompleting, kReturning };
struct MissionHome {
  double x = 0., y = 0., yaw = 0.;  // Map frame, metres/radians.
};
struct MissionPosition {
  double x = 0., y = 0.;
};
struct MissionCompletion {
  bool success = false;
  std::optional<double> home_error_m;
  std::string message;
};
// One mission per instance, one writer. Nanosecond stamps share the mission
// clock. ROS I/O and action ordering remain the adapter's responsibility.
class MissionLifecycle {
 public:
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
