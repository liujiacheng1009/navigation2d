#pragma once
#include <cstdint>
namespace navigation2d {
struct NavigationState;
}
namespace navigation2d::exploration {
class GoalExecutionClock {
 public:
  virtual ~GoalExecutionClock() = default;
  virtual std::int64_t ExecutionNowNs() const = 0;
};
// Goal-local execution timing. Nanoseconds use the caller's single ROS time
// domain; reads are lazy to retain timeout/stall short-circuit ordering.
class GoalExecutionWatch {
 public:
  void SetTimeout(double seconds) { timeout_s_ = seconds; }
  void Start(std::int64_t started_ns, std::int64_t progress_ns);
  void ObserveProgress(double metres, const GoalExecutionClock&);
  [[nodiscard]] std::int64_t started_ns() const { return started_ns_; }
  [[nodiscard]] std::int64_t progress_ns() const { return progress_ns_; }
  [[nodiscard]] double Elapsed(const GoalExecutionClock&) const;
  [[nodiscard]] bool TimedOut(const GoalExecutionClock&) const;
  [[nodiscard]] bool ProgressOlderThan(double seconds, const GoalExecutionClock&) const;
  [[nodiscard]] bool FailureDue(const NavigationState&, bool returning, double front,
                                bool corridor_pinch, bool forward_open,
                                const GoalExecutionClock&) const;

 private:
  double timeout_s_ = 55., best_progress_m_ = 0.;
  std::int64_t started_ns_ = 0, progress_ns_ = 0;
};
}  // namespace navigation2d::exploration
