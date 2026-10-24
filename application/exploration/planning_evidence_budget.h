#pragma once
#include <cstddef>
namespace navigation2d::exploration {
// Single owner for deferred planning-evidence intent. Budgets are metres of
// driven trail, not permission to carve occupied map cells. Consume only when
// a positive map resolution and at least two trail samples are available.
class PlanningEvidenceBudget {
 public:
  void RequestStartReplay() { reopen_start_ = true; }
  void RequestTrailReplay(double metres) {
    replay_trail_ = true;
    budget_m_ = metres;
  }
  void ResumeTrailReplay() { replay_trail_ = true; }
  // A rejected retreat cancels intent but preserves its deferred budget, as
  // before. A later Resume can consume that budget once evidence is ready.
  void CancelTrailReplay() { replay_trail_ = false; }
  [[nodiscard]] double Consume(double resolution_m, std::size_t trail_samples) {
    if (reopen_start_) {
      replay_trail_ = true;
      reopen_start_ = false;
    }
    if (!replay_trail_ || !(resolution_m > 0.) || trail_samples < 2) {
      return 0.;
    }
    replay_trail_ = false;
    const double metres = budget_m_ > 0. ? budget_m_ : 2.;
    budget_m_ = 0.;
    return metres;
  }

 private:
  bool reopen_start_ = false, replay_trail_ = false;
  double budget_m_ = 0.;
};
}  // namespace navigation2d::exploration
