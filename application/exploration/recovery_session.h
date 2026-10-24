#pragma once
#include <cmath>
namespace navigation2d::exploration {
// Per-goal recovery budget. Counters survive resume/replan of the same
// viewpoint so side-speck peel cannot reset itself into an infinite loop.
struct GoalRecoverySession {
  double goal_x = 0.;
  double goal_y = 0.;
  int side_peels = 0;
  int open_replans = 0;
  // Open-forward wall scrape: peel→resume same chord is the red-box sawtooth.
  int wall_scrapes = 0;
  bool resume_after_escape = false;
  bool active = false;

  bool SameGoal(double x, double y, double tol = 0.5) const {
    return active && std::hypot(x - goal_x, y - goal_y) <= tol;
  }

  void Begin(double x, double y) {
    if (SameGoal(x, y)) {
      return;
    }
    goal_x = x;
    goal_y = y;
    side_peels = 0;
    open_replans = 0;
    wall_scrapes = 0;
    resume_after_escape = false;
    active = true;
  }

  void Clear() {
    active = false;
    side_peels = 0;
    open_replans = 0;
    wall_scrapes = 0;
    resume_after_escape = false;
  }
};

}  // namespace navigation2d::exploration
