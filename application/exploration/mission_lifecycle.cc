#include "navigation2d/application/exploration/mission_lifecycle.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
namespace navigation2d::exploration {
void MissionLifecycle::Start(MissionHome home, std::size_t initial_known_cells,
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
bool MissionLifecycle::Finish() {
  if (finished()) {
    return false;
  }
  stage_ = Stage::kFinished;
  return true;
}
bool MissionLifecycle::TimedOut(std::int64_t now_ns, double budget_s) const {
  return started() && (now_ns - started_ns_) / 1e9 > budget_s;
}
MissionCompletion MissionLifecycle::EvaluateCompletion(bool success, std::string message,
                                                       std::optional<MissionPosition> current,
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
