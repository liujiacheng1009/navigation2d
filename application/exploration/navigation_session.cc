#include "navigation2d/application/exploration/navigation_session.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace navigation2d::exploration {

std::string InitialPlanResult::failure_reason() const {
  if (has_route()) {
    return {};
  }
  return state.planning_failure_reason.empty() ? "no route" : state.planning_failure_reason;
}

NavigationSession::NavigationSession(std::shared_ptr<const AlgorithmRegistry> algorithms)
    : algorithms_(std::move(algorithms)) {
  if (!algorithms_) {
    throw std::invalid_argument("navigation session requires an algorithm registry");
  }
}

// Keep the registry available in the moved-from session: continuous goal
// handoff prepares a candidate there while retaining the old execution for
// rollback. Moving ownership must not disable the next Begin call.
NavigationSession::NavigationSession(NavigationSession&& other) noexcept
    : algorithms_(other.algorithms_),
      system_(std::move(other.system_)),
      stage_(other.stage_),
      goal_(other.goal_),
      goal_tolerance_m_(other.goal_tolerance_m_) {}

NavigationSession& NavigationSession::operator=(NavigationSession&& other) noexcept {
  if (this != &other) {
    algorithms_ = other.algorithms_;
    system_ = std::move(other.system_);
    stage_ = other.stage_;
    goal_ = other.goal_;
    goal_tolerance_m_ = other.goal_tolerance_m_;
  }
  return *this;
}

void NavigationSession::Begin(const NavigationConfig& config, const std::string& map_path,
                              Pose2d goal, std::vector<std::uint64_t> banned_signatures) {
  system_ = std::make_unique<NavigationSystem>(config, map_path, algorithms_);
  stage_ = SessionStage::kPreparing;
  goal_ = goal;
  goal_tolerance_m_ = config.goal_xy_tolerance;
  system_->BanPathSignatures(std::move(banned_signatures));
  system_->SetGoal(goal);
}

InitialPlanResult NavigationSession::Probe(const Pose2d& pose, Twist2d measured_velocity,
                                           double timestamp) {
  if (stage() != SessionStage::kPreparing) {
    throw std::logic_error("Probe requires a preparing navigation session");
  }
  InitialPlanResult result{system_->ComputeCommand(pose, measured_velocity, timestamp)};
  const double distance = std::hypot(X(goal_) - X(pose), Y(goal_) - Y(pose));
  if (distance > goal_tolerance_m_ && result.state.global_path_length_m <= 0.) {
    const auto& reason = result.state.planning_failure_reason;
    if (reason.find("start occupied") != std::string::npos) {
      result.failure = InitialPlanFailure::kStartOccupied;
    } else if (reason.find("goal occupied") != std::string::npos) {
      result.failure = InitialPlanFailure::kGoalOccupied;
    } else {
      result.failure = InitialPlanFailure::kNoRoute;
    }
  } else {
    stage_ = SessionStage::kReady;
  }
  return result;
}

void NavigationSession::Accept() {
  if (stage() != SessionStage::kReady) {
    throw std::logic_error("Accept requires a successful navigation probe");
  }
  stage_ = SessionStage::kActive;
}

Cancellation NavigationSession::Cancel() noexcept {
  const auto outcome =
      !system_ ? Cancellation::kNoSession
               : (stage_ == SessionStage::kActive ? Cancellation::kStoppedActive
                                                  : Cancellation::kDiscardedPreparation);
  system_.reset();
  stage_ = SessionStage::kEmpty;
  return outcome;
}

NavigationState NavigationSession::Step(const Pose2d& pose, Twist2d measured_velocity,
                                        double timestamp) {
  if (stage() != SessionStage::kActive) {
    throw std::logic_error("Step requires an accepted navigation session");
  }
  return system_->ComputeCommand(pose, measured_velocity, timestamp);
}

void NavigationSession::UpdateCollisionScan(const Pose2d& sensor_pose, const LaserScan& scan) {
  if (!system_) {
    throw std::logic_error("collision scan requires a navigation session");
  }
  system_->UpdateCollisionMonitorLaserScan(sensor_pose, scan);
}

const Path& NavigationSession::path() const {
  if (!system_) {
    throw std::logic_error("path requires a navigation session");
  }
  return system_->path();
}

SessionStage NavigationSession::stage() const noexcept {
  return system_ ? stage_ : SessionStage::kEmpty;
}

}  // namespace navigation2d::exploration
