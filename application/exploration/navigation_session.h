#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "navigation2d/application/algorithm_registry.h"
#include "navigation2d/application/navigation_system.h"

namespace navigation2d::exploration {

enum class SessionStage { kEmpty, kPreparing, kReady, kActive };
enum class InitialPlanFailure {
  kNone,
  kNoRoute,
  kStartOccupied,
  kGoalOccupied,
  kCancelled,
  kBudgetExceeded,
  kInvalidInput,
  kBackendError,
  kExcludedRoute,
  kInvalidRoute
};
enum class Cancellation { kNoSession, kDiscardedPreparation, kStoppedActive };

struct InitialPlanResult {
  NavigationState state;
  InitialPlanFailure failure = InitialPlanFailure::kNone;
  [[nodiscard]] bool has_route() const { return failure == InitialPlanFailure::kNone; }
  // Preserve the planner's original text; an empty no-route reason has the
  // same fallback as the explorer's pre-extraction diagnostic.
  [[nodiscard]] std::string failure_reason() const;
};

// Owns one tentative/accepted navigation execution, without ROS or a clock.
// Single-writer, synchronous API. All poses/path/scan use the snapshot's local
// map frame (metres/radians); timestamps are seconds in the caller's time domain.
// Begin replaces the previous instance; exceptions propagate to the adapter,
// which must Cancel before retrying. No initial command may be published until
// Probe succeeds AND the caller's live-map/clearance checks pass, then Accept.
// Cancellation destroys algorithm state; the adapter still owns publishing stop.
class NavigationSession {
 public:
  explicit NavigationSession(
      std::shared_ptr<const AlgorithmRegistry> algorithms = AlgorithmRegistry::Builtins());
  NavigationSession(NavigationSession&& other) noexcept;
  NavigationSession& operator=(NavigationSession&& other) noexcept;
  NavigationSession(const NavigationSession&) = delete;
  NavigationSession& operator=(const NavigationSession&) = delete;

  void Begin(const NavigationConfig& config, const std::string& map_path, Pose2d goal,
             std::vector<std::uint64_t> banned_signatures);
  void Begin(const NavigationConfig& config, const std::string& map_path, Pose2d goal,
             std::vector<std::uint64_t> banned_signatures, NavigationExecutionOptions options);
  [[nodiscard]] InitialPlanResult Probe(const Pose2d& pose, Twist2d measured_velocity,
                                        double timestamp);
  [[nodiscard]] InitialPlanResult Probe(const Pose2d& pose, Twist2d measured_velocity,
                                        double timestamp, const AlgorithmBudget& budget);
  void Accept();
  Cancellation Cancel() noexcept;
  [[nodiscard]] NavigationState Step(const Pose2d& pose, Twist2d measured_velocity,
                                     double timestamp);
  [[nodiscard]] NavigationState Step(const Pose2d& pose, Twist2d measured_velocity,
                                     double timestamp, const AlgorithmBudget& budget);
  const PlannedPathMetadata& path_metadata() const;
  // Collision-only integration preserves static-map authority. Allowed during
  // preparation so Probe observes the same near-field evidence as execution.
  void UpdateCollisionScan(const Pose2d& sensor_pose, const LaserScan& scan);
  // Borrow valid until the next non-const operation; no algorithm pointer escapes.
  [[nodiscard]] const Path& path() const;
  [[nodiscard]] SessionStage stage() const noexcept;
  [[nodiscard]] bool has_session() const noexcept { return system_ != nullptr; }

 private:
  std::shared_ptr<const AlgorithmRegistry> algorithms_;
  std::unique_ptr<NavigationSystem> system_;
  SessionStage stage_ = SessionStage::kEmpty;
  Pose2d goal_ = MakePose2d(0., 0., 0.);
  double goal_tolerance_m_ = 0.;
};

}  // namespace navigation2d::exploration
