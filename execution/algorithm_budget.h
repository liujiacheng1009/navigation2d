#pragma once
#include <chrono>
#include <stop_token>

namespace navigation2d {
enum class AlgorithmStop { kNone, kCancelled, kBudgetExceeded };
// Cooperative, monotonic wall-time budget, independent of ROS/simulation time.
// Legacy adapters check before/after a synchronous call and discard late output;
// they cannot interrupt a backend that does not poll this token/deadline.
struct AlgorithmBudget {
  std::stop_token cancellation;
  std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::time_point::max();
  [[nodiscard]] AlgorithmStop Check() const noexcept {
    if (cancellation.stop_requested()) {
      return AlgorithmStop::kCancelled;
    }
    if (deadline != std::chrono::steady_clock::time_point::max() &&
        std::chrono::steady_clock::now() >= deadline) {
      return AlgorithmStop::kBudgetExceeded;
    }
    return AlgorithmStop::kNone;
  }
};
}  // namespace navigation2d
