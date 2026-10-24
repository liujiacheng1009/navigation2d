#include "navigation2d/ros/exploration/completion_reporter.h"

#include "rclcpp/logging.hpp"
namespace navigation2d::exploration_ros {
void ReportCompletion(const rclcpp::Logger& logger, const exploration::CompletionNotice& notice) {
  using exploration::CompletionNoticeKind;
  switch (notice.kind) {
    case CompletionNoticeKind::kEmpty:
      RCLCPP_WARN(logger, "Completing probe found no leftover approach: raw_cells=%zu rounds=%d",
                  notice.raw_cells, notice.rounds);
      break;
    case CompletionNoticeKind::kThresholdCell:
      RCLCPP_WARN(logger,
                  "Completing leftover is on the current doorway cell (%.2f, %.2f); skipping",
                  notice.goal.x, notice.goal.y);
      break;
    case CompletionNoticeKind::kEntryProbe:
      RCLCPP_WARN(logger, "Completing leftover unplannable at (%.2f, %.2f); trying entry probe",
                  notice.goal.x, notice.goal.y);
      break;
    case CompletionNoticeKind::kUnplannable:
      RCLCPP_WARN(logger, "Completing leftover approach is unplannable at (%.2f, %.2f)",
                  notice.goal.x, notice.goal.y);
      break;
    case CompletionNoticeKind::kCandidatesReady:
      RCLCPP_INFO(logger, "Completing probe finished; normal candidates=%zu tour=%zu",
                  notice.candidates, notice.tour_size);
      break;
  }
}
}  // namespace navigation2d::exploration_ros
