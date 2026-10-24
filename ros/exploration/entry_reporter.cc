#include "navigation2d/ros/exploration/entry_reporter.h"

#include "rclcpp/logging.hpp"
namespace navigation2d::exploration_ros {
void ReportEntry(const rclcpp::Logger& logger, const exploration::EntryNotice& notice,
                 const std::string& failure, EntryStatusSink& sink) {
  using exploration::EntryNoticeKind;
  const auto& mouth = notice.mouth;
  const auto& target = notice.target;
  switch (notice.kind) {
    case EntryNoticeKind::kApproaching:
      RCLCPP_INFO(logger,
                  "Entry approach standoff: frontier=(%.2f,%.2f) "
                  "range=%.2f step=(%.2f,%.2f)",
                  mouth.frontier_x, mouth.frontier_y, notice.range_m, target.x, target.y);
      sink.EntryStatus("COMPLETING", "approaching sealed-room mouth");
      break;
    case EntryNoticeKind::kUnavailable:
      RCLCPP_WARN(logger,
                  "Entry probe unavailable: frontier=(%.2f,%.2f) attempts=%d "
                  "gap=%.2f free_depth=%.2f",
                  mouth.frontier_x, mouth.frontier_y, notice.attempts, notice.gap_m,
                  notice.free_depth_m);
      break;
    case EntryNoticeKind::kEntered: {
      using exploration::EntryArrival;
      const bool driving = notice.arrival == EntryArrival::kDriving;
      sink.EntryStatus(driving ? "NAVIGATING" : "SELECTING",
                       driving ? "room entered; finishing probe inside active region"
                               : "room entered; finishing active region first");
      switch (notice.arrival) {
        case EntryArrival::kBeforeProbe:
          RCLCPP_INFO(logger, "room entered at pose=(%.2f,%.2f) (already past mouth)",
                      notice.robot_x, notice.robot_y);
          break;
        case EntryArrival::kCrawl:
          RCLCPP_INFO(logger, "room entered at pose=(%.2f,%.2f) (open-loop crawl)", notice.robot_x,
                      notice.robot_y);
          break;
        case EntryArrival::kDriving:
          RCLCPP_INFO(logger, "room entered at pose=(%.2f,%.2f) (mid-drive mouth cross)",
                      notice.robot_x, notice.robot_y);
          break;
        case EntryArrival::kNearProbe:
          RCLCPP_INFO(logger, "room entered at pose=(%.2f,%.2f) (arrived near probe)",
                      notice.robot_x, notice.robot_y);
          break;
        case EntryArrival::kNavigationSucceeded:
        case EntryArrival::kSelection:
          break;
      }
      break;
    }
    case EntryNoticeKind::kTooNear:
    case EntryNoticeKind::kPlanRejected:
      RCLCPP_WARN(logger,
                  "Entry probe rejected: target=(%.2f,%.2f) depth=%.2f attempt=%d "
                  "gap=%.2f reason=%s",
                  target.x, target.y, notice.minimum_depth_m, notice.attempts, notice.gap_m,
                  notice.kind == EntryNoticeKind::kTooNear
                      ? "target within goal tolerance of robot"
                      : (failure.empty() ? "unknown" : failure.c_str()));
      break;
    case EntryNoticeKind::kProbing:
      RCLCPP_INFO(logger,
                  "Entry probe started: frontier=(%.2f,%.2f) target=(%.2f,%.2f) "
                  "depth=%.2f attempt=%d gap=%.2f easy=%s",
                  mouth.frontier_x, mouth.frontier_y, target.x, target.y, notice.minimum_depth_m,
                  notice.attempts, notice.gap_m, notice.easy ? "yes" : "no");
      sink.EntryStatus("COMPLETING", "crossing frontier entry and waiting for map growth");
      break;
  }
}
}  // namespace navigation2d::exploration_ros
