#include "navigation2d/ros/exploration/entry_reporter.h"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("entry status contract failed");
  }
}
struct Sink final : navigation2d::exploration_ros::EntryStatusSink {
  std::vector<std::pair<std::string, std::string>> statuses;
  void EntryStatus(const char* phase, const char* reason) override {
    statuses.emplace_back(phase, reason);
  }
};
}  // namespace
int main(int argc, char** argv) {
  using namespace navigation2d::exploration;
  using navigation2d::exploration_ros::ReportEntry;
  rclcpp::init(argc, argv);
  const auto logger = rclcpp::get_logger("entry_reporter_test");
  Sink sink;
  EntryNotice notice{EntryNoticeKind::kEntered, {}, {}};
  for (auto arrival :
       {EntryArrival::kBeforeProbe, EntryArrival::kCrawl, EntryArrival::kDriving,
        EntryArrival::kNearProbe, EntryArrival::kNavigationSucceeded, EntryArrival::kSelection}) {
    sink.statuses.clear();
    notice.arrival = arrival;
    ReportEntry(logger, notice, {}, sink);
    Require(sink.statuses.size() == 1);
    const bool driving = arrival == EntryArrival::kDriving;
    Require(sink.statuses.front().first == (driving ? "NAVIGATING" : "SELECTING"));
    Require(sink.statuses.front().second ==
            (driving ? "room entered; finishing probe inside active region"
                     : "room entered; finishing active region first"));
  }
  sink.statuses.clear();
  for (auto kind :
       {EntryNoticeKind::kUnavailable, EntryNoticeKind::kTooNear, EntryNoticeKind::kPlanRejected}) {
    notice.kind = kind;
    ReportEntry(logger, notice, "no route", sink);
  }
  Require(sink.statuses.empty());
  notice.kind = EntryNoticeKind::kApproaching;
  ReportEntry(logger, notice, {}, sink);
  Require(sink.statuses.size() == 1 && sink.statuses.back().first == "COMPLETING");
  Require(sink.statuses.back().second == "approaching sealed-room mouth");
  notice.kind = EntryNoticeKind::kProbing;
  ReportEntry(logger, notice, {}, sink);
  Require(sink.statuses.size() == 2 && sink.statuses.back().first == "COMPLETING");
  Require(sink.statuses.back().second == "crossing frontier entry and waiting for map growth");
  rclcpp::shutdown();
}
