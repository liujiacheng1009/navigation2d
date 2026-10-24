#include <cmath>
#include <stdexcept>

#include "navigation2d/ros/exploration/pose_feedback.h"
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("pose feedback contract failed");
  }
}
void Near(double a, double b) { Require(std::abs(a - b) < 1e-12); }
}  // namespace
int main() {
  using namespace navigation2d::exploration_ros;
  PoseFeedback localized;
  Require(!localized.pose() && localized.trail().empty() && localized.distance_m() == 0.);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp.sec = 3;
  localized.ObserveLocalization(pose);
  pose.pose.position.x = .05;
  localized.ObserveLocalization(pose);
  Require(localized.trail().size() == 1);             // Strictly greater than 5 cm.
  Require(localized.pose()->pose.position.x == .05);  // Latest pose still updates.
  pose.pose.position.x = .1;
  localized.ObserveLocalization(pose);
  pose.pose.position.x = .2;
  localized.ObserveLocalization(pose);
  Require(localized.trail().size() == 3);
  Near(localized.distance_m(), .2);
  Require(localized.pose()->header == pose.header);
  PoseFeedback truth;
  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "world";
  odometry.header.stamp.sec = 42;
  odometry.pose.pose.position.x = 10.;
  odometry.pose.pose.position.y = 20.;
  odometry.pose.pose.orientation.z = std::sin(std::acos(-1.) / 4.);
  odometry.pose.pose.orientation.w = std::cos(std::acos(-1.) / 4.);
  truth.ObserveGroundTruth(odometry);
  Near(truth.pose()->pose.position.x, 0.);
  Near(truth.pose()->pose.position.y, 0.);
  Require(truth.pose()->header.frame_id == "map" && truth.pose()->header.stamp.sec == 42);
  odometry.pose.pose.position.y = 21.;
  truth.ObserveGroundTruth(odometry);
  Near(truth.pose()->pose.position.x, 1.);
  Near(truth.pose()->pose.position.y, 0.);
  Near(truth.pose()->pose.orientation.z, 0.);
  Near(truth.distance_m(), 1.);
}
