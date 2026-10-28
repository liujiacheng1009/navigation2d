#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigation2d/geometry/robot_geometry.h"
#include "navigation2d/test/ros_loop_room.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

namespace navigation2d::test {
// Synthetic chassis + sensors only. Run exclusively through the network/IPC
// isolated Docker runner. It never connects to a simulator or a hardware node.
class ExplorerRosLoopScene final : public rclcpp::Node {
 public:
  ExplorerRosLoopScene(RosLoopScenario scenario, const std::string& trace)
      : Node("explorer_loop_scene"),
        scenario_(scenario),
        room_(scenario == RosLoopScenario::kObstacle),
        frontier_(scenario == RosLoopScenario::kFrontier || scenario == RosLoopScenario::kObstacle),
        trace_(trace) {
    const auto durable = rclcpp::QoS(1).transient_local().reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/localization2d/map", durable);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/localization2d/pose",
                                                                  rclcpp::SensorDataQoS());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS());
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS());
    commands_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist& command) {
          if (!std::isfinite(command.linear.x) || !std::isfinite(command.angular.z) ||
              std::abs(command.linear.x) > .5 || std::abs(command.angular.z) > 1.5) {
            throw std::runtime_error("non-finite or out-of-profile synthetic chassis command");
          }
          CheckSourcePause(command);
          linear_ = command.linear.x;
          angular_ = command.angular.z;
          ++commands;
          moving_commands += std::abs(linear_) > .03;
          trace_ << now().nanoseconds() << ',' << x_ << ',' << y_ << ',' << yaw_ << ',' << linear_
                 << ',' << angular_ << '\n';
        });
    status_ = create_subscription<std_msgs::msg::String>(
        "/exploration/status", durable, [this](const std_msgs::msg::String& message) {
          returning |= message.data.find("\"state\":\"RETURNING\"") != std::string::npos;
          if (!ready_ && message.data.find("pose ready") != std::string::npos) {
            ready_ = std::chrono::steady_clock::now();
          }
        });
    paths_ = create_subscription<nav_msgs::msg::Path>(
        "/navigation/path", durable,
        [this](const nav_msgs::msg::Path& path) { planned_paths += path.poses.size() > 1; });
    map_.header.frame_id = "map";
    // A 4 x 3 m room at 2 cm exceeds the unchanged 25,000-cell
    // initial-map maturity requirement without inventing a completed goal.
    map_.info.width = RosLoopRoom::kWidth;
    map_.info.height = RosLoopRoom::kHeight;
    map_.info.resolution = RosLoopRoom::kResolution;
    map_.info.origin.orientation.w = 1.;
    BuildMap(!frontier_);
    if (!trace_) {
      throw std::runtime_error("cannot open synthetic command trace");
    }
    trace_.precision(17);
    trace_ << "timestamp_ns,x,y,yaw,linear,angular\n";
    previous_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { Frame(); });
  }
  double home_error() const { return std::hypot(x_ - 1.15, y_ - 1.5); }
  bool requires_reveal() const { return frontier_; }
  int commands = 0, moving_commands = 0, planned_paths = 0, pause_stopped_commands = 0;
  bool scan_resumed = false;
  bool returning = false, map_revealed = false;
  double driven_distance = 0.;

 private:
  void CheckSourcePause(const geometry_msgs::msg::Twist& command) {
    if (scenario_ != RosLoopScenario::kScanPause) {
      return;
    }
    const auto time = std::chrono::steady_clock::now();
    if (!scan_pause_started_ && std::abs(command.linear.x) > .03) {
      scan_pause_started_ = time;
    }
    if (scan_pause_started_) {
      const double age = std::chrono::duration<double>(time - *scan_pause_started_).count();
      if (age >= .55 && age <= 1.05) {
        if (command.linear.x != 0. || command.angular.z != 0.) {
          throw std::runtime_error("stale laser source did not hold zero velocity");
        }
        ++pause_stopped_commands;
      }
    }
  }
  void BuildMap(bool revealed) {
    map_.data.resize(RosLoopRoom::kWidth * RosLoopRoom::kHeight);
    for (int y = 0; y < RosLoopRoom::kHeight; ++y) {
      for (int x = 0; x < RosLoopRoom::kWidth; ++x) {
        map_.data[y * RosLoopRoom::kWidth + x] = room_.Cell(x, y, revealed);
      }
    }
    map_.header.stamp = now();
    map_pub_->publish(map_);
  }
  void Frame() {
    const auto time = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(time - previous_).count();
    previous_ = time;
    // Return-only case: an external relocation during bootstrap supplies a
    // nontrivial path home. This displacement is not counted as driven motion.
    if (!frontier_ && ready_ && !relocated_ &&
        std::chrono::duration<double>(time - *ready_).count() > 1.) {
      x_ = 2.35;
      yaw_ = std::numbers::pi;
      relocated_ = true;
    }
    const double next_yaw = yaw_ + angular_ * dt;
    if (std::abs(angular_) < 1e-9) {
      x_ += linear_ * std::cos(yaw_) * dt;
      y_ += linear_ * std::sin(yaw_) * dt;
    } else {
      x_ += linear_ / angular_ * (std::sin(next_yaw) - std::sin(yaw_));
      y_ -= linear_ / angular_ * (std::cos(next_yaw) - std::cos(yaw_));
    }
    yaw_ = std::atan2(std::sin(next_yaw), std::cos(next_yaw));
    driven_distance += std::abs(linear_) * dt;
    if (x_ < .27 || x_ > 3.73 || y_ < .27 || y_ > 2.73 ||
        room_.HitsObstacle(x_, y_, kChassisRadius())) {
      throw std::runtime_error("synthetic chassis reached room collision boundary");
    }
    if (frontier_ && !map_revealed && x_ > 1.8) {
      map_revealed = true;
      BuildMap(true);
    }
    // Online mapping publishes repeated snapshots even without new obstacles.
    // Region completion deliberately needs several such observations.
    if (++frames_ % 50 == 0) {
      BuildMap(!frontier_ || map_revealed);
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = now();
    pose.pose.position.x = x_;
    pose.pose.position.y = y_;
    pose.pose.orientation.z = std::sin(yaw_ / 2.);
    pose.pose.orientation.w = std::cos(yaw_ / 2.);
    pose_pub_->publish(pose);
    nav_msgs::msg::Odometry odom;
    odom.header = pose.header;
    odom.pose.pose = pose.pose;
    odom.twist.twist.linear.x = linear_;
    odom.twist.twist.angular.z = angular_;
    odom_pub_->publish(odom);
    if (scan_pause_started_) {
      if (std::chrono::duration<double>(time - *scan_pause_started_).count() < 1.2) {
        return;
      }
      scan_resumed = true;
    }
    sensor_msgs::msg::LaserScan scan;
    scan.header = pose.header;
    scan.angle_min = -std::numbers::pi;
    scan.angle_increment = 2. * std::numbers::pi / 180.;
    scan.range_min = .02;
    scan.range_max = 8.;
    const double sx = x_ + kLidarForwardOffset() * std::cos(yaw_);
    const double sy = y_ + kLidarForwardOffset() * std::sin(yaw_);
    for (int i = 0; i < 180; ++i) {
      const double a = yaw_ + scan.angle_min + i * scan.angle_increment;
      scan.ranges.push_back(static_cast<float>(room_.Range(sx, sy, a)));
    }
    scan_pub_->publish(scan);
  }
  RosLoopScenario scenario_;
  RosLoopRoom room_;
  bool frontier_, relocated_ = false;
  int frames_ = 0;
  double x_ = 1.15, y_ = 1.5, yaw_ = 0., linear_ = 0., angular_ = 0.;
  std::optional<std::chrono::steady_clock::time_point> ready_, scan_pause_started_;
  std::chrono::steady_clock::time_point previous_;
  std::ofstream trace_;
  nav_msgs::msg::OccupancyGrid map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr commands_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr paths_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace navigation2d::test
