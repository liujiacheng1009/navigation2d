#include "navigation2d/test/motion_output_benchmark.h"

#include "navigation2d/ros/exploration/motion_output.h"
#include "rclcpp/rclcpp.hpp"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("motion_output_candidate");
  auto output = navigation2d::exploration_ros::MakeRosMotionOutput(
      *node, navigation2d::test::MotionBenchmarkTopic());
  navigation2d::test::MeasureMotionOutput(
      node, [&](navigation2d::Twist2d command) { output->Send(command); });
  rclcpp::shutdown();
}
