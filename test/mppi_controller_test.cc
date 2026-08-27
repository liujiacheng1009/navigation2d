#include <cassert>
#include <cmath>
#include <fstream>

#include "navigation2d/control/mppi_controller.h"
#include "navigation2d/costmap/grid_2d.h"

int main() {
  const char* map_path = "/tmp/navigation2d_mppi_test.json";
  std::ofstream output(map_path);
  output << R"({"width":80,"height":40,"resolution":0.1,"cells":[)";
  for (int index = 0; index < 3200; ++index) output << (index ? ",0" : "0");
  output << "]}";
  output.close();

  navigation2d::NavigationConfig config;
  config.mppi_batch_size = 64;
  config.mppi_time_steps = 20;
  navigation2d::LayeredCostmap first_map(
      navigation2d::Grid2d::Load(map_path), config);
  navigation2d::LayeredCostmap second_map(
      navigation2d::Grid2d::Load(map_path), config);
  navigation2d::Path path;
  for (int index = 0; index <= 40; ++index)
    path.push_back(navigation2d::MakePose2d(1. + index * .1, 2., 0.));
  const auto pose = navigation2d::MakePose2d(1., 2., 0.);
  navigation2d::MppiController first(config), second(config);
  const auto first_command = first.Compute(path, pose, {}, first_map);
  const auto second_command = second.Compute(path, pose, {}, second_map);
  assert(std::isfinite(first_command.linear));
  assert(std::isfinite(first_command.angular));
  assert(first_command.linear >= -config.max_reverse_velocity);
  assert(first_command.linear <= config.max_linear_acceleration * config.control_period);
  assert(std::abs(first_command.angular) <=
         config.max_angular_acceleration * config.control_period);
  assert(first_command.linear == second_command.linear);
  assert(first_command.angular == second_command.angular);

  first_map.MarkObstacle(1.1, 2.);
  assert(first.CollisionImminent(pose, {config.desired_linear_velocity, 0.}, first_map));
}
