#include <chrono>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include "navigation2/application/navigation_system.h"

int main(int argc, char** argv) try {
  if (argc != 9) throw std::runtime_error("usage: navigation2d_benchmark CASE WORLD SX SY SYAW GX GY GYAW");
  const auto started = std::chrono::steady_clock::now();
  navigation2d::NavigationSystem system;
  const auto result = system.Run(argv[2], {std::stod(argv[3]), std::stod(argv[4]), std::stod(argv[5])},
                                 {std::stod(argv[6]), std::stod(argv[7]), std::stod(argv[8])});
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  std::cout << std::fixed << std::setprecision(6)
            << "{\"id\":\"" << argv[1] << "\",\"status\":\"" << result.status
            << "\",\"goal_error_m\":" << result.goal_error_m
            << ",\"collision_delta\":" << result.collisions
            << ",\"sim_duration_s\":" << result.duration_s
            << ",\"steps\":" << result.steps << ",\"wall_seconds\":" << wall
            << ",\"trace\":[";
  for (size_t i = 0; i < result.trajectory.size(); ++i) {
    if (i) std::cout << ',';
    const auto& pose = result.trajectory[i];
    std::cout << "{\"elapsed_s\":" << i * .06 << ",\"x\":" << pose.x
              << ",\"y\":" << pose.y << ",\"yaw\":" << pose.yaw << '}';
  }
  std::cout << "]}\n";
  return result.status == "SUCCEEDED" && result.collisions == 0 ? 0 : 1;
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n'; return 2;
}
