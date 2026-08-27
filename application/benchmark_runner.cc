#include <chrono>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include "navigation2d/simulation/navigation_simulator.h"

int main(int argc, char** argv) try {
  if (argc < 10) throw std::runtime_error("usage: navigation2d_benchmark CONFIG CASE WORLD SX SY SYAW GX GY GYAW [--obstacle APPEAR DISAPPEAR X Y]...");
  const auto started = std::chrono::steady_clock::now();
  auto config = navigation2d::NavigationConfig::Load(argv[1]);
  std::vector<navigation2d::simulation::ObstacleEvent> obstacles;
  for (int i = 10; i < argc;) {
    const std::string option = argv[i];
    if (option == "--obstacle" && i + 4 < argc) {
      obstacles.push_back({std::stod(argv[i + 1]), std::stod(argv[i + 2]),
                           std::stod(argv[i + 3]), std::stod(argv[i + 4])}); i += 5;
    } else if (option == "--planner" && i + 1 < argc) {
      config.planner = argv[i + 1]; i += 2;
    } else if (option == "--controller" && i + 1 < argc) {
      config.controller = argv[i + 1]; i += 2;
    } else throw std::runtime_error("invalid benchmark option: " + option);
  }
  navigation2d::simulation::NavigationSimulator simulator(config);
  const auto result = simulator.Run(
      argv[3], navigation2d::MakePose2d(std::stod(argv[4]), std::stod(argv[5]), std::stod(argv[6])),
      navigation2d::MakePose2d(std::stod(argv[7]), std::stod(argv[8]), std::stod(argv[9])), obstacles);
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  std::cout << std::fixed << std::setprecision(6)
            << "{\"id\":\"" << argv[2] << "\",\"status\":\"" << result.status
            << "\",\"planner\":\"" << config.planner << "\",\"controller\":\"" << config.controller
            << "\",\"goal_error_m\":" << result.goal_error_m
            << ",\"goal_heading_error_rad\":" << result.goal_heading_error_rad
            << ",\"collision_delta\":" << result.collisions
            << ",\"sim_duration_s\":" << result.duration_s
            << ",\"steps\":" << result.steps << ",\"wall_seconds\":" << wall
            << ",\"replans\":" << result.replans
            << ",\"emergency_stops\":" << result.emergency_stops
            << ",\"recoveries\":" << result.recoveries
            << ",\"global_path_length_m\":" << result.global_path_length_m
            << ",\"costmap_digest\":" << result.costmap_digest
            << ",\"trace\":[";
  for (size_t i = 0; i < result.trajectory.size(); ++i) {
    if (i) std::cout << ',';
    const auto& pose = result.trajectory[i];
    std::cout << "{\"elapsed_s\":" << i * .06 << ",\"x\":" << navigation2d::X(pose)
              << ",\"y\":" << navigation2d::Y(pose)
              << ",\"yaw\":" << navigation2d::Yaw(pose) << '}';
  }
  std::cout << "]}\n";
  return result.status == "SUCCEEDED" && result.collisions == 0 ? 0 : 1;
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n'; return 2;
}
