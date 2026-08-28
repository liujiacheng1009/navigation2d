#include <chrono>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include "navigation2d/simulation/navigation_simulator.h"

namespace {
double Percentile(std::vector<double> samples, double quantile) {
  if (samples.empty()) return 0.;
  std::sort(samples.begin(), samples.end());
  const double position = quantile * static_cast<double>(samples.size() - 1);
  const auto lower = static_cast<std::size_t>(position);
  const auto upper = std::min(lower + 1, samples.size() - 1);
  const double fraction = position - static_cast<double>(lower);
  return samples[lower] + fraction * (samples[upper] - samples[lower]);
}
}

int main(int argc, char** argv) try {
  if (argc < 10) throw std::runtime_error("usage: navigation2d_benchmark CONFIG CASE WORLD SX SY SYAW GX GY GYAW [--obstacle APPEAR DISAPPEAR X Y] [--moving-obstacle APPEAR DISAPPEAR X Y VX VY]...");
  const auto started = std::chrono::steady_clock::now();
  auto config = navigation2d::NavigationConfig::Load(argv[1]);
  std::vector<navigation2d::simulation::ObstacleEvent> obstacles;
  for (int i = 10; i < argc;) {
    const std::string option = argv[i];
    if (option == "--obstacle" && i + 4 < argc) {
      obstacles.push_back({std::stod(argv[i + 1]), std::stod(argv[i + 2]),
                           std::stod(argv[i + 3]), std::stod(argv[i + 4])}); i += 5;
    } else if (option == "--moving-obstacle" && i + 6 < argc) {
      obstacles.push_back({std::stod(argv[i + 1]), std::stod(argv[i + 2]),
                           std::stod(argv[i + 3]), std::stod(argv[i + 4]),
                           std::stod(argv[i + 5]), std::stod(argv[i + 6])}); i += 7;
    } else if (option == "--planner" && i + 1 < argc) {
      config.planner = argv[i + 1]; i += 2;
    } else if (option == "--controller" && i + 1 < argc) {
      config.controller = argv[i + 1]; i += 2;
    } else if (option == "--mpc-solver" && i + 1 < argc) {
      config.mpc_solver = argv[i + 1]; i += 2;
    } else throw std::runtime_error("invalid benchmark option: " + option);
  }
  if (config.mpc_solver != "shooting" && config.mpc_solver != "acados")
    throw std::runtime_error("invalid MPC solver: " + config.mpc_solver);
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
            << ",\"controller_solve_samples\":" << result.controller_solve_samples_us.size()
            << ",\"controller_solve_p50_us\":" << Percentile(result.controller_solve_samples_us, .50)
            << ",\"controller_solve_p95_us\":" << Percentile(result.controller_solve_samples_us, .95)
            << ",\"controller_solve_p99_us\":" << Percentile(result.controller_solve_samples_us, .99)
            << ",\"replans\":" << result.replans
            << ",\"emergency_stops\":" << result.emergency_stops
            << ",\"recoveries\":" << result.recoveries
            << ",\"global_path_length_m\":" << result.global_path_length_m
            << ",\"global_plan_expansions\":" << result.global_planning.expansions
            << ",\"global_plan_generated\":" << result.global_planning.generated
            << ",\"global_plan_seconds\":" << result.global_planning.elapsed_s
            << ",\"global_plan_first_solution_seconds\":" << result.global_planning.first_solution_s
            << ",\"global_plan_suboptimality_bound\":" << result.global_planning.suboptimality_bound
            << ",\"obstacle_heuristic_cache_hits\":" << result.obstacle_heuristic_cache_hits
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
