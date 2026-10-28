#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "navigation2d/ros/exploration/explorer_node.h"
#include "navigation2d/test/explorer_ros_loop_scene.h"
#include "yaml-cpp/yaml.h"

int main(int argc, char** argv) {
  try {
    if (argc != 4 || std::getenv("NAVIGATION2D_ISOLATED_ROS_TEST") == nullptr) {
      throw std::runtime_error("use the isolated ROS loop runner: scenario planner controller");
    }
    const std::string scenario = argv[1], planner = argv[2], controller = argv[3];
    if (scenario != "return" && scenario != "frontier") {
      throw std::runtime_error("unknown synthetic scenario");
    }
    const auto folder = std::filesystem::path("/tmp/explorer-ros-loops") /
                        (scenario + "-" + planner + "-" + controller);
    std::filesystem::create_directories(folder);
    const auto result = (folder / "result.json").string();
    std::filesystem::remove(result);
    setenv("SWEEPNAV_EXPLORATION_RESULT", result.c_str(), 1);
    setenv("SWEEPNAV_EXPLORATION_TIMEOUT", "90", 1);
    setenv("SWEEPNAV_SIMULATION_SPEED", "1", 1);
    rclcpp::init(1, argv);
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--log-level", "warn"});
    for (const std::string prefix : {"explore", "entry", "return"}) {
      options.append_parameter_override(prefix + ".global_planner", planner);
      options.append_parameter_override(prefix + ".local_controller", controller);
    }
    auto scene = std::make_shared<navigation2d::test::ExplorerRosLoopScene>(
        scenario == "frontier", (folder / "commands.csv").string());
    auto explorer = navigation2d::exploration_ros::MakeAutonomousExplorer(
        navigation2d::AlgorithmRegistry::Builtins(), options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(scene);
    executor.add_node(explorer);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(105);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_once(std::chrono::milliseconds(20));
    }
    if (rclcpp::ok()) {
      rclcpp::shutdown();
      throw std::runtime_error("real ROS mission did not finish before test deadline");
    }
    const auto report = YAML::LoadFile(result);
    const bool success =
        report["status"].as<std::string>() == "COMPLETE" &&
        report["planning"].as<std::string>() == "navigation2d/" + planner + "+" + controller &&
        report["return_error_m"].as<double>() <= .35 && scene->home_error() <= .35 &&
        scene->returning && scene->moving_commands > 10 && scene->planned_paths > 0 &&
        scene->driven_distance > .5 && (scenario != "frontier" || scene->map_revealed);
    std::cout << "scenario=" << scenario << " planner=" << planner << " controller=" << controller
              << " commands=" << scene->commands << " moving=" << scene->moving_commands
              << " paths=" << scene->planned_paths << " distance=" << scene->driven_distance
              << " home_error=" << scene->home_error() << " success=" << success << '\n';
    return success ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
