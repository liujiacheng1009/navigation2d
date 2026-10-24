#include "navigation2d/application/algorithm_registry.h"

#include <unistd.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "navigation2d/application/navigation_system.h"

namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("registry contract failed");
  }
}
template <class F>
void Reject(F&& f) {
  bool rejected = false;
  try {
    f();
  } catch (const std::exception&) {
    rejected = true;
  }
  Require(rejected);
}
class StraightPlanner final : public navigation2d::GlobalPlanner {
 public:
  navigation2d::Path Plan(const navigation2d::LayeredCostmap&, const navigation2d::Pose2d& start,
                          const navigation2d::Pose2d& goal) const override {
    return {start, goal};
  }
};
class ConstantController final : public navigation2d::LocalController {
 public:
  navigation2d::Twist2d Compute(
      const navigation2d::Path&, const navigation2d::Pose2d&, navigation2d::Twist2d,
      const navigation2d::LayeredCostmap&,
      const std::vector<navigation2d::PredictedObstacle>&) const override {
    return {.1, 0.};
  }
  bool CollisionImminent(const navigation2d::Pose2d&, navigation2d::Twist2d,
                         const navigation2d::LayeredCostmap&) const override {
    return false;
  }
};
struct MapFile {
  std::filesystem::path directory;
  MapFile() {
    char name[] = "/tmp/navigation2d-registry-XXXXXX";
    const char* created = mkdtemp(name);
    if (!created) {
      throw std::runtime_error("mkdtemp failed");
    }
    directory = created;
    std::ofstream out(directory / "map.json");
    out << R"({"width":50,"height":30,"resolution":0.1,"cells":[)";
    for (int i = 0; i < 1500; ++i) {
      out << (i ? ",0" : "0");
    }
    out << "]}";
  }
  ~MapFile() {
    std::error_code error;
    std::filesystem::remove_all(directory, error);
  }
};
}  // namespace
int main() {
  using namespace navigation2d;
  NavigationConfig c;
  for (const auto* planner : {"astar", "theta_star", "dijkstra", "state_lattice"}) {
    c.planner = planner;
    Require(AlgorithmRegistry::Builtins()->CreatePlanner(c) != nullptr);
  }
  for (const auto* controller : {"rpp", "dwa", "mppi", "mpc"}) {
    c.controller = controller;
    Require(AlgorithmRegistry::Builtins()->CreateController(c) != nullptr);
  }
  c.planner = "astar";
  c.controller = "rpp";
  Reject([&] { AlgorithmRegistry::Builtins()->Validate(c, {true, false}); });
  auto registry = std::make_shared<AlgorithmRegistry>(*AlgorithmRegistry::Builtins());
  registry->RegisterPlanner("test_straight",
                            [](const auto&) { return std::make_unique<StraightPlanner>(); });
  registry->RegisterController("test_constant",
                               [](const auto&) { return std::make_unique<ConstantController>(); });
  Reject([&] {
    registry->RegisterPlanner("test_straight",
                              [](const auto&) { return std::make_unique<StraightPlanner>(); });
  });
  Reject([&] { registry->RegisterController("", {}); });
  c.planner = "missing";
  Reject([&] { registry->Validate(c); });
  c.planner = "test_straight";
  c.controller = "test_constant";
  c.map_resolution = .1;
  c.robot_radius = .15;
  c.collision_monitor_enabled = false;
  c.smoother_enabled = false;
  MapFile fixture;
  // All currently linked algorithms use the shared session and safety path.
  for (const auto* global : {"dijkstra", "astar", "theta_star", "state_lattice"}) {
    for (const auto* local : {"rpp", "dwa", "mppi", "mpc"}) {
      auto combination = c;
      combination.planner = global;
      combination.controller = local;
      NavigationSystem built_in(combination, (fixture.directory / "map.json").string());
      built_in.SetGoal(MakePose2d(3., 1.5, 0.));
      const auto observed = built_in.ComputeCommand(MakePose2d(1., 1.5, 0.), {}, 0.);
      Require(std::isfinite(observed.command.linear) && std::isfinite(observed.command.angular));
      Require(observed.global_path_length_m > 0.);
    }
  }
  NavigationSystem session(c, (fixture.directory / "map.json").string(), registry);
  Require(session.planner_id() == "test_straight" && session.controller_id() == "test_constant");
  session.SetGoal(MakePose2d(3., 1.5, 0.));
  const auto state = session.ComputeCommand(MakePose2d(1., 1.5, 0.), {}, 0.);
  Require(state.status == NavigationStatus::kNavigating);
  Require(state.global_path_length_m > 1.9);
  Require(state.command.linear > 0. && std::isfinite(state.command.linear));
  session.ClearGoal();
  Require(session.ComputeCommand(MakePose2d(1., 1.5, 0.), {}, .06).status ==
          NavigationStatus::kIdle);
  Reject([&] { NavigationSystem invalid(c, "unused", nullptr); });
  registry->RegisterPlanner("null", [](const auto&) { return std::unique_ptr<GlobalPlanner>{}; });
  c.planner = "null";
  Reject([&] { static_cast<void>(registry->CreatePlanner(c)); });
}
