#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <source_location>
#include <stdexcept>
#include <utility>

#include "navigation2d/application/exploration/navigation_session.h"

namespace {
void Require(bool condition,
             const std::source_location location = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("navigation session contract failed at line " +
                             std::to_string(location.line()));
  }
}
template <class F>
void Reject(F&& action) {
  bool rejected = false;
  try {
    action();
  } catch (const std::exception&) {
    rejected = true;
  }
  Require(rejected);
}
struct MapFile {
  std::filesystem::path directory;
  MapFile() {
    char name[] = "/tmp/exploration-session-XXXXXX";
    const char* created = mkdtemp(name);
    if (!created) {
      throw std::runtime_error("mkdtemp failed");
    }
    directory = created;
    std::ofstream out(directory / "map.json");
    out << R"({"width":50,"height":30,"resolution":0.1,"cells":[)";
    for (int i = 0; i < 1500; ++i) {
      if (i) {
        out << ',';
      }
      // An occupied island provides deterministic start/goal rejection.
      out << ((i % 50 >= 35 && i % 50 <= 40 && i / 50 >= 10 && i / 50 <= 20) ? 254 : 0);
    }
    out << "]}";
  }
  ~MapFile() {
    std::error_code ignored;
    std::filesystem::remove_all(directory, ignored);
  }
};
class NoRoutePlanner final : public navigation2d::GlobalPlanner {
 public:
  navigation2d::Path Plan(const navigation2d::LayeredCostmap&, const navigation2d::Pose2d&,
                          const navigation2d::Pose2d&) const override {
    return {};
  }
};
}  // namespace

int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  MapFile map;
  const auto map_path = (map.directory / "map.json").string();
  NavigationConfig config;
  config.planner = "astar";
  config.controller = "rpp";
  config.map_resolution = .1;
  config.robot_radius = .15;
  config.smoother_enabled = false;
  config.collision_monitor_enabled = false;
  const auto start = MakePose2d(1., 1.5, 0.);
  const auto goal = MakePose2d(3., 1.5, 0.);
  const auto occupied = MakePose2d(3.75, 1.5, 0.);
  NavigationSession session;
  Require(!session.has_session() && session.stage() == SessionStage::kEmpty);
  Require(session.Cancel() == Cancellation::kNoSession);
  Reject([&] { session.Accept(); });
  Reject([&] { static_cast<void>(session.Step(start, {}, 0.)); });
  session.Begin(config, map_path, goal, {});
  Require(session.stage() == SessionStage::kPreparing);
  Reject([&] { session.Accept(); });
  Reject([&] { static_cast<void>(session.Step(start, {}, 0.)); });
  const auto initial = session.Probe(start, {}, 0.);
  Require(initial.has_route() && initial.failure_reason().empty());
  Require(!session.path().empty() && session.stage() == SessionStage::kReady);
  Reject([&] { static_cast<void>(session.Step(start, {}, .06)); });
  session.Accept();
  Require(session.stage() == SessionStage::kActive);
  // Compare the exact underlying lifecycle, including a cycle after acceptance.
  NavigationSystem original(config, map_path);
  original.BanPathSignatures({});
  original.SetGoal(goal);
  const auto original_initial = original.ComputeCommand(start, {}, 0.);
  Require(initial.state.path_signature == original_initial.path_signature);
  Require(initial.state.command.linear == original_initial.command.linear);
  const auto actual = session.Step(start, {}, .06);
  const auto expected = original.ComputeCommand(start, {}, .06);
  Require(actual.command.linear == expected.command.linear);
  Require(actual.command.angular == expected.command.angular);
  Require(actual.status == expected.status && actual.path_signature == expected.path_signature);

  // Continuous handoff: hold previous execution, reject a new candidate,
  // then restore the previous algorithm state without reinitializing it.
  auto previous = std::move(session);
  Require(!session.has_session() && session.stage() == SessionStage::kEmpty);
  session.Begin(config, map_path, occupied, {});
  const auto bad_goal = session.Probe(start, {}, .12);
  Require(bad_goal.failure == InitialPlanFailure::kGoalOccupied);
  Require(bad_goal.failure_reason().find("goal occupied") != std::string::npos);
  Reject([&] { session.Accept(); });
  Require(session.Cancel() == Cancellation::kDiscardedPreparation);
  session = std::move(previous);
  Require(session.stage() == SessionStage::kActive && !previous.has_session());
  const auto restored = session.Step(start, {}, .12);
  const auto continued = original.ComputeCommand(start, {}, .12);
  Require(restored.command.linear == continued.command.linear);
  Require(restored.command.angular == continued.command.angular);
  Require(session.Cancel() == Cancellation::kStoppedActive);
  Require(session.Cancel() == Cancellation::kNoSession);
  Reject([&] { static_cast<void>(session.Step(start, {}, .18)); });

  session.Begin(config, map_path, goal, {});
  Require(session.Probe(occupied, {}, 0.).failure == InitialPlanFailure::kStartOccupied);
  Require(session.Cancel() == Cancellation::kDiscardedPreparation);
  // Already at goal is valid even when no nonzero-length route is needed.
  session.Begin(config, map_path, start, {});
  Require(session.Probe(start, {}, 0.).has_route());
  session.Accept();
  Require(session.Step(start, {}, .06).status == NavigationStatus::kSucceeded);
  session.Cancel();

  auto registry = std::make_shared<AlgorithmRegistry>(*AlgorithmRegistry::Builtins());
  registry->RegisterPlanner("no_route",
                            [](const auto&) { return std::make_unique<NoRoutePlanner>(); });
  registry->RegisterPlanner("broken", [](const auto&) -> std::unique_ptr<GlobalPlanner> {
    throw std::runtime_error("injected initialization failure");
  });
  NavigationSession injected(registry);
  config.planner = "no_route";
  injected.Begin(config, map_path, goal, {});
  Require(injected.Probe(start, {}, 0.).failure == InitialPlanFailure::kNoRoute);
  Reject([&] { injected.Accept(); });
  Require(injected.Cancel() == Cancellation::kDiscardedPreparation);
  config.planner = "broken";
  Reject([&] { injected.Begin(config, map_path, goal, {}); });
  injected.Cancel();
  Require(injected.stage() == SessionStage::kEmpty);
  config.planner = "astar";
  injected.Begin(config, map_path, goal, {});
  Require(injected.Probe(start, {}, 0.).has_route());
  injected.Accept();
  Require(injected.Cancel() == Cancellation::kStoppedActive);
  Reject([&] { NavigationSession invalid(nullptr); });
}
