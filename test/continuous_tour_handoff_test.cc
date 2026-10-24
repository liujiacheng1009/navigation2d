#include "navigation2d/application/exploration/continuous_tour_handoff.h"

#include <fstream>
#include <source_location>
#include <stdexcept>
#include <string>

#include "navigation2d/exploration/frontier_explorer.h"
#include "temporary_directory.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
void Require(bool value, std::source_location location = std::source_location::current()) {
  if (!value) {
    throw std::runtime_error("continuous handoff at " + std::to_string(location.line()));
  }
}
class CountedController final : public LocalController {
 public:
  explicit CountedController(int& alive) : alive_(alive) { ++alive_; }
  ~CountedController() override { --alive_; }
  Twist2d Compute(const Path&, const Pose2d&, Twist2d, const LayeredCostmap&,
                  const std::vector<PredictedObstacle>&) const override {
    return {};
  }
  bool CollisionImminent(const Pose2d&, Twist2d, const LayeredCostmap&) const override {
    return false;
  }

 private:
  int& alive_;
};
struct Actions final : ContinuousHandoffActions {
  NavigationSession& navigation;
  const NavigationConfig& config;
  const std::string& map;
  int& alive;
  bool accept = false;
  int calls = 0, reports = 0;
  Actions(NavigationSession& n, const NavigationConfig& c, const std::string& m, int& count)
      : navigation(n), config(c), map(m), alive(count) {}
  bool NavigateHandoff(const ExplorationGoal& goal) override {
    ++calls;
    navigation.Begin(config, map, MakePose2d(goal.x, goal.y, goal.yaw), {});
    if (!accept) {
      return false;
    }
    Require(navigation.Probe(MakePose2d(1.4, 1.5, 0.), {}, 0.).has_route());
    navigation.Accept();
    return true;
  }
  void HandoffAccepted() override {
    Require(alive == 2);  // The previous controller survives bookkeeping/reporting.
    ++reports;
  }
};
}  // namespace
int main() {
  test::TemporaryDirectory temporary;
  const auto map_path = (temporary.path() / "map.json").string();
  {
    std::ofstream out(map_path);
    out << R"({"width":50,"height":30,"resolution":0.1,"cells":[)";
    for (int i = 0; i < 1500; ++i) {
      if (i) {
        out << ',';
      }
      out << 0;
    }
    out << "]}";
  }
  int alive = 0;
  auto registry = std::make_shared<AlgorithmRegistry>(*AlgorithmRegistry::Builtins());
  registry->RegisterController("handoff-test", [&](const NavigationConfig&) {
    return std::make_unique<CountedController>(alive);
  });
  NavigationConfig config;
  config.map_resolution = .1;
  config.controller = "handoff-test";
  NavigationSession navigation(registry);
  navigation.Begin(config, map_path, MakePose2d(1.5, 1.5, 0.), {});
  Require(navigation.Probe(MakePose2d(1.4, 1.5, 0.), {}, 0.).has_route());
  navigation.Accept();
  const auto* original_route = navigation.path().data();
  Require(alive == 1);
  FrontierExplorer explorer;
  ExplorationGrid observed{50, 30, .1, 0., 0., std::vector<std::int8_t>(1500, 0)};
  for (int row = 0; row < 30; ++row) {
    for (int col = 40; col < 50; ++col) {
      observed.cells[row * 50 + col] = -1;
    }
  }
  explorer.UpdateMap(observed);
  ExplorationGoal active;
  active.x = 1.5;
  active.y = 1.5;
  active.frontier_x = 3.95;
  active.frontier_y = 1.5;
  auto next = active;
  next.x = 1.8;
  auto resolved = active;
  resolved.frontier_x = 100.;
  TourExecution tour;
  tour.RetryLater(active);
  tour.RetryLater(resolved);
  tour.RetryLater(next);
  Actions actions(navigation, config, map_path, alive);
  Require(!TryContinuousTourHandoff(tour, explorer, navigation, active, {1.4, 1.5, 0.}, actions));
  Require(actions.calls == 1 && actions.reports == 0 && alive == 1);
  Require(navigation.stage() == SessionStage::kActive &&
          navigation.path().data() == original_route);
  Require(tour.goals().size() == 3 && explorer.failed_goals() == 1);
  actions.accept = true;
  Require(TryContinuousTourHandoff(tour, explorer, navigation, active, {1.4, 1.5, 0.}, actions));
  Require(actions.calls == 2 && actions.reports == 1 && alive == 1);
  Require(tour.goals().size() == 1 && tour.goals().front().x == next.x);
  Require(explorer.completed_goals() == 1 && navigation.stage() == SessionStage::kActive);
  tour.Suspend();
  auto behind = active;
  behind.x = 1.;
  tour.RetryLater(active);
  tour.RetryLater(behind);
  Require(!TryContinuousTourHandoff(tour, explorer, navigation, active, {1.4, 1.5, 0.}, actions));
  Require(actions.calls == 2);  // A moving handoff cannot reverse direction.
}
