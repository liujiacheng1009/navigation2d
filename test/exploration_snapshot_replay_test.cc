#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>

#include "navigation2d/application/exploration/planning_snapshot.h"
#include "navigation2d/application/exploration/status_serializer.h"
#include "navigation2d/geometry/vehicle_config.h"

int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  FrontierExplorerConfig vehicle;
  ApplyVehicleConfig(std::string(NAVIGATION2D_SOURCE_DIR) + "/config/navigation2d.yaml", vehicle);
  for (int sample = 0; sample < 8; ++sample) {
    ExplorationGrid map{40,
                        30,
                        static_cast<double>(.1F),
                        sample % 2 ? -2. : -1.,
                        -1.,
                        std::vector<std::int8_t>(1200, -1)};
    for (std::size_t i = 0; i < map.cells.size(); ++i) {
      if (i % 7 == 0) {
        map.cells[i] = 100;
      } else if (i % 3 == 0) {
        map.cells[i] = 0;
      }
    }
    FrontierExplorer explorer(vehicle);
    explorer.UpdateMap(map);
    SnapshotRequest request;
    request.robot = MakePose2d(0., 0., sample % 2 ? .7 : 0.);
    request.goal.x = 1.;
    request.goal.y = .1;
    if (sample < 6) {
      if (auto settled = explorer.SettleGoal(request.goal)) {
        request.goal = *settled;
      }
    }
    if (sample % 4 >= 2) {
      ExplorationGoal mouth;
      mouth.x = .4;
      mouth.frontier_x = .6;
      request.entry = mouth;
    }
    const std::vector<float> ranges{.8F, 1.2F, std::numeric_limits<float>::infinity(), .6F};
    request.scan = ScanEvidenceView{static_cast<double>(-.3F), static_cast<double>(.2F),
                                    static_cast<double>(.02F), 5., ranges};
    const std::vector<std::pair<double, double>> trail{{-.8, 0.}, {-.4, 0.}, {0., 0.}};
    request.trail = trail;
    request.trail_budget_m = sample >= 4 ? 2. : 0.;
    request.returning = sample >= 6;
    request.map_revision = 1000 + sample;
    const auto result = BuildPlanningSnapshot(explorer.observation_map(), request);
    const auto expected = YAML::LoadFile(std::string(NAVIGATION2D_SOURCE_DIR) +
                                         "/test/fixtures/exploration_snapshot/snapshot-" +
                                         std::to_string(sample) + ".json");
    const auto cells = expected["cells"].as<std::vector<int>>();
    if (cells.size() != result.cells.size()) {
      throw std::runtime_error("snapshot size changed");
    }
    for (std::size_t i = 0; i < cells.size(); ++i) {
      const int actual = result.snapshot.free_mask[i] ? 0 : (result.cells[i] < 0 ? 255 : 254);
      if (cells[i] != actual) {
        throw std::runtime_error("frozen snapshot mismatch sample=" + std::to_string(sample) +
                                 " cell=" + std::to_string(i));
      }
    }
  }
  const auto expected = YAML::LoadFile(std::string(NAVIGATION2D_SOURCE_DIR) +
                                       "/test/fixtures/exploration_snapshot/status.json");
  const std::string state = "SELFTEST", message = "quote\" slash\\", source = "localization2d",
                    failure;
  const std::deque<ExplorationGoal> tour;
  const ExplorationGoal active;
  const NavigationTelemetry telemetry;
  const auto serialized = SerializeExplorationStatus(
      {state, message, "exploring", expected["known_cells"].as<std::size_t>(), 0, 0, 0,
       expected["unresolved_frontier_cells"].as<std::size_t>(), 0, tour, active, telemetry.state(),
       failure, 0., 0., 0., 0., false, source});
  std::ifstream frozen(std::string(NAVIGATION2D_SOURCE_DIR) +
                       "/test/fixtures/exploration_snapshot/status.json");
  const std::string original((std::istreambuf_iterator<char>(frozen)), {});
  if (original != serialized) {
    std::ofstream("/tmp/exploration-current-status.json") << serialized;
    throw std::runtime_error("status byte contract changed");
  }
}
