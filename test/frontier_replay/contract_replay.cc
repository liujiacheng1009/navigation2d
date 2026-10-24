#include <cstdint>
#include <string>
#include <utility>

#include "navigation2d/test/frontier_replay/contract_output.h"
#include "navigation2d/test/frontier_replay/contract_timings.h"
using namespace navigation2d;
using namespace navigation2d::test;
namespace {
ContractTimings timings;
ExplorationGrid Scenario(int variant) {
  ExplorationGrid map;
  map.width = 60;
  map.height = 40;
  map.resolution = .1;
  map.origin_x = variant % 2 ? -2. : 0.;
  map.origin_y = variant % 3 ? -1. : 0.;
  map.cells.assign(2400, 100);
  const auto fill = [&](int x0, int x1, int y0, int y1, std::int8_t value) {
    for (int y = y0; y < y1; ++y) {
      for (int x = x0; x < x1; ++x) {
        map.cells[y * map.width + x] = value;
      }
    }
  };
  fill(4, 30, 10, 30, 0);
  fill(30, 55, 10, 30, -1);
  if (variant % 4 == 1) {
    fill(29, 32, 10, 30, 100);
  }
  if (variant % 4 == 2) {
    fill(29, 32, 10, 30, 100);
    fill(29, 32, 17, 23, 0);
  }
  if (variant % 4 == 3) {
    fill(29, 32, 10, 30, 100);
    fill(29, 32, 18, 22, 0);
    fill(40, 50, 14, 26, 0);
  }
  fill(10, 15, 30, 35, -1);
  return map;
}
void Snapshot(FrontierExplorer& explorer, int scenario, int epoch, double rx, double ry) {
  const auto start = ContractTimings::Clock::now();
  const auto selected = explorer.SelectGoals(rx, ry);
  const auto tour = explorer.BuildTour(rx, ry, .4);
  const auto leftovers = explorer.SelectLeftoverApproaches(rx, ry, 3);
  timings.Observe(start);
  std::cout << "{\"scenario\":" << scenario << ",\"epoch\":" << epoch << ",\"selected\":";
  Goals(selected);
  std::cout << ",\"tour\":";
  Goals(tour);
  std::cout << ",\"leftovers\":";
  Goals(leftovers);
  std::cout << ",\"known\":" << explorer.KnownCells()
            << ",\"raw\":" << explorer.raw_frontier_cells()
            << ",\"clusters\":" << explorer.raw_frontier_clusters()
            << ",\"candidates\":" << explorer.candidate_frontier_goals()
            << ",\"executable\":" << explorer.executable_frontier_goals()
            << ",\"unresolved\":" << explorer.unresolved_frontier_cells()
            << ",\"reachable_unknown\":" << explorer.ReachableUnknownCells(rx, ry)
            << ",\"completed\":" << explorer.completed_goals()
            << ",\"failed\":" << explorer.failed_goals()
            << ",\"active_region\":" << explorer.ActiveRegion()
            << ",\"regions_complete\":" << explorer.RegionsComplete()
            << ",\"audit\":" << std::quoted(explorer.RegionAudit())
            << ",\"cover\":" << explorer.ShouldCoverResiduals() << ",\"debug_cells\":[";
  bool first = true;
  for (const auto& cell : explorer.debug_snapshot().cells) {
    if (!first) {
      std::cout << ',';
    }
    std::cout << '[' << cell.x << ',' << cell.y << ',' << cell.cluster_id << ']';
    first = false;
  }
  std::cout << "],\"debug_cluster_goals\":";
  DebugGoals(explorer.debug_snapshot().cluster_goals);
  std::cout << ",\"debug_goals\":";
  DebugGoals(explorer.debug_snapshot().goals);
  std::cout << "}" << std::endl;
  if (!selected.empty()) {
    const auto& goal = selected.front();
    std::cout << "{\"settled\":";
    OptionalGoal(explorer.SettleGoal(goal));
    std::cout << ",\"entry\":";
    OptionalGoal(explorer.SelectEntryProbe(goal, rx, ry, .3, .8));
    std::cout << ",\"goal_frontier\":" << explorer.GoalStillFrontier(goal)
              << ",\"region_frontier\":" << explorer.GoalRegionStillFrontier(goal)
              << ",\"room_beyond\":" << explorer.UnenteredRoomBeyond(goal) << "}" << std::endl;
    explorer.CommitRegion(goal);
    explorer.RecordAttempt(goal, epoch % 2 == 0, false, epoch % 3 == 0);
  }
}
}  // namespace
int main(int argc, char** argv) {
  const bool benchmark = argc == 2 && std::string(argv[1]) == "--benchmark";
  const auto start = ContractTimings::Clock::now();
  ContractOutputCapture output(benchmark);
  std::cout << std::setprecision(17);
  for (int scenario = 0; scenario < 12; ++scenario) {
    FrontierExplorerConfig config;
    config.minimum_frontier_cells = scenario % 2 ? 3 : 6;
    config.required_frontier_observations = scenario % 3 + 1;
    config.footprint_clearance = scenario % 2 ? .3 : .38;
    FrontierExplorer explorer(config);
    auto map = Scenario(scenario);
    const double rx = map.origin_x + 1.5, ry = map.origin_y + 2.;
    for (int epoch = 0; epoch < 6; ++epoch) {
      if (epoch == 2) {
        explorer.CoolRegion(rx + .5, ry, 1.5, 500);
      }
      if (epoch == 3) {
        explorer.ClearFailedBlacklist();
      }
      if (epoch == 4) {
        explorer.ClearBlacklist();
      }
      if (epoch == 5) {
        map.cells[20 * map.width + 30] = 0;
      }
      explorer.UpdateMap(map);
      Snapshot(explorer, scenario, epoch, rx, ry);
    }
  }
  if (benchmark) {
    timings.Report(output.Release(), start);
  }
}
