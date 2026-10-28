#include <bit>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include "navigation2d/exploration/frontier_explorer.h"

namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("frontier boundary evidence contract failed");
  }
}
struct Fingerprint {
  std::uint64_t value = 1469598103934665603ULL;
  void Add(std::uint64_t input) {
    value ^= input;
    value *= 1099511628211ULL;
  }
  void Add(double input) { Add(std::bit_cast<std::uint64_t>(input)); }
};
}  // namespace

int main() {
  using namespace navigation2d;
  FrontierExplorer explorer;
  ExplorationGoal goal;
  Require(!explorer.GoalStillFrontier(goal) && !explorer.GoalRegionStillFrontier(goal) &&
          !explorer.UnenteredRoomBeyond(goal) && explorer.MouthFreeDepth(goal, 0., 0.) == 0.);
  // A failed execution graph provides no new reachability evidence. It must
  // not erase the observed unfinished boundary or manufacture executable goals.
  FrontierExplorer isolated;
  ExplorationGrid pocket;
  pocket.width = pocket.height = 7;
  pocket.resolution = .1;
  pocket.cells.assign(49, -1);
  pocket.cells[3 * 7 + 3] = 0;
  pocket.cells[3 * 7 + 4] = 100;
  isolated.UpdateMap(pocket);
  const auto unresolved = isolated.unresolved_frontier_cells();
  Require(unresolved > 0);
  Require(isolated.SelectGoals(.35, .35).empty());
  Require(isolated.raw_frontier_cells() == 0 && isolated.raw_frontier_clusters() == 0 &&
          isolated.unresolved_frontier_cells() == unresolved &&
          isolated.debug_snapshot().cluster_goals.empty());
  ExplorationGrid map;
  map.width = map.height = 30;
  map.resolution = .1;
  map.origin_x = -1.;
  map.origin_y = 2.;
  map.cells.assign(900, 0);
  map.cells[15 * 30 + 15] = -1;
  goal.x = map.origin_x + 1.25;
  goal.y = map.origin_y + 1.55;
  goal.frontier_x = map.origin_x + 1.45;
  goal.frontier_y = goal.y;
  explorer.UpdateMap(map);
  Require(explorer.GoalStillFrontier(goal) && explorer.GoalRegionStillFrontier(goal));
  Require(!explorer.UnenteredRoomBeyond(goal));  // One unknown cell is not a room.
  map.cells[15 * 30 + 15] = 0;
  explorer.UpdateMap(map);
  Require(!explorer.GoalStillFrontier(goal) && !explorer.GoalRegionStillFrontier(goal));
  Require(explorer.MouthFreeDepth(goal, goal.x, goal.y) > 1.);
  Fingerprint fingerprint;
  for (int scenario = 0; scenario < 32; ++scenario) {
    map.cells.assign(900, 100);
    // Vary observed throat width, unknown-room depth and a blocking jamb.
    for (int row = 3; row < 27; ++row) {
      for (int col = 3; col < 27; ++col) {
        if (col < 15) {
          map.cells[row * 30 + col] = 0;
        }
        if (col >= 15 && col < 16 + scenario % 10 && row >= 9 && row < 12 + scenario % 10) {
          map.cells[row * 30 + col] = scenario % 4 == 0 ? 0 : -1;
        }
      }
    }
    if (scenario % 3 == 0) {
      map.cells[15 * 30 + 14] = 100;
    }
    const auto observed = map.cells;
    explorer.UpdateMap(map);
    for (int row = 5; row < 25; row += 2) {
      for (int col = 8; col < 24; col += 2) {
        goal.frontier_x = map.origin_x + (col + .5) * map.resolution;
        goal.frontier_y = map.origin_y + (row + .5) * map.resolution;
        goal.x = goal.frontier_x - .3;
        goal.y = goal.frontier_y;
        fingerprint.Add(static_cast<std::uint64_t>(explorer.GoalStillFrontier(goal)));
        fingerprint.Add(static_cast<std::uint64_t>(explorer.GoalRegionStillFrontier(goal)));
        fingerprint.Add(static_cast<std::uint64_t>(explorer.UnenteredRoomBeyond(goal)));
        fingerprint.Add(explorer.MouthFreeDepth(goal, goal.x, goal.y));
      }
    }
    for (double remaining : {0., 4., 6., -1.}) {
      fingerprint.Add(static_cast<std::uint64_t>(explorer.ShouldPreemptForNearerMouth(
          map.origin_x + .85, map.origin_y + 1.55, goal, remaining)));
    }
    for (int repeat = 0; repeat < 2; ++repeat) {
      const auto candidates = explorer.SelectGoals(map.origin_x + .85, map.origin_y + 1.55);
      fingerprint.Add(static_cast<std::uint64_t>(candidates.size()));
      for (const auto& candidate : candidates) {
        Require(std::isfinite(candidate.x) && std::isfinite(candidate.y));
        fingerprint.Add(candidate.x);
        fingerprint.Add(candidate.y);
        fingerprint.Add(candidate.frontier_x);
        fingerprint.Add(candidate.frontier_y);
        fingerprint.Add(candidate.score);
        fingerprint.Add(candidate.information_gain);
      }
      fingerprint.Add(static_cast<std::uint64_t>(explorer.raw_frontier_cells()));
      fingerprint.Add(static_cast<std::uint64_t>(explorer.unresolved_frontier_cells()));
      fingerprint.Add(static_cast<std::uint64_t>(explorer.raw_frontier_clusters()));
      fingerprint.Add(static_cast<std::uint64_t>(explorer.candidate_frontier_goals()));
      fingerprint.Add(static_cast<std::uint64_t>(explorer.executable_frontier_goals()));
      explorer.RefreshDebugFrontierCells();
      const auto& snapshot = explorer.debug_snapshot();
      fingerprint.Add(static_cast<std::uint64_t>(snapshot.cells.size()));
      for (const auto& cell : snapshot.cells) {
        fingerprint.Add(cell.x);
        fingerprint.Add(cell.y);
        fingerprint.Add(static_cast<std::uint64_t>(cell.cluster_id));
      }
      for (const auto* goals : {&snapshot.cluster_goals, &snapshot.goals}) {
        fingerprint.Add(static_cast<std::uint64_t>(goals->size()));
        for (const auto& debug : *goals) {
          fingerprint.Add(debug.x);
          fingerprint.Add(debug.y);
          fingerprint.Add(debug.frontier_x);
          fingerprint.Add(debug.frontier_y);
          fingerprint.Add(debug.score);
          fingerprint.Add(debug.information_gain);
          fingerprint.Add(static_cast<std::uint64_t>(debug.cluster_id));
          fingerprint.Add(static_cast<std::uint64_t>(debug.frontier_cells));
        }
      }
    }
    Require(map.cells == observed);
  }
  std::cout << "boundary-query-fingerprint " << fingerprint.value << '\n';
}
