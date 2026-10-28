#include "navigation2d/exploration/region_graph_observation.h"

#include <algorithm>
#include <cmath>

#include "navigation2d/exploration/frontier_types.h"
namespace navigation2d::exploration_internal {
namespace {
constexpr int dx[] = {-1, 1, 0, 0};
constexpr int dy[] = {0, 0, -1, 1};
}  // namespace
void ObserveRegionGraph(const ExplorationGrid& g, std::span<const RegionId> labels,
                        std::map<RegionId, RegionRecord>& regions) {
  const int n = static_cast<int>(g.cells.size());
  auto neighbor = [&](int i, int d) {
    const int c = i % g.width + dx[d], r = i / g.width + dy[d];
    return c < 0 || r < 0 || c >= g.width || r >= g.height ? -1 : r * g.width + c;
  };
  for (auto& [id, r] : regions) {
    (void)id;
    r.visible = false;
    r.free_cells = r.frontier_cells = r.candidates = 0;
    r.passage = false;
    r.neighbors.clear();
  }
  std::map<RegionId, std::uint64_t> hashes;
  for (int i = 0; i < n; ++i) {
    if (labels[i]) {
      auto& r = regions.at(labels[i]);
      r.visible = true;
      ++r.free_cells;
      bool frontier = false;
      for (int d = 0; d < 4; ++d) {
        const int j = neighbor(i, d);
        if (j >= 0 && g.cells[j] < 0) {
          frontier = true;
        }
        if (j >= 0 && labels[j] && labels[j] != labels[i]) {
          r.neighbors.insert(labels[j]);
        }
      }
      if (frontier) {
        ++r.frontier_cells;
      }
      // Commutative world-cell digest unaffected by origin padding / iteration
      // order. No global known-cell growth can complete or unblock a room.
      const auto wx = static_cast<std::int64_t>(
          std::llround((g.origin_x + (i % g.width + .5) * g.resolution) / g.resolution));
      const auto wy = static_cast<std::int64_t>(
          std::llround((g.origin_y + (i / g.width + .5) * g.resolution) / g.resolution));
      std::uint64_t h = static_cast<std::uint64_t>(wx) * 0x9e3779b97f4a7c15ULL;
      h ^= static_cast<std::uint64_t>(wy) * 0xbf58476d1ce4e5b9ULL;
      h ^= frontier ? 0x94d049bb133111ebULL : 0;
      hashes[r.id] += h ^ (h >> 31);
    }
  }
  for (auto& [id, r] : regions) {
    if (r.visible) {
      const bool changed = r.evidence != hashes[id];
      r.stable_empty_updates =
          r.frontier_cells ? 0 : (changed ? 1 : std::min(2u, r.stable_empty_updates + 1));
      if (r.stable_empty_updates >= 2) {
        r.state = RegionState::kComplete;
      } else if (changed || r.state == RegionState::kComplete) {
        r.state = RegionState::kPending;
      }
      r.evidence = hashes[id];
    }
  }
}
}  // namespace navigation2d::exploration_internal
