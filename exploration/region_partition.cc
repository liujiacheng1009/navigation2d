#include "navigation2d/exploration/region_partition.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <utility>

#include "navigation2d/exploration/frontier_types.h"

namespace navigation2d::exploration_internal {
RegionPartition PartitionRegions(const ExplorationGrid& g, double clearance) {
  constexpr int dx[] = {-1, 1, 0, 0};
  constexpr int dy[] = {0, 0, -1, 1};
  const int n = static_cast<int>(g.cells.size());
  auto neighbor = [&](int i, int d) {
    const int c = i % g.width + dx[d], r = i / g.width + dy[d];
    return c < 0 || r < 0 || c >= g.width || r >= g.height ? -1 : r * g.width + c;
  };
  // Erode observed free space at a chassis-derived scale, then geodesically
  // grow the cores back to the actual walls/unknown boundary. A narrow neck
  // becomes an edge between cores, not a circular "room" around a goal.
  // Small components without a core still get a region (no area threshold).
  std::vector<int> distance(n, n), components(n, 0);
  std::deque<int> queue;
  for (int i = 0; i < n; ++i) {
    if (g.cells[i] != 0) {
      distance[i] = 0;
      queue.push_back(i);
    } else if (i % g.width == 0 || i % g.width == g.width - 1 || i / g.width == 0 ||
               i / g.width == g.height - 1) {
      distance[i] = 1;
      queue.push_back(i);
    }
  }
  while (!queue.empty()) {
    const int i = queue.front();
    queue.pop_front();
    for (int d = 0; d < 4; ++d) {
      const int j = neighbor(i, d);
      if (j >= 0 && distance[j] > distance[i] + 1) {
        distance[j] = distance[i] + 1;
        queue.push_back(j);
      }
    }
  }
  const int erosion = std::max(1, static_cast<int>(std::ceil(2. * clearance / g.resolution)));
  int count = 0;
  for (int seed = 0; seed < n; ++seed) {
    if (g.cells[seed] != 0 || distance[seed] <= erosion || components[seed]) {
      continue;
    }
    components[seed] = ++count;
    queue.push_back(seed);
    while (!queue.empty()) {
      const int i = queue.front();
      queue.pop_front();
      for (int d = 0; d < 4; ++d) {
        const int j = neighbor(i, d);
        if (j >= 0 && g.cells[j] == 0 && distance[j] > erosion && !components[j]) {
          components[j] = count;
          queue.push_back(j);
        }
      }
    }
  }
  // Keep the eroded cores separate.  The previous implementation grew each
  // core through every free cell, which reconnected rooms through a narrow
  // doorway and made the scheduler lose its DFS context.  The remaining
  // corridor cells are assigned below by a multi-source wavefront, so they
  // still belong to a region without becoming a bridge between regions.
  for (int i = 0; i < n; ++i) {
    if (components[i]) {
      queue.push_back(i);
    }
  }
  auto grow_core = [&]() {
    while (!queue.empty()) {
      const int i = queue.front();
      queue.pop_front();
      for (int d = 0; d < 4; ++d) {
        const int j = neighbor(i, d);
        if (j >= 0 && g.cells[j] == 0 && distance[j] > erosion && !components[j]) {
          components[j] = components[i];
          queue.push_back(j);
        }
      }
    }
  };
  grow_core();
  if (count == 0) {
    // A map that contains only corridors still needs one stable region.  Do
    // not manufacture one region per cell when there is no eroded core.
    for (int seed = 0; seed < n; ++seed) {
      if (g.cells[seed] != 0) {
        continue;
      }
      if (components[seed]) {
        continue;
      }
      components[seed] = ++count;
      queue.push_back(seed);
      while (!queue.empty()) {
        const int i = queue.front();
        queue.pop_front();
        for (int d = 0; d < 4; ++d) {
          const int j = neighbor(i, d);
          if (j >= 0 && g.cells[j] == 0 && !components[j]) {
            components[j] = count;
            queue.push_back(j);
          }
        }
      }
    }
  } else {
    // Assign non-core free cells to the nearest core.  A tie is deterministic
    // and the label boundary remains an explicit graph edge for BFS fallback.
    std::deque<int> wave;
    for (int i = 0; i < n; ++i) {
      if (components[i]) {
        wave.push_back(i);
      }
    }
    while (!wave.empty()) {
      const int i = wave.front();
      wave.pop_front();
      for (int d = 0; d < 4; ++d) {
        const int j = neighbor(i, d);
        if (j >= 0 && g.cells[j] == 0 && !components[j]) {
          components[j] = components[i];
          wave.push_back(j);
        }
      }
    }
  }

  // Every disconnected observed component needs an identity, even if another
  // component has an eroded core. Previously narrow islands were label zero.
  for (int seed = 0; seed < n; ++seed) {
    if (g.cells[seed] != 0 || components[seed]) {
      continue;
    }
    components[seed] = ++count;
    queue.push_back(seed);
    while (!queue.empty()) {
      const int i = queue.front();
      queue.pop_front();
      for (int d = 0; d < 4; ++d) {
        const int j = neighbor(i, d);
        if (j >= 0 && g.cells[j] == 0 && !components[j]) {
          components[j] = count;
          queue.push_back(j);
        }
      }
    }
  }

  return {std::move(components), count};
}
}  // namespace navigation2d::exploration_internal
