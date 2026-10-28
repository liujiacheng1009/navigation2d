#pragma once
#include <cstddef>
#include <cstdint>
#include <set>

namespace navigation2d {
using RegionId = std::uint64_t;
enum class RegionState { kPending, kActive, kWaiting, kBlocked, kComplete };
struct RegionRecord {
  RegionId id = 0, parent = 0;
  RegionState state = RegionState::kPending;
  bool visible = false;
  // Set by the explorer when this region still has a passable door.
  // Raw frontier_cells do not keep the room open.
  bool passage = false;
  std::size_t free_cells = 0, frontier_cells = 0, candidates = 0;
  unsigned stable_empty_updates = 0;
  std::uint64_t evidence = 0;
  std::set<RegionId> neighbors;
};

}  // namespace navigation2d
