#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "navigation2d/planning/search_core.h"

namespace navigation2d::planning_internal {

using NeighborExpansion = std::function<void(int, const SuccessorVisitor&)>;
using PairHeuristic = std::function<double(int, int)>;

struct IncrementalSearchResult {
  bool found = false;
  std::vector<int> path;
  SearchDiagnostics diagnostics;
  bool reused = false;
  std::size_t repaired_states = 0;
};

// Goal-rooted D* Lite. g/rhs and the lazy priority queue persist across calls;
// moving starts update km, while changed_states names graph vertices whose
// outgoing edge costs changed.
class DStarLite {
 public:
  DStarLite(int state_count, int goal, NeighborExpansion successors,
            NeighborExpansion predecessors, PairHeuristic heuristic);
  ~DStarLite();
  DStarLite(DStarLite&&) noexcept;
  DStarLite& operator=(DStarLite&&) noexcept;
  DStarLite(const DStarLite&) = delete;
  DStarLite& operator=(const DStarLite&) = delete;

  IncrementalSearchResult Plan(int start, const std::vector<int>& changed_states,
                               const SearchOptions& options = {});

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace navigation2d::planning_internal
