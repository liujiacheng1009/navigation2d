#pragma once

#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

namespace navigation2d::planning_internal {

struct SearchSuccessor {
  int state = -1;
  double transition_cost = 0.;
  // Defaults to the state currently being expanded. Graphs such as Theta*
  // may instead relax a successor directly from the current state's parent.
  int predecessor = -1;
};

struct SearchOptions {
  double heuristic_weight = 1.;
  std::size_t max_expansions = std::numeric_limits<std::size_t>::max();
  double max_planning_time_s = std::numeric_limits<double>::infinity();
};

struct SearchDiagnostics {
  std::size_t expansions = 0;
  std::size_t generated = 0;
  double elapsed_s = 0.;
  double first_solution_s = 0.;
  bool expansion_limit_reached = false;
  bool time_limit_reached = false;
};

struct SearchResult {
  bool found = false;
  std::vector<double> cost_to_come;
  std::vector<int> parent;
  SearchDiagnostics diagnostics;
  double suboptimality_bound = std::numeric_limits<double>::infinity();
};

using SearchHeuristic = std::function<double(int)>;
using GoalPredicate = std::function<bool(int)>;
using SuccessorVisitor = std::function<void(const SearchSuccessor&)>;
using SearchExpansion = std::function<void(int, int, const SuccessorVisitor&)>;

// Generic best-first search over a dense integer state space. A state encoder
// owned by the graph maps 2D or SE(2) nodes to [0, state_count). Equal-priority
// entries are expanded in insertion order, making fixed inputs deterministic.
SearchResult BestFirstSearch(int state_count, int start, const GoalPredicate& is_goal,
                             const SearchHeuristic& heuristic,
                             const SearchExpansion& expand,
                             const SearchOptions& options = {});

// ARA*: reuses g-values and OPEN/INCONS while decreasing epsilon. Returns the
// best solution available within the shared expansion/time budget.
SearchResult AnytimeRepairingAStar(int state_count, int start, int goal,
                                   const SearchHeuristic& heuristic,
                                   const SearchExpansion& expand,
                                   double initial_weight, double weight_decrement,
                                   const SearchOptions& options = {});

std::vector<int> RestoreStatePath(const SearchResult& result, int start, int goal);

}  // namespace navigation2d::planning_internal
