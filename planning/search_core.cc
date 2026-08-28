#include "navigation2d/planning/search_core.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <queue>
#include <stdexcept>

namespace navigation2d::planning_internal {
namespace {
struct QueueEntry {
  double priority;
  double cost_to_come;
  std::uint64_t sequence;
  int state;
};

struct LaterEntry {
  bool operator()(const QueueEntry& lhs, const QueueEntry& rhs) const {
    if (lhs.priority != rhs.priority) return lhs.priority > rhs.priority;
    return lhs.sequence > rhs.sequence;
  }
};
}  // namespace

SearchResult BestFirstSearch(int state_count, int start, const GoalPredicate& is_goal,
                             const SearchHeuristic& heuristic,
                             const SearchExpansion& expand, const SearchOptions& options) {
  if (state_count <= 0 || start < 0 || start >= state_count)
    throw std::invalid_argument("invalid search state space");
  if (!(options.heuristic_weight >= 1.) || options.max_planning_time_s <= 0.)
    throw std::invalid_argument("invalid search options");

  SearchResult result;
  result.cost_to_come.assign(state_count, std::numeric_limits<double>::infinity());
  result.parent.assign(state_count, -1);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, LaterEntry> open;
  std::uint64_t sequence = 0;
  result.cost_to_come[start] = 0.;
  open.push({options.heuristic_weight * heuristic(start), 0., sequence++, start});
  const auto started = std::chrono::steady_clock::now();

  while (!open.empty()) {
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    if (elapsed >= options.max_planning_time_s) {
      result.diagnostics.time_limit_reached = true;
      break;
    }
    if (result.diagnostics.expansions >= options.max_expansions) {
      result.diagnostics.expansion_limit_reached = true;
      break;
    }
    const QueueEntry current = open.top();
    open.pop();
    if (current.cost_to_come != result.cost_to_come[current.state]) continue;
    ++result.diagnostics.expansions;
    if (is_goal(current.state)) {
      result.found = true;
      break;
    }
    expand(current.state, result.parent[current.state],
           [&](const SearchSuccessor& successor) {
      if (successor.state < 0 || successor.state >= state_count ||
          !std::isfinite(successor.transition_cost) || successor.transition_cost < 0.)
        throw std::runtime_error("invalid search successor");
      ++result.diagnostics.generated;
      const int predecessor = successor.predecessor < 0 ? current.state : successor.predecessor;
      if (predecessor < 0 || predecessor >= state_count ||
          !std::isfinite(result.cost_to_come[predecessor]))
        throw std::runtime_error("invalid search predecessor");
      const double candidate = result.cost_to_come[predecessor] + successor.transition_cost;
      if (candidate >= result.cost_to_come[successor.state]) return;
      result.cost_to_come[successor.state] = candidate;
      result.parent[successor.state] = predecessor;
      open.push({candidate + options.heuristic_weight * heuristic(successor.state),
                 candidate, sequence++, successor.state});
    });
  }
  result.diagnostics.elapsed_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();
  return result;
}

std::vector<int> RestoreStatePath(const SearchResult& result, int start, int goal) {
  if (!result.found || start < 0 || goal < 0 ||
      start >= static_cast<int>(result.parent.size()) ||
      goal >= static_cast<int>(result.parent.size())) return {};
  std::vector<int> reversed;
  for (int state = goal;; state = result.parent[state]) {
    reversed.push_back(state);
    if (state == start) break;
    if (result.parent[state] < 0) throw std::runtime_error("broken search parent chain");
  }
  return {reversed.rbegin(), reversed.rend()};
}

}  // namespace navigation2d::planning_internal
