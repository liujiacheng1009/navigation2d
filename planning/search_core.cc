#include "navigation2d/planning/search_core.h"

#include <chrono>
#include <algorithm>
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
      result.diagnostics.first_solution_s = elapsed;
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
  if (result.found) result.suboptimality_bound = options.heuristic_weight;
  return result;
}

SearchResult AnytimeRepairingAStar(int state_count, int start, int goal,
                                   const SearchHeuristic& heuristic,
                                   const SearchExpansion& expand,
                                   double initial_weight, double weight_decrement,
                                   const SearchOptions& options) {
  if (state_count <= 0 || start < 0 || goal < 0 || start >= state_count || goal >= state_count ||
      initial_weight < 1. || weight_decrement <= 0. || options.max_planning_time_s <= 0.)
    throw std::invalid_argument("invalid ARA* search parameters");
  SearchResult result;
  result.cost_to_come.assign(state_count, std::numeric_limits<double>::infinity());
  result.parent.assign(state_count, -1);
  std::vector<bool> closed(state_count, false), in_incons(state_count, false);
  std::vector<int> incons;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, LaterEntry> open;
  std::uint64_t sequence = 0;
  double weight = initial_weight;
  result.cost_to_come[start] = 0.;
  open.push({weight * heuristic(start), 0., sequence++, start});
  const auto started = std::chrono::steady_clock::now();
  const auto exhausted = [&] {
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    if (elapsed >= options.max_planning_time_s) {
      result.diagnostics.time_limit_reached = true;
      return true;
    }
    if (result.diagnostics.expansions >= options.max_expansions) {
      result.diagnostics.expansion_limit_reached = true;
      return true;
    }
    return false;
  };
  while (true) {
    while (!open.empty()) {
      if (exhausted()) break;
      const QueueEntry current = open.top();
      if (current.priority >= result.cost_to_come[goal]) break;
      open.pop();
      if (current.cost_to_come != result.cost_to_come[current.state]) continue;
      closed[current.state] = true;
      ++result.diagnostics.expansions;
      expand(current.state, result.parent[current.state],
             [&](const SearchSuccessor& successor) {
        if (successor.state < 0 || successor.state >= state_count ||
            !std::isfinite(successor.transition_cost) || successor.transition_cost < 0.)
          throw std::runtime_error("invalid ARA* successor");
        ++result.diagnostics.generated;
        const int predecessor = successor.predecessor < 0 ? current.state : successor.predecessor;
        if (predecessor < 0 || predecessor >= state_count ||
            !std::isfinite(result.cost_to_come[predecessor]))
          throw std::runtime_error("invalid ARA* predecessor");
        const double candidate = result.cost_to_come[predecessor] + successor.transition_cost;
        if (candidate >= result.cost_to_come[successor.state]) return;
        result.cost_to_come[successor.state] = candidate;
        result.parent[successor.state] = predecessor;
        if (!closed[successor.state]) {
          open.push({candidate + weight * heuristic(successor.state), candidate,
                     sequence++, successor.state});
        } else if (!in_incons[successor.state]) {
          incons.push_back(successor.state);
          in_incons[successor.state] = true;
        }
      });
    }
    if (std::isfinite(result.cost_to_come[goal])) {
      if (!result.found)
        result.diagnostics.first_solution_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
      result.found = true;
      result.suboptimality_bound = weight;
    }
    if (exhausted() || weight <= 1. || (!result.found && open.empty())) break;
    weight = std::max(1., weight - weight_decrement);
    std::vector<int> candidates;
    while (!open.empty()) {
      const auto entry = open.top(); open.pop();
      if (entry.cost_to_come == result.cost_to_come[entry.state]) candidates.push_back(entry.state);
    }
    candidates.insert(candidates.end(), incons.begin(), incons.end());
    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
    std::fill(closed.begin(), closed.end(), false);
    for (int state : incons) in_incons[state] = false;
    incons.clear();
    for (int state : candidates)
      open.push({result.cost_to_come[state] + weight * heuristic(state),
                 result.cost_to_come[state], sequence++, state});
    if (open.empty()) break;
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
