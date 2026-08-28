#include "navigation2d/planning/incremental_search.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>

namespace navigation2d::planning_internal {
namespace {
constexpr double kInfinity = std::numeric_limits<double>::infinity();
struct Key { double first; double second; };
bool Less(const Key& lhs, const Key& rhs) {
  return lhs.first < rhs.first || (lhs.first == rhs.first && lhs.second < rhs.second);
}
struct Entry { Key key; std::uint64_t sequence; std::uint64_t version; int state; };
struct Later {
  bool operator()(const Entry& lhs, const Entry& rhs) const {
    if (Less(lhs.key, rhs.key)) return false;
    if (Less(rhs.key, lhs.key)) return true;
    return lhs.sequence > rhs.sequence;
  }
};
}  // namespace

struct DStarLite::Impl {
  Impl(int count, int target, NeighborExpansion next, NeighborExpansion previous,
       PairHeuristic estimate)
      : state_count(count), goal(target), successors(std::move(next)),
        predecessors(std::move(previous)), heuristic(std::move(estimate)),
        g(count, kInfinity), rhs(count, kInfinity), best_next(count, -1), version(count, 0) {
    if (count <= 0 || target < 0 || target >= count) throw std::invalid_argument("invalid D* Lite graph");
    rhs[goal] = 0.;
  }

  Key CalculateKey(int state) const {
    const double value = std::min(g[state], rhs[state]);
    return {value + heuristic(start, state) + km, value};
  }
  void Push(int state) {
    ++version[state];
    open.push({CalculateKey(state), sequence++, version[state], state});
  }
  void UpdateVertex(int state) {
    if (state != goal) {
      rhs[state] = kInfinity;
      best_next[state] = -1;
      successors(state, [&](const SearchSuccessor& edge) {
        if (edge.state < 0 || edge.state >= state_count || edge.transition_cost < 0. ||
            !std::isfinite(edge.transition_cost)) throw std::runtime_error("invalid D* Lite edge");
        const double candidate = edge.transition_cost + g[edge.state];
        if (candidate < rhs[state]) { rhs[state] = candidate; best_next[state] = edge.state; }
      });
    }
    ++version[state];  // Invalidates any previous queue entry.
    if (g[state] != rhs[state]) Push(state);
  }
  void DiscardStale() {
    while (!open.empty() && open.top().version != version[open.top().state]) open.pop();
  }

  int state_count;
  int goal;
  int start = -1;
  int last_start = -1;
  double km = 0.;
  bool planned = false;
  NeighborExpansion successors;
  NeighborExpansion predecessors;
  PairHeuristic heuristic;
  std::vector<double> g, rhs;
  std::vector<int> best_next;
  std::vector<std::uint64_t> version;
  std::priority_queue<Entry, std::vector<Entry>, Later> open;
  std::uint64_t sequence = 0;
};

DStarLite::DStarLite(int state_count, int goal, NeighborExpansion successors,
                     NeighborExpansion predecessors, PairHeuristic heuristic)
    : impl_(std::make_unique<Impl>(state_count, goal, std::move(successors),
                                  std::move(predecessors), std::move(heuristic))) {}
DStarLite::~DStarLite() = default;
DStarLite::DStarLite(DStarLite&&) noexcept = default;
DStarLite& DStarLite::operator=(DStarLite&&) noexcept = default;

IncrementalSearchResult DStarLite::Plan(int start, const std::vector<int>& changed_states,
                                        const SearchOptions& options) {
  auto& data = *impl_;
  if (start < 0 || start >= data.state_count || options.max_planning_time_s <= 0.)
    throw std::invalid_argument("invalid D* Lite plan request");
  IncrementalSearchResult result;
  result.reused = data.planned;
  data.start = start;
  if (data.last_start < 0) {
    data.last_start = start;
    data.Push(data.goal);
  } else {
    data.km += data.heuristic(data.last_start, start);
    data.last_start = start;
  }
  std::vector<int> affected = changed_states;
  std::sort(affected.begin(), affected.end());
  affected.erase(std::unique(affected.begin(), affected.end()), affected.end());
  for (int state : affected) {
    if (state < 0 || state >= data.state_count) continue;
    data.UpdateVertex(state);
  }
  result.repaired_states = affected.size();
  const auto started = std::chrono::steady_clock::now();
  while (true) {
    data.DiscardStale();
    const Key start_key = data.CalculateKey(start);
    if ((data.open.empty() || !Less(data.open.top().key, start_key)) &&
        data.rhs[start] == data.g[start]) break;
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    if (elapsed >= options.max_planning_time_s) { result.diagnostics.time_limit_reached = true; break; }
    if (result.diagnostics.expansions >= options.max_expansions) {
      result.diagnostics.expansion_limit_reached = true; break;
    }
    if (data.open.empty()) break;
    const Entry entry = data.open.top(); data.open.pop();
    if (entry.version != data.version[entry.state]) continue;
    const Key fresh = data.CalculateKey(entry.state);
    if (Less(entry.key, fresh)) {
      data.Push(entry.state);
    } else if (data.g[entry.state] > data.rhs[entry.state]) {
      data.g[entry.state] = data.rhs[entry.state];
      ++result.diagnostics.expansions;
      data.predecessors(entry.state, [&](const SearchSuccessor& edge) {
        ++result.diagnostics.generated;
        data.UpdateVertex(edge.state);
      });
    } else {
      data.g[entry.state] = kInfinity;
      ++result.diagnostics.expansions;
      data.UpdateVertex(entry.state);
      data.predecessors(entry.state, [&](const SearchSuccessor& edge) {
        ++result.diagnostics.generated;
        data.UpdateVertex(edge.state);
      });
    }
  }
  result.diagnostics.elapsed_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();
  data.planned = true;
  if (!std::isfinite(data.g[start])) return result;
  result.path.push_back(start);
  int state = start;
  for (int steps = 0; state != data.goal && steps <= data.state_count; ++steps) {
    data.UpdateVertex(state);
    state = data.best_next[state];
    if (state < 0) { result.path.clear(); return result; }
    result.path.push_back(state);
  }
  result.found = !result.path.empty() && result.path.back() == data.goal;
  if (result.found) result.diagnostics.first_solution_s = result.diagnostics.elapsed_s;
  return result;
}

}  // namespace navigation2d::planning_internal
