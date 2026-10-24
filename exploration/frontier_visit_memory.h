#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>
namespace navigation2d::frontier_internal {
struct VisitCooling {
  double x = 0., y = 0.;
  double radius = 0.;
  std::size_t known_cells = 0;
  // Successful observation viewpoints are cooled for a larger map-growth
  // budget so the planner cannot immediately revisit the same room edge.
  std::size_t required_growth = 600;
  // A successful doorway is an observation already performed in this
  // mission. It must not become eligible again merely because the map grew.
  bool persistent = false;
};

// Query definitions stay inline for per-candidate geometry filtering.
// Mission-local visit evidence. Temporary exclusions expire at the specified
// known-cell growth; successful persistent observations survive map growth.
class FrontierVisitMemory {
 public:
  void Add(VisitCooling cooling) { entries_.push_back(cooling); }
  bool Contains(double x, double y, bool boundary, std::size_t known_cells) const {
    const std::size_t known = known_cells;
    return std::any_of(entries_.begin(), entries_.end(), [&](const auto& entry) {
      // Eligible again after the map has materially expanded (or forever when
      // persistent). Temporary mouth/pocket cools must also apply to frontier
      // queries: the old `boundary ⇒ persistent-only` gate ignored every failed
      // doorway cool, so nearest-first kept re-queuing the same alcove crumbs.
      if (!(entry.persistent || known < entry.known_cells + entry.required_growth)) {
        return false;
      }
      // Tiny pose-only cools stay viewpoint-local; mouth/pocket radii (>=.40)
      // suppress the frontier component as well.
      if (boundary && !entry.persistent && entry.radius < .40) {
        return false;
      }
      return std::hypot(x - entry.x, y - entry.y) < entry.radius;
    });
  }
  bool Persistent(double x, double y) const {
    return std::any_of(entries_.begin(), entries_.end(), [&](const auto& entry) {
      return entry.persistent && std::hypot(x - entry.x, y - entry.y) < entry.radius;
    });
  }
  bool WidePocket(double x, double y, std::size_t known_now) const {
    return std::any_of(entries_.begin(), entries_.end(), [&](const auto& entry) {
      if (entry.persistent || entry.radius < 1.5) {
        return false;
      }
      if (known_now >= entry.known_cells + entry.required_growth) {
        return false;
      }
      return std::hypot(x - entry.x, y - entry.y) < entry.radius;
    });
  }
  void ClearFailures();
  void Clear() { entries_.clear(); }

 private:
  std::vector<VisitCooling> entries_;
};
}  // namespace navigation2d::frontier_internal
