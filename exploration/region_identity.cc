#include "navigation2d/exploration/region_identity.h"

#include <algorithm>
#include <cmath>

#include "navigation2d/exploration/frontier_types.h"

namespace navigation2d::exploration_internal {
int RegionIdentity::Cell(double x, double y) const {
  if (resolution_ <= 0. || !std::isfinite(x) || !std::isfinite(y)) {
    return -1;
  }
  const int c = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  const int r = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return c < 0 || r < 0 || c >= width_ || r >= height_ ? -1 : r * width_ + c;
}
RegionId RegionIdentity::RegionAt(double x, double y) const {
  const int i = Cell(x, y);
  return i < 0 || static_cast<std::size_t>(i) >= labels_.size() ? 0 : labels_[i];
}

RegionIdentityAssignment RegionIdentity::Assign(const ExplorationGrid& g,
                                                const std::vector<int>& components, int count) {
  RegionIdentityAssignment result;
  result.first_new_id = next_id_;
  const int n = static_cast<int>(g.cells.size());
  // Preserve ownership evidence across temporary missing cells and merges.
  // Matching only the preceding raster loses every absorbed/disappearing ID.
  // Resolution changes start a new identity domain; padding does not.
  if (resolution_ > 0. && std::abs(resolution_ - g.resolution) > 1e-9) {
    history_.clear();
    labels_.clear();
    width_ = height_ = 0;
    result.domain_reset = true;
  }
  ++revision_;
  constexpr std::uint64_t kHistoryUpdates = 64;
  for (auto it = history_.begin(); it != history_.end();) {
    if (revision_ - it->second.seen > kHistoryUpdates) {
      it = history_.erase(it);
    } else {
      ++it;
    }
  }
  const auto world_cell = [&](int i) {
    return std::make_pair(static_cast<std::int64_t>(std::floor(
                              (g.origin_x + (i % g.width + .5) * g.resolution) / g.resolution)),
                          static_cast<std::int64_t>(std::floor(
                              (g.origin_y + (i / g.width + .5) * g.resolution) / g.resolution)));
  };
  std::map<std::pair<int, RegionId>, std::size_t> overlaps;
  for (int i = 0; i < n; ++i) {
    if (components[i]) {
      const auto remembered = history_.find(world_cell(i));
      const auto old = remembered != history_.end()
                           ? remembered->second.owner
                           : RegionAt(g.origin_x + (i % g.width + .5) * g.resolution,
                                      g.origin_y + (i / g.width + .5) * g.resolution);
      if (old) {
        ++overlaps[{components[i], old}];
      }
    }
  }
  auto& matches = result.matches;
  for (auto [key, size] : overlaps) {
    matches.emplace_back(size, key.first, key.second);
  }
  std::sort(matches.rbegin(), matches.rend());
  result.ids.assign(count + 1, 0);
  result.parents.assign(count + 1, 0);
  auto& ids = result.ids;
  auto& parents = result.parents;
  std::set<RegionId> used;
  for (auto [size, c, old] : matches) {
    (void)size;
    if (!parents[c]) {
      parents[c] = old;
    }
    if (!ids[c] && !used.count(old)) {
      ids[c] = old;
      used.insert(old);
    }
  }
  for (int c = 1; c <= count; ++c) {
    if (!ids[c]) {
      ids[c] = next_id_++;
    }
  }
  std::vector<RegionId> labels(n, 0);
  for (int i = 0; i < n; ++i) {
    if (components[i]) {
      labels[i] = ids[components[i]];
    }
  }
  for (int i = 0; i < n; ++i) {
    if (labels[i]) {
      const auto key = world_cell(i);
      auto [it, inserted] = history_.try_emplace(key, CellHistory{labels[i], revision_});
      // Do not overwrite an absorbed region's cells with the merge winner:
      // they are what lets a later split recover its original identity.
      // On a true split the old owner is already assigned to another child.
      // Transfer these cells to this child once, so subsequent updates reuse it.
      if (it->second.owner != labels[i] && used.count(it->second.owner)) {
        it->second.owner = labels[i];
      }
      it->second.seen = revision_;
      (void)inserted;
    }
  }
  width_ = g.width;
  height_ = g.height;
  resolution_ = g.resolution;
  origin_x_ = g.origin_x;
  origin_y_ = g.origin_y;
  labels_ = std::move(labels);
  return result;
}
void RegionIdentity::RetainOwners(std::set<RegionId>& retained) const {
  for (const auto& [cell, history] : history_) {
    (void)cell;
    retained.insert(history.owner);
  }
}
}  // namespace navigation2d::exploration_internal
