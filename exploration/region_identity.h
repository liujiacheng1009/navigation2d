#pragma once
#include <cstdint>
#include <map>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "navigation2d/exploration/region_types.h"
namespace navigation2d {
struct ExplorationGrid;
namespace exploration_internal {
struct RegionIdentityAssignment {
  bool domain_reset = false;
  RegionId first_new_id = 0;
  std::vector<RegionId> ids, parents;
  std::vector<std::tuple<std::size_t, int, RegionId>> matches;
};
// Single owner of stable raster identities and world-cell history. No mission
// state/DFS decisions. Assign requires a valid grid and matching partition.
class RegionIdentity {
 public:
  RegionIdentityAssignment Assign(const ExplorationGrid&, const std::vector<int>& components,
                                  int count);
  RegionId RegionAt(double x, double y) const;
  const std::vector<RegionId>& labels() const { return labels_; }
  void RetainOwners(std::set<RegionId>& retained) const;

 private:
  int Cell(double x, double y) const;
  int width_ = 0, height_ = 0;
  double resolution_ = 0., origin_x_ = 0., origin_y_ = 0.;
  RegionId next_id_ = 1;
  std::vector<RegionId> labels_;
  struct CellHistory {
    RegionId owner;
    std::uint64_t seen;
  };
  std::map<std::pair<std::int64_t, std::int64_t>, CellHistory> history_;
  std::uint64_t revision_ = 0;
};
}  // namespace exploration_internal
}  // namespace navigation2d
