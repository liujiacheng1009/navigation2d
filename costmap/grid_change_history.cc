#include "navigation2d/costmap/grid_change_history.h"

#include <algorithm>
#include <utility>
namespace navigation2d::costmap_internal {
void GridChangeHistory::Record(std::span<const std::uint8_t> previous,
                               std::span<const std::uint8_t> current) {
  std::vector<int> changed;
  for (std::size_t index = 0; index < current.size(); ++index) {
    if (current[index] != previous[index]) {
      changed.push_back(static_cast<int>(index));
    }
  }
  if (!changed.empty()) {
    ++revision_;
    changes_.push_back({revision_, std::move(changed)});
    constexpr std::size_t kHistoryLimit = 64;
    if (changes_.size() > kHistoryLimit) {
      changes_.erase(changes_.begin());
    }
  }
}

std::vector<int> GridChangeHistory::Since(std::uint64_t revision, std::size_t cells) const {
  if (revision >= revision_) {
    return {};
  }
  if (changes_.empty() || revision + 1 < changes_.front().revision) {
    std::vector<int> all(cells);
    for (std::size_t index = 0; index < all.size(); ++index) {
      all[index] = static_cast<int>(index);
    }
    return all;
  }
  std::vector<int> changed;
  for (const auto& set : changes_) {
    if (set.revision > revision) {
      changed.insert(changed.end(), set.cells.begin(), set.cells.end());
    }
  }
  std::sort(changed.begin(), changed.end());
  changed.erase(std::unique(changed.begin(), changed.end()), changed.end());
  return changed;
}
}  // namespace navigation2d::costmap_internal
