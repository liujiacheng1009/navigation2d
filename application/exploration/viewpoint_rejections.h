#pragma once
#include <cstdint>
#include <optional>

#include "navigation2d/exploration/frontier_types.h"
namespace navigation2d::exploration {
// One writer owns spatial scrape identity and the currently demoted viewpoint.
// Coordinates share the exploration frame (metres); deadlines use the mission
// clock in nanoseconds. A zero deadline disables cooling as in the legacy node.
class ViewpointRejections {
 public:
  int NoteScrape(const ExplorationGoal&);
  void ResetScrapes();
  void Demote(const ExplorationGoal&, std::int64_t until_ns);
  void ClearDemotion() { demoted_.reset(); }
  [[nodiscard]] bool HasDemotion() const { return demoted_ && demoted_->until_ns != 0; }
  [[nodiscard]] bool RecentlyDemoted(const ExplorationGoal&, std::int64_t now_ns) const;

 private:
  static bool Nearby(const ExplorationGoal&, const ExplorationGoal&);
  struct Demotion {
    ExplorationGoal goal;
    std::int64_t until_ns;
  };
  std::optional<ExplorationGoal> scrape_goal_;
  int scrape_count_ = 0;
  std::optional<Demotion> demoted_;
};
}  // namespace navigation2d::exploration
