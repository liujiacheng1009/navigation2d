#pragma once
#include <cstdint>
#include <vector>

#include "navigation2d/exploration/frontier_types.h"
namespace navigation2d::frontier_internal {
// Mission-local candidate identity. A Filter call is one observation epoch;
// duplicate candidates in the same epoch never increment persistence twice.
class FrontierTracks {
 public:
  std::vector<ExplorationGoal> Filter(std::vector<ExplorationGoal> goals,
                                      double association_radius_m, int required_observations);

 private:
  struct FrontierTrack {
    double x = 0., y = 0.;
    double goal_x = 0., goal_y = 0.;
    int observations = 0;
    std::uint64_t last_observation = 0;
  };
  std::vector<FrontierTrack> frontier_tracks_;
  std::uint64_t stability_observation_ = 0;
};
}  // namespace navigation2d::frontier_internal
