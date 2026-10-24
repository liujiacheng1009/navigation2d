#include "navigation2d/exploration/frontier_tracks.h"

#include <algorithm>
#include <cmath>
#include <limits>
namespace navigation2d::frontier_internal {
std::vector<ExplorationGoal> FrontierTracks::Filter(std::vector<ExplorationGoal> goals,
                                                    double association_radius_m,
                                                    int required_observations) {
  // Frontiers are sensor-derived hypotheses, not facts.  A clipped range ray
  // can create a large enough one-frame unknown/free boundary to pass the
  // geometry test.  Track components by their representative position across
  // map revisions and promote only persistent components to navigation work.
  ++stability_observation_;
  const auto stale = std::remove_if(frontier_tracks_.begin(), frontier_tracks_.end(),
                                    [&](const FrontierTrack& track) {
                                      return stability_observation_ > track.last_observation + 2;
                                    });
  frontier_tracks_.erase(stale, frontier_tracks_.end());
  std::vector<ExplorationGoal> stable;
  for (const auto& goal : goals) {
    auto track = std::min_element(
        frontier_tracks_.begin(), frontier_tracks_.end(),
        [&](const FrontierTrack& first, const FrontierTrack& second) {
          const double first_distance =
              std::min(std::hypot(first.x - goal.frontier_x, first.y - goal.frontier_y),
                       std::hypot(first.goal_x - goal.x, first.goal_y - goal.y));
          const double second_distance =
              std::min(std::hypot(second.x - goal.frontier_x, second.y - goal.frontier_y),
                       std::hypot(second.goal_x - goal.x, second.goal_y - goal.y));
          return first_distance < second_distance;
        });
    const double association_distance =
        track == frontier_tracks_.end()
            ? std::numeric_limits<double>::infinity()
            : std::min(std::hypot(track->x - goal.frontier_x, track->y - goal.frontier_y),
                       std::hypot(track->goal_x - goal.x, track->goal_y - goal.y));
    if (track == frontier_tracks_.end() || association_distance > association_radius_m) {
      frontier_tracks_.push_back(
          {goal.frontier_x, goal.frontier_y, goal.x, goal.y, 1, stability_observation_});
      if (required_observations <= 1) {
        stable.push_back(goal);
      }
      continue;
    }
    if (track->last_observation != stability_observation_) {
      ++track->observations;
    }
    track->x = goal.frontier_x;
    track->y = goal.frontier_y;
    track->goal_x = goal.x;
    track->goal_y = goal.y;
    track->last_observation = stability_observation_;
    if (track->observations >= required_observations) {
      stable.push_back(goal);
    }
  }
  return stable;
}

}  // namespace navigation2d::frontier_internal
