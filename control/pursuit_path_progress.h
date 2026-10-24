#pragma once
#include <Eigen/Core>
#include <cstddef>

#include "navigation2d/geometry/path_2d.h"
namespace navigation2d::pursuit_internal {
struct PathProjection {
  std::size_t segment;
  double t;
};
class PathProgress {
 public:
  // Nonempty path. Endpoint/size identity retains the original 5 cm test.
  // Returns true after resetting progress for a materially different route.
  [[nodiscard]] bool Observe(const Path&);
  // At least two poses; projection searches forward over 1.25 m of arc.
  // The projection is a value tied to this exact path, never retained here.
  [[nodiscard]] PathProjection Project(const Path&, const Pose2d&);

 private:
  std::size_t progress_index_ = 0, path_size_ = 0;
  double path_start_x_ = 0., path_start_y_ = 0.;
  double path_goal_x_ = 0., path_goal_y_ = 0.;
};
// Same path as the projection; lookahead in metres, coordinates in its frame.
Eigen::Vector2d CarrotAt(const Path&, PathProjection, double lookahead);
}  // namespace navigation2d::pursuit_internal
