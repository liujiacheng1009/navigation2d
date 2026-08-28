#pragma once

namespace navigation2d {

// A time-aligned, constant-velocity obstacle prediction.  sigma_x/y describe
// one standard deviation of the predicted position in the world frame.
// Consumers inflate these values by their configured confidence multiplier.
struct PredictedObstacle {
  double x = 0.;
  double y = 0.;
  double vx = 0.;
  double vy = 0.;
  double radius = .20;
  double sigma_x = 0.;
  double sigma_y = 0.;
  double age_s = 0.;
};

}  // namespace navigation2d
