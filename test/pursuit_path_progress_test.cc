#include "navigation2d/control/pursuit_path_progress.h"

#include <cassert>
#include <cmath>
namespace {
using namespace navigation2d;
using namespace navigation2d::pursuit_internal;
void Near(const Eigen::Vector2d& point, double x, double y) {
  assert((point - Eigen::Vector2d(x, y)).norm() < 1e-12);
}
}  // namespace
int main() {
  using namespace navigation2d;
  using namespace navigation2d::pursuit_internal;
  Path path{MakePose2d(0., 0., 0.), MakePose2d(2., 0., 0.), MakePose2d(2., 2., 0.)};
  PathProgress progress;
  assert(progress.Observe(path));
  assert(!progress.Observe(path));
  const auto projection = progress.Project(path, MakePose2d(1., .2, 0.));
  assert(projection.segment == 0 && projection.t == .5);
  Near(CarrotAt(path, projection, 1.5), 2., .5);
  Near(CarrotAt(path, projection, 10.), 2., 2.);
  Near(CarrotAt(path, projection, 0.), 1., 0.);
  Path degenerate{MakePose2d(0., 0., 0.), MakePose2d(0., 0., 0.), MakePose2d(1., 0., 0.),
                  MakePose2d(1., 0., 0.), MakePose2d(1., 1., 0.)};
  assert(progress.Observe(degenerate));
  const auto duplicate_projection = progress.Project(degenerate, MakePose2d(.3, .1, 0.));
  assert(duplicate_projection.segment == 1);
  Near(CarrotAt(degenerate, duplicate_projection, .9), 1., .2);
  Path long_path;
  for (int i = 0; i < 8; ++i) {
    long_path.push_back(MakePose2d(i, 0., 0.));
  }
  assert(progress.Observe(long_path));
  // A far pose cannot skip the bounded forward arc window in one projection.
  assert(progress.Project(long_path, MakePose2d(6., 0., 0.)).segment == 1);
  assert(progress.Project(long_path, MakePose2d(6., 0., 0.)).segment == 2);
  // Moving back physically must not rewind the committed path cursor.
  assert(progress.Project(long_path, MakePose2d(.1, 0., 0.)).segment == 2);
  auto small_shift = long_path;
  small_shift.back() = MakePose2d(7.049, 0., 0.);
  assert(!progress.Observe(small_shift));
  assert(progress.Project(small_shift, MakePose2d(.1, 0., 0.)).segment == 2);
  small_shift.back() = MakePose2d(7.051, 0., 0.);
  assert(progress.Observe(small_shift));
  assert(progress.Project(small_shift, MakePose2d(.1, 0., 0.)).segment == 0);
  small_shift.push_back(MakePose2d(8., 0., 0.));
  assert(progress.Observe(small_shift));
  Path singleton{MakePose2d(3., 2., 1.)};
  assert(progress.Observe(singleton) && !progress.Observe(singleton));
}
