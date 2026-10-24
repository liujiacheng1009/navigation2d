#include "navigation2d/application/exploration/viewpoint_rejections.h"

#include <cassert>
#include <limits>
int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  ViewpointRejections history;
  ExplorationGoal a{};
  a.frontier_x = 100.;
  ExplorationGoal b = a, c = a;
  b.x = 1.1;
  b.frontier_x = 200.;
  c.x = 2.2;
  c.frontier_x = 300.;
  assert(!history.HasDemotion() && !history.RecentlyDemoted(a, 0));
  assert(history.NoteScrape(a) == 1);
  assert(history.NoteScrape(b) == 2);
  assert(history.NoteScrape(c) == 1);  // Match the original anchor, not a transitive chain.
  history.ResetScrapes();
  assert(history.NoteScrape(c) == 1);
  history.Demote(a, 100);
  assert(history.HasDemotion() && history.RecentlyDemoted(a, 99));
  assert(!history.RecentlyDemoted(a, 100) && !history.RecentlyDemoted(a, 101));
  auto boundary = a;
  boundary.x = 1.2;
  boundary.frontier_x = 200.;
  assert(!history.RecentlyDemoted(boundary, 50));
  boundary.x = 1.199;
  assert(history.RecentlyDemoted(boundary, 50));
  boundary.x = 10.;
  boundary.frontier_x = 101.5;
  assert(!history.RecentlyDemoted(boundary, 50));
  boundary.frontier_x = 101.499;
  assert(history.RecentlyDemoted(boundary, 50));
  boundary.x = std::numeric_limits<double>::quiet_NaN();
  assert(history.RecentlyDemoted(boundary, 50));  // Frontier match retains legacy precedence.
  history.ClearDemotion();
  assert(!history.HasDemotion() && !history.RecentlyDemoted(a, 1));
  history.Demote(a, 0);
  assert(!history.HasDemotion());
  history.Demote(c, 1000);
  assert(history.RecentlyDemoted(c, 1) && !history.RecentlyDemoted(a, 1));
}
