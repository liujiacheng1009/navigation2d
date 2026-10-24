#include "navigation2d/exploration/frontier_tracks.h"

#include <cassert>
using namespace navigation2d;
using navigation2d::frontier_internal::FrontierTracks;
ExplorationGoal Goal(double x) {
  ExplorationGoal goal;
  goal.x = x;
  goal.frontier_x = x;
  return goal;
}
int main() {
  FrontierTracks tracks;
  const auto first = Goal(1.);
  assert(tracks.Filter({first, first}, .5, 2).empty());
  auto shifted = Goal(1.5);
  const auto promoted = tracks.Filter({shifted}, .5, 2);
  assert(promoted.size() == 1 && promoted.front().x == 1.5);
  // Either the representative frontier OR the viewpoint can preserve identity.
  shifted.frontier_x = 20.;
  assert(tracks.Filter({shifted}, .5, 3).size() == 1);
  assert(tracks.Filter({}, .5, 3).empty());
  assert(tracks.Filter({}, .5, 3).empty());
  // More than two missed epochs drops the identity before association.
  assert(tracks.Filter({shifted}, .5, 2).empty());
  assert(tracks.Filter({shifted}, .5, 2).size() == 1);
  FrontierTracks order;
  const auto output = order.Filter({Goal(3.), Goal(1.), Goal(2.)}, .1, 1);
  assert(output.size() == 3);
  assert(output[0].x == 3. && output[1].x == 1. && output[2].x == 2.);
  auto copied = order;
  order.Filter({}, .1, 1);
  order.Filter({}, .1, 1);
  order.Filter({}, .1, 1);
  assert(copied.Filter({Goal(3.)}, .1, 2).size() == 1);
  assert(order.Filter({Goal(3.)}, .1, 2).empty());
}
