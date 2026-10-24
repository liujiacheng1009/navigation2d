#include "navigation2d/exploration/frontier_visit_memory.h"

#include <cassert>
using navigation2d::frontier_internal::FrontierVisitMemory;
int main() {
  FrontierVisitMemory visits;
  visits.Add({0., 0., .25, 100, 500, false});
  assert(visits.Contains(0., 0., false, 599));
  assert(!visits.Contains(0., 0., false, 600));
  assert(!visits.Contains(0., 0., true, 100));
  assert(!visits.Contains(.25, 0., false, 100));
  visits.Add({2., 0., .4, 100, 500, false});
  assert(visits.Contains(2., 0., true, 599));
  assert(!visits.Contains(2., 0., true, 600));
  visits.Add({4., 0., .25, 100, 500, true});
  assert(visits.Contains(4., 0., true, 999999));
  assert(visits.Persistent(4., 0.));
  assert(!visits.Persistent(2., 0.));
  visits.Add({8., 0., 1.5, 100, 500, false});
  assert(visits.WidePocket(8., 0., 599));
  assert(!visits.WidePocket(8., 0., 600));
  assert(!visits.WidePocket(2., 0., 100));
  assert(!visits.WidePocket(4., 0., 100));
  auto copy = visits;
  visits.ClearFailures();
  assert(!visits.Contains(0., 0., false, 100));
  assert(!visits.Contains(2., 0., true, 100));
  assert(visits.Contains(4., 0., true, 999999));
  assert(copy.Contains(2., 0., true, 100));
  visits.Clear();
  assert(!visits.Persistent(4., 0.));
}
