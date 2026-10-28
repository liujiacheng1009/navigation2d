#include <algorithm>

#include "navigation2d/test/goal_tracking_fixture.h"
using namespace navigation2d;
using namespace navigation2d::test;
namespace {
bool Has(const TrackingScene& scene, const char* event) {
  return std::find(scene.events.begin(), scene.events.end(), event) != scene.events.end();
}
}  // namespace
int main() {
  TrackingScene handoff;
  handoff.handoff = true;
  const auto prior_controls = handoff.plugin.calls.controls;
  handoff.RunTrack();
  assert(handoff.events == std::vector<std::string>{"handoff"});
  assert(handoff.commands.empty() && handoff.plugin.calls.controls == prior_controls);
  TrackingScene immobile;
  immobile.motion.Observe(1., 1., 1000000000LL);
  immobile.now_ns = 10000000000LL;
  immobile.RunTrack();
  assert(Has(immobile, "immobility-report") && !Has(immobile, "capture"));
  assert(immobile.commands.size() == 1 && immobile.commands.front().linear == 0. &&
         immobile.commands.front().angular == 0.);
  assert(!immobile.navigation.has_session() && immobile.motion.recoveries() == 1);

  TrackingScene normal;
  normal.RunTrack();
  assert(normal.commands.size() == 1 && Has(normal, "capture") && Has(normal, "report-cycle"));
  assert(normal.telemetry.command_counts().rpp == 1 && normal.navigation.has_session());
  const auto where = [&](const char* name) {
    return std::find(normal.events.begin(), normal.events.end(), name);
  };
  assert(where("capture") < where("motion") && where("motion") < where("report-cycle"));

  TrackingScene cut;
  cut.cut = true;
  cut.RunTrack();
  assert(cut.commands.size() == 1 && cut.commands.front().linear == 0.);
  assert(Has(cut, "cut-replan") && !Has(cut, "capture"));
  assert(cut.telemetry.command_counts().rpp == 0);
  TrackingScene home_cut;
  home_cut.cut = true;
  home_cut.returning.AcceptNavigationMode(true);
  home_cut.routes.Cool(42);
  home_cut.RunTrack();
  assert(!home_cut.navigation.has_session() && home_cut.routes.size() == 0);
  assert(home_cut.events.back() == "return-recovery" && Has(home_cut, "replay-start"));

  TrackingScene home;
  home.local_pose = MakePose2d(2., 1., 0.);
  home.returning.AcceptNavigationMode(true);
  home.RunTrack();
  assert(home.events.back() == "return-complete" && !home.navigation.has_session());

  TrackingScene near;
  // The local/world offset is -0.75 m: both frames have 0.25 m remaining.
  near.goal.x = 1.25;
  near.local_pose = MakePose2d(1.75, 1., 0.);
  near.RunTrack();
  assert(!near.navigation.has_session() && Has(near, "defer") && !Has(near, "route-check"));
  assert(near.commands.size() == 1 && near.commands.front().linear == 0.);
  TrackingScene boundary;
  boundary.goal.x = 1.28;
  boundary.local_pose = MakePose2d(1.72, 1., 0.);
  boundary.RunTrack();
  assert(boundary.navigation.has_session() && Has(boundary, "capture"));
}
