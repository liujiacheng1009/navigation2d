#include <stdexcept>

#include "navigation2d/application/exploration/viewpoint_rejections.h"
#include "navigation2d/test/goal_tracking_fixture.h"
using namespace navigation2d;
using namespace navigation2d::exploration;
using namespace navigation2d::test;
int main() {
  TrackingScene same;
  ViewpointRejections rejections;
  same.recovery.Begin(2., 1.);
  same.recovery.wall_scrapes = 2;
  same.recovery.side_peels = 1;
  same.recovery.resume_after_escape = true;
  assert(same.motion.TryOpenReplan([] { return true; }));
  assert(rejections.NoteScrape(same.goal) == 1);
  assert(rejections.NoteScrape(same.goal) == 2);
  rejections.Demote(same.goal, 10000000000LL);
  std::int64_t time = 2000000000LL;
  same.on_clock = [&] {
    time += 1000000000LL;
    return time;
  };
  same.on_status = [&] {
    assert(same.explorer.ActiveRegion() == 0);
    assert(same.watch.started_ns() == 3000000000LL);
    assert(same.watch.progress_ns() == 4000000000LL);
  };
  same.on_accepted_report = [&](const auto& goal, const auto& state) {
    assert(&goal == &same.goal && &state == &same.state);
    assert(same.explorer.ActiveRegion() != 0);
  };
  same.execution.CommitAccepted({same.goal, false, same.state}, same.motion, rejections, same,
                                same);
  assert((same.events ==
          std::vector<std::string>{"clock", "clock", "status:NAVIGATING", "accepted-report"}));
  assert(same.recovery.wall_scrapes == 2 && same.recovery.side_peels == 1 &&
         same.recovery.resume_after_escape);
  assert(same.motion.TryOpenReplan([] { return true; }));
  assert(rejections.NoteScrape(same.goal) == 1 && rejections.HasDemotion());

  same.on_status = nullptr;
  same.on_accepted_report = nullptr;
  same.PrepareGoal(2.6);
  same.execution.CommitAccepted({same.goal, false, same.state}, same.motion, rejections, same,
                                same);
  assert(same.recovery.wall_scrapes == 0 && same.recovery.side_peels == 0 &&
         !same.recovery.resume_after_escape);

  TrackingScene home;
  home.execution.CommitAccepted({home.goal, true, home.state}, home.motion, rejections, home, home);
  assert(home.returning.returning() && home.explorer.ActiveRegion() == 0);
  assert(home.message == "returning to exploration start");
  TrackingScene leftover;
  leftover.goal.leftover_approach = true;
  leftover.execution.CommitAccepted({leftover.goal, false, leftover.state}, leftover.motion,
                                    rejections, leftover, leftover);
  assert(leftover.events[2] == "status:COMPLETING");

  TrackingScene before_region;
  before_region.returning.AcceptNavigationMode(true);
  before_region.on_status = [] { throw std::runtime_error("status"); };
  try {
    before_region.execution.CommitAccepted({before_region.goal, false, before_region.state},
                                           before_region.motion, rejections, before_region,
                                           before_region);
    assert(false);
  } catch (const std::runtime_error&) {
  }
  assert(!before_region.returning.returning() && before_region.recovery.active);
  assert(before_region.explorer.ActiveRegion() == 0 &&
         before_region.events.back() == "status:NAVIGATING");

  TrackingScene after_region;
  after_region.on_accepted_report = [](const auto&, const auto&) {
    throw std::runtime_error("report");
  };
  try {
    after_region.execution.CommitAccepted({after_region.goal, false, after_region.state},
                                          after_region.motion, rejections, after_region,
                                          after_region);
    assert(false);
  } catch (const std::runtime_error&) {
  }
  assert(after_region.explorer.ActiveRegion() != 0 &&
         after_region.events.back() == "accepted-report");
}
