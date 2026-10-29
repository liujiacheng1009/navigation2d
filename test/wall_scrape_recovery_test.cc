#include <cassert>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/application/exploration/recovery_session.h"
#include "navigation2d/application/exploration/wall_scrape_actions.h"
#include "navigation2d/geometry/robot_geometry.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
struct Scene : WallScrapeActions {
  GoalRecoverySession recovery;
  ScrapeRoute route{11, kSwingClearance()};
  ScrapeRoute next{22, kSwingClearance()};
  mutable std::vector<std::string> events;
  bool accepted = true, reset = false, throw_retry = false, unstick_result = true;
  Scene() { recovery.Begin(1., 2.); }
  ScrapeRoute ReadScrapeRoute() const override {
    events.push_back("read");
    return route;
  }
  void CoolRouteSignature(std::uint64_t id) override {
    events.push_back("cool:" + std::to_string(id));
  }
  void CancelRecoveryNavigation() override { events.push_back("cancel"); }
  void ReportWallScrape(const char* reason, int count) override {
    assert(std::string(reason) == "wall" && count == recovery.wall_scrapes);
    events.push_back("report:" + std::to_string(count));
  }
  bool RetryScrapedGoal() override {
    events.push_back("retry");
    if (throw_retry) {
      throw std::runtime_error("retry");
    }
    if (reset) {
      recovery.Clear();
      recovery.Begin(3., 4.);
    }
    route = next;
    return accepted;
  }
  bool UnstickFromPinch(const char* reason) override {
    events.push_back("unstick:" + std::string(reason));
    return unstick_result;
  }
  void PublishStatus(const std::string& phase, const std::string& message) override {
    assert(phase == "NAVIGATING" && message == "replanned off wall after scrape stop");
    events.push_back("status");
  }
  bool Run() { return recovery.BreakWallScrape("wall", *this); }
};
}  // namespace
int main() {
  for (int scrapes : {-1, 0, 1}) {
    for (bool accepted : {false, true}) {
      Scene after_peel;
      after_peel.recovery.wall_scrapes = scrapes;
      after_peel.recovery.side_peels = 3;
      after_peel.unstick_result = accepted;
      assert(after_peel.recovery.LeavePinchAfterPeel(after_peel) == (scrapes > 0 && accepted));
      assert(after_peel.events == (scrapes > 0
                                      ? std::vector<std::string>{
                                            "unstick:after wall peel; leave pinch pocket"}
                                      : std::vector<std::string>{}));
    }
  }
  Scene first;
  first.reset = true;
  assert(first.Run());
  assert(first.recovery.wall_scrapes == 1);
  assert((first.events == std::vector<std::string>{"read", "cool:11", "cancel", "report:1", "retry",
                                                   "read", "status"}));
  // Even an accepted nested dispatch that opens a session cannot buy a second
  // retry. Its new signature is cooled before the demotion decision.
  first.events.clear();
  assert(first.Run());
  assert((first.events ==
          std::vector<std::string>{"read", "cool:22", "cancel", "report:2", "unstick:wall"}));
  Scene prior_peel;
  prior_peel.recovery.side_peels = 1;
  prior_peel.unstick_result = false;
  assert(!prior_peel.Run());
  assert(prior_peel.events.back() == "unstick:wall");
  Scene lethal;
  lethal.route = {0, -1.};
  assert(lethal.Run());
  assert((lethal.events == std::vector<std::string>{"read", "cancel", "report:1", "unstick:wall"}));
  for (const double margin :
       {0., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    Scene invalid;
    invalid.route.clearance_m = margin;
    assert(invalid.Run());
    assert(invalid.events.back() == "unstick:wall");
    assert(invalid.events.size() == 5);  // No same-route retry.
  }
  Scene rejected;
  rejected.accepted = false;
  assert(rejected.Run());
  assert(rejected.events.back() == "unstick:pinch replan failed");
  Scene still_pinched;
  still_pinched.next.clearance_m = -1.;
  assert(still_pinched.Run());
  assert(still_pinched.recovery.wall_scrapes == 2);
  assert(still_pinched.events[6] == "cool:22");
  assert(still_pinched.events.back() == "unstick:replanned path still pinch-scrape");
  Scene missing_signature;
  missing_signature.next = {0, -1.};
  assert(missing_signature.Run());
  assert(missing_signature.events[6] == "unstick:replanned path still pinch-scrape");
  Scene exception;
  exception.throw_retry = true;
  try {
    exception.Run();
    assert(false);
  } catch (const std::runtime_error&) {
  }
  assert(exception.recovery.wall_scrapes == 1 && exception.events.back() == "retry");
}
