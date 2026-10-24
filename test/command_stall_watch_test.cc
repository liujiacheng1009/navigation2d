#include "navigation2d/application/command_stall_watch.h"

#include <source_location>
#include <stdexcept>
#include <string>

namespace {
void Require(bool value, std::source_location location = std::source_location::current()) {
  if (!value) {
    throw std::runtime_error("stall contract failed at line " + std::to_string(location.line()));
  }
}
}  // namespace

int main() {
  using namespace navigation2d;
  const auto fixed = MakePose2d(0., 0., 0.);
  CommandStallWatch stopped(.06);
  for (int i = 0; i < 16; ++i) {
    Require(!stopped.Observe({fixed, 0., {}, 0., i * .06, true, true}));
  }
  Require(stopped.Observe({fixed, 0., {}, 0., .96, true, true}));
  // Removing a published command ends the streak rather than counting a stall.
  Require(!stopped.Observe({fixed, 0., {}, 0., 1., false, true}));
  Require(!stopped.Observe({fixed, 0., {}, 0., 2., true, true}));
  // Large planning clock jumps cannot stand in for enough executed cycles.
  for (int i = 0; i < 15; ++i) {
    Require(!stopped.Observe({fixed, 0., {}, 0., 10., true, true}));
  }
  Require(stopped.Observe({fixed, 0., {}, 0., 10., true, true}));
  stopped.RestartCycleBudget();
  for (int i = 0; i < 15; ++i) {
    Require(!stopped.Observe({fixed, 0., {}, 0., 10., true, true}));
  }
  Require(stopped.Observe({fixed, 0., {}, 0., 10., true, true}));
  stopped.Reset();
  Require(!stopped.Observe({fixed, 0., {}, 0., 10., true, true}));

  CommandStallWatch rotating(.06), translating(.06), measured(.06), moving(.06), route(.06);
  for (int i = 0; i <= 20; ++i) {
    const auto turned = MakePose2d(0., 0., i * .04);
    Require(!rotating.Observe({turned, 0., {0., .05}, 0., i * .06, true, false}));
    const bool stalled = translating.Observe({turned, 0., {0., .05}, 0., i * .06, true, true});
    if (i == 20) {
      Require(stalled);  // Spin against a wall is not translation progress.
    }
    Require(!measured.Observe({fixed, 0., {.02, 0.}, 0., i * .06, true, true}));
    const auto displaced = MakePose2d(i * .003, 0., 0.);
    Require(!moving.Observe({displaced, 0., {}, 0., i * .06, true, true}));
    Require(!route.Observe({fixed, i * .003, {}, 0., i * .06, true, true}));
  }
  CommandStallWatch slow_period(.2);
  Require(!slow_period.Observe({fixed, 0., {}, 0., 0., true, true}));
  for (int i = 0; i < 7; ++i) {
    Require(!slow_period.Observe({fixed, 0., {}, 0., 5., true, true}));
  }
  Require(slow_period.Observe({fixed, 0., {}, 0., 5., true, true}));
}
