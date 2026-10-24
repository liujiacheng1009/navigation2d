#include "navigation2d/application/docking_servo.h"

#include <cmath>
#include <stdexcept>
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("docking servo contract failed");
  }
}
void Near(double a, double b) { Require(std::abs(a - b) < 1e-12); }
}  // namespace
int main() {
  using namespace navigation2d;
  const DockingSettings settings{.04, .12, .5, 1., .06};
  const auto robot = MakePose2d(0., 0., 0.);
  const auto front = MakePose2d(.2, 0., 0.);
  DockingServo servo(settings);
  const auto first = servo.Advance(robot, front, .2, {});
  Require(first.distance_progress && !first.yaw_progress && !first.succeeded);
  Near(first.command.linear, .03);
  Near(first.command.angular, 0.);
  Require(!servo.Advance(robot, front, .2, {}).distance_progress);
  Require(servo.Advance(MakePose2d(.002, 0., 0.), front, .198, {}).distance_progress);
  // Acceleration limiting follows the last published command rather than odom.
  const auto decelerating = servo.Advance(robot, front, .2, {.2, 0.});
  Near(decelerating.command.linear, .17);
  servo.Reset();
  const auto behind = servo.Advance(robot, MakePose2d(-.2, 0., 0.), .2, {});
  Near(behind.command.linear, -.03);
  Near(behind.command.angular, 0.);
  servo.Reset();
  const auto side = servo.Advance(robot, MakePose2d(0., .2, 0.), .2, {});
  Near(side.command.linear, 0.);
  Near(side.command.angular, .06);
  servo.Reset();
  const auto docking = MakePose2d(.02, 0., .4);
  const auto aligning = servo.Advance(robot, docking, .02, {});
  Require(aligning.distance_progress && aligning.yaw_progress && !aligning.succeeded);
  Near(aligning.command.angular, .06);
  Require(!servo.Advance(robot, docking, .02, {}).yaw_progress);
  const auto turned = MakePose2d(0., 0., .1);
  Require(servo.Advance(turned, docking, .02, {}).yaw_progress);
  servo.Advance(MakePose2d(-.025, 0., .1), docking, .045,
                {});  // Position latch survives 1 cm hysteresis.
  Require(!servo.Advance(turned, docking, .02, {}).yaw_progress);
  servo.Advance(MakePose2d(-.04, 0., .1), docking, .06,
                {});  // Leaving that band resets yaw progress.
  Require(servo.Advance(turned, docking, .02, {}).yaw_progress);
  const auto reached = servo.Advance(MakePose2d(.02, 0., .4), docking, 0., {.2, .3});
  Require(reached.succeeded);
  Near(reached.command.linear, 0.);
  Near(reached.command.angular, 0.);
}
