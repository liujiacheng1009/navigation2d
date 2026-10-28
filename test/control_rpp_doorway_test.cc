#include <cassert>
#include <fstream>

#include "navigation2d/control/regulated_pure_pursuit.h"
#include "navigation2d/costmap/layered_costmap.h"

int main() {
  // Doorway regression: parallel walls leave ~0.12 m margin outside the
  // 0.22 m footprint while the forward corridor is free. Old inflation-cost
  // regulation crawled to ≤0.25×; clearance+trust must keep a healthy speed.
  const char* door_map_path = "/tmp/navigation2d_rpp_doorway_test.json";
  {
    std::ofstream door_output(door_map_path);
    door_output << R"({"width":80,"height":40,"resolution":0.05,"cells":[)";
    for (int y = 0; y < 40; ++y) {
      for (int x = 0; x < 80; ++x) {
        // Free corridor |y| < 0.34 m around y=1.0 → walls at ±0.34 m from
        // centreline. Robot at (1,1) has clearance 0.34 m, margin 0.12 m.
        const double wy = (y + .5) * .05;
        const bool wall = wy < .66 || wy > 1.34;
        door_output << ((x || y) ? "," : "") << (wall ? 254 : 0);
      }
    }
    door_output << "]}";
  }
  navigation2d::NavigationConfig door_config;
  door_config.map_resolution = .05;
  door_config.robot_radius = .22;
  door_config.inflation_radius = .30;
  door_config.desired_linear_velocity = .24;
  door_config.collision_horizon = .35;
  door_config.regulated_min_speed = .08;
  navigation2d::LayeredCostmap door_costmap(navigation2d::Grid2d::Load(door_map_path), door_config);
  navigation2d::Path door_path;
  for (int index = 0; index <= 30; ++index) {
    door_path.push_back(navigation2d::MakePose2d(1. + .05 * index, 1., 0.));
  }
  navigation2d::RegulatedPurePursuit door_rpp(door_config);
  // Warm up path projection with zero speed, then command from rest so the
  // acceleration clamp does not hide the clearance scale result.
  door_rpp.Compute(door_path, navigation2d::MakePose2d(1., 1., 0.), {}, door_costmap);
  auto door_cmd =
      door_rpp.Compute(door_path, navigation2d::MakePose2d(1., 1., 0.), {}, door_costmap);
  // Several cycles to finish accel ramp toward desired speed.
  for (int i = 0; i < 20; ++i) {
    door_cmd =
        door_rpp.Compute(door_path, navigation2d::MakePose2d(1., 1., 0.), door_cmd, door_costmap);
  }
  assert(door_cmd.linear > .15);
  assert(door_rpp.Diagnostics().maneuver == navigation2d::ControllerManeuver::kTracking);

  // Tight sides + blocked forward → trust off, soft band applies; still above
  // crawl floor, but clearly slower than the clear doorway case.
  const char* pinch_map_path = "/tmp/navigation2d_rpp_pinch_test.json";
  {
    std::ofstream pinch_output(pinch_map_path);
    pinch_output << R"({"width":80,"height":40,"resolution":0.05,"cells":[)";
    for (int y = 0; y < 40; ++y) {
      for (int x = 0; x < 80; ++x) {
        const double wx = (x + .5) * .05;
        const double wy = (y + .5) * .05;
        const bool wall = wy < .75 || wy > 1.25 || wx > 1.45;
        pinch_output << ((x || y) ? "," : "") << (wall ? 254 : 0);
      }
    }
    pinch_output << "]}";
  }
  navigation2d::LayeredCostmap pinch_costmap(navigation2d::Grid2d::Load(pinch_map_path),
                                             door_config);
  navigation2d::Path pinch_path;
  for (int index = 0; index <= 8; ++index) {
    pinch_path.push_back(navigation2d::MakePose2d(1. + .05 * index, 1., 0.));
  }
  navigation2d::RegulatedPurePursuit pinch_rpp(door_config);
  navigation2d::Twist2d pinch_cmd{};
  for (int i = 0; i < 25; ++i) {
    pinch_cmd = pinch_rpp.Compute(pinch_path, navigation2d::MakePose2d(1., 1., 0.), pinch_cmd,
                                  pinch_costmap);
  }
  assert(pinch_cmd.linear >= door_config.regulated_min_speed - 1e-9);
  assert(pinch_cmd.linear + 1e-9 < door_cmd.linear);
}
