#include "navigation2d/application/exploration/navigation_target_record.h"

#include <unistd.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "navigation2d/application/exploration/navigation_profile.h"
#include "navigation2d/application/exploration/navigation_session.h"

namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("accepted navigation target contract failed");
  }
}
template <class F>
void Reject(F&& operation) {
  bool rejected = false;
  try {
    operation();
  } catch (const std::logic_error&) {
    rejected = true;
  }
  Require(rejected);
}
struct MapFile {
  std::filesystem::path directory;
  MapFile() {
    char name[] = "/tmp/navigation-target-XXXXXX";
    const auto* created = mkdtemp(name);
    if (!created) {
      throw std::runtime_error("mkdtemp failed");
    }
    directory = created;
    std::ofstream out(directory / "map.json");
    out << R"({"width":50,"height":30,"resolution":0.1,"cells":[)";
    for (int index = 0; index < 1500; ++index) {
      out << (index ? ",0" : "0");
    }
    out << "]}";
  }
  ~MapFile() {
    std::error_code ignored;
    std::filesystem::remove_all(directory, ignored);
  }
};
}  // namespace

int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  NavigationTargetRecord record;
  AlgorithmSelection selected;
  Require(record.target() == nullptr && record.legacy_goal().x == 0.);
  Require(record.algorithms().global == selected.global &&
          record.algorithms().local == selected.local);
  ExplorationGoal goal{.x = 3.,
                       .y = 1.5,
                       .frontier_x = 3.2,
                       .frontier_y = 1.5,
                       .frontier_cells = 12,
                       .region_id = 42};
  NavigationSession session;
  Reject([&] { record.Accept(session, goal, selected); });
  Require(record.target() == nullptr && session.stage() == SessionStage::kEmpty);
  MapFile map;
  NavigationConfig config;
  config.planner = "astar";
  config.controller = "rpp";
  config.robot_radius = .15;
  config.map_resolution = .1;
  config.smoother_enabled = false;
  config.collision_monitor_enabled = false;
  selected.global = config.planner;
  selected.local = config.controller;
  const auto map_path = (map.directory / "map.json").string();
  const auto start = MakePose2d(1., 1.5, 0.);
  session.Begin(config, map_path, MakePose2d(goal.x, goal.y, goal.yaw), {});
  Reject([&] { record.Accept(session, goal, selected); });
  Require(record.target() == nullptr && session.stage() == SessionStage::kPreparing);
  Require(session.Probe(start, {}, 1.).has_route());
  record.Accept(session, goal, selected);
  Require(session.stage() == SessionStage::kActive && record.target());
  Require(record.target()->x == 3. && record.target()->frontier_cells == 12 &&
          record.target()->region_id == 42 && record.algorithms().global == "astar");
  goal.x = 4.;
  selected.global = "changed_after_commit";
  Require(record.target()->x == 3. && record.algorithms().global == "astar");
  Reject([&] { record.Accept(session, goal, selected); });
  Require(record.target()->x == 3. && session.stage() == SessionStage::kActive);
  session.Cancel();
  Require(record.target() && record.target()->region_id == 42);
  // A later rejected attempt must not replace the last accepted task metadata.
  session.Begin(config, map_path, MakePose2d(9., 9., 0.), {});
  Require(!session.Probe(start, {}, 2.).has_route());
  Reject([&] { record.Accept(session, goal, selected); });
  Require(record.target()->x == 3. && record.algorithms().global == "astar");
  record.Clear();
  Require(record.target() == nullptr && std::isnan(record.legacy_goal().x) &&
          std::isnan(record.legacy_goal().y) && std::isnan(record.legacy_goal().frontier_x) &&
          std::isnan(record.legacy_goal().frontier_y));
  Require(record.legacy_goal().region_id == 0 && record.algorithms().global == "astar");
  session.Cancel();
  selected.global = config.planner;
  goal.x = 2.5;
  goal.region_id = 77;
  session.Begin(config, map_path, MakePose2d(goal.x, goal.y, goal.yaw), {});
  Require(session.Probe(start, {}, 3.).has_route());
  record.Accept(session, goal, selected);
  Require(record.target() && record.target()->region_id == 77 && record.target()->x == 2.5);
}
