#include "navigation2d/application/route_preparation.h"

#include <unistd.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <source_location>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "navigation2d/planning/path_geometry.h"
#include "navigation2d/planning/route_identity.h"

namespace {
void Require(bool ok, std::source_location location = std::source_location::current()) {
  if (!ok) {
    throw std::runtime_error("route contract failed at line " + std::to_string(location.line()));
  }
}
template <class F>
void Reject(F&& action, const std::string& reason) {
  try {
    action();
  } catch (const std::runtime_error& error) {
    Require(error.what() == reason);
    return;
  }
  throw std::runtime_error("expected route rejection: " + reason);
}
struct MapFile {
  std::filesystem::path directory;
  MapFile() {
    char name[] = "/tmp/route-preparation-XXXXXX";
    auto* created = mkdtemp(name);
    if (!created) {
      throw std::runtime_error("mkdtemp failed");
    }
    directory = created;
    std::ofstream out(directory / "map.json");
    out << R"({"width":50,"height":30,"resolution":0.1,"cells":[)";
    for (int i = 0; i < 1500; ++i) {
      if (i) {
        out << ',';
      }
      out << ((i % 50 >= 20 && i % 50 <= 24 && i / 50 >= 14 && i / 50 <= 16) ? 254 : 0);
    }
    out << "]}";
  }
  ~MapFile() {
    std::error_code ignored;
    std::filesystem::remove_all(directory, ignored);
  }
};
class StraightPlanner final : public navigation2d::GlobalPlanner {
 public:
  navigation2d::Path Plan(const navigation2d::LayeredCostmap&, const navigation2d::Pose2d& start,
                          const navigation2d::Pose2d& goal) const override {
    return {start, goal};
  }
};
class ThrowPlanner final : public navigation2d::GlobalPlanner {
 public:
  navigation2d::Path Plan(const navigation2d::LayeredCostmap&, const navigation2d::Pose2d&,
                          const navigation2d::Pose2d&) const override {
    throw std::runtime_error("injected planner failure");
  }
};
}  // namespace

int main() {
  using namespace navigation2d;
  using namespace navigation2d::planning_internal;
  MapFile fixture;
  NavigationConfig config;
  config.robot_radius = .15;
  config.smoother_enabled = false;
  config.planner = "custom_straight";
  LayeredCostmap map(Grid2d::Load((fixture.directory / "map.json").string()), config);
  const auto start = MakePose2d(.5, 1.5, 0.);
  const auto goal = MakePose2d(4., 1.5, 0.);
  std::unordered_set<std::uint64_t> banned;
  const RouteRequest request{start, goal, banned};
  StraightPlanner straight;
  Reject([&] { static_cast<void>(PrepareRoute(straight, map, config, {}, request)); },
         "planned path failed dense footprint validation");
  const auto detour = PrepareRoute(straight, map, config, {.corner_grid_retry = true}, request);
  Require(PathFootprintValid(detour.path, map, config.robot_radius));
  Require(detour.signature == FirstSegmentSignature(detour.path, start, .1));
  bool leaves_chord = false;
  for (const auto& point : detour.path) {
    leaves_chord |= std::abs(Y(point) - 1.5) > .2;
  }
  Require(leaves_chord);
  banned.insert(detour.signature);
  config.planner = "astar";
  Reject(
      [&] {
        static_cast<void>(
            PrepareRoute(straight, map, config, {.corner_grid_retry = true}, request));
      },
      "failed first-segment replay");
  config.planner = "custom_straight";
  Reject(
      [&] {
        static_cast<void>(
            PrepareRoute(straight, map, config, {.corner_grid_retry = true}, request));
      },
      "failed first-segment replay");
  banned.clear();
  ThrowPlanner failing;
  Reject([&] { static_cast<void>(PrepareRoute(failing, map, config, {}, request)); },
         "injected planner failure");
  const auto fallback = PrepareRoute(failing, map, config, {.grid_fallback = true}, request);
  Require(PathFootprintValid(fallback.path, map, config.robot_radius));
  Require(fallback.signature == detour.signature);

  // Navigation densification differs intentionally from grid-search densification:
  // discard zero-length translations, retaining the final requested goal yaw.
  const auto dense =
      DensifyRoute({MakePose2d(0., 0., .2), MakePose2d(1., 0., .5), MakePose2d(1., 0., 1.)}, .25);
  Require(dense.size() == 5 && std::abs(Yaw(dense.front()) - .2) < 1e-12);
  Require(std::abs(X(dense.back()) - 1.) < 1e-12 && std::abs(Yaw(dense.back()) - 1.) < 1e-12);
  Require(DensifyRoute({}, .25).empty());
  const Path loop{MakePose2d(0., 0., 0.), MakePose2d(4., 0., 0.), MakePose2d(4., 4., 0.),
                  MakePose2d(0., 4., 0.), MakePose2d(0., .1, 0.)};
  Require(PathProgress(loop, MakePose2d(0., .1, 0.), 0.) < 1.25);
  Require(std::abs(DistanceToPath(loop, MakePose2d(2., 1., 0.)) - 1.) < 1e-12);
}
