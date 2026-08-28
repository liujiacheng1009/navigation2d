#include "navigation2d/planning/planner_factory.h"

#include <stdexcept>

#include "navigation2d/planning/astar_planner.h"
#include "navigation2d/planning/dijkstra_planner.h"
#include "navigation2d/planning/theta_star_planner.h"
#include "navigation2d/planning/state_lattice_planner.h"

namespace navigation2d {

std::unique_ptr<GlobalPlanner> MakeGlobalPlanner(const std::string& name,
                                                 double clearance) {
  if (name == "dijkstra") return std::make_unique<DijkstraPlanner>(clearance);
  if (name == "astar") return std::make_unique<AStarPlanner>(clearance);
  if (name == "theta_star") return std::make_unique<ThetaStarPlanner>(clearance);
  throw std::runtime_error("unknown planner: " + name);
}

std::unique_ptr<GlobalPlanner> MakeGlobalPlanner(const NavigationConfig& config) {
  if (config.planner == "state_lattice") return std::make_unique<StateLatticePlanner>(config);
  return MakeGlobalPlanner(config.planner, config.robot_radius);
}

}  // namespace navigation2d
