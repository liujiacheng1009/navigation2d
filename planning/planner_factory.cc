#include "navigation2/planning/planner_factory.h"

#include <stdexcept>

#include "navigation2/planning/astar_planner.h"
#include "navigation2/planning/dijkstra_planner.h"
#include "navigation2/planning/theta_star_planner.h"

namespace navigation2d {

std::unique_ptr<GlobalPlanner> MakeGlobalPlanner(const std::string& name,
                                                 double clearance) {
  if (name == "dijkstra") return std::make_unique<DijkstraPlanner>(clearance);
  if (name == "astar") return std::make_unique<AStarPlanner>(clearance);
  if (name == "theta_star") return std::make_unique<ThetaStarPlanner>(clearance);
  throw std::runtime_error("unknown planner: " + name);
}

}  // namespace navigation2d
