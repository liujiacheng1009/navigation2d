#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <utility>

#include "navigation2d/costmap/coordinate_validation.h"
#include "navigation2d/planning/global_planner.h"
namespace navigation2d {
namespace {
GlobalPlanStatus LegacyFailure(const std::string& reason) {
  if (reason.find("start occupied") != std::string::npos) {
    return GlobalPlanStatus::kStartOccupied;
  }
  if (reason.find("goal occupied") != std::string::npos) {
    return GlobalPlanStatus::kGoalOccupied;
  }
  if (reason.find("no path") != std::string::npos || reason.find("no route") != std::string::npos) {
    return GlobalPlanStatus::kNoPath;
  }
  return GlobalPlanStatus::kBackendError;
}
GlobalPlanStatus Stopped(AlgorithmStop stop) {
  return stop == AlgorithmStop::kCancelled ? GlobalPlanStatus::kCancelled
                                           : GlobalPlanStatus::kBudgetExceeded;
}
}  // namespace
Path GlobalPlanner::Plan(const LayeredCostmap&, const Pose2d&, const Pose2d&) const {
  throw std::logic_error("planner does not implement the legacy Plan API");
}
GlobalPlanResult GlobalPlanner::PlanWithContext(const GlobalPlanRequest& request) const {
  GlobalPlanResult result;
  result.metadata = {std::string(request.frame_id), request.costmap.revision(), HeadingSemantics(),
                     true};
  if (const auto stop = request.budget.Check(); stop != AlgorithmStop::kNone) {
    result.status = Stopped(stop);
    result.reason =
        stop == AlgorithmStop::kCancelled ? "planning cancelled" : "planning budget exceeded";
    return result;
  }
  const auto& grid = request.costmap.grid();
  if (grid.width() <= 0 || grid.height() <= 0 || !std::isfinite(grid.resolution()) ||
      grid.resolution() <= 0. || !costmap_internal::IndexablePose(grid, request.start) ||
      !costmap_internal::IndexablePose(grid, request.goal) || request.frame_id.empty() ||
      !request.start.matrix().allFinite() || !request.goal.matrix().allFinite()) {
    result.status = GlobalPlanStatus::kInvalidInput;
    result.reason = "invalid planning frame or pose";
    return result;
  }
  try {
    result = PlanRequest(request);
    if (const auto stop = request.budget.Check(); stop != AlgorithmStop::kNone) {
      result.status = Stopped(stop);
      result.reason =
          stop == AlgorithmStop::kCancelled ? "planning cancelled" : "planning budget exceeded";
      result.path.clear();
      return result;
    }
    if (result.status == GlobalPlanStatus::kNotRun) {
      result.status = GlobalPlanStatus::kInvalidOutput;
      result.reason = "planner returned no outcome";
    }
    if (!result.succeeded()) {
      result.path.clear();
      return result;
    }
    if (result.metadata.frame_id != request.frame_id ||
        result.metadata.map_revision != request.costmap.revision()) {
      result.status = GlobalPlanStatus::kInvalidOutput;
      result.reason = "planner returned a mismatched frame or map revision";
      result.path.clear();
      return result;
    }
    if (result.path.empty()) {
      result.status = GlobalPlanStatus::kNoPath;
      result.reason = "planner returned an empty path";
      return result;
    }
    for (const auto& pose : result.path) {
      if (!pose.matrix().allFinite()) {
        result.status = GlobalPlanStatus::kInvalidOutput;
        result.reason = "planner returned a non-finite path";
        result.path.clear();
        return result;
      }
    }
    for (const auto& pose : result.path) {
      if (X(pose) < 0. || Y(pose) < 0. || X(pose) >= grid.width() * grid.resolution() ||
          Y(pose) >= grid.height() * grid.resolution()) {
        result.status = GlobalPlanStatus::kInvalidOutput;
        result.reason = "planner returned a path outside the map";
        result.path.clear();
        return result;
      }
    }
    result.status = GlobalPlanStatus::kSuccess;
  } catch (const std::exception& error) {
    result.status = GlobalPlanStatus::kBackendError;
    result.reason = error.what();
    result.path.clear();
  } catch (...) {
    result.status = GlobalPlanStatus::kBackendError;
    result.reason = "planner threw a non-standard exception";
    result.path.clear();
  }
  return result;
}
GlobalPlanResult GlobalPlanner::PlanRequest(const GlobalPlanRequest& request) const {
  GlobalPlanResult result;
  result.metadata = {std::string(request.frame_id), request.costmap.revision(), HeadingSemantics(),
                     true};
  try {
    result.path = Plan(request.costmap, request.start, request.goal);
    result.status = GlobalPlanStatus::kSuccess;
  } catch (const std::exception& error) {
    result.status = LegacyFailure(error.what());
    result.reason = error.what();
  }
  return result;
}

}  // namespace navigation2d
