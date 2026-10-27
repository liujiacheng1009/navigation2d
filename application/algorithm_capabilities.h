#pragma once
namespace navigation2d {
struct PlannerCapabilities {
  bool owns_smoothing = false;
  bool grid_fallback = false;
  bool corner_grid_retry = false;
  bool reverse_paths = false;
};
struct ControllerCapabilities {
  bool dynamic_predictions = false;
  bool guidance_candidates = false;
  bool reverse_tracking = false;
  bool in_place_rotation = false;
};
struct AlgorithmRequirements {
  bool dynamic_predictions = false;
  bool guidance_candidates = false;
  bool reverse_paths = false;
  bool in_place_rotation = false;
};
}  // namespace navigation2d
