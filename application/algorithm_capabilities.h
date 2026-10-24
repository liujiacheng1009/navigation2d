#pragma once
namespace navigation2d {
struct PlannerCapabilities {
  bool owns_smoothing = false;
  bool grid_fallback = false;
  bool corner_grid_retry = false;
};
struct ControllerCapabilities {
  bool dynamic_predictions = false;
  bool guidance_candidates = false;
};
struct AlgorithmRequirements {
  bool dynamic_predictions = false;
  bool guidance_candidates = false;
};
}  // namespace navigation2d
