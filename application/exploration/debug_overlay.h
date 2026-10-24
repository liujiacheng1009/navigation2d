#pragma once

#include <cstdint>
#include <span>
#include <string>

#include "navigation2d/exploration/frontier_types.h"
#include "navigation2d/exploration/frontier_debug_snapshot.h"
#include "navigation2d/geometry/pose_2d.h"

namespace navigation2d::exploration {
// Display-only calculations: never grant planning/execution free-space evidence.
// Synchronous borrowed spans, both sized width*height. Valid grid dimensions
// and positive resolution are required. No allocation in the painting calls.
// Input painting uses robot coordinates relative to navigation snapshot origin,
// preserving the adapter's original convention even if live map origin changed.
void PaintPlanningDebugInput(const ExplorationGrid& grid, const Pose2d& robot_local,
                             std::span<std::int8_t> output);
// Output must initially be a copy of input, in separate nonoverlapping storage.
// Only known-free cells receive inflation; unknown and occupied stay unchanged.
void PaintInflatedDebugMap(const ExplorationGrid& grid, std::span<const std::int8_t> input,
                           std::span<std::int8_t> output);
// Retains existing JSON schema, float formatting and <=4000-cell sampling rule.
std::string SerializeFrontierDebug(const FrontierDebugSnapshot& snap);
}  // namespace navigation2d::exploration
