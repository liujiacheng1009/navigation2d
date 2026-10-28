#pragma once
namespace navigation2d {
// Application admission rejection, independent of a planner backend's status.
// Excluded means an already-cooled first segment could not be replaced.
enum class RouteRejection { kNone, kSignatureExcluded, kFootprintInvalid };
}  // namespace navigation2d
