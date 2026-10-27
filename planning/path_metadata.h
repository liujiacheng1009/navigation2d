#pragma once
#include <cstdint>
#include <string>
namespace navigation2d {
enum class PathHeadingSemantics { kUnspecified, kForwardTangent, kBodyHeading };
struct PlannedPathMetadata {
  std::string frame_id;
  // Revision is meaningful only within the same costmap instance/lifetime.
  std::uint64_t map_revision = 0;
  // ForwardTangent forbids reverse segments. BodyHeading encodes body yaw:
  // tangent dot heading determines forward/reverse; zero translation rotates.
  PathHeadingSemantics headings = PathHeadingSemantics::kUnspecified;
  // Only legacy Plan adapters may opt into the historical tangent projection.
  bool legacy_forward_projection_allowed = false;
};
}  // namespace navigation2d
