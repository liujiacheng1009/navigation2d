#include "navigation2d/application/exploration/recovery_policy.h"

#include <algorithm>
#include <cmath>

#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
double SectorMinRange(const std::optional<ScanEvidenceView>& scan, double center_angle,
                      double half_width) {
  if (!scan) {
    return 0.;
  }
  double minimum = 10.;
  bool any = false;
  for (std::size_t beam = 0; beam < scan->ranges.size(); ++beam) {
    const double range = scan->ranges[beam];
    if (!std::isfinite(range) || range < scan->range_min) {
      continue;
    }
    const double angle = scan->angle_min + static_cast<double>(beam) * scan->angle_increment;
    if (std::abs(std::atan2(std::sin(angle - center_angle), std::cos(angle - center_angle))) >
        half_width) {
      continue;
    }
    minimum = std::min(minimum, range);
    any = true;
  }
  return any ? minimum : 0.;
}

BodyRanges ReadBodyRanges(const std::optional<ScanEvidenceView>& scan) {
  const double cone = navigation2d::kBodyConeHalfAngle();
  const double pi = std::acos(-1.);
  BodyRanges ranges;
  ranges.front = SectorMinRange(scan, 0., cone);
  ranges.back = SectorMinRange(scan, pi, cone);
  // See kLidarForwardOffset(): the back ray starts ahead of the chassis
  // centre. Keep a positive value so a wall between the lidar and the tail
  // still counts as inside the body rather than "no return".
  if (ranges.back > 0.) {
    ranges.back = std::max(1e-3, ranges.back - navigation2d::kLidarForwardOffset());
  }
  ranges.left = SectorMinRange(scan, pi / 2., cone);
  ranges.right = SectorMinRange(scan, -pi / 2., cone);
  ranges.side =
      std::min(ranges.left > 0. ? ranges.left : 10., ranges.right > 0. ? ranges.right : 10.);
  return ranges;
}

std::optional<EscapeDecision> SelectEscape(const BodyRanges& body, bool returning) {
  const double front = body.front;
  const double back = body.back;
  const double left = body.left;
  const double right = body.right;
  Twist2d command;
  double hold_s = 1.6;
  // Mild side bias only (≤0.35 rad/s). Large w during reverse was the
  // "spin to free" habit that corrupted scan matching mid-escape.
  const double side_bias = (left > 0. && right > 0. && std::abs(left - right) > .12)
                               ? ((left > right) ? .30 : -.30)
                               : 0.;
  if (back >= navigation2d::kPlanningRadius()) {
    // Priority 1: back straight out of the tip. Odom coasts fine; map yaw
    // does not matter. Tail must clear the planning circle.
    command.linear = -.20;
    command.angular = side_bias;
    hold_s = returning ? 2.2 : 1.8;
  } else if (navigation2d::AxisSwingClear(front)) {
    // Priority 2: creep forward into open, slight bias off the near wall.
    command.linear = .16;
    command.angular = side_bias;
    hold_s = 1.4;
  } else if (navigation2d::AxisSwingClear(std::max(left, right))) {
    // Priority 3 (last): short body-frame turn toward freer flank, then the
    // next escape tick will reverse. Keep spin ≤0.8 s.
    command.angular = left >= right ? .55 : -.55;
    hold_s = .70;
  } else {
    return std::nullopt;
  }
  return EscapeDecision{command, hold_s};
}
Twist2d AdjustEscape(Twist2d command, const BodyRanges& body) {
  const double front = body.front;
  const double back = body.back;
  const double left = body.left;
  const double right = body.right;
  if (std::abs(command.linear) > .05 && left > 0. && right > 0. && std::abs(left - right) > .12) {
    const double bias = (left > right) ? .30 : -.30;
    // Keep the planned sign of rotation if any; otherwise gently bias.
    if (std::abs(command.angular) < .15) {
      command.angular = bias;
    } else if ((command.angular > 0.) != (bias > 0.)) {
      command.angular = .5 * command.angular + .5 * bias;
    }
  }
  if (command.linear > 0. && front > 0. && front < navigation2d::kPlanningRadius()) {
    command.linear = 0.;
    // Prefer reverse over spin when the nose closes mid-escape.
    if (back >= navigation2d::kPlanningRadius()) {
      command.linear = -.16;
      command.angular = 0.;
    } else if (std::abs(command.angular) < .20) {
      command.angular = (left >= right) ? .50 : -.50;
    }
  }
  if (command.linear < 0. && navigation2d::HitInsideBody(back)) {
    command.linear = 0.;
  }
  return command;
}
}  // namespace navigation2d::exploration
