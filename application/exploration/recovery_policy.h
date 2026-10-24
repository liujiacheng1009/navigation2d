#pragma once
#include <optional>

#include "navigation2d/application/exploration/planning_snapshot.h"
#include "navigation2d/control/local_controller.h"
namespace navigation2d::exploration {
// One aperture for every stall/escape sample (kBodyConeHalfAngle()).
struct BodyRanges {
  double front = 0.;
  double back = 0.;
  double left = 0.;
  double right = 0.;
  double side = 0.;
};

struct EscapeDecision {
  Twist2d command;
  double hold_s;
};
double SectorMinRange(const std::optional<ScanEvidenceView>& scan, double center_angle,
                      double half_width);
BodyRanges ReadBodyRanges(const std::optional<ScanEvidenceView>& scan);
std::optional<EscapeDecision> SelectEscape(const BodyRanges& body, bool returning);
Twist2d AdjustEscape(Twist2d command, const BodyRanges& body);
}  // namespace navigation2d::exploration
