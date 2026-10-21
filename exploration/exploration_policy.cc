#include "navigation2d/exploration/exploration_policy.h"

#include <cmath>
#include <vector>

namespace navigation2d {

bool AdmitExploringGoal(const ExploringGoalFacts& facts, int minimum_cells) {
  if (facts.passable_mouth || facts.unentered_room) return true;
  if (minimum_cells > 0 && facts.frontier_cells < minimum_cells) return false;
  if (!(facts.cell_area > 0.) || facts.information_gain <= facts.cell_area)
    return false;
  const double pocket = kRoomPocketDepthM() * kRoomPocketDepthM();
  return facts.information_gain >= pocket;
}

bool RoomStillOpen(bool has_admitted_goal, bool has_passable_door) {
  return has_admitted_goal || has_passable_door;
}

bool ShouldCoverResiduals(bool exploring_work_remains, bool coverage_enabled,
                          bool residual_remains) {
  return coverage_enabled && residual_remains && !exploring_work_remains;
}

TipLedger::Tip* TipLedger::Find(double x, double y) {
  for (auto& tip : tips_) {
    if (std::hypot(tip.x - x, tip.y - y) <= kSameTipM()) return &tip;
  }
  return nullptr;
}

const TipLedger::Tip* TipLedger::Find(double x, double y) const {
  for (const auto& tip : tips_) {
    if (std::hypot(tip.x - x, tip.y - y) <= kSameTipM()) return &tip;
  }
  return nullptr;
}

void TipLedger::NoteUnreached(double x, double y, bool passable_door) {
  if (!std::isfinite(x) || !std::isfinite(y)) return;
  Tip* tip = Find(x, y);
  if (tip == nullptr) {
    tips_.push_back(Tip{x, y, 0, false});
    tip = &tips_.back();
  }
  if (tip->blocked) return;
  ++tip->failures;
  const int limit = passable_door ? kDoorGiveUpAttempts : kOpenTipGiveUpAttempts;
  if (tip->failures >= limit) tip->blocked = true;
}

void TipLedger::NoteNoNewArea(double x, double y) {
  if (!std::isfinite(x) || !std::isfinite(y)) return;
  Tip* tip = Find(x, y);
  if (tip == nullptr) {
    tips_.push_back(Tip{x, y, 0, true});
    return;
  }
  tip->blocked = true;
}

void TipLedger::NoteProgress(double x, double y) {
  Tip* tip = Find(x, y);
  if (tip == nullptr) return;
  tip->failures = 0;
  tip->blocked = false;
}

bool TipLedger::Blocked(double x, double y) const {
  const Tip* tip = Find(x, y);
  return tip != nullptr && tip->blocked;
}

}  // namespace navigation2d
