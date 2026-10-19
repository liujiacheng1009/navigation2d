#pragma once

#include <vector>

namespace navigation2d {

// Depth of unknown that already counts as a room pocket in CorridorMouthAt.
// A fringe thinner than this square is not exploring work.
constexpr double kRoomPocketDepthM = 0.55;
// Viewpoints this close are the same tip. Same distance used to merge one mouth.
constexpr double kSameTipM = 0.80;
// Open residuals are blocked after this many unreachable attempts.
constexpr int kOpenTipGiveUpAttempts = 3;
// A passable door is real work, so it survives more failures than an open tip
// before the room is allowed to end.
constexpr int kDoorGiveUpAttempts = 6;

struct ExploringGoalFacts {
  int frontier_cells = 0;
  double information_gain = 0.;
  double cell_area = 0.;
  bool passable_mouth = false;
  bool unentered_room = false;
};

// Exploring candidates are a large-enough frontier with more than one cell of
// unknown, or a passable door / unentered room. One-cell specks stay out.
bool AdmitExploringGoal(const ExploringGoalFacts& facts, int minimum_cells);

// The room stays open while it still has an admitted goal or a passable door.
// Raw frontier cells alone do not keep it open.
bool RoomStillOpen(bool has_admitted_goal, bool has_passable_door);

// Coverage finish runs after exploring work is gone. Mapping leaves this off.
bool ShouldCoverResiduals(bool exploring_work_remains, bool coverage_enabled,
                          bool residual_remains);

// Failure count for one tip. Blacklist is this count, not another tour sort.
class TipLedger {
 public:
  void NoteUnreached(double x, double y, bool passable_door);
  // Reached the pose and the boundary did not open into new area.
  void NoteNoNewArea(double x, double y);
  void NoteProgress(double x, double y);
  bool Blocked(double x, double y) const;

 private:
  struct Tip {
    double x = 0.;
    double y = 0.;
    int failures = 0;
    bool blocked = false;
  };
  Tip* Find(double x, double y);
  const Tip* Find(double x, double y) const;
  std::vector<Tip> tips_;
};

}  // namespace navigation2d
