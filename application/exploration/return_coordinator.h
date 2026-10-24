#pragma once
#include <optional>
#include <span>
#include <utility>

#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
enum class ReturnStepKind { kDirect, kBreadcrumb, kRepeated };
struct ReturnStep {
  ReturnStepKind kind;
  ExplorationGoal goal;
};
class ReturnCoordinator {
 public:
  void MarkPartial() { partial_ = true; }
  bool partial() const { return partial_; }
  void UseBreadcrumbs(bool enabled) { breadcrumbs_ = enabled; }
  bool breadcrumbs() const { return breadcrumbs_; }
  void ResetEscapes() { escapes_ = 0; }
  void CountEscape() { ++escapes_; }
  int escapes() const { return escapes_; }
  ExplorationGoal Home(const ExplorationGrid* map, ExplorationGoal home) const;
  ReturnStep NextStep(std::span<const std::pair<double, double>> trail, const ExplorationGoal& home,
                      double rx, double ry, const ExplorationGoal& active_goal) const;

 private:
  bool partial_ = false;
  bool breadcrumbs_ = false;
  int escapes_ = 0;
};
}  // namespace navigation2d::exploration
