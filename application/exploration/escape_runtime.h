#pragma once
#include <cstdint>

#include "navigation2d/application/exploration/escape_execution.h"
namespace navigation2d::exploration {
// Synchronous body-frame observations and continuation/output effects. Only
// methods that need time read the mission clock. Deadline conversion remains
// at the adapter boundary to preserve its duration rounding/time domain.
class EscapeRuntime {
 public:
  virtual ~EscapeRuntime() = default;
  virtual bool EscapeInputsReady() const = 0;
  virtual std::int64_t ExecutionNowNs() const = 0;
  virtual BodyRanges EscapeBodyRanges() const = 0;
  virtual bool EscapeStartOccupied() const = 0;
  virtual bool EscapeReturning() const = 0;
  virtual std::int64_t EscapeDeadlineNs(double hold_s) const = 0;
  virtual void ReportEscapeSkipped(const BodyRanges&) = 0;
  virtual void ReportEscapeStarted(const BodyRanges&, Twist2d, double hold_s) = 0;
  virtual void EscapeDrivingStatus() = 0;
  virtual EscapeExitContext EscapeExitState() const = 0;
  virtual void BeginReturn(bool partial) = 0;
  virtual void ResumeEntryAfterEscape() = 0;
  virtual bool BreakWallScrapeAfterPeel() = 0;
  virtual bool NavigateAfterEscape() = 0;
  virtual void EscapeGoalResumed() = 0;
  virtual void DemoteActiveGoal(const char* reason) = 0;
  virtual void FinishCompleting() = 0;
  virtual void SelectAfterEscape() = 0;
};
}  // namespace navigation2d::exploration
