#include <cassert>

#include "navigation2d/application/exploration/navigation_telemetry.h"

int main() {
  using namespace navigation2d;
  exploration::NavigationTelemetry telemetry;
  NavigationState state;
  state.controller_diagnostics.backend = ControllerBackend::kMppi;
  state.recoveries = 2;
  telemetry.Capture(state, {.1, .2});
  // Observing a tentative/rejected or preempted frame is not execution.
  assert(telemetry.command_counts().mppi == 0);
  assert(telemetry.state().recoveries == 2);
  assert(telemetry.TakeRecoveryNotice(2));
  assert(!telemetry.TakeRecoveryNotice(2));
  assert(!telemetry.TakeRecoveryNotice(1));
  assert(!telemetry.TakeRecoveryNotice(0));
  assert(telemetry.TakeRecoveryNotice(3));
  telemetry.NoteCommandBackend(ControllerBackend::kMppi);
  telemetry.NoteCommandBackend(ControllerBackend::kMppi);
  telemetry.NoteCommandBackend(ControllerBackend::kAcados);
  telemetry.NoteCommandBackend(ControllerBackend::kRpp);
  telemetry.NoteCommandBackend(ControllerBackend::kDwa);
  telemetry.NoteCommandBackend(ControllerBackend::kNone);
  assert(telemetry.command_counts().mppi == 2);
  assert(telemetry.command_counts().acados == 1);
  assert(telemetry.command_counts().rpp == 1);
  // A new preparation resets only the recovery reporting cursor. Totals
  // remain mission-wide, including failed attempts and accepted replans.
  telemetry.BeginPreparation();
  assert(!telemetry.TakeRecoveryNotice(0));
  assert(telemetry.TakeRecoveryNotice(1));
  assert(telemetry.command_counts().mppi == 2);
  assert(telemetry.command_counts().acados == 1);
  assert(telemetry.command_counts().rpp == 1);
  assert(telemetry.state().recoveries == 2);
}
