#include <stdexcept>

#include "navigation2d/application/exploration/escape_execution.h"
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("escape lifecycle contract failed");
  }
}
}  // namespace
int main() {
  using namespace navigation2d::exploration;
  EscapeExecution escape;
  Require(escape.phase(100) == EscapeExecution::Phase::kInactive);
  escape.Begin({-.1, .2}, 200);
  Require(escape.scheduled() && escape.Busy(199));
  Require(escape.phase(199) == EscapeExecution::Phase::kDriving);
  Require(escape.phase(200) == EscapeExecution::Phase::kExpired);
  Require(!escape.Busy(200));
  const BodyRanges open{2., 2., 2., 2., 2.};
  const auto expected = AdjustEscape({-.1, .2}, open);
  const auto command = escape.Command(open);
  Require(command.linear == expected.linear && command.angular == expected.angular);
  escape.ResumeReturn();
  escape.Unschedule();
  Require(!escape.scheduled() && escape.resume_return());
  Require(escape.ConsumeReturnIntent());
  Require(!escape.ConsumeReturnIntent());
  escape.ResumeReturn();
  escape.ClearReturnIntent();
  Require(!escape.resume_return());
  escape.Begin({.1, -.2}, 400);
  Require(escape.phase(300) == EscapeExecution::Phase::kDriving);
}
