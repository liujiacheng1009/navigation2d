#include "navigation2d/application/exploration/exploration_mission.h"

#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
namespace {
using namespace navigation2d::exploration;
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("mission phase ordering contract failed");
  }
}
struct Runtime final : MissionRuntime, MotionOutput {
  mutable std::vector<std::string> events;
  std::int64_t time_ns = 1000000000LL;
  bool ready = true, escape = false, idle = false;
  std::int64_t deadline = 0;
  ExplorationMission* mission = nullptr;
  std::int64_t ExecutionNowNs() const override {
    events.push_back("clock");
    return time_ns;
  }
  std::optional<MissionHome> ReadyMissionHome() const override {
    events.push_back("ready");
    return ready ? std::optional<MissionHome>{{1., 2., .3}} : std::nullopt;
  }
  std::size_t KnownCells() const override {
    events.push_back("known");
    return 120;
  }
  void ResetMissionRecovery() override { events.push_back("reset"); }
  void InitializeMissionProgress(std::size_t known) override {
    Require(known == 120);
    events.push_back("initialize");
  }
  void MissionBootstrapScheduled(std::int64_t value) override {
    Require(mission->started() && mission->initial_known_cells() == 120);
    deadline = value;
    events.push_back("bootstrap");
  }
  void Finish(bool success, const std::string& message) override {
    Require(!success && message == "exploration timed out");
    events.push_back("finish");
    mission->Finish();
  }
  void NoteKnownProgress(std::size_t known) override {
    Require(known == 120);
    events.push_back("progress");
  }
  void NotePoseMotion() override { events.push_back("pose"); }
  bool TickEscape() override {
    events.push_back("escape");
    return escape;
  }
  bool TickIdleNavigation() override {
    events.push_back("idle");
    return idle;
  }
  void TickNavigation() override { events.push_back("track"); }
  void Send(navigation2d::Twist2d command) override {
    Require(command.linear == 0. && command.angular == 0.);
    events.push_back("stop");
  }
};
}  // namespace
int main() {
  ExplorationMission mission;
  Runtime runtime;
  runtime.mission = &mission;
  const auto tick = [&] {
    runtime.events.clear();
    mission.Tick(60., runtime, runtime);
  };
  runtime.ready = false;
  tick();
  Require(runtime.events == std::vector<std::string>{"ready"} && !mission.started());
  runtime.ready = true;
  tick();
  Require(runtime.events == std::vector<std::string>{"ready", "clock", "reset", "known",
                                                     "initialize", "clock", "bootstrap", "clock",
                                                     "known", "progress", "pose", "escape", "clock",
                                                     "stop"});
  Require(runtime.deadline == 9000000000LL && mission.home()->x == 1.);
  runtime.time_ns = runtime.deadline;
  tick();
  Require(runtime.events == std::vector<std::string>{"ready", "clock", "known", "progress", "pose",
                                                     "escape", "clock", "idle", "track"});
  runtime.idle = true;
  tick();
  Require(runtime.events.back() == "idle");
  runtime.escape = true;
  tick();
  Require(runtime.events ==
          std::vector<std::string>{"ready", "clock", "known", "progress", "pose", "escape"});
  runtime.ready = false;
  runtime.time_ns = 100000000000LL;
  tick();
  Require(runtime.events ==
          std::vector<std::string>{"ready"});  // Missing input still precedes timeout.
  runtime.ready = true;
  tick();
  Require(runtime.events == std::vector<std::string>{"ready", "clock", "finish"});
  Require(mission.finished());
  tick();
  Require(runtime.events.empty());
  ExplorationMission overflow;
  runtime.mission = &overflow;
  runtime.time_ns = std::numeric_limits<std::int64_t>::max();
  bool rejected = false;
  try {
    overflow.Tick(60., runtime, runtime);
  } catch (const std::overflow_error&) {
    rejected = true;
  }
  Require(rejected && !overflow.started());
}
