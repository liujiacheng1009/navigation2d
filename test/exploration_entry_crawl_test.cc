#include <cmath>
#include <stdexcept>

#include "navigation2d/application/exploration/entry_coordinator.h"
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("entry crawl contract failed");
  }
}
}  // namespace
int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  ExplorationGoal goal;
  goal.x = 2.;
  EntryCoordinator entry;
  entry.StartCrawl();
  const auto forward = entry.AdvanceCrawl({0., 0., 0., false}, goal);
  Require(forward.state == EntryCrawlStep::State::kDriving);
  Require(forward.command.linear == .18 && forward.command.angular == 0.);
  const auto turn = entry.AdvanceCrawl({0., 0., 1., false}, goal);
  Require(turn.command.linear == 0. && turn.command.angular == -.90);
  const auto tracking = entry.AdvanceCrawl({0., 0., .5, false}, goal);
  Require(tracking.command.linear == .18 && tracking.command.angular == -.70);
  const auto blocked = entry.AdvanceCrawl({0., 0., 0., true}, goal);
  Require(blocked.state == EntryCrawlStep::State::kBlocked && entry.crawl_ticks() == 0);
  entry.StartCrawl();
  const auto arrived = entry.AdvanceCrawl({2., 0., 0., false}, goal);
  Require(arrived.state == EntryCrawlStep::State::kBlocked && entry.crawl_ticks() == 0);
  entry.StartCrawl();
  for (int i = 0; i < 39; ++i) {
    Require(entry.AdvanceCrawl({0., 0., 0., false}, goal).state == EntryCrawlStep::State::kDriving);
  }
  Require(entry.crawl_ticks() == 40);
  const auto last = entry.AdvanceCrawl({0., 0., 0., false}, goal);
  Require(last.state == EntryCrawlStep::State::kLastCommand && last.command.linear == .18);
  Require(entry.crawl_ticks() == 41);  // Adapter publishes before stopping/retrying.
  entry.StopCrawl();
  Require(entry.crawl_ticks() == 0);
}
