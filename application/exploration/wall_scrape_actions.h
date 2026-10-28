#pragma once
#include <cstdint>
#include <string>
namespace navigation2d::exploration {
struct ScrapeRoute {
  std::uint64_t signature;
  double clearance_m;
};
// Synchronous effects of a scrape recovery. A retry can replace the active
// route and reset the per-goal session; observations are deliberately lazy.
class WallScrapeActions {
 public:
  virtual ~WallScrapeActions() = default;
  virtual ScrapeRoute ReadScrapeRoute() const = 0;
  virtual void CoolRouteSignature(std::uint64_t) = 0;
  virtual void CancelRecoveryNavigation() = 0;
  virtual void ReportWallScrape(const char* reason, int scrapes) = 0;
  // Reset the latest-map budget, then attempt the same goal on the latest map.
  virtual bool RetryScrapedGoal() = 0;
  virtual bool UnstickFromPinch(const char* reason) = 0;
  virtual void PublishStatus(const std::string& phase, const std::string& message) = 0;
};
}  // namespace navigation2d::exploration
