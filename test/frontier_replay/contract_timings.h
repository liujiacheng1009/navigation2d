#pragma once
#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
namespace navigation2d::test {
// Restore std::cout even when a diagnostic query throws.
class ContractOutputCapture {
 public:
  explicit ContractOutputCapture(bool enabled) : original_(std::cout.rdbuf()) {
    if (enabled) {
      std::cout.rdbuf(captured_.rdbuf());
    }
  }
  ContractOutputCapture(const ContractOutputCapture&) = delete;
  ContractOutputCapture& operator=(const ContractOutputCapture&) = delete;
  ~ContractOutputCapture() { std::cout.rdbuf(original_); }
  std::string Release() {
    std::cout.rdbuf(original_);
    return captured_.str();
  }

 private:
  std::ostringstream captured_;
  std::streambuf* original_;
};
// Test-driver timings only. Each sample spans SelectGoals, BuildTour and
// SelectLeftoverApproaches; wall/RSS also include contract-output formatting.
class ContractTimings {
 public:
  using Clock = std::chrono::steady_clock;
  void Observe(Clock::time_point start) {
    samples_.push_back(std::chrono::duration<double, std::micro>(Clock::now() - start).count());
  }
  void Report(const std::string& captured, Clock::time_point start) {
    std::uint64_t hash = 1469598103934665603ULL;
    for (unsigned char byte : captured) {
      hash ^= byte;
      hash *= 1099511628211ULL;
    }
    std::sort(samples_.begin(), samples_.end());
    const auto percentile = [&](double ratio) {
      return samples_[static_cast<std::size_t>(ratio * (samples_.size() - 1))];
    };
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    std::cout << "{\"contract_hash\":" << hash << ",\"samples\":" << samples_.size()
              << ",\"tick_p50_us\":" << percentile(.50) << ",\"tick_p95_us\":" << percentile(.95)
              << ",\"tick_p99_us\":" << percentile(.99)
              << ",\"wall_seconds\":" << std::chrono::duration<double>(Clock::now() - start).count()
              << ",\"peak_rss_kb\":" << usage.ru_maxrss << "}" << std::endl;
  }

 private:
  std::vector<double> samples_;
};
}  // namespace navigation2d::test
