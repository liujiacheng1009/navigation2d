#pragma once
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <system_error>
namespace navigation2d::test {
class TemporaryDirectory {
 public:
  TemporaryDirectory() {
    char pattern[] = "/tmp/navigation-fixture-XXXXXX";
    const auto* created = mkdtemp(pattern);
    if (!created) {
      throw std::runtime_error("cannot create fixture directory");
    }
    try {
      path_ = created;
    } catch (...) {
      rmdir(created);
      throw;
    }
  }
  TemporaryDirectory(const TemporaryDirectory&) = delete;
  TemporaryDirectory& operator=(const TemporaryDirectory&) = delete;
  ~TemporaryDirectory() {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }
  const std::filesystem::path& path() const& { return path_; }
  const std::filesystem::path& path() const&& = delete;

 private:
  std::filesystem::path path_;
};
}  // namespace navigation2d::test
