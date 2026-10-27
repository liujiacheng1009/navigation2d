#pragma once
#include <cstdint>
namespace navigation2d {
enum Cost : std::uint8_t {
  kFree = 0,
  kInscribed = 253,
  kLethal = 254,
  kUnknown = 255,
};
}  // namespace navigation2d
