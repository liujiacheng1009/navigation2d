#pragma once

#include <charconv>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace navigation2d {
// Scalar configuration transport, independent of YAML and concrete algorithms.
// Descriptor parsers select keys and produce their own typed configuration.
using AlgorithmParameters = std::map<std::string, std::string>;
using AlgorithmParameterSets = std::map<std::string, AlgorithmParameters>;

inline const AlgorithmParameters& ParametersFor(const AlgorithmParameterSets& sets,
                                                const std::string& id) {
  static const AlgorithmParameters empty;
  const auto found = sets.find(id);
  return found == sets.end() ? empty : found->second;
}

template <class T>
T ReadAlgorithmParameter(const std::string& text, const std::string& key) {
  if constexpr (std::is_same_v<T, std::string>) {
    return text;
  } else if constexpr (std::is_same_v<T, bool>) {
    if (text == "true") {
      return true;
    }
    if (text == "false") {
      return false;
    }
  } else {
    T value{};
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error == std::errc{} && end == text.data() + text.size()) {
      if constexpr (std::is_floating_point_v<T>) {
        if (std::isfinite(value)) {
          return value;
        }
      } else {
        return value;
      }
    }
  }
  throw std::invalid_argument("invalid algorithm parameter: " + key);
}
}  // namespace navigation2d
