#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace l_system_package {
inline std::uint64_t HashBytes(const void* data, const std::size_t byte_count) {
  constexpr std::uint64_t kOffsetBasis = 14695981039346656037ULL;
  constexpr std::uint64_t kPrime = 1099511628211ULL;
  std::uint64_t hash = kOffsetBasis;
  const auto* bytes = static_cast<const std::uint8_t*>(data);
  for (std::size_t i = 0; i < byte_count; ++i) {
    hash ^= bytes[i];
    hash *= kPrime;
  }
  return hash;
}
}  // namespace l_system_package
