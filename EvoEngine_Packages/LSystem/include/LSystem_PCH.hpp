#pragma once

#include <cstddef>
#include <cstdint>

#include "EvoEngine_Package_PCH.hpp"

namespace l_system_package {
[[nodiscard]] inline std::uint64_t HashBytes(const void* data, std::size_t size) noexcept {
  constexpr std::uint64_t kOffsetBasis = 14695981039346656037ULL;
  constexpr std::uint64_t kPrime = 1099511628211ULL;

  const auto* bytes = static_cast<const std::uint8_t*>(data);
  std::uint64_t hash = kOffsetBasis;
  for (std::size_t i = 0; i < size; ++i) {
    hash ^= static_cast<std::uint64_t>(bytes[i]);
    hash *= kPrime;
  }
  return hash;
}
}  // namespace l_system_package
