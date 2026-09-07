#pragma once

#include "GiHistoryBudget.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace evo_engine {

inline bool IsDdgiHistoryCountSupported(const int count) {
  return count >= 5 && count <= 30 && count % 5 == 0;
}

// Packed uint storage avoids requiring 16-bit storage-buffer shader support.
struct DdgiHistoryLayout {
  enum BufferIndex { IrradianceRing, IrradianceSum, ProbeOrigins, VisibilityRing, VisibilitySum, BufferCount };
  std::array<uint64_t, BufferCount> buffer_bytes{};

  static bool Calculate(const uint64_t probe_count, const uint32_t irradiance_size, const uint32_t visibility_size,
                        const int history_count, DdgiHistoryLayout& result) {
    if (!probe_count || !irradiance_size || !visibility_size || !IsDdgiHistoryCountSupported(history_count))
      return false;
    DdgiHistoryLayout candidate;
    for (size_t i = 0; i < BufferCount; ++i) {
      auto bytes = probe_count;
      if (i == ProbeOrigins) {
        if (!MultiplyGiHistoryBytes(bytes, 16))
          return false;
        candidate.buffer_bytes[i] = bytes;
        continue;
      }
      const bool visibility = i == VisibilityRing || i == VisibilitySum;
      const bool ring = i == IrradianceRing || i == VisibilityRing;
      const auto size = visibility ? visibility_size : irradiance_size;
      if (!MultiplyGiHistoryBytes(bytes, size) || !MultiplyGiHistoryBytes(bytes, size) ||
          !MultiplyGiHistoryBytes(bytes, visibility ? (ring ? 4 : 8) : (ring ? 8 : 16)) ||
          !MultiplyGiHistoryBytes(bytes, ring ? history_count : 1))
        return false;
      candidate.buffer_bytes[i] = bytes;
    }
    result = candidate;
    return true;
  }

  // The caller supplies actual device allocation requirements, including padding.
  static bool AddAllocationBytes(const std::array<uint64_t, BufferCount>& allocation_bytes, GiHistoryBudget& budget) {
    auto candidate = budget;
    for (const auto bytes : allocation_bytes)
      if (!candidate.Add(bytes))
        return false;
    budget = candidate;
    return true;
  }
};

inline uint16_t QuantizeDdgiHistorySample(const float value, const float bound) {
  if (!std::isfinite(value) || !std::isfinite(bound) || bound <= 0.0f)
    return 0;
  return static_cast<uint16_t>(std::floor(std::clamp(value / bound, 0.0f, 1.0f) * 65535.0f + 0.5f));
}

inline float DecodeDdgiHistorySum(const uint32_t sum, const float bound, const uint32_t history_count) {
  return static_cast<float>(sum) * (bound / (65535.0f * static_cast<float>(history_count)));
}

}  // namespace evo_engine
