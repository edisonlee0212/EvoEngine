#pragma once

#include <algorithm>
#include <cstdint>

#include "GiHistoryBudget.hpp"

namespace evo_engine {

struct SdfgiProbeLayout {
  uint32_t horizontal = 0;
  uint32_t vertical = 0;
  uint32_t columns = 0;
  uint32_t rows = 0;

  [[nodiscard]] uint64_t ProbeCount() const {
    return uint64_t{horizontal} * horizontal * vertical;
  }

  [[nodiscard]] uint32_t Index(const uint32_t x, const uint32_t y, const uint32_t z) const {
    return x + horizontal * (z + horizontal * y);
  }

  [[nodiscard]] bool HistoryBytes(const uint32_t cascades, const uint32_t history, uint64_t& bytes) const {
    bytes = 0;
    if (!columns || !rows || cascades < 1 || cascades > 8 || history < 5 || history > 30 || history % 5)
      return false;
    // Sixteen coefficients, signed16 samples, signed32 sums; one shared scrolling copy.
    bytes = uint64_t{columns} * rows;
    return MultiplyGiHistoryBytes(bytes, 16) && MultiplyGiHistoryBytes(bytes, uint64_t{history} * 8 + 16) &&
           MultiplyGiHistoryBytes(bytes, uint64_t{cascades} + 1);
  }

  [[nodiscard]] static SdfgiProbeLayout Create(const uint32_t x, const uint32_t y, const uint32_t spacing,
                                               const uint32_t max_image_dimension) {
    if (x < 64 || x > 256 || x % 16 || y < 64 || y > 256 || y % 16 ||
        (spacing != 1 && spacing != 2 && spacing != 4 && spacing != 8))
      return {};
    SdfgiProbeLayout result;
    result.horizontal = x / spacing + 1;
    result.vertical = y / spacing + 1;
    result.columns = std::min(result.horizontal * result.horizontal, max_image_dimension / 8);
    if (!result.columns)
      return {};
    result.rows = static_cast<uint32_t>((result.ProbeCount() + result.columns - 1) / result.columns);
    if (uint64_t{result.rows} * 16 > max_image_dimension)
      return {};
    return result;
  }
};

}  // namespace evo_engine
