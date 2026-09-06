#pragma once

#include <cstdint>
#include <limits>

namespace evo_engine {

inline constexpr uint64_t kGiHistoryBudgetBytes = uint64_t{4} << 30;

// Retiring generations are intentionally excluded from the steady-state budget.
struct GiHistoryBudget {
  uint64_t bytes = 0;

  [[nodiscard]] bool CanAdd(const uint64_t additional) const {
    return bytes < kGiHistoryBudgetBytes && additional < kGiHistoryBudgetBytes - bytes;
  }

  bool Add(const uint64_t additional) {
    if (!CanAdd(additional))
      return false;
    bytes += additional;
    return true;
  }
};

inline bool MultiplyGiHistoryBytes(uint64_t& bytes, const uint64_t count) {
  if (count != 0 && bytes > std::numeric_limits<uint64_t>::max() / count)
    return false;
  bytes *= count;
  return true;
}

}  // namespace evo_engine
