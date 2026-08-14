#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace evo_engine {

struct DdgiEmissiveSamplingStats {
  uint32_t nee_attempt_count = 0;
  uint32_t zero_pdf_reject_count = 0;
  uint32_t emitter_backface_reject_count = 0;
  uint32_t alpha_mask_reject_count = 0;
  uint32_t invalid_sample_reject_count = 0;
  uint32_t receiver_backface_reject_count = 0;
  uint32_t shadowed_sample_count = 0;
  uint32_t zero_radiance_sample_count = 0;
  uint32_t nonzero_contribution_count = 0;
};

static_assert(std::is_standard_layout_v<DdgiEmissiveSamplingStats>);
static_assert(sizeof(DdgiEmissiveSamplingStats) == 36);
static_assert(alignof(DdgiEmissiveSamplingStats) == alignof(uint32_t));
static_assert(offsetof(DdgiEmissiveSamplingStats, nee_attempt_count) == 0);
static_assert(offsetof(DdgiEmissiveSamplingStats, nonzero_contribution_count) == 32);

}  // namespace evo_engine
