#pragma once

#include <cstdint>
#include <limits>

#include "volk.h"

namespace evo_engine::raw_g_buffer {

inline constexpr VkFormat kMetadataFormat = VK_FORMAT_R32G32B32A32_UINT;
inline constexpr uint32_t kClearValue = std::numeric_limits<uint32_t>::max();

}  // namespace evo_engine::raw_g_buffer
