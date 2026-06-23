#pragma once
#include "EvoEngine_SDK_PCH.hpp"

namespace evo_engine {
[[nodiscard]] VkDescriptorImageInfo CreateDdgiFallbackImageInfo();
[[nodiscard]] bool IsValidDescriptorImageInfo(const VkDescriptorImageInfo& image_info);
}  // namespace evo_engine
