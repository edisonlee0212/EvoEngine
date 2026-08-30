#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include "EvoEngineAPI.hpp"

namespace evo_engine::bc7_texture_codec {

struct MipLevel {
  glm::uvec2 resolution = glm::uvec2(0);
  std::vector<glm::vec4> pixels;
};

struct MipChain {
  std::vector<MipLevel> levels;
  uint64_t content_hash = 0;
};

EVOENGINE_API bool DecodeDds(const std::filesystem::path& path, MipChain& result, std::string* error = nullptr);
EVOENGINE_API bool DecodeDds(const std::vector<std::byte>& bytes, MipChain& result, std::string* error = nullptr);
EVOENGINE_API bool WriteDds(const std::filesystem::path& path, const std::vector<MipLevel>& levels, bool srgb,
                            bool perceptual, std::string* error = nullptr);
EVOENGINE_API bool ValidateDds(const std::filesystem::path& path, glm::uvec2 expected_resolution, bool srgb,
                               std::string* error = nullptr);

}  // namespace evo_engine::bc7_texture_codec
