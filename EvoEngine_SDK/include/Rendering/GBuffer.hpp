#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include <glm/glm.hpp>
#include "volk.h"

namespace evo_engine::raw_g_buffer {

inline constexpr VkFormat kAttributeFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
inline constexpr VkFormat kMetadataFormat = VK_FORMAT_R32G32B32A32_UINT;
inline constexpr VkFormat kDepthFormat = VK_FORMAT_D32_SFLOAT;

inline constexpr uint32_t kMaterialIndexMask = 0x3fffffffu;
inline constexpr uint32_t kVertexColorReplacesBaseColorBit = 0x40000000u;
inline constexpr uint32_t kNegativeTangentHandednessBit = 0x80000000u;
inline constexpr uint32_t kClearValue = std::numeric_limits<uint32_t>::max();

struct Metadata {
  uint32_t instance_index = kClearValue;
  uint32_t material_index = kMaterialIndexMask;
  uint32_t info_index = kClearValue;
  glm::vec4 vertex_color = glm::vec4(1.0f);
  bool vertex_color_replaces_base_color = false;
  bool negative_tangent_handedness = false;
};

inline uint32_t PackUnorm8(const glm::vec4& value) {
  const glm::uvec4 quantized =
      glm::uvec4(glm::floor(glm::clamp(value, glm::vec4(0.0f), glm::vec4(1.0f)) * 255.0f + glm::vec4(0.5f)));
  return quantized.r | quantized.g << 8u | quantized.b << 16u | quantized.a << 24u;
}

inline glm::vec4 UnpackUnorm8(const uint32_t value) {
  return glm::vec4(value & 0xffu, value >> 8u & 0xffu, value >> 16u & 0xffu, value >> 24u & 0xffu) / 255.0f;
}

inline glm::uvec4 PackMetadata(const Metadata& metadata) {
  uint32_t material_and_flags = metadata.material_index & kMaterialIndexMask;
  if (metadata.vertex_color_replaces_base_color) {
    material_and_flags |= kVertexColorReplacesBaseColorBit;
  }
  if (metadata.negative_tangent_handedness) {
    material_and_flags |= kNegativeTangentHandednessBit;
  }
  return {metadata.instance_index, material_and_flags, metadata.info_index, PackUnorm8(metadata.vertex_color)};
}

inline Metadata UnpackMetadata(const glm::uvec4& value) {
  Metadata metadata;
  metadata.instance_index = value.x;
  metadata.material_index = value.y & kMaterialIndexMask;
  metadata.info_index = value.z;
  metadata.vertex_color = UnpackUnorm8(value.w);
  metadata.vertex_color_replaces_base_color = (value.y & kVertexColorReplacesBaseColorBit) != 0u;
  metadata.negative_tangent_handedness = (value.y & kNegativeTangentHandednessBit) != 0u;
  return metadata;
}

inline glm::uvec4 ClearMetadata() {
  return glm::uvec4(kClearValue);
}

inline glm::vec2 OctEncode(const glm::vec3& direction) {
  const float length_squared = glm::dot(direction, direction);
  glm::vec3 normalized =
      length_squared > 0.0f ? direction * glm::inversesqrt(length_squared) : glm::vec3(0.0f, 0.0f, 1.0f);
  normalized /= glm::abs(normalized.x) + glm::abs(normalized.y) + glm::abs(normalized.z);
  glm::vec2 encoded = glm::vec2(normalized);
  if (normalized.z < 0.0f) {
    const glm::vec2 sign = {encoded.x >= 0.0f ? 1.0f : -1.0f, encoded.y >= 0.0f ? 1.0f : -1.0f};
    encoded = (1.0f - glm::abs(glm::vec2(encoded.y, encoded.x))) * sign;
  }
  return encoded;
}

inline glm::vec3 OctDecode(const glm::vec2& encoded) {
  glm::vec3 direction(encoded, 1.0f - glm::abs(encoded.x) - glm::abs(encoded.y));
  if (direction.z < 0.0f) {
    const glm::vec2 sign = {direction.x >= 0.0f ? 1.0f : -1.0f, direction.y >= 0.0f ? 1.0f : -1.0f};
    const glm::vec2 unfolded = (1.0f - glm::abs(glm::vec2(direction.y, direction.x))) * sign;
    direction.x = unfolded.x;
    direction.y = unfolded.y;
  }
  return glm::normalize(direction);
}

}  // namespace evo_engine::raw_g_buffer
