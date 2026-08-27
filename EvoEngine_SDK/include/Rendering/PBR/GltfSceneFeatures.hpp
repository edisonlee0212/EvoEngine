#pragma once

#include "GltfMaterial.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace evo_engine {

enum class GltfSceneFeature : uint32_t {
  Transmission = 1u << 0u,
  Volume = 1u << 1u,
  VolumeScatter = 1u << 2u,
  Clearcoat = 1u << 3u,
  Iridescence = 1u << 4u,
  Anisotropy = 1u << 5u,
  Sheen = 1u << 6u,
  Dispersion = 1u << 7u,
  DiffuseTransmission = 1u << 8u,
  Retroreflection = 1u << 9u,
  Unlit = 1u << 10u,
  Specular = 1u << 11u,
  Ior = 1u << 12u,
  SpecularGlossiness = 1u << 13u,
  TextureTransform = 1u << 14u,
};

constexpr uint32_t kGltfSceneAllFeatures = (1u << 15u) - 1u;

[[nodiscard]] EVOENGINE_API uint32_t PromoteGltfSceneFeatures(uint32_t feature_mask);
[[nodiscard]] EVOENGINE_API uint32_t DetectGltfSceneFeatures(const std::vector<GltfShadeMaterial>& materials,
                                                             const std::vector<GltfTextureInfo>& texture_infos);
[[nodiscard]] EVOENGINE_API std::string BuildGltfSceneFeatureDefines(uint32_t feature_mask);
[[nodiscard]] EVOENGINE_API std::string FormatGltfSceneFeatureMask(uint32_t feature_mask);

}  // namespace evo_engine
