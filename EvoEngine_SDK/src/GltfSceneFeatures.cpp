#include "GltfSceneFeatures.hpp"

#include <array>
#include <iomanip>
#include <sstream>

using namespace evo_engine;

namespace {
constexpr uint32_t Feature(const GltfSceneFeature feature) {
  return static_cast<uint32_t>(feature);
}

bool HasTexture(const uint16_t slot, const std::vector<GltfTextureInfo>& texture_infos) {
  return slot > 0u && slot < texture_infos.size() && texture_infos[slot].index >= 0;
}

std::array<uint16_t, 22> GetTextureSlots(const GltfShadeMaterial& material) {
  return {material.pbr_base_color_texture,
          material.normal_texture,
          material.pbr_metallic_roughness_texture,
          material.emissive_texture,
          material.occlusion_texture,
          material.transmission_texture,
          material.thickness_texture,
          material.clearcoat_texture,
          material.clearcoat_roughness_texture,
          material.clearcoat_normal_texture,
          material.specular_texture,
          material.specular_color_texture,
          material.iridescence_texture,
          material.iridescence_thickness_texture,
          material.anisotropy_texture,
          material.sheen_color_texture,
          material.sheen_roughness_texture,
          material.pbr_diffuse_texture,
          material.pbr_specular_glossiness_texture,
          material.diffuse_transmission_texture,
          material.diffuse_transmission_color_texture,
          material.retroreflection_texture};
}

bool UsesTextureTransform(const GltfShadeMaterial& material, const std::vector<GltfTextureInfo>& texture_infos) {
  const glm::mat3x2 identity(1.0f);
  for (const auto slot : GetTextureSlots(material)) {
    if (HasTexture(slot, texture_infos) && texture_infos[slot].uv_transform != identity) {
      return true;
    }
  }
  return false;
}
}  // namespace

uint32_t evo_engine::PromoteGltfSceneFeatures(uint32_t feature_mask) {
  feature_mask &= kGltfSceneAllFeatures;
  if ((feature_mask & Feature(GltfSceneFeature::VolumeScatter)) != 0u) {
    feature_mask |= Feature(GltfSceneFeature::Volume);
  }
  if ((feature_mask & Feature(GltfSceneFeature::Volume)) != 0u) {
    feature_mask |= Feature(GltfSceneFeature::Transmission);
  }
  return feature_mask;
}

uint32_t evo_engine::DetectGltfSceneFeatures(const std::vector<GltfShadeMaterial>& materials,
                                             const std::vector<GltfTextureInfo>& texture_infos) {
  uint32_t result = 0;
  for (const auto& material : materials) {
    if (material.transmission_factor != 0.0f || HasTexture(material.transmission_texture, texture_infos))
      result |= Feature(GltfSceneFeature::Transmission);
    if (material.thickness_factor != 0.0f || HasTexture(material.thickness_texture, texture_infos))
      result |= Feature(GltfSceneFeature::Volume);
    if (material.multiscatter_color_factor != glm::vec3(0.0f) || material.scatter_anisotropy != 0.0f)
      result |= Feature(GltfSceneFeature::VolumeScatter);
    if (material.clearcoat_factor != 0.0f || material.clearcoat_roughness != 0.0f ||
        HasTexture(material.clearcoat_texture, texture_infos) ||
        HasTexture(material.clearcoat_roughness_texture, texture_infos) ||
        HasTexture(material.clearcoat_normal_texture, texture_infos))
      result |= Feature(GltfSceneFeature::Clearcoat);
    if (material.iridescence_factor != 0.0f || material.iridescence_ior != 1.3f ||
        material.iridescence_thickness_minimum != 100.0f || material.iridescence_thickness_maximum != 400.0f ||
        HasTexture(material.iridescence_texture, texture_infos) ||
        HasTexture(material.iridescence_thickness_texture, texture_infos))
      result |= Feature(GltfSceneFeature::Iridescence);
    if (material.anisotropy_strength != 0.0f || material.anisotropy_rotation != glm::vec2(1.0f, 0.0f) ||
        HasTexture(material.anisotropy_texture, texture_infos))
      result |= Feature(GltfSceneFeature::Anisotropy);
    if (material.sheen_color_factor != glm::vec3(0.0f) || material.sheen_roughness_factor != 0.0f ||
        HasTexture(material.sheen_color_texture, texture_infos) ||
        HasTexture(material.sheen_roughness_texture, texture_infos))
      result |= Feature(GltfSceneFeature::Sheen);
    if (material.dispersion > 0.0f)
      result |= Feature(GltfSceneFeature::Dispersion);
    if (material.diffuse_transmission_factor != 0.0f || material.diffuse_transmission_color != glm::vec3(1.0f) ||
        HasTexture(material.diffuse_transmission_texture, texture_infos) ||
        HasTexture(material.diffuse_transmission_color_texture, texture_infos))
      result |= Feature(GltfSceneFeature::DiffuseTransmission);
    if (material.retroreflection_factor != 0.0f || HasTexture(material.retroreflection_texture, texture_infos))
      result |= Feature(GltfSceneFeature::Retroreflection);
    if (material.unlit != 0)
      result |= Feature(GltfSceneFeature::Unlit);
    if (material.specular_factor != 1.0f || material.specular_color_factor != glm::vec3(1.0f) ||
        HasTexture(material.specular_texture, texture_infos) ||
        HasTexture(material.specular_color_texture, texture_infos))
      result |= Feature(GltfSceneFeature::Specular);
    if (material.ior != 1.5f)
      result |= Feature(GltfSceneFeature::Ior);
    if (material.pbr_model == static_cast<int32_t>(GltfPbrModel::SpecularGlossiness) ||
        material.pbr_diffuse_factor != glm::vec4(1.0f) || material.pbr_specular_factor != glm::vec3(1.0f) ||
        material.pbr_glossiness_factor != 1.0f || HasTexture(material.pbr_diffuse_texture, texture_infos) ||
        HasTexture(material.pbr_specular_glossiness_texture, texture_infos))
      result |= Feature(GltfSceneFeature::SpecularGlossiness);
    if (UsesTextureTransform(material, texture_infos))
      result |= Feature(GltfSceneFeature::TextureTransform);
  }
  return PromoteGltfSceneFeatures(result);
}

std::string evo_engine::BuildGltfSceneFeatureDefines(const uint32_t feature_mask) {
  const auto mask = PromoteGltfSceneFeatures(feature_mask);
  const std::array<std::pair<const char*, GltfSceneFeature>, 15> features = {{
      {"TRANSMISSION", GltfSceneFeature::Transmission},
      {"VOLUME", GltfSceneFeature::Volume},
      {"VOLUME_SCATTER", GltfSceneFeature::VolumeScatter},
      {"CLEARCOAT", GltfSceneFeature::Clearcoat},
      {"IRIDESCENCE", GltfSceneFeature::Iridescence},
      {"ANISOTROPY", GltfSceneFeature::Anisotropy},
      {"SHEEN", GltfSceneFeature::Sheen},
      {"DISPERSION", GltfSceneFeature::Dispersion},
      {"DIFFUSE_TRANSMISSION", GltfSceneFeature::DiffuseTransmission},
      {"RETROREFLECTION", GltfSceneFeature::Retroreflection},
      {"UNLIT", GltfSceneFeature::Unlit},
      {"SPECULAR", GltfSceneFeature::Specular},
      {"IOR", GltfSceneFeature::Ior},
      {"SPECULAR_GLOSSINESS", GltfSceneFeature::SpecularGlossiness},
      {"TEXTURE_TRANSFORM", GltfSceneFeature::TextureTransform},
  }};
  std::string result;
  for (const auto& [name, feature] : features) {
    result += "#define EE_GLTF_USE_" + std::string(name) + " " + ((mask & Feature(feature)) != 0u ? "1\n" : "0\n");
  }
  return result;
}

std::string evo_engine::FormatGltfSceneFeatureMask(const uint32_t feature_mask) {
  std::ostringstream stream;
  stream << "0x" << std::hex << std::setfill('0') << std::setw(4)
         << (PromoteGltfSceneFeatures(feature_mask) & kGltfSceneAllFeatures);
  return stream.str();
}
