#pragma once

/*
 * Struct layout derived from vk_gltf_renderer's shaderio::GltfShadeMaterial and
 * shaderio::GltfTextureInfo, licensed under Apache-2.0 by NVIDIA CORPORATION.
 */

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#ifndef MAT_EXT_VAL
#  define MAT_EXT_VAL 1
#endif

#ifndef MAT_EXT_IOR
#  define MAT_EXT_IOR MAT_EXT_VAL
#endif
#ifndef MAT_EXT_TRANSMISSION
#  define MAT_EXT_TRANSMISSION MAT_EXT_VAL
#endif
#ifndef MAT_EXT_VOLUME
#  define MAT_EXT_VOLUME MAT_EXT_VAL
#endif
#ifndef MAT_EXT_VOLUME_SCATTER
#  define MAT_EXT_VOLUME_SCATTER MAT_EXT_VAL
#endif
#ifndef MAT_EXT_CLEARCOAT
#  define MAT_EXT_CLEARCOAT MAT_EXT_VAL
#endif
#ifndef MAT_EXT_IRIDESCENCE
#  define MAT_EXT_IRIDESCENCE MAT_EXT_VAL
#endif
#ifndef MAT_EXT_ANISOTROPY
#  define MAT_EXT_ANISOTROPY MAT_EXT_VAL
#endif
#ifndef MAT_EXT_SHEEN
#  define MAT_EXT_SHEEN MAT_EXT_VAL
#endif
#ifndef MAT_EXT_DISPERSION
#  define MAT_EXT_DISPERSION MAT_EXT_VAL
#endif
#ifndef MAT_EXT_DIFFUSE_TRANSMISSION
#  define MAT_EXT_DIFFUSE_TRANSMISSION MAT_EXT_VAL
#endif
#ifndef MAT_EXT_RETROREFLECTION
#  define MAT_EXT_RETROREFLECTION MAT_EXT_VAL
#endif
#ifndef MAT_EXT_UNLIT
#  define MAT_EXT_UNLIT MAT_EXT_VAL
#endif
#ifndef MAT_EXT_SPECULAR
#  define MAT_EXT_SPECULAR MAT_EXT_VAL
#endif
#ifndef MAT_EXT_TEXTURE_TRANSFORM
#  define MAT_EXT_TEXTURE_TRANSFORM MAT_EXT_VAL
#endif

namespace evo_engine {

enum class GltfAlphaMode : int32_t {
  Opaque = 0,
  Mask = 1,
  Blend = 2,
};

enum class GltfTextureColorSpace : int32_t {
  Linear = 0,
  Srgb = 1,
};

struct GltfTextureInfo {
#if MAT_EXT_TEXTURE_TRANSFORM
  glm::mat3x2 uv_transform = glm::mat3x2(1.0f);
#endif
  int32_t index = -1;
  int32_t tex_coord = 0;
  int32_t color_space = static_cast<int32_t>(GltfTextureColorSpace::Linear);
  int32_t padding = 0;
};

inline bool operator!=(const GltfTextureInfo& lhs, const GltfTextureInfo& rhs) {
#if MAT_EXT_TEXTURE_TRANSFORM
  if (lhs.uv_transform != rhs.uv_transform)
    return true;
#endif
  if (lhs.index != rhs.index)
    return true;
  if (lhs.tex_coord != rhs.tex_coord)
    return true;
  if (lhs.color_space != rhs.color_space)
    return true;
  return false;
}

inline bool operator==(const GltfTextureInfo& lhs, const GltfTextureInfo& rhs) {
  return !(lhs != rhs);
}

struct alignas(8) GltfShadeMaterial {
  glm::vec4 pbr_base_color_factor = glm::vec4(1.0f);
  glm::vec3 emissive_factor = glm::vec3(0.0f);
  float normal_texture_scale = 1.0f;

  float pbr_roughness_factor = 1.0f;
  float pbr_metallic_factor = 1.0f;
  int32_t alpha_mode = static_cast<int32_t>(GltfAlphaMode::Opaque);
  float alpha_cutoff = 0.5f;

  float occlusion_strength = 1.0f;
  int32_t double_sided = 0;

#if MAT_EXT_VOLUME
  glm::vec3 attenuation_color = glm::vec3(1.0f);
#endif

#if MAT_EXT_IOR
  float ior = 1.5f;
#endif

#if MAT_EXT_TRANSMISSION
  float transmission_factor = 0.0f;
#endif

#if MAT_EXT_VOLUME
  float thickness_factor = 0.0f;
  float attenuation_distance = 0.0f;
#endif

#if MAT_EXT_CLEARCOAT
  float clearcoat_factor = 0.0f;
#endif

#if MAT_EXT_SPECULAR
  glm::vec3 specular_color_factor = glm::vec3(1.0f);
#endif

#if MAT_EXT_CLEARCOAT
  float clearcoat_roughness = 0.0f;
#endif

#if MAT_EXT_SPECULAR
  float specular_factor = 1.0f;
#endif

#if MAT_EXT_UNLIT
  int32_t unlit = 0;
#endif

#if MAT_EXT_IRIDESCENCE
  float iridescence_factor = 0.0f;
  float iridescence_thickness_minimum = 100.0f;
  float iridescence_thickness_maximum = 400.0f;
  float iridescence_ior = 1.3f;
#endif

#if MAT_EXT_ANISOTROPY
  glm::vec2 anisotropy_rotation = glm::vec2(1.0f, 0.0f);
#endif

#if MAT_EXT_SHEEN
  glm::vec3 sheen_color_factor = glm::vec3(0.0f);
#endif

#if MAT_EXT_ANISOTROPY
  float anisotropy_strength = 0.0f;
#endif

#if MAT_EXT_SHEEN
  float sheen_roughness_factor = 0.0f;
#endif

#if MAT_EXT_DISPERSION
  float dispersion = 0.0f;
#endif

#if MAT_EXT_DIFFUSE_TRANSMISSION
  glm::vec3 diffuse_transmission_color = glm::vec3(1.0f);
  float diffuse_transmission_factor = 0.0f;
#endif

#if MAT_EXT_RETROREFLECTION
  float retroreflection_factor = 0.0f;
#endif

#if MAT_EXT_VOLUME_SCATTER
  glm::vec3 multiscatter_color_factor = glm::vec3(0.0f);
  float scatter_anisotropy = 0.0f;
#endif

  uint16_t pbr_base_color_texture = 0;
  uint16_t normal_texture = 0;
  uint16_t pbr_metallic_roughness_texture = 0;
  uint16_t emissive_texture = 0;
  uint16_t occlusion_texture = 0;

#if MAT_EXT_TRANSMISSION
  uint16_t transmission_texture = 0;
#endif

#if MAT_EXT_VOLUME
  uint16_t thickness_texture = 0;
#endif

#if MAT_EXT_CLEARCOAT
  uint16_t clearcoat_texture = 0;
  uint16_t clearcoat_roughness_texture = 0;
  uint16_t clearcoat_normal_texture = 0;
#endif

#if MAT_EXT_SPECULAR
  uint16_t specular_texture = 0;
  uint16_t specular_color_texture = 0;
#endif

#if MAT_EXT_IRIDESCENCE
  uint16_t iridescence_texture = 0;
  uint16_t iridescence_thickness_texture = 0;
#endif

#if MAT_EXT_ANISOTROPY
  uint16_t anisotropy_texture = 0;
#endif

#if MAT_EXT_SHEEN
  uint16_t sheen_color_texture = 0;
  uint16_t sheen_roughness_texture = 0;
#endif

#if MAT_EXT_DIFFUSE_TRANSMISSION
  uint16_t diffuse_transmission_texture = 0;
  uint16_t diffuse_transmission_color_texture = 0;
#endif

#if MAT_EXT_RETROREFLECTION
  uint16_t retroreflection_texture = 0;
#endif

  uint32_t nested_priority = 0;
#if MAT_EXT_CLEARCOAT
  float clearcoat_normal_texture_scale = 1.0f;
#else
  float clearcoat_padding = 0.0f;
#endif
  uint32_t padding1 = 0;
};

inline bool GltfMaterialRequiresTransparentPass(const GltfShadeMaterial& material) {
  if (material.alpha_mode == static_cast<int32_t>(GltfAlphaMode::Blend)) {
    return true;
  }
#if MAT_EXT_TRANSMISSION
  if (material.transmission_factor > 0.0f) {
    return true;
  }
#endif
#if MAT_EXT_DIFFUSE_TRANSMISSION
  if (material.diffuse_transmission_factor > 0.0f) {
    return true;
  }
#endif
  return false;
}

enum class GltfRasterMaterialClass : uint8_t {
  Opaque,
  Masked,
  Forward,
};

inline GltfRasterMaterialClass ClassifyGltfRasterMaterial(const GltfShadeMaterial& material,
                                                          const bool blending_enabled) {
  if (blending_enabled || GltfMaterialRequiresTransparentPass(material)) {
    return GltfRasterMaterialClass::Forward;
  }
  if (material.alpha_mode == static_cast<int32_t>(GltfAlphaMode::Mask)) {
    return GltfRasterMaterialClass::Masked;
  }
  return GltfRasterMaterialClass::Opaque;
}

inline bool operator!=(const GltfShadeMaterial& lhs, const GltfShadeMaterial& rhs) {
  if (lhs.pbr_base_color_factor != rhs.pbr_base_color_factor)
    return true;
  if (lhs.emissive_factor != rhs.emissive_factor)
    return true;
  if (lhs.normal_texture_scale != rhs.normal_texture_scale)
    return true;
  if (lhs.pbr_roughness_factor != rhs.pbr_roughness_factor)
    return true;
  if (lhs.pbr_metallic_factor != rhs.pbr_metallic_factor)
    return true;
  if (lhs.alpha_mode != rhs.alpha_mode)
    return true;
  if (lhs.alpha_cutoff != rhs.alpha_cutoff)
    return true;
  if (lhs.occlusion_strength != rhs.occlusion_strength)
    return true;
  if (lhs.double_sided != rhs.double_sided)
    return true;
#if MAT_EXT_VOLUME
  if (lhs.attenuation_color != rhs.attenuation_color)
    return true;
#endif
#if MAT_EXT_IOR
  if (lhs.ior != rhs.ior)
    return true;
#endif
#if MAT_EXT_TRANSMISSION
  if (lhs.transmission_factor != rhs.transmission_factor)
    return true;
#endif
#if MAT_EXT_VOLUME
  if (lhs.thickness_factor != rhs.thickness_factor)
    return true;
  if (lhs.attenuation_distance != rhs.attenuation_distance)
    return true;
#endif
#if MAT_EXT_CLEARCOAT
  if (lhs.clearcoat_factor != rhs.clearcoat_factor)
    return true;
#endif
#if MAT_EXT_SPECULAR
  if (lhs.specular_color_factor != rhs.specular_color_factor)
    return true;
#endif
#if MAT_EXT_CLEARCOAT
  if (lhs.clearcoat_roughness != rhs.clearcoat_roughness)
    return true;
#endif
#if MAT_EXT_SPECULAR
  if (lhs.specular_factor != rhs.specular_factor)
    return true;
#endif
#if MAT_EXT_UNLIT
  if (lhs.unlit != rhs.unlit)
    return true;
#endif
#if MAT_EXT_IRIDESCENCE
  if (lhs.iridescence_factor != rhs.iridescence_factor)
    return true;
  if (lhs.iridescence_thickness_minimum != rhs.iridescence_thickness_minimum)
    return true;
  if (lhs.iridescence_thickness_maximum != rhs.iridescence_thickness_maximum)
    return true;
  if (lhs.iridescence_ior != rhs.iridescence_ior)
    return true;
#endif
#if MAT_EXT_ANISOTROPY
  if (lhs.anisotropy_rotation != rhs.anisotropy_rotation)
    return true;
#endif
#if MAT_EXT_SHEEN
  if (lhs.sheen_color_factor != rhs.sheen_color_factor)
    return true;
#endif
#if MAT_EXT_ANISOTROPY
  if (lhs.anisotropy_strength != rhs.anisotropy_strength)
    return true;
#endif
#if MAT_EXT_SHEEN
  if (lhs.sheen_roughness_factor != rhs.sheen_roughness_factor)
    return true;
#endif
#if MAT_EXT_DISPERSION
  if (lhs.dispersion != rhs.dispersion)
    return true;
#endif
#if MAT_EXT_DIFFUSE_TRANSMISSION
  if (lhs.diffuse_transmission_color != rhs.diffuse_transmission_color)
    return true;
  if (lhs.diffuse_transmission_factor != rhs.diffuse_transmission_factor)
    return true;
#endif
#if MAT_EXT_RETROREFLECTION
  if (lhs.retroreflection_factor != rhs.retroreflection_factor)
    return true;
#endif
#if MAT_EXT_VOLUME_SCATTER
  if (lhs.multiscatter_color_factor != rhs.multiscatter_color_factor)
    return true;
  if (lhs.scatter_anisotropy != rhs.scatter_anisotropy)
    return true;
#endif
  if (lhs.pbr_base_color_texture != rhs.pbr_base_color_texture)
    return true;
  if (lhs.normal_texture != rhs.normal_texture)
    return true;
  if (lhs.pbr_metallic_roughness_texture != rhs.pbr_metallic_roughness_texture)
    return true;
  if (lhs.emissive_texture != rhs.emissive_texture)
    return true;
  if (lhs.occlusion_texture != rhs.occlusion_texture)
    return true;
#if MAT_EXT_TRANSMISSION
  if (lhs.transmission_texture != rhs.transmission_texture)
    return true;
#endif
#if MAT_EXT_VOLUME
  if (lhs.thickness_texture != rhs.thickness_texture)
    return true;
#endif
#if MAT_EXT_CLEARCOAT
  if (lhs.clearcoat_texture != rhs.clearcoat_texture)
    return true;
  if (lhs.clearcoat_roughness_texture != rhs.clearcoat_roughness_texture)
    return true;
  if (lhs.clearcoat_normal_texture != rhs.clearcoat_normal_texture)
    return true;
#endif
#if MAT_EXT_SPECULAR
  if (lhs.specular_texture != rhs.specular_texture)
    return true;
  if (lhs.specular_color_texture != rhs.specular_color_texture)
    return true;
#endif
#if MAT_EXT_IRIDESCENCE
  if (lhs.iridescence_texture != rhs.iridescence_texture)
    return true;
  if (lhs.iridescence_thickness_texture != rhs.iridescence_thickness_texture)
    return true;
#endif
#if MAT_EXT_ANISOTROPY
  if (lhs.anisotropy_texture != rhs.anisotropy_texture)
    return true;
#endif
#if MAT_EXT_SHEEN
  if (lhs.sheen_color_texture != rhs.sheen_color_texture)
    return true;
  if (lhs.sheen_roughness_texture != rhs.sheen_roughness_texture)
    return true;
#endif
#if MAT_EXT_DIFFUSE_TRANSMISSION
  if (lhs.diffuse_transmission_texture != rhs.diffuse_transmission_texture)
    return true;
  if (lhs.diffuse_transmission_color_texture != rhs.diffuse_transmission_color_texture)
    return true;
#endif
#if MAT_EXT_RETROREFLECTION
  if (lhs.retroreflection_texture != rhs.retroreflection_texture)
    return true;
#endif
  if (lhs.nested_priority != rhs.nested_priority)
    return true;
#if MAT_EXT_CLEARCOAT
  if (lhs.clearcoat_normal_texture_scale != rhs.clearcoat_normal_texture_scale)
    return true;
#else
  if (lhs.clearcoat_padding != rhs.clearcoat_padding)
    return true;
#endif
  if (lhs.padding1 != rhs.padding1)
    return true;
  return false;
}

inline bool operator==(const GltfShadeMaterial& lhs, const GltfShadeMaterial& rhs) {
  return !(lhs != rhs);
}

struct GltfMaterialData {
  GltfShadeMaterial shade_material{};
  std::vector<GltfTextureInfo> texture_infos{GltfTextureInfo{}};
};

}  // namespace evo_engine
