#ifndef EE_GLTF_MATERIAL_GLSL
#define EE_GLTF_MATERIAL_GLSL

#extension GL_ARB_gpu_shader_int64 : enable
#extension GL_EXT_shader_explicit_arithmetic_types_int16 : require
#extension GL_EXT_scalar_block_layout : require

/*
 * Struct layout derived from vk_gltf_renderer's shaderio::GltfShadeMaterial and
 * shaderio::GltfTextureInfo, licensed under Apache-2.0 by NVIDIA CORPORATION.
 */

#ifndef MAT_EXT_VAL
#define MAT_EXT_VAL 1
#endif

#ifndef MAT_EXT_SPECULAR_GLOSSINESS
#define MAT_EXT_SPECULAR_GLOSSINESS MAT_EXT_VAL
#endif
#ifndef MAT_EXT_IOR
#define MAT_EXT_IOR MAT_EXT_VAL
#endif
#ifndef MAT_EXT_TRANSMISSION
#define MAT_EXT_TRANSMISSION MAT_EXT_VAL
#endif
#ifndef MAT_EXT_VOLUME
#define MAT_EXT_VOLUME MAT_EXT_VAL
#endif
#ifndef MAT_EXT_VOLUME_SCATTER
#define MAT_EXT_VOLUME_SCATTER MAT_EXT_VAL
#endif
#ifndef MAT_EXT_CLEARCOAT
#define MAT_EXT_CLEARCOAT MAT_EXT_VAL
#endif
#ifndef MAT_EXT_IRIDESCENCE
#define MAT_EXT_IRIDESCENCE MAT_EXT_VAL
#endif
#ifndef MAT_EXT_ANISOTROPY
#define MAT_EXT_ANISOTROPY MAT_EXT_VAL
#endif
#ifndef MAT_EXT_SHEEN
#define MAT_EXT_SHEEN MAT_EXT_VAL
#endif
#ifndef MAT_EXT_DISPERSION
#define MAT_EXT_DISPERSION MAT_EXT_VAL
#endif
#ifndef MAT_EXT_DIFFUSE_TRANSMISSION
#define MAT_EXT_DIFFUSE_TRANSMISSION MAT_EXT_VAL
#endif
#ifndef MAT_EXT_RETROREFLECTION
#define MAT_EXT_RETROREFLECTION MAT_EXT_VAL
#endif
#ifndef MAT_EXT_UNLIT
#define MAT_EXT_UNLIT MAT_EXT_VAL
#endif
#ifndef MAT_EXT_SPECULAR
#define MAT_EXT_SPECULAR MAT_EXT_VAL
#endif
#ifndef MAT_EXT_TEXTURE_TRANSFORM
#define MAT_EXT_TEXTURE_TRANSFORM MAT_EXT_VAL
#endif

// MAT_EXT_* controls the storage ABI. Camera-ray variants only override EE_GLTF_USE_* behavior gates.
#ifndef EE_GLTF_USE_TRANSMISSION
#define EE_GLTF_USE_TRANSMISSION MAT_EXT_TRANSMISSION
#endif
#ifndef EE_GLTF_USE_VOLUME
#define EE_GLTF_USE_VOLUME MAT_EXT_VOLUME
#endif
#ifndef EE_GLTF_USE_VOLUME_SCATTER
#define EE_GLTF_USE_VOLUME_SCATTER MAT_EXT_VOLUME_SCATTER
#endif
#ifndef EE_GLTF_USE_CLEARCOAT
#define EE_GLTF_USE_CLEARCOAT MAT_EXT_CLEARCOAT
#endif
#ifndef EE_GLTF_USE_IRIDESCENCE
#define EE_GLTF_USE_IRIDESCENCE MAT_EXT_IRIDESCENCE
#endif
#ifndef EE_GLTF_USE_ANISOTROPY
#define EE_GLTF_USE_ANISOTROPY MAT_EXT_ANISOTROPY
#endif
#ifndef EE_GLTF_USE_SHEEN
#define EE_GLTF_USE_SHEEN MAT_EXT_SHEEN
#endif
#ifndef EE_GLTF_USE_DISPERSION
#define EE_GLTF_USE_DISPERSION MAT_EXT_DISPERSION
#endif
#ifndef EE_GLTF_USE_DIFFUSE_TRANSMISSION
#define EE_GLTF_USE_DIFFUSE_TRANSMISSION MAT_EXT_DIFFUSE_TRANSMISSION
#endif
#ifndef EE_GLTF_USE_RETROREFLECTION
#define EE_GLTF_USE_RETROREFLECTION MAT_EXT_RETROREFLECTION
#endif
#ifndef EE_GLTF_USE_UNLIT
#define EE_GLTF_USE_UNLIT MAT_EXT_UNLIT
#endif
#ifndef EE_GLTF_USE_SPECULAR
#define EE_GLTF_USE_SPECULAR MAT_EXT_SPECULAR
#endif
#ifndef EE_GLTF_USE_IOR
#define EE_GLTF_USE_IOR MAT_EXT_IOR
#endif
#ifndef EE_GLTF_USE_SPECULAR_GLOSSINESS
#define EE_GLTF_USE_SPECULAR_GLOSSINESS MAT_EXT_SPECULAR_GLOSSINESS
#endif
#ifndef EE_GLTF_USE_TEXTURE_TRANSFORM
#define EE_GLTF_USE_TEXTURE_TRANSFORM MAT_EXT_TEXTURE_TRANSFORM
#endif

const int EE_GLTF_PBR_MODEL_METALLIC_ROUGHNESS = 0;
const int EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS = 1;

const int EE_GLTF_ALPHA_MODE_OPAQUE = 0;
const int EE_GLTF_ALPHA_MODE_MASK = 1;
const int EE_GLTF_ALPHA_MODE_BLEND = 2;

const int EE_GLTF_TEXTURE_COLOR_SPACE_LINEAR = 0;
const int EE_GLTF_TEXTURE_COLOR_SPACE_SRGB = 1;

struct GltfTextureInfo {
#if MAT_EXT_TEXTURE_TRANSFORM
  mat3x2 uv_transform;
#endif
  int index;
  int tex_coord;
  int color_space;
  int padding;
};

struct GltfShadeMaterial {
  vec4 pbr_base_color_factor;
  vec3 emissive_factor;
  float normal_texture_scale;

  float pbr_roughness_factor;
  float pbr_metallic_factor;
  int alpha_mode;
  float alpha_cutoff;

  float occlusion_strength;
  int double_sided;

#if MAT_EXT_VOLUME
  vec3 attenuation_color;
#endif

#if MAT_EXT_IOR
  float ior;
#endif

#if MAT_EXT_TRANSMISSION
  float transmission_factor;
#endif

#if MAT_EXT_VOLUME
  float thickness_factor;
  float attenuation_distance;
#endif

#if MAT_EXT_CLEARCOAT
  float clearcoat_factor;
#endif

#if MAT_EXT_SPECULAR
  vec3 specular_color_factor;
#endif

#if MAT_EXT_CLEARCOAT
  float clearcoat_roughness;
#endif

#if MAT_EXT_SPECULAR
  float specular_factor;
#endif

#if MAT_EXT_UNLIT
  int unlit;
#endif

#if MAT_EXT_IRIDESCENCE
  float iridescence_factor;
  float iridescence_thickness_minimum;
  float iridescence_thickness_maximum;
  float iridescence_ior;
#endif

#if MAT_EXT_ANISOTROPY
  vec2 anisotropy_rotation;
#endif

#if MAT_EXT_SHEEN
  vec3 sheen_color_factor;
#endif

#if MAT_EXT_ANISOTROPY
  float anisotropy_strength;
#endif

#if MAT_EXT_SHEEN
  float sheen_roughness_factor;
#endif

#if MAT_EXT_DISPERSION
  float dispersion;
#endif

#if MAT_EXT_SPECULAR_GLOSSINESS
  int pbr_model;

  vec4 pbr_diffuse_factor;
  vec3 pbr_specular_factor;
  float pbr_glossiness_factor;
#endif

#if MAT_EXT_DIFFUSE_TRANSMISSION
  vec3 diffuse_transmission_color;
  float diffuse_transmission_factor;
#endif

#if MAT_EXT_RETROREFLECTION
  float retroreflection_factor;
#endif

#if MAT_EXT_VOLUME_SCATTER
  vec3 multiscatter_color_factor;
  float scatter_anisotropy;
#endif

  uint16_t pbr_base_color_texture;
  uint16_t normal_texture;
  uint16_t pbr_metallic_roughness_texture;
  uint16_t emissive_texture;
  uint16_t occlusion_texture;

#if MAT_EXT_TRANSMISSION
  uint16_t transmission_texture;
#endif

#if MAT_EXT_VOLUME
  uint16_t thickness_texture;
#endif

#if MAT_EXT_CLEARCOAT
  uint16_t clearcoat_texture;
  uint16_t clearcoat_roughness_texture;
  uint16_t clearcoat_normal_texture;
#endif

#if MAT_EXT_SPECULAR
  uint16_t specular_texture;
  uint16_t specular_color_texture;
#endif

#if MAT_EXT_IRIDESCENCE
  uint16_t iridescence_texture;
  uint16_t iridescence_thickness_texture;
#endif

#if MAT_EXT_ANISOTROPY
  uint16_t anisotropy_texture;
#endif

#if MAT_EXT_SHEEN
  uint16_t sheen_color_texture;
  uint16_t sheen_roughness_texture;
#endif

#if MAT_EXT_SPECULAR_GLOSSINESS
  uint16_t pbr_diffuse_texture;
  uint16_t pbr_specular_glossiness_texture;
#endif

#if MAT_EXT_DIFFUSE_TRANSMISSION
  uint16_t diffuse_transmission_texture;
  uint16_t diffuse_transmission_color_texture;
#endif

#if MAT_EXT_RETROREFLECTION
  uint16_t retroreflection_texture;
#endif

  uint nested_priority;
#if MAT_EXT_CLEARCOAT
  float clearcoat_normal_texture_scale;
#else
  float clearcoat_padding;
#endif
  uint padding1;
};

#ifdef EE_GLTF_MATERIALS_BLOCK_BINDING
layout(scalar, set = EE_GLTF_MATERIALS_BLOCK_SET, binding = EE_GLTF_MATERIALS_BLOCK_BINDING) readonly buffer
    EE_GLTF_MATERIAL_BLOCK {
  GltfShadeMaterial EE_GLTF_MATERIALS[];
};
#endif

#ifdef EE_GLTF_TEXTURE_INFOS_BLOCK_BINDING
layout(scalar, set = EE_GLTF_TEXTURE_INFOS_BLOCK_SET, binding = EE_GLTF_TEXTURE_INFOS_BLOCK_BINDING) readonly buffer
    EE_GLTF_TEXTURE_INFO_BLOCK {
  GltfTextureInfo EE_GLTF_TEXTURE_INFOS[];
};
#endif

#endif
