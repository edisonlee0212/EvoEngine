// @evoengine-dialect native
// Struct layout mirrors Rendering/PBR/GltfMaterial.hpp with every material extension enabled.
public
static const int EE_GLTF_PBR_MODEL_METALLIC_ROUGHNESS = 0;
public
static const int EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS = 1;
public
static const int EE_GLTF_ALPHA_MODE_OPAQUE = 0;
public
static const int EE_GLTF_ALPHA_MODE_MASK = 1;
public
static const int EE_GLTF_ALPHA_MODE_BLEND = 2;
public
static const int EE_GLTF_TEXTURE_COLOR_SPACE_LINEAR = 0;
public
static const int EE_GLTF_TEXTURE_COLOR_SPACE_SRGB = 1;
public
static const uint EE_GLTF_FEATURE_TRANSMISSION = 1u << 0u;
public
static const uint EE_GLTF_FEATURE_VOLUME = 1u << 1u;
public
static const uint EE_GLTF_FEATURE_VOLUME_SCATTER = 1u << 2u;
public
static const uint EE_GLTF_FEATURE_CLEARCOAT = 1u << 3u;
public
static const uint EE_GLTF_FEATURE_IRIDESCENCE = 1u << 4u;
public
static const uint EE_GLTF_FEATURE_ANISOTROPY = 1u << 5u;
public
static const uint EE_GLTF_FEATURE_SHEEN = 1u << 6u;
public
static const uint EE_GLTF_FEATURE_DISPERSION = 1u << 7u;
public
static const uint EE_GLTF_FEATURE_DIFFUSE_TRANSMISSION = 1u << 8u;
public
static const uint EE_GLTF_FEATURE_RETROREFLECTION = 1u << 9u;
public
static const uint EE_GLTF_FEATURE_UNLIT = 1u << 10u;
public
static const uint EE_GLTF_FEATURE_SPECULAR = 1u << 11u;
public
static const uint EE_GLTF_FEATURE_IOR = 1u << 12u;
public
static const uint EE_GLTF_FEATURE_SPECULAR_GLOSSINESS = 1u << 13u;
public
static const uint EE_GLTF_FEATURE_TEXTURE_TRANSFORM = 1u << 14u;
public
static const uint EE_GLTF_ALL_FEATURES = (1u << 15u) - 1u;

public
struct GltfTextureInfo {
 public
  float3x2 uv_transform;
 public
  int index;
 public
  int tex_coord;
 public
  int color_space;
 public
  int padding;
};

public
float2 EE_GLTF_TRANSFORM_UV(GltfTextureInfo texture_info, float3 uv) {
  return mul(uv, texture_info.uv_transform);
}

public
struct GltfShadeMaterial {
 public
  float4 pbr_base_color_factor;
 public
  float3 emissive_factor;
 public
  float normal_texture_scale;
 public
  float pbr_roughness_factor;
 public
  float pbr_metallic_factor;
 public
  int alpha_mode;
 public
  float alpha_cutoff;
 public
  float occlusion_strength;
 public
  int double_sided;
 public
  float3 attenuation_color;
 public
  float ior;
 public
  float transmission_factor;
 public
  float thickness_factor;
 public
  float attenuation_distance;
 public
  float clearcoat_factor;
 public
  float3 specular_color_factor;
 public
  float clearcoat_roughness;
 public
  float specular_factor;
 public
  int unlit;
 public
  float iridescence_factor;
 public
  float iridescence_thickness_minimum;
 public
  float iridescence_thickness_maximum;
 public
  float iridescence_ior;
 public
  float2 anisotropy_rotation;
 public
  float3 sheen_color_factor;
 public
  float anisotropy_strength;
 public
  float sheen_roughness_factor;
 public
  float dispersion;
 public
  int pbr_model;
 public
  float4 pbr_diffuse_factor;
 public
  float3 pbr_specular_factor;
 public
  float pbr_glossiness_factor;
 public
  float3 diffuse_transmission_color;
 public
  float diffuse_transmission_factor;
 public
  float retroreflection_factor;
 public
  float3 multiscatter_color_factor;
 public
  float scatter_anisotropy;
 public
  uint16_t pbr_base_color_texture;
 public
  uint16_t normal_texture;
 public
  uint16_t pbr_metallic_roughness_texture;
 public
  uint16_t emissive_texture;
 public
  uint16_t occlusion_texture;
 public
  uint16_t transmission_texture;
 public
  uint16_t thickness_texture;
 public
  uint16_t clearcoat_texture;
 public
  uint16_t clearcoat_roughness_texture;
 public
  uint16_t clearcoat_normal_texture;
 public
  uint16_t specular_texture;
 public
  uint16_t specular_color_texture;
 public
  uint16_t iridescence_texture;
 public
  uint16_t iridescence_thickness_texture;
 public
  uint16_t anisotropy_texture;
 public
  uint16_t sheen_color_texture;
 public
  uint16_t sheen_roughness_texture;
 public
  uint16_t pbr_diffuse_texture;
 public
  uint16_t pbr_specular_glossiness_texture;
 public
  uint16_t diffuse_transmission_texture;
 public
  uint16_t diffuse_transmission_color_texture;
 public
  uint16_t retroreflection_texture;
 public
  uint nested_priority;
 public
  float clearcoat_normal_texture_scale;
 public
  uint padding1;
};

[[vk::binding(11, 0)]] public
StructuredBuffer<GltfShadeMaterial, ScalarDataLayout> EE_GLTF_MATERIALS;

[[vk::binding(12, 0)]] public
StructuredBuffer<GltfTextureInfo, ScalarDataLayout> EE_GLTF_TEXTURE_INFOS;
