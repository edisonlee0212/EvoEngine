#ifndef EE_GLTF_RASTER_MATERIAL_GLSL
#define EE_GLTF_RASTER_MATERIAL_GLSL

#include "GltfMaterial.glsl"

const float EE_GLTF_MICROFACET_MIN_ROUGHNESS = 0.0014142;

#ifndef EE_GLTF_RASTER_MATERIAL_SET
#define EE_GLTF_RASTER_MATERIAL_SET 3
#endif
#define EE_GLTF_RASTER_BASE_COLOR_TEXTURE_BINDING 0
#define EE_GLTF_RASTER_METALLIC_ROUGHNESS_TEXTURE_BINDING 1
#define EE_GLTF_RASTER_NORMAL_TEXTURE_BINDING 2
#define EE_GLTF_RASTER_EMISSIVE_TEXTURE_BINDING 3
#define EE_GLTF_RASTER_OCCLUSION_TEXTURE_BINDING 4
#define EE_GLTF_RASTER_CLEARCOAT_TEXTURE_BINDING 5
#define EE_GLTF_RASTER_CLEARCOAT_ROUGHNESS_TEXTURE_BINDING 6
#define EE_GLTF_RASTER_CLEARCOAT_NORMAL_TEXTURE_BINDING 7

const int EE_GLTF_RASTER_TEXTURE_UNSUPPORTED_EXTENSION = -1;
const int EE_GLTF_RASTER_TEXTURE_BASE_COLOR = 0;
const int EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS = 1;
const int EE_GLTF_RASTER_TEXTURE_NORMAL = 2;
const int EE_GLTF_RASTER_TEXTURE_EMISSIVE = 3;
const int EE_GLTF_RASTER_TEXTURE_OCCLUSION = 4;
const int EE_GLTF_RASTER_TEXTURE_CLEARCOAT = 5;
const int EE_GLTF_RASTER_TEXTURE_CLEARCOAT_ROUGHNESS = 6;
const int EE_GLTF_RASTER_TEXTURE_CLEARCOAT_NORMAL = 7;

#ifdef EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES
layout(set = EE_GLTF_RASTER_MATERIAL_SET, binding = EE_GLTF_RASTER_BASE_COLOR_TEXTURE_BINDING) uniform sampler2D
    EE_GLTF_RASTER_BASE_COLOR_TEXTURE;
layout(set = EE_GLTF_RASTER_MATERIAL_SET, binding = EE_GLTF_RASTER_METALLIC_ROUGHNESS_TEXTURE_BINDING) uniform sampler2D
    EE_GLTF_RASTER_METALLIC_ROUGHNESS_TEXTURE;
layout(set = EE_GLTF_RASTER_MATERIAL_SET, binding = EE_GLTF_RASTER_NORMAL_TEXTURE_BINDING) uniform sampler2D
    EE_GLTF_RASTER_NORMAL_TEXTURE;
layout(set = EE_GLTF_RASTER_MATERIAL_SET, binding = EE_GLTF_RASTER_EMISSIVE_TEXTURE_BINDING) uniform sampler2D
    EE_GLTF_RASTER_EMISSIVE_TEXTURE;
layout(set = EE_GLTF_RASTER_MATERIAL_SET, binding = EE_GLTF_RASTER_OCCLUSION_TEXTURE_BINDING) uniform sampler2D
    EE_GLTF_RASTER_OCCLUSION_TEXTURE;
layout(set = EE_GLTF_RASTER_MATERIAL_SET, binding = EE_GLTF_RASTER_CLEARCOAT_TEXTURE_BINDING) uniform sampler2D
    EE_GLTF_RASTER_CLEARCOAT_TEXTURE;
layout(set = EE_GLTF_RASTER_MATERIAL_SET, binding = EE_GLTF_RASTER_CLEARCOAT_ROUGHNESS_TEXTURE_BINDING) uniform sampler2D
    EE_GLTF_RASTER_CLEARCOAT_ROUGHNESS_TEXTURE;
layout(set = EE_GLTF_RASTER_MATERIAL_SET, binding = EE_GLTF_RASTER_CLEARCOAT_NORMAL_TEXTURE_BINDING) uniform sampler2D
    EE_GLTF_RASTER_CLEARCOAT_NORMAL_TEXTURE;
#endif

struct GltfRasterMaterial {
  vec4 base_color;
  vec3 specular_f0;
  float specular_f90;
  vec3 emissive;
  float clearcoat;
  float clearcoat_roughness;
  float metallic;
  float roughness;
  float occlusion;
  int alpha_mode;
  float alpha_cutoff;
  float transmission;
  vec3 attenuation_color;
  float attenuation_distance;
  float thickness;
  vec3 diffuse_transmission_color;
  float diffuse_transmission_factor;
  vec3 multiscatter_color_factor;
  vec3 scatter_coefficient;
  float scatter_anisotropy;
};

struct GltfTexCoords {
  vec2 uv0;
  vec2 uv1;
  vec2 uv2;
  vec2 uv3;
  vec4 gradients;
};

GltfTexCoords EE_GLTF_MAKE_TEX_COORDS(vec2 uv0, vec2 uv1, vec2 uv2, vec2 uv3, vec4 gradients) {
  return GltfTexCoords(uv0, uv1, uv2, uv3, gradients);
}

GltfTexCoords EE_GLTF_MAKE_TEX_COORDS(vec2 uv0, vec2 uv1) {
  return EE_GLTF_MAKE_TEX_COORDS(uv0, uv1, vec2(0.0), vec2(0.0), vec4(0.0));
}

bool EE_GLTF_HAS_TEXTURE(uint16_t texture_info_slot) {
  return uint(texture_info_slot) > 0u;
}

vec2 EE_GLTF_SELECT_TEX_COORD(int tex_coord, GltfTexCoords tex_coords) {
  if (tex_coord == 1) return tex_coords.uv1;
  if (tex_coord == 2) return tex_coords.uv2;
  if (tex_coord == 3) return tex_coords.uv3;
  return tex_coords.uv0;
}

vec2 EE_GLTF_TEXTURE_UV(GltfTextureInfo texture_info, GltfTexCoords tex_coords) {
  vec2 uv = EE_GLTF_SELECT_TEX_COORD(texture_info.tex_coord, tex_coords);
#if EE_GLTF_USE_TEXTURE_TRANSFORM
  uv = texture_info.uv_transform * vec3(uv, 1.0);
#endif
  return uv;
}

vec3 EE_GLTF_SRGB_TO_LINEAR(vec3 encoded) {
  const vec3 low = encoded / 12.92;
  const vec3 high = pow(max((encoded + 0.055) / 1.055, vec3(0.0)), vec3(2.4));
  return mix(high, low, lessThanEqual(encoded, vec3(0.04045)));
}

vec4 EE_GLTF_DECODE_TEXTURE_SAMPLE(const GltfTextureInfo texture_info, vec4 sample_value) {
  if (texture_info.color_space == EE_GLTF_TEXTURE_COLOR_SPACE_SRGB) {
    sample_value.rgb = EE_GLTF_SRGB_TO_LINEAR(sample_value.rgb);
  }
  return sample_value;
}

#ifndef EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES
vec4 EE_GLTF_SAMPLE_BINDLESS_TEXTURE(
    GltfTextureInfo texture_info, vec2 uv, vec2 ddx_uv, vec2 ddy_uv, bool use_grad) {
  if (use_grad) {
    return textureGrad(EE_TEXTURE_2DS[nonuniformEXT(texture_info.index)], uv, ddx_uv, ddy_uv);
  }
#ifdef EE_GLTF_USE_EXPLICIT_TEXTURE_LOD
  return textureLod(EE_TEXTURE_2DS[nonuniformEXT(texture_info.index)], uv, EE_GLTF_TEXTURE_LOD);
#else
  return texture(EE_TEXTURE_2DS[nonuniformEXT(texture_info.index)], uv);
#endif
}

vec4 EE_GLTF_SAMPLE_BINDLESS_TEXTURE_LOD0(GltfTextureInfo texture_info, vec2 uv) {
  return textureLod(EE_TEXTURE_2DS[nonuniformEXT(texture_info.index)], uv, 0.0);
}
#endif

#ifdef EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES
vec4 EE_GLTF_SAMPLE_FIXED_RASTER_TEXTURE(int texture_slot, vec2 uv, vec2 ddx_uv, vec2 ddy_uv, bool use_grad,
                                         vec4 fallback) {
  switch (texture_slot) {
    case EE_GLTF_RASTER_TEXTURE_BASE_COLOR:
      if (use_grad)
        return textureGrad(EE_GLTF_RASTER_BASE_COLOR_TEXTURE, uv, ddx_uv, ddy_uv);
#ifdef EE_GLTF_USE_EXPLICIT_TEXTURE_LOD
      return textureLod(EE_GLTF_RASTER_BASE_COLOR_TEXTURE, uv, EE_GLTF_TEXTURE_LOD);
#else
      return texture(EE_GLTF_RASTER_BASE_COLOR_TEXTURE, uv);
#endif
    case EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS:
      if (use_grad)
        return textureGrad(EE_GLTF_RASTER_METALLIC_ROUGHNESS_TEXTURE, uv, ddx_uv, ddy_uv);
#ifdef EE_GLTF_USE_EXPLICIT_TEXTURE_LOD
      return textureLod(EE_GLTF_RASTER_METALLIC_ROUGHNESS_TEXTURE, uv, EE_GLTF_TEXTURE_LOD);
#else
      return texture(EE_GLTF_RASTER_METALLIC_ROUGHNESS_TEXTURE, uv);
#endif
    case EE_GLTF_RASTER_TEXTURE_NORMAL:
      if (use_grad)
        return textureGrad(EE_GLTF_RASTER_NORMAL_TEXTURE, uv, ddx_uv, ddy_uv);
#ifdef EE_GLTF_USE_EXPLICIT_TEXTURE_LOD
      return textureLod(EE_GLTF_RASTER_NORMAL_TEXTURE, uv, EE_GLTF_TEXTURE_LOD);
#else
      return texture(EE_GLTF_RASTER_NORMAL_TEXTURE, uv);
#endif
    case EE_GLTF_RASTER_TEXTURE_EMISSIVE:
      if (use_grad)
        return textureGrad(EE_GLTF_RASTER_EMISSIVE_TEXTURE, uv, ddx_uv, ddy_uv);
#ifdef EE_GLTF_USE_EXPLICIT_TEXTURE_LOD
      return textureLod(EE_GLTF_RASTER_EMISSIVE_TEXTURE, uv, EE_GLTF_TEXTURE_LOD);
#else
      return texture(EE_GLTF_RASTER_EMISSIVE_TEXTURE, uv);
#endif
    case EE_GLTF_RASTER_TEXTURE_OCCLUSION:
      if (use_grad)
        return textureGrad(EE_GLTF_RASTER_OCCLUSION_TEXTURE, uv, ddx_uv, ddy_uv);
#ifdef EE_GLTF_USE_EXPLICIT_TEXTURE_LOD
      return textureLod(EE_GLTF_RASTER_OCCLUSION_TEXTURE, uv, EE_GLTF_TEXTURE_LOD);
#else
      return texture(EE_GLTF_RASTER_OCCLUSION_TEXTURE, uv);
#endif
    case EE_GLTF_RASTER_TEXTURE_CLEARCOAT:
      if (use_grad)
        return textureGrad(EE_GLTF_RASTER_CLEARCOAT_TEXTURE, uv, ddx_uv, ddy_uv);
#ifdef EE_GLTF_USE_EXPLICIT_TEXTURE_LOD
      return textureLod(EE_GLTF_RASTER_CLEARCOAT_TEXTURE, uv, EE_GLTF_TEXTURE_LOD);
#else
      return texture(EE_GLTF_RASTER_CLEARCOAT_TEXTURE, uv);
#endif
    case EE_GLTF_RASTER_TEXTURE_CLEARCOAT_ROUGHNESS:
      if (use_grad)
        return textureGrad(EE_GLTF_RASTER_CLEARCOAT_ROUGHNESS_TEXTURE, uv, ddx_uv, ddy_uv);
#ifdef EE_GLTF_USE_EXPLICIT_TEXTURE_LOD
      return textureLod(EE_GLTF_RASTER_CLEARCOAT_ROUGHNESS_TEXTURE, uv, EE_GLTF_TEXTURE_LOD);
#else
      return texture(EE_GLTF_RASTER_CLEARCOAT_ROUGHNESS_TEXTURE, uv);
#endif
    case EE_GLTF_RASTER_TEXTURE_CLEARCOAT_NORMAL:
      if (use_grad)
        return textureGrad(EE_GLTF_RASTER_CLEARCOAT_NORMAL_TEXTURE, uv, ddx_uv, ddy_uv);
#ifdef EE_GLTF_USE_EXPLICIT_TEXTURE_LOD
      return textureLod(EE_GLTF_RASTER_CLEARCOAT_NORMAL_TEXTURE, uv, EE_GLTF_TEXTURE_LOD);
#else
      return texture(EE_GLTF_RASTER_CLEARCOAT_NORMAL_TEXTURE, uv);
#endif
  }
  return fallback;
}

vec4 EE_GLTF_SAMPLE_FIXED_RASTER_TEXTURE_LOD0(int texture_slot, vec2 uv, vec4 fallback) {
  switch (texture_slot) {
    case EE_GLTF_RASTER_TEXTURE_BASE_COLOR:
      return textureLod(EE_GLTF_RASTER_BASE_COLOR_TEXTURE, uv, 0.0);
    case EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS:
      return textureLod(EE_GLTF_RASTER_METALLIC_ROUGHNESS_TEXTURE, uv, 0.0);
    case EE_GLTF_RASTER_TEXTURE_NORMAL:
      return textureLod(EE_GLTF_RASTER_NORMAL_TEXTURE, uv, 0.0);
    case EE_GLTF_RASTER_TEXTURE_EMISSIVE:
      return textureLod(EE_GLTF_RASTER_EMISSIVE_TEXTURE, uv, 0.0);
    case EE_GLTF_RASTER_TEXTURE_OCCLUSION:
      return textureLod(EE_GLTF_RASTER_OCCLUSION_TEXTURE, uv, 0.0);
    case EE_GLTF_RASTER_TEXTURE_CLEARCOAT:
      return textureLod(EE_GLTF_RASTER_CLEARCOAT_TEXTURE, uv, 0.0);
    case EE_GLTF_RASTER_TEXTURE_CLEARCOAT_ROUGHNESS:
      return textureLod(EE_GLTF_RASTER_CLEARCOAT_ROUGHNESS_TEXTURE, uv, 0.0);
    case EE_GLTF_RASTER_TEXTURE_CLEARCOAT_NORMAL:
      return textureLod(EE_GLTF_RASTER_CLEARCOAT_NORMAL_TEXTURE, uv, 0.0);
  }
  return fallback;
}
#endif

vec4 EE_GLTF_SAMPLE_TEXTURE_SLOT(
    uint16_t texture_info_slot, int texture_slot, GltfTexCoords tex_coords, vec4 fallback) {
  if (!EE_GLTF_HAS_TEXTURE(texture_info_slot)) {
    return fallback;
  }
  GltfTextureInfo texture_info = EE_GLTF_TEXTURE_INFOS[uint(texture_info_slot)];
  if (texture_info.index < 0) {
    return fallback;
  }

  vec2 uv = EE_GLTF_TEXTURE_UV(texture_info, tex_coords);
  const float tex_grad = tex_coords.gradients[clamp(texture_info.tex_coord, 0, 3)];
  vec4 sample_value;
  if (tex_grad > 0.0) {
#if EE_GLTF_USE_TEXTURE_TRANSFORM
    vec2 ddx_uv = texture_info.uv_transform * vec3(tex_grad, 0.0, 0.0);
    vec2 ddy_uv = texture_info.uv_transform * vec3(0.0, tex_grad, 0.0);
#else
    vec2 ddx_uv = vec2(tex_grad, 0.0);
    vec2 ddy_uv = vec2(0.0, tex_grad);
#endif
#ifdef EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES
    sample_value = EE_GLTF_SAMPLE_FIXED_RASTER_TEXTURE(texture_slot, uv, ddx_uv, ddy_uv, true, fallback);
#else
    sample_value = EE_GLTF_SAMPLE_BINDLESS_TEXTURE(texture_info, uv, ddx_uv, ddy_uv, true);
#endif
  } else {
    const vec2 ddx_uv = vec2(0.0);
    const vec2 ddy_uv = vec2(0.0);
#ifdef EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES
    sample_value = EE_GLTF_SAMPLE_FIXED_RASTER_TEXTURE(texture_slot, uv, ddx_uv, ddy_uv, false, fallback);
#else
    sample_value = EE_GLTF_SAMPLE_BINDLESS_TEXTURE(texture_info, uv, ddx_uv, ddy_uv, false);
#endif
  }
  return EE_GLTF_DECODE_TEXTURE_SAMPLE(texture_info, sample_value);
}

vec4 EE_GLTF_SAMPLE_TEXTURE(
    uint16_t texture_info_slot, GltfTexCoords tex_coords, vec4 fallback) {
  return EE_GLTF_SAMPLE_TEXTURE_SLOT(
      texture_info_slot, EE_GLTF_RASTER_TEXTURE_UNSUPPORTED_EXTENSION, tex_coords, fallback);
}

vec4 EE_GLTF_SAMPLE_TEXTURE_SLOT(
    uint16_t texture_info_slot, int texture_slot, vec2 tex_coord_0, vec2 tex_coord_1, vec4 fallback,
    vec2 tex_gradients) {
  return EE_GLTF_SAMPLE_TEXTURE_SLOT(
      texture_info_slot, texture_slot,
      EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, vec2(0.0), vec2(0.0), vec4(tex_gradients, 0.0, 0.0)),
      fallback);
}

vec4 EE_GLTF_SAMPLE_TEXTURE(
    uint16_t texture_info_slot, vec2 tex_coord_0, vec2 tex_coord_1, vec4 fallback, vec2 tex_gradients) {
  return EE_GLTF_SAMPLE_TEXTURE_SLOT(texture_info_slot, EE_GLTF_RASTER_TEXTURE_UNSUPPORTED_EXTENSION, tex_coord_0,
                                     tex_coord_1, fallback, tex_gradients);
}

vec4 EE_GLTF_SAMPLE_TEXTURE_SLOT(
    uint16_t texture_info_slot, int texture_slot, vec2 tex_coord_0, vec2 tex_coord_1, vec4 fallback,
    float tex_grad) {
  return EE_GLTF_SAMPLE_TEXTURE_SLOT(texture_info_slot, texture_slot, tex_coord_0, tex_coord_1, fallback,
                                     vec2(tex_grad));
}

vec4 EE_GLTF_SAMPLE_TEXTURE(
    uint16_t texture_info_slot, vec2 tex_coord_0, vec2 tex_coord_1, vec4 fallback, float tex_grad) {
  return EE_GLTF_SAMPLE_TEXTURE(texture_info_slot, tex_coord_0, tex_coord_1, fallback, vec2(tex_grad));
}

vec4 EE_GLTF_SAMPLE_TEXTURE(uint16_t texture_info_slot, vec2 tex_coord_0, vec2 tex_coord_1, vec4 fallback) {
  return EE_GLTF_SAMPLE_TEXTURE(texture_info_slot, tex_coord_0, tex_coord_1, fallback, 0.0);
}

vec4 EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(
    uint16_t texture_info_slot, int texture_slot, GltfTexCoords tex_coords, vec4 fallback) {
  if (!EE_GLTF_HAS_TEXTURE(texture_info_slot)) {
    return fallback;
  }
  GltfTextureInfo texture_info = EE_GLTF_TEXTURE_INFOS[uint(texture_info_slot)];
  if (texture_info.index < 0) {
    return fallback;
  }

  const vec2 uv = EE_GLTF_TEXTURE_UV(texture_info, tex_coords);
#ifdef EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES
  const vec4 sample_value = EE_GLTF_SAMPLE_FIXED_RASTER_TEXTURE_LOD0(texture_slot, uv, fallback);
#else
  const vec4 sample_value = EE_GLTF_SAMPLE_BINDLESS_TEXTURE_LOD0(texture_info, uv);
#endif
  return EE_GLTF_DECODE_TEXTURE_SAMPLE(texture_info, sample_value);
}

vec4 EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(
    uint16_t texture_info_slot, int texture_slot, vec2 tex_coord_0, vec2 tex_coord_1, vec4 fallback) {
  return EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(
      texture_info_slot, texture_slot, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1), fallback);
}

vec4 EE_GLTF_SAMPLE_TEXTURE_LOD0(uint16_t texture_info_slot, GltfTexCoords tex_coords, vec4 fallback) {
  return EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(
      texture_info_slot, EE_GLTF_RASTER_TEXTURE_UNSUPPORTED_EXTENSION, tex_coords, fallback);
}

vec4 EE_GLTF_SAMPLE_TEXTURE_LOD0(uint16_t texture_info_slot, vec2 tex_coord_0, vec2 tex_coord_1, vec4 fallback) {
  return EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(texture_info_slot, EE_GLTF_RASTER_TEXTURE_UNSUPPORTED_EXTENSION, tex_coord_0,
                                          tex_coord_1, fallback);
}

#if EE_GLTF_USE_VOLUME_SCATTER
vec3 EE_GLTF_MULTI_TO_SINGLE_SCATTER_ALBEDO(vec3 rho_ms) {
  vec3 t = 4.09712 + 4.20863 * rho_ms -
           sqrt(9.59217 + 41.6808 * rho_ms + 17.7126 * rho_ms * rho_ms);
  return 1.0 - t * t;
}
#endif

GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(
    uint material_index, GltfTexCoords tex_coords, vec4 vertex_color) {
  GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  GltfRasterMaterial surface;
  vertex_color = clamp(vertex_color, vec4(0.0), vec4(1.0));
  surface.base_color = material.pbr_base_color_factor * vertex_color;
  surface.specular_f0 = vec3(0.04);
  surface.specular_f90 = 1.0;
  surface.emissive = material.emissive_factor;
  surface.clearcoat = 0.0;
  surface.clearcoat_roughness = EE_GLTF_MICROFACET_MIN_ROUGHNESS;
  surface.metallic = material.pbr_metallic_factor;
  surface.roughness = material.pbr_roughness_factor;
  surface.occlusion = material.occlusion_strength;
  surface.alpha_mode = material.alpha_mode;
  surface.alpha_cutoff = material.alpha_cutoff;
  surface.transmission = 0.0;
  surface.attenuation_color = vec3(1.0);
  surface.attenuation_distance = 1.0;
  surface.thickness = 0.0;
  surface.diffuse_transmission_color = vec3(1.0);
  surface.diffuse_transmission_factor = 0.0;
  surface.multiscatter_color_factor = vec3(0.0);
  surface.scatter_coefficient = vec3(0.0);
  surface.scatter_anisotropy = 0.0;

#if EE_GLTF_USE_SPECULAR_GLOSSINESS
  if (material.pbr_model == EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS) {
    vec4 diffuse = material.pbr_diffuse_factor * vertex_color;
    vec3 specular = material.pbr_specular_factor;
    float glossiness = material.pbr_glossiness_factor;

    diffuse *= EE_GLTF_SAMPLE_TEXTURE_SLOT(
        material.pbr_diffuse_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR, tex_coords, vec4(1.0));
    vec4 specular_glossiness = EE_GLTF_SAMPLE_TEXTURE_SLOT(
        material.pbr_specular_glossiness_texture, EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS, tex_coords,
        vec4(1.0));
    specular *= specular_glossiness.rgb;
    glossiness *= specular_glossiness.a;

    surface.specular_f0 = clamp(specular, vec3(0.0), vec3(1.0));
    surface.base_color.rgb = diffuse.rgb * (1.0 - max(surface.specular_f0.r,
                                                       max(surface.specular_f0.g, surface.specular_f0.b)));
    surface.base_color.a = diffuse.a;
    surface.metallic = 0.0;
    surface.roughness = max(1.0 - glossiness, EE_GLTF_MICROFACET_MIN_ROUGHNESS);
  } else
#endif
  {
    surface.base_color *= EE_GLTF_SAMPLE_TEXTURE_SLOT(
        material.pbr_base_color_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR, tex_coords, vec4(1.0));
    vec4 metallic_roughness = EE_GLTF_SAMPLE_TEXTURE_SLOT(
        material.pbr_metallic_roughness_texture, EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS, tex_coords,
        vec4(1.0));
    surface.roughness *= metallic_roughness.g;
    surface.metallic *= metallic_roughness.b;
    surface.roughness = max(surface.roughness, EE_GLTF_MICROFACET_MIN_ROUGHNESS);
    surface.metallic = clamp(surface.metallic, 0.0, 1.0);
    float dielectric_f0 = 0.04;
    float dielectric_specular_f90 = 1.0;
#if EE_GLTF_USE_IOR
    const float material_ior = material.ior == 0.0 ? 0.0 : max(material.ior, 1.0);
    dielectric_f0 = pow((material_ior - 1.0) / max(material_ior + 1.0, 0.000001), 2.0);
#endif
    vec3 dielectric_specular_f0 = vec3(dielectric_f0);
#if EE_GLTF_USE_SPECULAR
    float specular_weight = material.specular_factor;
    specular_weight *= EE_GLTF_SAMPLE_TEXTURE(material.specular_texture, tex_coords, vec4(1.0)).a;
    vec3 specular_color = material.specular_color_factor;
    specular_color *= EE_GLTF_SAMPLE_TEXTURE(material.specular_color_texture, tex_coords, vec4(1.0)).rgb;
    dielectric_specular_f0 =
        clamp(dielectric_specular_f0 * max(specular_color, vec3(0.0)), vec3(0.0), vec3(1.0)) *
        clamp(specular_weight, 0.0, 1.0);
    dielectric_specular_f90 = clamp(specular_weight, 0.0, 1.0);
#endif
    surface.specular_f0 = mix(dielectric_specular_f0, max(surface.base_color.rgb, vec3(0.0)), surface.metallic);
    surface.specular_f90 = mix(dielectric_specular_f90, 1.0, surface.metallic);
  }

  if (EE_GLTF_HAS_TEXTURE(material.occlusion_texture)) {
    float occlusion = EE_GLTF_SAMPLE_TEXTURE_SLOT(
        material.occlusion_texture, EE_GLTF_RASTER_TEXTURE_OCCLUSION, tex_coords, vec4(1.0)).r;
    surface.occlusion = 1.0 + surface.occlusion * (occlusion - 1.0);
  }

#if EE_GLTF_USE_CLEARCOAT
  surface.clearcoat = clamp(material.clearcoat_factor, 0.0, 1.0);
  surface.clearcoat *= EE_GLTF_SAMPLE_TEXTURE_SLOT(
      material.clearcoat_texture, EE_GLTF_RASTER_TEXTURE_CLEARCOAT, tex_coords, vec4(1.0)).r;
  surface.clearcoat_roughness = max(material.clearcoat_roughness, EE_GLTF_MICROFACET_MIN_ROUGHNESS);
  surface.clearcoat_roughness *= EE_GLTF_SAMPLE_TEXTURE_SLOT(
      material.clearcoat_roughness_texture, EE_GLTF_RASTER_TEXTURE_CLEARCOAT_ROUGHNESS, tex_coords,
      vec4(1.0)).g;
#endif

#if EE_GLTF_USE_TRANSMISSION
  surface.transmission = material.transmission_factor;
  if (EE_GLTF_HAS_TEXTURE(material.transmission_texture)) {
    surface.transmission *= EE_GLTF_SAMPLE_TEXTURE(material.transmission_texture, tex_coords, vec4(1.0)).r;
  }
#endif

#if EE_GLTF_USE_VOLUME
  surface.attenuation_color = material.attenuation_color;
  surface.attenuation_distance = material.attenuation_distance;
  surface.thickness = material.thickness_factor;
  if (EE_GLTF_HAS_TEXTURE(material.thickness_texture)) {
    surface.thickness *= EE_GLTF_SAMPLE_TEXTURE(material.thickness_texture, tex_coords, vec4(1.0)).g;
  }
#endif

#if EE_GLTF_USE_DIFFUSE_TRANSMISSION
  surface.diffuse_transmission_factor = material.diffuse_transmission_factor;
  if (EE_GLTF_HAS_TEXTURE(material.diffuse_transmission_texture)) {
    surface.diffuse_transmission_factor *=
        EE_GLTF_SAMPLE_TEXTURE(material.diffuse_transmission_texture, tex_coords, vec4(1.0)).a;
  }
  surface.diffuse_transmission_color = material.diffuse_transmission_color;
  if (EE_GLTF_HAS_TEXTURE(material.diffuse_transmission_color_texture)) {
    surface.diffuse_transmission_color *=
        EE_GLTF_SAMPLE_TEXTURE(material.diffuse_transmission_color_texture, tex_coords, vec4(1.0)).rgb;
  }
#endif

#if EE_GLTF_USE_VOLUME_SCATTER
  surface.multiscatter_color_factor = max(material.multiscatter_color_factor, vec3(0.0));
  surface.scatter_anisotropy = clamp(material.scatter_anisotropy, -0.999, 0.999);
  if (any(greaterThan(surface.multiscatter_color_factor, vec3(0.0)))) {
    vec3 single_scatter_albedo = EE_GLTF_MULTI_TO_SINGLE_SCATTER_ALBEDO(surface.multiscatter_color_factor);
    vec3 attenuation_coefficient =
        -log(max(surface.attenuation_color, vec3(0.001))) / max(surface.attenuation_distance, 0.001);
    surface.scatter_coefficient = attenuation_coefficient * single_scatter_albedo;
  }
#endif

  surface.emissive *= EE_GLTF_SAMPLE_TEXTURE_SLOT(
      material.emissive_texture, EE_GLTF_RASTER_TEXTURE_EMISSIVE, tex_coords, vec4(1.0)).rgb;
  surface.emissive = max(vec3(0.0), surface.emissive);
  return surface;
}

GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec2 tex_coord_2, vec2 tex_coord_3,
    vec4 vertex_color, vec4 tex_gradients) {
  return EE_EVALUATE_GLTF_RASTER_SURFACE(
      material_index,
      EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, tex_gradients), vertex_color);
}

GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec2 tex_coord_2, vec2 tex_coord_3,
    vec4 vertex_color) {
  return EE_EVALUATE_GLTF_RASTER_SURFACE(
      material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color, vec4(0.0));
}

GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec4 vertex_color, vec2 tex_gradients) {
  return EE_EVALUATE_GLTF_RASTER_SURFACE(
      material_index,
      EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, vec2(0.0), vec2(0.0), vec4(tex_gradients, 0.0, 0.0)),
      vertex_color);
}

GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec4 vertex_color, float tex_grad) {
  return EE_EVALUATE_GLTF_RASTER_SURFACE(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, vec2(0.0), vec2(0.0), vec4(tex_grad)),
      vertex_color);
}

GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec2 tex_gradients) {
  return EE_EVALUATE_GLTF_RASTER_SURFACE(
      material_index,
      EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, vec2(0.0), vec2(0.0), vec4(tex_gradients, 0.0, 0.0)),
      vec4(1.0));
}

GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, float tex_grad) {
  return EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, tex_coord_0, tex_coord_1, vec4(1.0), vec2(tex_grad));
}

GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec4 vertex_color) {
  return EE_EVALUATE_GLTF_RASTER_SURFACE(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1), vertex_color);
}

GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(uint material_index, vec2 tex_coord_0, vec2 tex_coord_1) {
  return EE_EVALUATE_GLTF_RASTER_SURFACE(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1), vec4(1.0));
}

vec3 EE_GLTF_RASTER_REBASE_SPECULAR_F0(uint material_index, GltfRasterMaterial surface, vec3 base_color) {
#if EE_GLTF_USE_SPECULAR_GLOSSINESS
  GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  if (material.pbr_model == EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS) {
    return surface.specular_f0;
  }
#endif
  const float metallic = clamp(surface.metallic, 0.0, 1.0);
  const vec3 rebased_base_color = max(base_color, vec3(0.0));
  if (metallic >= 0.999999) {
    return rebased_base_color;
  }
  const vec3 original_base_color = max(surface.base_color.rgb, vec3(0.0));
  const vec3 dielectric_specular_f0 =
      clamp((surface.specular_f0 - original_base_color * metallic) / max(1.0 - metallic, 0.000001), vec3(0.0),
            vec3(1.0));
  return mix(dielectric_specular_f0, rebased_base_color, metallic);
}

vec3 EE_GLTF_SAFE_NORMALIZE(vec3 value, vec3 fallback) {
  float length_squared = dot(value, value);
  return length_squared > 1e-12 ? value * inversesqrt(length_squared) : fallback;
}

vec3 EE_GLTF_FALLBACK_TANGENT(vec3 normal) {
  vec3 axis = abs(normal.z) < 0.999 ? vec3(0.0, 0.0, 1.0) : vec3(0.0, 1.0, 0.0);
  return EE_GLTF_SAFE_NORMALIZE(cross(axis, normal), vec3(1.0, 0.0, 0.0));
}

vec3 EE_EVALUATE_GLTF_RASTER_NORMAL(uint material_index, GltfTexCoords tex_coords, vec3 normal,
                                    vec3 tangent, float tangent_handedness) {
  GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  vec3 n = EE_GLTF_SAFE_NORMALIZE(normal, vec3(0.0, 1.0, 0.0));
  if (!EE_GLTF_HAS_TEXTURE(material.normal_texture)) {
    return n;
  }

  vec3 normal_vector = EE_GLTF_SAMPLE_TEXTURE_SLOT(
      material.normal_texture, EE_GLTF_RASTER_TEXTURE_NORMAL, tex_coords, vec4(0.5, 0.5, 1.0, 1.0)).xyz;
  normal_vector = normal_vector * 2.0 - 1.0;
  normal_vector.xy *= material.normal_texture_scale;

  vec3 t = tangent - n * dot(n, tangent);
  t = EE_GLTF_SAFE_NORMALIZE(t, EE_GLTF_FALLBACK_TANGENT(n));
  float bitangent_sign = tangent_handedness < 0.0 ? -1.0 : 1.0;
  vec3 b = cross(n, t) * bitangent_sign;
  return EE_GLTF_SAFE_NORMALIZE(mat3(t, b, n) * normal_vector, n);
}

vec3 EE_EVALUATE_GLTF_RASTER_NORMAL(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec2 tex_coord_2, vec2 tex_coord_3,
    vec3 normal, vec3 tangent, float tangent_handedness, vec4 tex_gradients) {
  return EE_EVALUATE_GLTF_RASTER_NORMAL(
      material_index,
      EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, tex_gradients),
      normal, tangent, tangent_handedness);
}

vec3 EE_EVALUATE_GLTF_RASTER_NORMAL(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec2 tex_coord_2, vec2 tex_coord_3,
    vec3 normal, vec3 tangent, float tangent_handedness) {
  return EE_EVALUATE_GLTF_RASTER_NORMAL(
      material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, normal, tangent,
      tangent_handedness, vec4(0.0));
}

vec3 EE_EVALUATE_GLTF_RASTER_NORMAL(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec3 normal, vec3 tangent,
    float tangent_handedness, vec2 tex_gradients) {
  return EE_EVALUATE_GLTF_RASTER_NORMAL(
      material_index,
      EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, vec2(0.0), vec2(0.0), vec4(tex_gradients, 0.0, 0.0)),
      normal, tangent, tangent_handedness);
}

vec3 EE_EVALUATE_GLTF_RASTER_NORMAL(uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec3 normal,
                                    vec3 tangent, float tangent_handedness, float tex_grad) {
  return EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coord_0, tex_coord_1, normal, tangent,
                                         tangent_handedness, vec2(tex_grad));
}

vec3 EE_EVALUATE_GLTF_RASTER_NORMAL(uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec3 normal,
                                    vec3 tangent, float tangent_handedness) {
  return EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coord_0, tex_coord_1, normal, tangent,
                                        tangent_handedness, vec2(0.0));
}

vec3 EE_EVALUATE_GLTF_RASTER_NORMAL(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec3 normal, vec3 tangent) {
  return EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coord_0, tex_coord_1, normal, tangent, 1.0);
}

vec3 EE_EVALUATE_GLTF_RASTER_CLEARCOAT_NORMAL(uint material_index, GltfTexCoords tex_coords,
                                               vec3 normal, vec3 tangent, float tangent_handedness);

vec3 EE_EVALUATE_GLTF_RASTER_CLEARCOAT_NORMAL(uint material_index, vec2 tex_coord_0, vec2 tex_coord_1,
                                               vec3 normal, vec3 tangent, float tangent_handedness,
                                               vec2 tex_gradients) {
  return EE_EVALUATE_GLTF_RASTER_CLEARCOAT_NORMAL(
      material_index,
      EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, vec2(0.0), vec2(0.0), vec4(tex_gradients, 0.0, 0.0)),
      normal, tangent, tangent_handedness);
}

vec3 EE_EVALUATE_GLTF_RASTER_CLEARCOAT_NORMAL(uint material_index, GltfTexCoords tex_coords,
                                               vec3 normal, vec3 tangent, float tangent_handedness) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  const vec3 n = EE_GLTF_SAFE_NORMALIZE(normal, vec3(0.0, 1.0, 0.0));
#if EE_GLTF_USE_CLEARCOAT
  if (EE_GLTF_HAS_TEXTURE(material.clearcoat_normal_texture)) {
    vec3 normal_vector = EE_GLTF_SAMPLE_TEXTURE_SLOT(
        material.clearcoat_normal_texture, EE_GLTF_RASTER_TEXTURE_CLEARCOAT_NORMAL, tex_coords,
        vec4(0.5, 0.5, 1.0, 1.0)).xyz;
    normal_vector = normal_vector * 2.0 - 1.0;
    normal_vector.xy *= material.clearcoat_normal_texture_scale;
    vec3 t = tangent - n * dot(n, tangent);
    t = EE_GLTF_SAFE_NORMALIZE(t, EE_GLTF_FALLBACK_TANGENT(n));
    const vec3 b = cross(n, t) * (tangent_handedness < 0.0 ? -1.0 : 1.0);
    return EE_GLTF_SAFE_NORMALIZE(mat3(t, b, n) * normal_vector, n);
  }
#endif
  return n;
}

vec3 EE_GLTF_RASTER_COATED_EMISSION(GltfRasterMaterial surface, vec3 clearcoat_normal,
                                     vec3 outgoing_direction) {
  const float cosine = clamp(abs(dot(EE_GLTF_SAFE_NORMALIZE(outgoing_direction, clearcoat_normal), clearcoat_normal)),
                             0.0, 1.0);
  const float fresnel = 0.04 + 0.96 * pow(max(1.0 - cosine, 0.0), 5.0);
  return surface.emissive * max(1.0 - clamp(surface.clearcoat, 0.0, 1.0) * fresnel, 0.0);
}

vec3 EE_GLTF_RASTER_COATED_EMISSION(uint material_index, GltfRasterMaterial surface, vec2 tex_coord_0,
                                     vec2 tex_coord_1, vec3 normal, vec3 tangent, float tangent_handedness,
                                     float facing_sign, vec3 outgoing_direction) {
  vec3 clearcoat_normal = EE_EVALUATE_GLTF_RASTER_CLEARCOAT_NORMAL(
      material_index, tex_coord_0, tex_coord_1, normal, tangent, tangent_handedness, vec2(0.0));
  clearcoat_normal = EE_GLTF_SAFE_NORMALIZE(facing_sign * clearcoat_normal, vec3(0.0, 1.0, 0.0));
  return EE_GLTF_RASTER_COATED_EMISSION(surface, clearcoat_normal, outgoing_direction);
}

vec3 EE_GLTF_RASTER_COATED_EMISSION(
    uint material_index, GltfRasterMaterial surface, vec2 tex_coord_0, vec2 tex_coord_1,
    vec2 tex_coord_2, vec2 tex_coord_3, vec3 normal, vec3 tangent, float tangent_handedness,
    float facing_sign, vec3 outgoing_direction) {
  vec3 clearcoat_normal = EE_EVALUATE_GLTF_RASTER_CLEARCOAT_NORMAL(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vec4(0.0)),
      normal, tangent, tangent_handedness);
  clearcoat_normal = EE_GLTF_SAFE_NORMALIZE(facing_sign * clearcoat_normal, vec3(0.0, 1.0, 0.0));
  return EE_GLTF_RASTER_COATED_EMISSION(surface, clearcoat_normal, outgoing_direction);
}

float EE_GLTF_RASTER_OPACITY(GltfRasterMaterial surface) {
  const float alpha = surface.alpha_mode == EE_GLTF_ALPHA_MODE_OPAQUE
                          ? 1.0
                          : clamp(surface.base_color.a, 0.0, 1.0);
  const float transmission = clamp(max(surface.transmission, surface.diffuse_transmission_factor), 0.0, 1.0);
  return alpha * (1.0 - transmission);
}

bool EE_GLTF_RASTER_SHOULD_DISCARD(GltfRasterMaterial surface) {
  if (surface.alpha_mode == EE_GLTF_ALPHA_MODE_OPAQUE) {
    return false;
  }
  if (surface.alpha_mode == EE_GLTF_ALPHA_MODE_MASK) {
    return surface.base_color.a < surface.alpha_cutoff;
  }
  return EE_GLTF_RASTER_OPACITY(surface) <= 0.0;
}

vec3 EE_GLTF_RASTER_FRESNEL(const vec3 f0, const vec3 f90, const float cos_theta) {
  return f0 + (f90 - f0) * pow(max(1.0 - clamp(cos_theta, 0.0, 1.0), 0.0), 5.0);
}

float EE_GLTF_RASTER_BASE_COLOR_ALPHA(uint material_index, GltfTexCoords tex_coords, float vertex_alpha) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  float base_color_alpha;
#if EE_GLTF_USE_SPECULAR_GLOSSINESS
  if (material.pbr_model == EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS) {
    base_color_alpha = material.pbr_diffuse_factor.a;
    base_color_alpha *= EE_GLTF_SAMPLE_TEXTURE_SLOT(
        material.pbr_diffuse_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR, tex_coords, vec4(1.0)).a;
  } else
#endif
  {
    base_color_alpha = material.pbr_base_color_factor.a;
    base_color_alpha *= EE_GLTF_SAMPLE_TEXTURE_SLOT(
        material.pbr_base_color_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR, tex_coords, vec4(1.0)).a;
  }
  base_color_alpha *= vertex_alpha;
  return base_color_alpha;
}

bool EE_GLTF_RASTER_ALPHA_MASK_PASSES(uint material_index, GltfTexCoords tex_coords, float vertex_alpha) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  return material.alpha_mode != EE_GLTF_ALPHA_MODE_MASK ||
         EE_GLTF_RASTER_BASE_COLOR_ALPHA(material_index, tex_coords, vertex_alpha) >= material.alpha_cutoff;
}

float EE_GLTF_RASTER_BASE_COLOR_ALPHA_LOD0(uint material_index, GltfTexCoords tex_coords, float vertex_alpha) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  float base_color_alpha;
#if EE_GLTF_USE_SPECULAR_GLOSSINESS
  if (material.pbr_model == EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS) {
    base_color_alpha = material.pbr_diffuse_factor.a;
    base_color_alpha *= EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(
        material.pbr_diffuse_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR, tex_coords, vec4(1.0)).a;
  } else
#endif
  {
    base_color_alpha = material.pbr_base_color_factor.a;
    base_color_alpha *= EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(
        material.pbr_base_color_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR, tex_coords, vec4(1.0)).a;
  }
  base_color_alpha *= vertex_alpha;
  return base_color_alpha;
}

bool EE_GLTF_RASTER_ALPHA_MASK_PASSES_LOD0(uint material_index, GltfTexCoords tex_coords, float vertex_alpha) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  return material.alpha_mode != EE_GLTF_ALPHA_MODE_MASK ||
         EE_GLTF_RASTER_BASE_COLOR_ALPHA_LOD0(material_index, tex_coords, vertex_alpha) >= material.alpha_cutoff;
}

float EE_GLTF_RASTER_OPACITY_LOD0(uint material_index, GltfTexCoords tex_coords, float vertex_alpha) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  if (material.alpha_mode == EE_GLTF_ALPHA_MODE_OPAQUE) {
    return 1.0;
  }
  const float base_color_alpha = EE_GLTF_RASTER_BASE_COLOR_ALPHA_LOD0(material_index, tex_coords, vertex_alpha);
  if (material.alpha_mode == EE_GLTF_ALPHA_MODE_MASK) {
    return base_color_alpha >= material.alpha_cutoff ? 1.0 : 0.0;
  }
  return base_color_alpha;
}

float EE_GLTF_RASTER_OPACITY_LOD0(uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, float vertex_alpha) {
  return EE_GLTF_RASTER_OPACITY_LOD0(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1), vertex_alpha);
}

float EE_GLTF_RASTER_OPACITY_LOD0(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec2 tex_coord_2, vec2 tex_coord_3,
    float vertex_alpha) {
  return EE_GLTF_RASTER_OPACITY_LOD0(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vec4(0.0)),
      vertex_alpha);
}

vec3 EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0(uint material_index, GltfTexCoords tex_coords,
                                              vec3 vertex_color, float cos_theta, float segment_length,
                                              inout bool is_inside, float min_transmission) {
#if !EE_GLTF_USE_TRANSMISSION && !EE_GLTF_USE_DIFFUSE_TRANSMISSION
  return vec3(0.0);
#else
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  vec3 base_color = material.pbr_base_color_factor.rgb * vertex_color;
#if EE_GLTF_USE_SPECULAR_GLOSSINESS
  if (material.pbr_model == EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS) {
    base_color = material.pbr_diffuse_factor.rgb * vertex_color;
    base_color *= EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(
        material.pbr_diffuse_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR, tex_coords, vec4(1.0)).rgb;
  } else
#endif
  {
    base_color *= EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(
        material.pbr_base_color_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR, tex_coords, vec4(1.0)).rgb;
  }

  float specular_transmission = 0.0;
#if EE_GLTF_USE_TRANSMISSION
  specular_transmission = material.transmission_factor;
  specular_transmission *= EE_GLTF_SAMPLE_TEXTURE_LOD0(material.transmission_texture, tex_coords, vec4(1.0)).r;
#endif
  float diffuse_transmission = 0.0;
  vec3 diffuse_transmission_color = vec3(1.0);
#if EE_GLTF_USE_DIFFUSE_TRANSMISSION
  diffuse_transmission = material.diffuse_transmission_factor;
  diffuse_transmission *=
      EE_GLTF_SAMPLE_TEXTURE_LOD0(material.diffuse_transmission_texture, tex_coords, vec4(1.0)).a;
  diffuse_transmission_color = material.diffuse_transmission_color;
  diffuse_transmission_color *=
      EE_GLTF_SAMPLE_TEXTURE_LOD0(material.diffuse_transmission_color_texture, tex_coords, vec4(1.0)).rgb;
#endif
  specular_transmission = clamp(specular_transmission, 0.0, 1.0);
  diffuse_transmission = clamp(diffuse_transmission, 0.0, 1.0);
  const float effective_diffuse_transmission = (1.0 - specular_transmission) * diffuse_transmission;
  if (max(specular_transmission, effective_diffuse_transmission) <= min_transmission) {
    return vec3(0.0);
  }

  float ior = 1.5;
#if EE_GLTF_USE_IOR
  ior = material.ior == 0.0 ? 0.0 : max(material.ior, 1.0);
#endif
  float ior_f0 = (ior - 1.0) / max(ior + 1.0, 0.000001);
  ior_f0 *= ior_f0;
  float specular_weight = 1.0;
  vec3 specular_color = vec3(1.0);
#if EE_GLTF_USE_SPECULAR
  specular_weight = material.specular_factor;
  specular_weight *= EE_GLTF_SAMPLE_TEXTURE_LOD0(material.specular_texture, tex_coords, vec4(1.0)).a;
  specular_color = material.specular_color_factor;
  specular_color *= EE_GLTF_SAMPLE_TEXTURE_LOD0(material.specular_color_texture, tex_coords, vec4(1.0)).rgb;
#endif
  specular_weight = clamp(specular_weight, 0.0, 1.0);
  const vec3 specular_f0 =
      clamp(vec3(ior_f0) * max(specular_color, vec3(0.0)), vec3(0.0), vec3(1.0)) * specular_weight;
  const vec3 fresnel = EE_GLTF_RASTER_FRESNEL(specular_f0, vec3(specular_weight), cos_theta);
  const float remaining_energy = 1.0 - max(fresnel.r, max(fresnel.g, fresnel.b));
  vec3 transmission = remaining_energy *
                      (specular_transmission * base_color +
                       effective_diffuse_transmission * diffuse_transmission_color);
  transmission = clamp(transmission, vec3(0.0), vec3(1.0));

#if EE_GLTF_USE_VOLUME
  if (material.thickness_factor > 0.0) {
    if (is_inside) {
      const vec3 absorption_coefficient =
          -log(max(material.attenuation_color, vec3(0.001))) / max(material.attenuation_distance, 0.001);
      vec3 scatter_coefficient = vec3(0.0);
#if EE_GLTF_USE_VOLUME_SCATTER
      scatter_coefficient = absorption_coefficient * EE_GLTF_MULTI_TO_SINGLE_SCATTER_ALBEDO(
                                                         max(material.multiscatter_color_factor, vec3(0.0)));
#endif
      const vec3 extinction = absorption_coefficient + scatter_coefficient;
      transmission *= exp(-segment_length * extinction);

      const float max_scatter = max(scatter_coefficient.x, max(scatter_coefficient.y, scatter_coefficient.z));
      if (max_scatter > 0.001) {
        const float max_extinction = max(extinction.x, max(extinction.y, extinction.z));
        transmission *= exp(-segment_length * max_extinction);
      }
    }
    is_inside = !is_inside;
  }
#endif

  float roughness = material.pbr_roughness_factor;
  float metallic = material.pbr_metallic_factor;
  const vec4 metallic_roughness = EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(
      material.pbr_metallic_roughness_texture, EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS, tex_coords,
      vec4(1.0));
  roughness *= metallic_roughness.g;
  metallic *= metallic_roughness.b;

  const float roughness_effect = 1.0 - roughness * roughness;
  const float transmission_attenuation = (1.0 - metallic) * mix(0.65, 1.0, roughness_effect);
  return transmission * transmission_attenuation;
#endif
}

vec3 EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec3 vertex_color, float cos_theta,
    float segment_length, inout bool is_inside, float min_transmission) {
  return EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1), vertex_color, cos_theta,
      segment_length, is_inside, min_transmission);
}

vec3 EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec2 tex_coord_2, vec2 tex_coord_3,
    vec3 vertex_color, float cos_theta, float segment_length, inout bool is_inside, float min_transmission) {
  return EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vec4(0.0)),
      vertex_color, cos_theta, segment_length, is_inside, min_transmission);
}

#endif
