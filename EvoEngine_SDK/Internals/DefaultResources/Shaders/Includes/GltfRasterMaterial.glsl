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

const int EE_GLTF_RASTER_TEXTURE_UNSUPPORTED_EXTENSION = -1;
const int EE_GLTF_RASTER_TEXTURE_BASE_COLOR = 0;
const int EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS = 1;
const int EE_GLTF_RASTER_TEXTURE_NORMAL = 2;
const int EE_GLTF_RASTER_TEXTURE_EMISSIVE = 3;
const int EE_GLTF_RASTER_TEXTURE_OCCLUSION = 4;

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
#endif

struct GltfRasterMaterial {
  vec4 base_color;
  vec3 emissive;
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

bool EE_GLTF_HAS_TEXTURE(uint16_t texture_info_slot) {
  return uint(texture_info_slot) > 0u;
}

vec2 EE_GLTF_SELECT_TEX_COORD(int tex_coord, vec2 tex_coord_0, vec2 tex_coord_1) {
  return tex_coord == 1 ? tex_coord_1 : tex_coord_0;
}

vec2 EE_GLTF_TEXTURE_UV(GltfTextureInfo texture_info, vec2 tex_coord_0, vec2 tex_coord_1) {
  vec2 uv = EE_GLTF_SELECT_TEX_COORD(texture_info.tex_coord, tex_coord_0, tex_coord_1);
#if MAT_EXT_TEXTURE_TRANSFORM
  uv = texture_info.uv_transform * vec3(uv, 1.0);
#endif
  return uv;
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
  }
  return fallback;
}
#endif

vec4 EE_GLTF_SAMPLE_TEXTURE_SLOT(
    uint16_t texture_info_slot, int texture_slot, vec2 tex_coord_0, vec2 tex_coord_1, vec4 fallback,
    float tex_grad) {
  if (!EE_GLTF_HAS_TEXTURE(texture_info_slot)) {
    return fallback;
  }
  GltfTextureInfo texture_info = EE_GLTF_TEXTURE_INFOS[uint(texture_info_slot)];
  if (texture_info.index < 0) {
    return fallback;
  }

  vec2 uv = EE_GLTF_TEXTURE_UV(texture_info, tex_coord_0, tex_coord_1);
  if (tex_grad > 0.0) {
#if MAT_EXT_TEXTURE_TRANSFORM
    vec2 ddx_uv = texture_info.uv_transform * vec3(tex_grad, 0.0, 0.0);
    vec2 ddy_uv = texture_info.uv_transform * vec3(0.0, tex_grad, 0.0);
#else
    vec2 ddx_uv = vec2(tex_grad, 0.0);
    vec2 ddy_uv = vec2(0.0, tex_grad);
#endif
#ifdef EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES
    return EE_GLTF_SAMPLE_FIXED_RASTER_TEXTURE(texture_slot, uv, ddx_uv, ddy_uv, true, fallback);
#else
    return EE_GLTF_SAMPLE_BINDLESS_TEXTURE(texture_info, uv, ddx_uv, ddy_uv, true);
#endif
  }
  const vec2 ddx_uv = vec2(0.0);
  const vec2 ddy_uv = vec2(0.0);
#ifdef EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES
  return EE_GLTF_SAMPLE_FIXED_RASTER_TEXTURE(texture_slot, uv, ddx_uv, ddy_uv, false, fallback);
#else
  return EE_GLTF_SAMPLE_BINDLESS_TEXTURE(texture_info, uv, ddx_uv, ddy_uv, false);
#endif
}

vec4 EE_GLTF_SAMPLE_TEXTURE(
    uint16_t texture_info_slot, vec2 tex_coord_0, vec2 tex_coord_1, vec4 fallback, float tex_grad) {
  return EE_GLTF_SAMPLE_TEXTURE_SLOT(texture_info_slot, EE_GLTF_RASTER_TEXTURE_UNSUPPORTED_EXTENSION, tex_coord_0,
                                     tex_coord_1, fallback, tex_grad);
}

vec4 EE_GLTF_SAMPLE_TEXTURE(uint16_t texture_info_slot, vec2 tex_coord_0, vec2 tex_coord_1, vec4 fallback) {
  return EE_GLTF_SAMPLE_TEXTURE(texture_info_slot, tex_coord_0, tex_coord_1, fallback, 0.0);
}

vec4 EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(
    uint16_t texture_info_slot, int texture_slot, vec2 tex_coord_0, vec2 tex_coord_1, vec4 fallback) {
  if (!EE_GLTF_HAS_TEXTURE(texture_info_slot)) {
    return fallback;
  }
  GltfTextureInfo texture_info = EE_GLTF_TEXTURE_INFOS[uint(texture_info_slot)];
  if (texture_info.index < 0) {
    return fallback;
  }

  const vec2 uv = EE_GLTF_TEXTURE_UV(texture_info, tex_coord_0, tex_coord_1);
#ifdef EE_GLTF_RASTER_FIXED_MATERIAL_TEXTURES
  return EE_GLTF_SAMPLE_FIXED_RASTER_TEXTURE_LOD0(texture_slot, uv, fallback);
#else
  return EE_GLTF_SAMPLE_BINDLESS_TEXTURE_LOD0(texture_info, uv);
#endif
}

vec4 EE_GLTF_SAMPLE_TEXTURE_LOD0(uint16_t texture_info_slot, vec2 tex_coord_0, vec2 tex_coord_1, vec4 fallback) {
  return EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(texture_info_slot, EE_GLTF_RASTER_TEXTURE_UNSUPPORTED_EXTENSION, tex_coord_0,
                                          tex_coord_1, fallback);
}

#if MAT_EXT_SPECULAR_GLOSSINESS
vec3 EE_GLTF_CONVERT_SPEC_GLOSS_TO_METALLIC_ROUGHNESS(
    vec3 diffuse_color, vec3 specular_color, float glossiness, out float metallic, out float roughness) {
  const float dielectric_specular = 0.04;
  float specular_intensity = max(specular_color.r, max(specular_color.g, specular_color.b));
  metallic = smoothstep(dielectric_specular + 0.01, dielectric_specular + 0.05, specular_intensity);

  vec3 base_color;
  if (metallic > 0.0) {
    base_color = specular_color;
  } else {
    base_color = diffuse_color / (1.0 - dielectric_specular * (1.0 - metallic));
    base_color = clamp(base_color, 0.0, 1.0);
  }

  roughness = max(1.0 - glossiness, EE_GLTF_MICROFACET_MIN_ROUGHNESS);
  return base_color;
}
#endif

#if MAT_EXT_VOLUME_SCATTER
vec3 EE_GLTF_MULTI_TO_SINGLE_SCATTER_ALBEDO(vec3 rho_ms) {
  vec3 t = 4.09712 + 4.20863 * rho_ms -
           sqrt(9.59217 + 41.6808 * rho_ms + 17.7126 * rho_ms * rho_ms);
  return 1.0 - t * t;
}
#endif

GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, float tex_grad) {
  GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  GltfRasterMaterial surface;
  surface.base_color = material.pbr_base_color_factor;
  surface.emissive = material.emissive_factor;
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

#if MAT_EXT_SPECULAR_GLOSSINESS
  if (material.pbr_model == EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS) {
    vec4 diffuse = material.pbr_diffuse_factor;
    vec3 specular = material.pbr_specular_factor;
    float glossiness = material.pbr_glossiness_factor;

    diffuse *= EE_GLTF_SAMPLE_TEXTURE_SLOT(material.pbr_diffuse_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR,
                                           tex_coord_0, tex_coord_1, vec4(1.0), tex_grad);
    vec4 specular_glossiness = EE_GLTF_SAMPLE_TEXTURE_SLOT(
        material.pbr_specular_glossiness_texture, EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS, tex_coord_0,
        tex_coord_1, vec4(1.0), tex_grad);
    specular *= specular_glossiness.rgb;
    glossiness *= specular_glossiness.a;

    surface.base_color.rgb = EE_GLTF_CONVERT_SPEC_GLOSS_TO_METALLIC_ROUGHNESS(
        diffuse.rgb, specular, glossiness, surface.metallic, surface.roughness);
    surface.base_color.a = diffuse.a;
  } else
#endif
  {
    surface.base_color *= EE_GLTF_SAMPLE_TEXTURE_SLOT(
        material.pbr_base_color_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR, tex_coord_0, tex_coord_1,
        vec4(1.0), tex_grad);
    vec4 metallic_roughness = EE_GLTF_SAMPLE_TEXTURE_SLOT(
        material.pbr_metallic_roughness_texture, EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS, tex_coord_0,
        tex_coord_1, vec4(1.0), tex_grad);
    surface.roughness *= metallic_roughness.g;
    surface.metallic *= metallic_roughness.b;
    surface.roughness = max(surface.roughness, EE_GLTF_MICROFACET_MIN_ROUGHNESS);
    surface.metallic = clamp(surface.metallic, 0.0, 1.0);
  }

  if (EE_GLTF_HAS_TEXTURE(material.occlusion_texture)) {
    float occlusion = EE_GLTF_SAMPLE_TEXTURE_SLOT(
        material.occlusion_texture, EE_GLTF_RASTER_TEXTURE_OCCLUSION, tex_coord_0, tex_coord_1, vec4(1.0),
        tex_grad).r;
    surface.occlusion = 1.0 + surface.occlusion * (occlusion - 1.0);
  }

#if MAT_EXT_TRANSMISSION
  surface.transmission = material.transmission_factor;
  if (EE_GLTF_HAS_TEXTURE(material.transmission_texture)) {
    surface.transmission *=
        EE_GLTF_SAMPLE_TEXTURE(material.transmission_texture, tex_coord_0, tex_coord_1, vec4(1.0), tex_grad).r;
  }
#endif

#if MAT_EXT_VOLUME
  surface.attenuation_color = material.attenuation_color;
  surface.attenuation_distance = material.attenuation_distance;
  surface.thickness = material.thickness_factor;
  if (EE_GLTF_HAS_TEXTURE(material.thickness_texture)) {
    surface.thickness *=
        EE_GLTF_SAMPLE_TEXTURE(material.thickness_texture, tex_coord_0, tex_coord_1, vec4(1.0), tex_grad).g;
  }
#endif

#if MAT_EXT_DIFFUSE_TRANSMISSION
  surface.diffuse_transmission_factor = material.diffuse_transmission_factor;
  if (EE_GLTF_HAS_TEXTURE(material.diffuse_transmission_texture)) {
    surface.diffuse_transmission_factor *=
        EE_GLTF_SAMPLE_TEXTURE(material.diffuse_transmission_texture, tex_coord_0, tex_coord_1, vec4(1.0),
                               tex_grad).a;
  }
  surface.diffuse_transmission_color = material.diffuse_transmission_color;
  if (EE_GLTF_HAS_TEXTURE(material.diffuse_transmission_color_texture)) {
    surface.diffuse_transmission_color *=
        EE_GLTF_SAMPLE_TEXTURE(material.diffuse_transmission_color_texture, tex_coord_0, tex_coord_1, vec4(1.0),
                               tex_grad).rgb;
  }
#endif

#if MAT_EXT_VOLUME_SCATTER
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
      material.emissive_texture, EE_GLTF_RASTER_TEXTURE_EMISSIVE, tex_coord_0, tex_coord_1, vec4(1.0),
      tex_grad).rgb;
  surface.emissive = max(vec3(0.0), surface.emissive);
  return surface;
}

GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(uint material_index, vec2 tex_coord_0, vec2 tex_coord_1) {
  return EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, tex_coord_0, tex_coord_1, 0.0);
}

vec3 EE_GLTF_SAFE_NORMALIZE(vec3 value, vec3 fallback) {
  float length_squared = dot(value, value);
  return length_squared > 1e-12 ? value * inversesqrt(length_squared) : fallback;
}

vec3 EE_GLTF_FALLBACK_TANGENT(vec3 normal) {
  vec3 axis = abs(normal.z) < 0.999 ? vec3(0.0, 0.0, 1.0) : vec3(0.0, 1.0, 0.0);
  return EE_GLTF_SAFE_NORMALIZE(cross(axis, normal), vec3(1.0, 0.0, 0.0));
}

vec3 EE_EVALUATE_GLTF_RASTER_NORMAL(uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec3 normal,
                                    vec3 tangent, float tangent_handedness, float tex_grad) {
  GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  vec3 n = EE_GLTF_SAFE_NORMALIZE(normal, vec3(0.0, 1.0, 0.0));
  if (!EE_GLTF_HAS_TEXTURE(material.normal_texture)) {
    return n;
  }

  vec3 normal_vector = EE_GLTF_SAMPLE_TEXTURE_SLOT(
      material.normal_texture, EE_GLTF_RASTER_TEXTURE_NORMAL, tex_coord_0, tex_coord_1,
      vec4(0.5, 0.5, 1.0, 1.0), tex_grad).xyz;
  normal_vector = normal_vector * 2.0 - 1.0;
  normal_vector.xy *= material.normal_texture_scale;

  vec3 t = tangent - n * dot(n, tangent);
  t = EE_GLTF_SAFE_NORMALIZE(t, EE_GLTF_FALLBACK_TANGENT(n));
  float bitangent_sign = tangent_handedness < 0.0 ? -1.0 : 1.0;
  vec3 b = cross(n, t) * bitangent_sign;
  return EE_GLTF_SAFE_NORMALIZE(mat3(t, b, n) * normal_vector, n);
}

vec3 EE_EVALUATE_GLTF_RASTER_NORMAL(uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec3 normal,
                                    vec3 tangent, float tangent_handedness) {
  return EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coord_0, tex_coord_1, normal, tangent,
                                        tangent_handedness, 0.0);
}

vec3 EE_EVALUATE_GLTF_RASTER_NORMAL(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec3 normal, vec3 tangent) {
  return EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coord_0, tex_coord_1, normal, tangent, 1.0);
}

float EE_GLTF_RASTER_OPACITY(GltfRasterMaterial surface) {
  const float alpha = clamp(surface.base_color.a, 0.0, 1.0);
  const float transmission = clamp(max(surface.transmission, surface.diffuse_transmission_factor), 0.0, 1.0);
  return alpha * (1.0 - transmission);
}

bool EE_GLTF_RASTER_SHOULD_DISCARD(GltfRasterMaterial surface) {
  if (surface.alpha_mode == EE_GLTF_ALPHA_MODE_MASK) {
    return surface.base_color.a < surface.alpha_cutoff;
  }
  return EE_GLTF_RASTER_OPACITY(surface) <= 0.0;
}

float EE_GLTF_RASTER_IOR_FRESNEL(const float ior, const float cos_theta) {
  const float safe_ior = max(ior, 1.0);
  float f0 = (1.0 - safe_ior) / (1.0 + safe_ior);
  f0 *= f0;
  return f0 + (1.0 - f0) * pow(max(1.0 - clamp(cos_theta, 0.0, 1.0), 0.0), 5.0);
}

float EE_GLTF_RASTER_OPACITY_LOD0(uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, float vertex_alpha) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  if (material.alpha_mode == EE_GLTF_ALPHA_MODE_OPAQUE) {
    return 1.0;
  }

  float base_color_alpha = 1.0;
#if MAT_EXT_SPECULAR_GLOSSINESS
  if (material.pbr_model == EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS) {
    base_color_alpha = material.pbr_diffuse_factor.a;
    base_color_alpha *= EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(
        material.pbr_diffuse_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR, tex_coord_0, tex_coord_1,
        vec4(1.0)).a;
  } else
#endif
  {
    base_color_alpha = material.pbr_base_color_factor.a;
    base_color_alpha *= EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(
        material.pbr_base_color_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR, tex_coord_0, tex_coord_1,
        vec4(1.0)).a;
  }

  base_color_alpha *= vertex_alpha;
  if (material.alpha_mode == EE_GLTF_ALPHA_MODE_MASK) {
    return base_color_alpha >= material.alpha_cutoff ? 1.0 : 0.0;
  }
  return base_color_alpha;
}

vec3 EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0(uint material_index, vec2 tex_coord_0, vec2 tex_coord_1,
                                             float cos_theta, float segment_length, inout bool is_inside,
                                             float min_transmission) {
#if !MAT_EXT_TRANSMISSION
  return vec3(0.0);
#else
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  if (material.transmission_factor <= min_transmission) {
    return vec3(0.0);
  }

  float ior = 1.5;
#if MAT_EXT_IOR
  ior = material.ior;
#endif
  const float fresnel = EE_GLTF_RASTER_IOR_FRESNEL(ior, cos_theta);
  vec3 transmission = material.transmission_factor * material.pbr_base_color_factor.rgb * (1.0 - fresnel);

#if MAT_EXT_VOLUME
  if (material.thickness_factor > 0.0) {
    if (is_inside) {
      const vec3 absorption_coefficient =
          -log(max(material.attenuation_color, vec3(0.001))) / max(material.attenuation_distance, 0.001);
      vec3 scatter_coefficient = vec3(0.0);
#if MAT_EXT_VOLUME_SCATTER
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
      material.pbr_metallic_roughness_texture, EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS, tex_coord_0,
      tex_coord_1, vec4(1.0));
  roughness *= metallic_roughness.g;
  metallic *= metallic_roughness.b;

  const float roughness_effect = 1.0 - roughness * roughness;
  const float transmission_attenuation = (1.0 - metallic) * mix(0.65, 1.0, roughness_effect);
  return transmission * transmission_attenuation;
#endif
}

#endif
