// @evoengine-dialect native
__exported import EvoEngine.GltfMaterial;
[[vk::binding(0, 3)]] public
Sampler2D<float4> EE_GLTF_RASTER_BASE_COLOR_TEXTURE;
[[vk::binding(1, 3)]] public
Sampler2D<float4> EE_GLTF_RASTER_METALLIC_ROUGHNESS_TEXTURE;
[[vk::binding(2, 3)]] public
Sampler2D<float4> EE_GLTF_RASTER_NORMAL_TEXTURE;
[[vk::binding(3, 3)]] public
Sampler2D<float4> EE_GLTF_RASTER_EMISSIVE_TEXTURE;
[[vk::binding(4, 3)]] public
Sampler2D<float4> EE_GLTF_RASTER_OCCLUSION_TEXTURE;
[[vk::binding(5, 3)]] public
Sampler2D<float4> EE_GLTF_RASTER_CLEARCOAT_TEXTURE;
[[vk::binding(6, 3)]] public
Sampler2D<float4> EE_GLTF_RASTER_CLEARCOAT_ROUGHNESS_TEXTURE;
[[vk::binding(7, 3)]] public
Sampler2D<float4> EE_GLTF_RASTER_CLEARCOAT_NORMAL_TEXTURE;
public
static const float EE_GLTF_MICROFACET_MIN_ROUGHNESS = 0.0014142;
public
static const int EE_GLTF_RASTER_TEXTURE_UNSUPPORTED_EXTENSION = -1;
public
static const int EE_GLTF_RASTER_TEXTURE_BASE_COLOR = 0;
public
static const int EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS = 1;
public
static const int EE_GLTF_RASTER_TEXTURE_NORMAL = 2;
public
static const int EE_GLTF_RASTER_TEXTURE_EMISSIVE = 3;
public
static const int EE_GLTF_RASTER_TEXTURE_OCCLUSION = 4;
public
static const int EE_GLTF_RASTER_TEXTURE_CLEARCOAT = 5;
public
static const int EE_GLTF_RASTER_TEXTURE_CLEARCOAT_ROUGHNESS = 6;
public
static const int EE_GLTF_RASTER_TEXTURE_CLEARCOAT_NORMAL = 7;
public
struct GltfRasterMaterial {
 public
  float4 base_color;
 public
  float3 specular_f0;
 public
  float specular_f90;
 public
  float3 emissive;
 public
  float clearcoat;
 public
  float clearcoat_roughness;
 public
  float metallic;
 public
  float roughness;
 public
  float occlusion;
 public
  int alpha_mode;
 public
  float alpha_cutoff;
 public
  float transmission;
 public
  float3 attenuation_color;
 public
  float attenuation_distance;
 public
  float thickness;
 public
  float3 diffuse_transmission_color;
 public
  float diffuse_transmission_factor;
 public
  float3 multiscatter_color_factor;
 public
  float3 scatter_coefficient;
 public
  float scatter_anisotropy;
};
public
struct GltfTexCoords {
 public
  float2 uv0;
 public
  float2 uv1;
 public
  float2 uv2;
 public
  float2 uv3;
 public
  float4 gradients;
};
public
GltfTexCoords EE_GLTF_MAKE_TEX_COORDS(float2 uv0, float2 uv1, float2 uv2, float2 uv3, float4 gradients) {
  return GltfTexCoords(uv0, uv1, uv2, uv3, gradients);
}
public
GltfTexCoords EE_GLTF_MAKE_TEX_COORDS(float2 uv0, float2 uv1) {
  return EE_GLTF_MAKE_TEX_COORDS(uv0, uv1, float2(0.0), float2(0.0), float4(0.0));
}
public
bool EE_GLTF_HAS_TEXTURE(uint16_t texture_info_slot) {
  return uint(texture_info_slot) > 0u;
}
public
float2 EE_GLTF_SELECT_TEX_COORD(int tex_coord, GltfTexCoords tex_coords) {
  if (tex_coord == 1)
    return tex_coords.uv1;
  if (tex_coord == 2)
    return tex_coords.uv2;
  if (tex_coord == 3)
    return tex_coords.uv3;
  return tex_coords.uv0;
}
public
float2 EE_GLTF_TEXTURE_UV(GltfTextureInfo texture_info, GltfTexCoords tex_coords) {
  float2 uv = EE_GLTF_SELECT_TEX_COORD(texture_info.tex_coord, tex_coords);
  uv = EE_GLTF_TRANSFORM_UV(texture_info, float3(uv, 1.0));
  return uv;
}
public
float3 EE_GLTF_SRGB_TO_LINEAR(float3 encoded) {
  const float3 low = encoded / 12.92;
  const float3 high = pow(max((encoded + 0.055) / 1.055, float3(0.0)), float3(2.4));
  return float3(encoded.x <= 0.04045 ? low.x : high.x, encoded.y <= 0.04045 ? low.y : high.y,
                encoded.z <= 0.04045 ? low.z : high.z);
}
public
float4 EE_GLTF_DECODE_TEXTURE_SAMPLE(const GltfTextureInfo texture_info, float4 sample_value) {
  if (texture_info.color_space == EE_GLTF_TEXTURE_COLOR_SPACE_SRGB) {
    sample_value.rgb = EE_GLTF_SRGB_TO_LINEAR(sample_value.rgb);
  }
  return sample_value;
}
public
float4 EE_GLTF_SAMPLE_FIXED_RASTER_TEXTURE(int texture_slot, float2 uv, float2 ddx_uv, float2 ddy_uv, bool use_grad,
                                           float4 fallback) {
  switch (texture_slot) {
    case EE_GLTF_RASTER_TEXTURE_BASE_COLOR:
      return use_grad ? EE_GLTF_RASTER_BASE_COLOR_TEXTURE.SampleGrad(uv, ddx_uv, ddy_uv)
                      : EE_GLTF_RASTER_BASE_COLOR_TEXTURE.Sample(uv);
    case EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS:
      return use_grad ? EE_GLTF_RASTER_METALLIC_ROUGHNESS_TEXTURE.SampleGrad(uv, ddx_uv, ddy_uv)
                      : EE_GLTF_RASTER_METALLIC_ROUGHNESS_TEXTURE.Sample(uv);
    case EE_GLTF_RASTER_TEXTURE_NORMAL:
      return use_grad ? EE_GLTF_RASTER_NORMAL_TEXTURE.SampleGrad(uv, ddx_uv, ddy_uv)
                      : EE_GLTF_RASTER_NORMAL_TEXTURE.Sample(uv);
    case EE_GLTF_RASTER_TEXTURE_EMISSIVE:
      return use_grad ? EE_GLTF_RASTER_EMISSIVE_TEXTURE.SampleGrad(uv, ddx_uv, ddy_uv)
                      : EE_GLTF_RASTER_EMISSIVE_TEXTURE.Sample(uv);
    case EE_GLTF_RASTER_TEXTURE_OCCLUSION:
      return use_grad ? EE_GLTF_RASTER_OCCLUSION_TEXTURE.SampleGrad(uv, ddx_uv, ddy_uv)
                      : EE_GLTF_RASTER_OCCLUSION_TEXTURE.Sample(uv);
    case EE_GLTF_RASTER_TEXTURE_CLEARCOAT:
      return use_grad ? EE_GLTF_RASTER_CLEARCOAT_TEXTURE.SampleGrad(uv, ddx_uv, ddy_uv)
                      : EE_GLTF_RASTER_CLEARCOAT_TEXTURE.Sample(uv);
    case EE_GLTF_RASTER_TEXTURE_CLEARCOAT_ROUGHNESS:
      return use_grad ? EE_GLTF_RASTER_CLEARCOAT_ROUGHNESS_TEXTURE.SampleGrad(uv, ddx_uv, ddy_uv)
                      : EE_GLTF_RASTER_CLEARCOAT_ROUGHNESS_TEXTURE.Sample(uv);
    case EE_GLTF_RASTER_TEXTURE_CLEARCOAT_NORMAL:
      return use_grad ? EE_GLTF_RASTER_CLEARCOAT_NORMAL_TEXTURE.SampleGrad(uv, ddx_uv, ddy_uv)
                      : EE_GLTF_RASTER_CLEARCOAT_NORMAL_TEXTURE.Sample(uv);
  }
  return fallback;
}
public
float4 EE_GLTF_SAMPLE_FIXED_RASTER_TEXTURE_LOD0(int texture_slot, float2 uv, float4 fallback) {
  switch (texture_slot) {
    case EE_GLTF_RASTER_TEXTURE_BASE_COLOR:
      return EE_GLTF_RASTER_BASE_COLOR_TEXTURE.SampleLevel(uv, 0.0f);
    case EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS:
      return EE_GLTF_RASTER_METALLIC_ROUGHNESS_TEXTURE.SampleLevel(uv, 0.0f);
    case EE_GLTF_RASTER_TEXTURE_NORMAL:
      return EE_GLTF_RASTER_NORMAL_TEXTURE.SampleLevel(uv, 0.0f);
    case EE_GLTF_RASTER_TEXTURE_EMISSIVE:
      return EE_GLTF_RASTER_EMISSIVE_TEXTURE.SampleLevel(uv, 0.0f);
    case EE_GLTF_RASTER_TEXTURE_OCCLUSION:
      return EE_GLTF_RASTER_OCCLUSION_TEXTURE.SampleLevel(uv, 0.0f);
    case EE_GLTF_RASTER_TEXTURE_CLEARCOAT:
      return EE_GLTF_RASTER_CLEARCOAT_TEXTURE.SampleLevel(uv, 0.0f);
    case EE_GLTF_RASTER_TEXTURE_CLEARCOAT_ROUGHNESS:
      return EE_GLTF_RASTER_CLEARCOAT_ROUGHNESS_TEXTURE.SampleLevel(uv, 0.0f);
    case EE_GLTF_RASTER_TEXTURE_CLEARCOAT_NORMAL:
      return EE_GLTF_RASTER_CLEARCOAT_NORMAL_TEXTURE.SampleLevel(uv, 0.0f);
  }
  return fallback;
}
public
float4 EE_GLTF_SAMPLE_TEXTURE_SLOT(uint16_t texture_info_slot, int texture_slot, GltfTexCoords tex_coords,
                                   float4 fallback) {
  if (!EE_GLTF_HAS_TEXTURE(texture_info_slot)) {
    return fallback;
  }
  GltfTextureInfo texture_info = EE_GLTF_TEXTURE_INFOS[uint(texture_info_slot)];
  if (texture_info.index < 0) {
    return fallback;
  }
  float2 uv = EE_GLTF_TEXTURE_UV(texture_info, tex_coords);
  const float tex_grad = tex_coords.gradients[clamp(texture_info.tex_coord, 0, 3)];
  float4 sample_value;
  if (tex_grad > 0.0) {
    float2 ddx_uv = EE_GLTF_TRANSFORM_UV(texture_info, float3(tex_grad, 0.0, 0.0));
    float2 ddy_uv = EE_GLTF_TRANSFORM_UV(texture_info, float3(0.0, tex_grad, 0.0));
    sample_value = EE_GLTF_SAMPLE_FIXED_RASTER_TEXTURE(texture_slot, uv, ddx_uv, ddy_uv, true, fallback);
  } else {
    const float2 ddx_uv = float2(0.0);
    const float2 ddy_uv = float2(0.0);
    sample_value = EE_GLTF_SAMPLE_FIXED_RASTER_TEXTURE(texture_slot, uv, ddx_uv, ddy_uv, false, fallback);
  }
  return EE_GLTF_DECODE_TEXTURE_SAMPLE(texture_info, sample_value);
}
public
float4 EE_GLTF_SAMPLE_TEXTURE(uint16_t texture_info_slot, GltfTexCoords tex_coords, float4 fallback) {
  return EE_GLTF_SAMPLE_TEXTURE_SLOT(texture_info_slot, EE_GLTF_RASTER_TEXTURE_UNSUPPORTED_EXTENSION, tex_coords,
                                     fallback);
}
public
float4 EE_GLTF_SAMPLE_TEXTURE_SLOT(uint16_t texture_info_slot, int texture_slot, float2 tex_coord_0, float2 tex_coord_1,
                                   float4 fallback, float2 tex_gradients) {
  return EE_GLTF_SAMPLE_TEXTURE_SLOT(
      texture_info_slot, texture_slot,
      EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, float2(0.0), float2(0.0), float4(tex_gradients, 0.0, 0.0)),
      fallback);
}
public
float4 EE_GLTF_SAMPLE_TEXTURE(uint16_t texture_info_slot, float2 tex_coord_0, float2 tex_coord_1, float4 fallback,
                              float2 tex_gradients) {
  return EE_GLTF_SAMPLE_TEXTURE_SLOT(texture_info_slot, EE_GLTF_RASTER_TEXTURE_UNSUPPORTED_EXTENSION, tex_coord_0,
                                     tex_coord_1, fallback, tex_gradients);
}
public
float4 EE_GLTF_SAMPLE_TEXTURE_SLOT(uint16_t texture_info_slot, int texture_slot, float2 tex_coord_0, float2 tex_coord_1,
                                   float4 fallback, float tex_grad) {
  return EE_GLTF_SAMPLE_TEXTURE_SLOT(texture_info_slot, texture_slot, tex_coord_0, tex_coord_1, fallback,
                                     float2(tex_grad));
}
public
float4 EE_GLTF_SAMPLE_TEXTURE(uint16_t texture_info_slot, float2 tex_coord_0, float2 tex_coord_1, float4 fallback,
                              float tex_grad) {
  return EE_GLTF_SAMPLE_TEXTURE(texture_info_slot, tex_coord_0, tex_coord_1, fallback, float2(tex_grad));
}
public
float4 EE_GLTF_SAMPLE_TEXTURE(uint16_t texture_info_slot, float2 tex_coord_0, float2 tex_coord_1, float4 fallback) {
  return EE_GLTF_SAMPLE_TEXTURE(texture_info_slot, tex_coord_0, tex_coord_1, fallback, 0.0);
}
public
float4 EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(uint16_t texture_info_slot, int texture_slot, GltfTexCoords tex_coords,
                                        float4 fallback) {
  if (!EE_GLTF_HAS_TEXTURE(texture_info_slot)) {
    return fallback;
  }
  GltfTextureInfo texture_info = EE_GLTF_TEXTURE_INFOS[uint(texture_info_slot)];
  if (texture_info.index < 0) {
    return fallback;
  }
  const float2 uv = EE_GLTF_TEXTURE_UV(texture_info, tex_coords);
  const float4 sample_value = EE_GLTF_SAMPLE_FIXED_RASTER_TEXTURE_LOD0(texture_slot, uv, fallback);
  return EE_GLTF_DECODE_TEXTURE_SAMPLE(texture_info, sample_value);
}
public
float4 EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(uint16_t texture_info_slot, int texture_slot, float2 tex_coord_0,
                                        float2 tex_coord_1, float4 fallback) {
  return EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(texture_info_slot, texture_slot,
                                          EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1), fallback);
}
public
float4 EE_GLTF_SAMPLE_TEXTURE_LOD0(uint16_t texture_info_slot, GltfTexCoords tex_coords, float4 fallback) {
  return EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(texture_info_slot, EE_GLTF_RASTER_TEXTURE_UNSUPPORTED_EXTENSION, tex_coords,
                                          fallback);
}
public
float4 EE_GLTF_SAMPLE_TEXTURE_LOD0(uint16_t texture_info_slot, float2 tex_coord_0, float2 tex_coord_1,
                                   float4 fallback) {
  return EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(texture_info_slot, EE_GLTF_RASTER_TEXTURE_UNSUPPORTED_EXTENSION, tex_coord_0,
                                          tex_coord_1, fallback);
}
public
float3 EE_GLTF_MULTI_TO_SINGLE_SCATTER_ALBEDO(float3 rho_ms) {
  float3 t = 4.09712 + 4.20863 * rho_ms - sqrt(9.59217 + 41.6808 * rho_ms + 17.7126 * rho_ms * rho_ms);
  return 1.0 - t * t;
}
public
GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(uint material_index, GltfTexCoords tex_coords, float4 vertex_color) {
  GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  GltfRasterMaterial surface;
  vertex_color = clamp(vertex_color, float4(0.0), float4(1.0));
  surface.base_color = material.pbr_base_color_factor * vertex_color;
  surface.specular_f0 = float3(0.04);
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
  surface.attenuation_color = float3(1.0);
  surface.attenuation_distance = 1.0;
  surface.thickness = 0.0;
  surface.diffuse_transmission_color = float3(1.0);
  surface.diffuse_transmission_factor = 0.0;
  surface.multiscatter_color_factor = float3(0.0);
  surface.scatter_coefficient = float3(0.0);
  surface.scatter_anisotropy = 0.0;
  if (material.pbr_model == EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS) {
    float4 diffuse = material.pbr_diffuse_factor * vertex_color;
    float3 specular = material.pbr_specular_factor;
    float glossiness = material.pbr_glossiness_factor;
    diffuse *= EE_GLTF_SAMPLE_TEXTURE_SLOT(material.pbr_diffuse_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR, tex_coords,
                                           float4(1.0));
    float4 specular_glossiness = EE_GLTF_SAMPLE_TEXTURE_SLOT(
        material.pbr_specular_glossiness_texture, EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS, tex_coords, float4(1.0));
    specular *= specular_glossiness.rgb;
    glossiness *= specular_glossiness.a;
    surface.specular_f0 = clamp(specular, float3(0.0), float3(1.0));
    surface.base_color.rgb =
        diffuse.rgb * (1.0 - max(surface.specular_f0.r, max(surface.specular_f0.g, surface.specular_f0.b)));
    surface.base_color.a = diffuse.a;
    surface.metallic = 0.0;
    surface.roughness = max(1.0 - glossiness, EE_GLTF_MICROFACET_MIN_ROUGHNESS);
  } else {
    surface.base_color *= EE_GLTF_SAMPLE_TEXTURE_SLOT(material.pbr_base_color_texture,
                                                      EE_GLTF_RASTER_TEXTURE_BASE_COLOR, tex_coords, float4(1.0));
    float4 metallic_roughness = EE_GLTF_SAMPLE_TEXTURE_SLOT(
        material.pbr_metallic_roughness_texture, EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS, tex_coords, float4(1.0));
    surface.roughness *= metallic_roughness.g;
    surface.metallic *= metallic_roughness.b;
    surface.roughness = max(surface.roughness, EE_GLTF_MICROFACET_MIN_ROUGHNESS);
    surface.metallic = clamp(surface.metallic, 0.0, 1.0);
    float dielectric_f0 = 0.04;
    float dielectric_specular_f90 = 1.0;
    const float material_ior = material.ior == 0.0 ? 0.0 : max(material.ior, 1.0);
    dielectric_f0 = pow((material_ior - 1.0) / max(material_ior + 1.0, 0.000001), 2.0);
    float3 dielectric_specular_f0 = float3(dielectric_f0);
    float specular_weight = material.specular_factor;
    specular_weight *= EE_GLTF_SAMPLE_TEXTURE(material.specular_texture, tex_coords, float4(1.0)).a;
    float3 specular_color = material.specular_color_factor;
    specular_color *= EE_GLTF_SAMPLE_TEXTURE(material.specular_color_texture, tex_coords, float4(1.0)).rgb;
    dielectric_specular_f0 =
        clamp(dielectric_specular_f0 * max(specular_color, float3(0.0)), float3(0.0), float3(1.0)) *
        clamp(specular_weight, 0.0, 1.0);
    dielectric_specular_f90 = clamp(specular_weight, 0.0, 1.0);
    surface.specular_f0 = lerp(dielectric_specular_f0, max(surface.base_color.rgb, float3(0.0)), surface.metallic);
    surface.specular_f90 = lerp(dielectric_specular_f90, 1.0, surface.metallic);
  }
  if (EE_GLTF_HAS_TEXTURE(material.occlusion_texture)) {
    float occlusion = EE_GLTF_SAMPLE_TEXTURE_SLOT(material.occlusion_texture, EE_GLTF_RASTER_TEXTURE_OCCLUSION,
                                                  tex_coords, float4(1.0))
                          .r;
    surface.occlusion = 1.0 + surface.occlusion * (occlusion - 1.0);
  }
  surface.clearcoat = clamp(material.clearcoat_factor, 0.0, 1.0);
  surface.clearcoat *=
      EE_GLTF_SAMPLE_TEXTURE_SLOT(material.clearcoat_texture, EE_GLTF_RASTER_TEXTURE_CLEARCOAT, tex_coords, float4(1.0))
          .r;
  surface.clearcoat_roughness = max(material.clearcoat_roughness, EE_GLTF_MICROFACET_MIN_ROUGHNESS);
  surface.clearcoat_roughness *=
      EE_GLTF_SAMPLE_TEXTURE_SLOT(material.clearcoat_roughness_texture, EE_GLTF_RASTER_TEXTURE_CLEARCOAT_ROUGHNESS,
                                  tex_coords, float4(1.0))
          .g;
  surface.transmission = material.transmission_factor;
  if (EE_GLTF_HAS_TEXTURE(material.transmission_texture)) {
    surface.transmission *= EE_GLTF_SAMPLE_TEXTURE(material.transmission_texture, tex_coords, float4(1.0)).r;
  }
  surface.attenuation_color = material.attenuation_color;
  surface.attenuation_distance = material.attenuation_distance;
  surface.thickness = material.thickness_factor;
  if (EE_GLTF_HAS_TEXTURE(material.thickness_texture)) {
    surface.thickness *= EE_GLTF_SAMPLE_TEXTURE(material.thickness_texture, tex_coords, float4(1.0)).g;
  }
  surface.diffuse_transmission_factor = material.diffuse_transmission_factor;
  if (EE_GLTF_HAS_TEXTURE(material.diffuse_transmission_texture)) {
    surface.diffuse_transmission_factor *=
        EE_GLTF_SAMPLE_TEXTURE(material.diffuse_transmission_texture, tex_coords, float4(1.0)).a;
  }
  surface.diffuse_transmission_color = material.diffuse_transmission_color;
  if (EE_GLTF_HAS_TEXTURE(material.diffuse_transmission_color_texture)) {
    surface.diffuse_transmission_color *=
        EE_GLTF_SAMPLE_TEXTURE(material.diffuse_transmission_color_texture, tex_coords, float4(1.0)).rgb;
  }
  surface.multiscatter_color_factor = max(material.multiscatter_color_factor, float3(0.0));
  surface.scatter_anisotropy = clamp(material.scatter_anisotropy, -0.999, 0.999);
  if (any(surface.multiscatter_color_factor > float3(0.0))) {
    float3 single_scatter_albedo = EE_GLTF_MULTI_TO_SINGLE_SCATTER_ALBEDO(surface.multiscatter_color_factor);
    float3 attenuation_coefficient =
        -log(max(surface.attenuation_color, float3(0.001))) / max(surface.attenuation_distance, 0.001);
    surface.scatter_coefficient = attenuation_coefficient * single_scatter_albedo;
  }
  surface.emissive *=
      EE_GLTF_SAMPLE_TEXTURE_SLOT(material.emissive_texture, EE_GLTF_RASTER_TEXTURE_EMISSIVE, tex_coords, float4(1.0))
          .rgb;
  surface.emissive = max(float3(0.0), surface.emissive);
  return surface;
}
public
GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(uint material_index, float2 tex_coord_0, float2 tex_coord_1,
                                                   float2 tex_coord_2, float2 tex_coord_3, float4 vertex_color,
                                                   float4 tex_gradients) {
  return EE_EVALUATE_GLTF_RASTER_SURFACE(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, tex_gradients),
      vertex_color);
}
public
GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(uint material_index, float2 tex_coord_0, float2 tex_coord_1,
                                                   float2 tex_coord_2, float2 tex_coord_3, float4 vertex_color) {
  return EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3,
                                         vertex_color, float4(0.0));
}
public
GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(uint material_index, float2 tex_coord_0, float2 tex_coord_1,
                                                   float4 vertex_color, float2 tex_gradients) {
  return EE_EVALUATE_GLTF_RASTER_SURFACE(
      material_index,
      EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, float2(0.0), float2(0.0), float4(tex_gradients, 0.0, 0.0)),
      vertex_color);
}
public
GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(uint material_index, float2 tex_coord_0, float2 tex_coord_1,
                                                   float4 vertex_color, float tex_grad) {
  return EE_EVALUATE_GLTF_RASTER_SURFACE(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, float2(0.0), float2(0.0), float4(tex_grad)),
      vertex_color);
}
public
GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(uint material_index, float2 tex_coord_0, float2 tex_coord_1,
                                                   float2 tex_gradients) {
  return EE_EVALUATE_GLTF_RASTER_SURFACE(
      material_index,
      EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, float2(0.0), float2(0.0), float4(tex_gradients, 0.0, 0.0)),
      float4(1.0));
}
public
GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(uint material_index, float2 tex_coord_0, float2 tex_coord_1,
                                                   float tex_grad) {
  return EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, tex_coord_0, tex_coord_1, float4(1.0), float2(tex_grad));
}
public
GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(uint material_index, float2 tex_coord_0, float2 tex_coord_1,
                                                   float4 vertex_color) {
  return EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1),
                                         vertex_color);
}
public
GltfRasterMaterial EE_EVALUATE_GLTF_RASTER_SURFACE(uint material_index, float2 tex_coord_0, float2 tex_coord_1) {
  return EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1),
                                         float4(1.0));
}
public
float3 EE_GLTF_RASTER_REBASE_SPECULAR_F0(uint material_index, GltfRasterMaterial surface, float3 base_color) {
  GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  if (material.pbr_model == EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS) {
    return surface.specular_f0;
  }
  const float metallic = clamp(surface.metallic, 0.0, 1.0);
  const float3 rebased_base_color = max(base_color, float3(0.0));
  if (metallic >= 0.999999) {
    return rebased_base_color;
  }
  const float3 original_base_color = max(surface.base_color.rgb, float3(0.0));
  const float3 dielectric_specular_f0 = clamp(
      (surface.specular_f0 - original_base_color * metallic) / max(1.0 - metallic, 0.000001), float3(0.0), float3(1.0));
  return lerp(dielectric_specular_f0, rebased_base_color, metallic);
}
public
float3 EE_GLTF_SAFE_NORMALIZE(float3 value, float3 fallback) {
  float length_squared = dot(value, value);
  return length_squared > 1e-12 ? value * rsqrt(length_squared) : fallback;
}
public
float3 EE_GLTF_FALLBACK_TANGENT(float3 normal) {
  float3 axis = abs(normal.z) < 0.999 ? float3(0.0, 0.0, 1.0) : float3(0.0, 1.0, 0.0);
  return EE_GLTF_SAFE_NORMALIZE(cross(axis, normal), float3(1.0, 0.0, 0.0));
}
public
float3 EE_EVALUATE_GLTF_RASTER_NORMAL(uint material_index, GltfTexCoords tex_coords, float3 normal, float3 tangent,
                                      float tangent_handedness) {
  GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  float3 n = EE_GLTF_SAFE_NORMALIZE(normal, float3(0.0, 1.0, 0.0));
  if (!EE_GLTF_HAS_TEXTURE(material.normal_texture)) {
    return n;
  }
  float3 normal_vector = EE_GLTF_SAMPLE_TEXTURE_SLOT(material.normal_texture, EE_GLTF_RASTER_TEXTURE_NORMAL, tex_coords,
                                                     float4(0.5, 0.5, 1.0, 1.0))
                             .xyz;
  normal_vector = normal_vector * 2.0 - 1.0;
  normal_vector.xy *= material.normal_texture_scale;
  float3 t = tangent - n * dot(n, tangent);
  t = EE_GLTF_SAFE_NORMALIZE(t, EE_GLTF_FALLBACK_TANGENT(n));
  float bitangent_sign = tangent_handedness < 0.0 ? -1.0 : 1.0;
  float3 b = cross(n, t) * bitangent_sign;
  return EE_GLTF_SAFE_NORMALIZE(mul(normal_vector, float3x3(t, b, n)), n);
}
public
float3 EE_EVALUATE_GLTF_RASTER_NORMAL(uint material_index, float2 tex_coord_0, float2 tex_coord_1, float2 tex_coord_2,
                                      float2 tex_coord_3, float3 normal, float3 tangent, float tangent_handedness,
                                      float4 tex_gradients) {
  return EE_EVALUATE_GLTF_RASTER_NORMAL(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, tex_gradients),
      normal, tangent, tangent_handedness);
}
public
float3 EE_EVALUATE_GLTF_RASTER_NORMAL(uint material_index, float2 tex_coord_0, float2 tex_coord_1, float2 tex_coord_2,
                                      float2 tex_coord_3, float3 normal, float3 tangent, float tangent_handedness) {
  return EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, normal,
                                        tangent, tangent_handedness, float4(0.0));
}
public
float3 EE_EVALUATE_GLTF_RASTER_NORMAL(uint material_index, float2 tex_coord_0, float2 tex_coord_1, float3 normal,
                                      float3 tangent, float tangent_handedness, float2 tex_gradients) {
  return EE_EVALUATE_GLTF_RASTER_NORMAL(
      material_index,
      EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, float2(0.0), float2(0.0), float4(tex_gradients, 0.0, 0.0)),
      normal, tangent, tangent_handedness);
}
public
float3 EE_EVALUATE_GLTF_RASTER_NORMAL(uint material_index, float2 tex_coord_0, float2 tex_coord_1, float3 normal,
                                      float3 tangent, float tangent_handedness, float tex_grad) {
  return EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coord_0, tex_coord_1, normal, tangent, tangent_handedness,
                                        float2(tex_grad));
}
public
float3 EE_EVALUATE_GLTF_RASTER_NORMAL(uint material_index, float2 tex_coord_0, float2 tex_coord_1, float3 normal,
                                      float3 tangent, float tangent_handedness) {
  return EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coord_0, tex_coord_1, normal, tangent, tangent_handedness,
                                        float2(0.0));
}
public
float3 EE_EVALUATE_GLTF_RASTER_NORMAL(uint material_index, float2 tex_coord_0, float2 tex_coord_1, float3 normal,
                                      float3 tangent) {
  return EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coord_0, tex_coord_1, normal, tangent, 1.0);
}
public
float3 EE_EVALUATE_GLTF_RASTER_CLEARCOAT_NORMAL(uint material_index, GltfTexCoords tex_coords, float3 normal,
                                                float3 tangent, float tangent_handedness);
public
float3 EE_EVALUATE_GLTF_RASTER_CLEARCOAT_NORMAL(uint material_index, float2 tex_coord_0, float2 tex_coord_1,
                                                float3 normal, float3 tangent, float tangent_handedness,
                                                float2 tex_gradients) {
  return EE_EVALUATE_GLTF_RASTER_CLEARCOAT_NORMAL(
      material_index,
      EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, float2(0.0), float2(0.0), float4(tex_gradients, 0.0, 0.0)),
      normal, tangent, tangent_handedness);
}
public
float3 EE_EVALUATE_GLTF_RASTER_CLEARCOAT_NORMAL(uint material_index, GltfTexCoords tex_coords, float3 normal,
                                                float3 tangent, float tangent_handedness) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  const float3 n = EE_GLTF_SAFE_NORMALIZE(normal, float3(0.0, 1.0, 0.0));
  if (EE_GLTF_HAS_TEXTURE(material.clearcoat_normal_texture)) {
    float3 normal_vector =
        EE_GLTF_SAMPLE_TEXTURE_SLOT(material.clearcoat_normal_texture, EE_GLTF_RASTER_TEXTURE_CLEARCOAT_NORMAL,
                                    tex_coords, float4(0.5, 0.5, 1.0, 1.0))
            .xyz;
    normal_vector = normal_vector * 2.0 - 1.0;
    normal_vector.xy *= material.clearcoat_normal_texture_scale;
    float3 t = tangent - n * dot(n, tangent);
    t = EE_GLTF_SAFE_NORMALIZE(t, EE_GLTF_FALLBACK_TANGENT(n));
    const float3 b = cross(n, t) * (tangent_handedness < 0.0 ? -1.0 : 1.0);
    return EE_GLTF_SAFE_NORMALIZE(mul(normal_vector, float3x3(t, b, n)), n);
  }
  return n;
}
public
float3 EE_GLTF_RASTER_COATED_EMISSION(GltfRasterMaterial surface, float3 clearcoat_normal, float3 outgoing_direction) {
  const float cosine =
      clamp(abs(dot(EE_GLTF_SAFE_NORMALIZE(outgoing_direction, clearcoat_normal), clearcoat_normal)), 0.0, 1.0);
  const float fresnel = 0.04 + 0.96 * pow(max(1.0 - cosine, 0.0), 5.0);
  return surface.emissive * max(1.0 - clamp(surface.clearcoat, 0.0, 1.0) * fresnel, 0.0);
}
public
float3 EE_GLTF_RASTER_COATED_EMISSION(uint material_index, GltfRasterMaterial surface, float2 tex_coord_0,
                                      float2 tex_coord_1, float3 normal, float3 tangent, float tangent_handedness,
                                      float facing_sign, float3 outgoing_direction) {
  float3 clearcoat_normal = EE_EVALUATE_GLTF_RASTER_CLEARCOAT_NORMAL(material_index, tex_coord_0, tex_coord_1, normal,
                                                                     tangent, tangent_handedness, float2(0.0));
  clearcoat_normal = EE_GLTF_SAFE_NORMALIZE(facing_sign * clearcoat_normal, float3(0.0, 1.0, 0.0));
  return EE_GLTF_RASTER_COATED_EMISSION(surface, clearcoat_normal, outgoing_direction);
}
public
float3 EE_GLTF_RASTER_COATED_EMISSION(uint material_index, GltfRasterMaterial surface, float2 tex_coord_0,
                                      float2 tex_coord_1, float2 tex_coord_2, float2 tex_coord_3, float3 normal,
                                      float3 tangent, float tangent_handedness, float facing_sign,
                                      float3 outgoing_direction) {
  float3 clearcoat_normal = EE_EVALUATE_GLTF_RASTER_CLEARCOAT_NORMAL(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, float4(0.0)), normal,
      tangent, tangent_handedness);
  clearcoat_normal = EE_GLTF_SAFE_NORMALIZE(facing_sign * clearcoat_normal, float3(0.0, 1.0, 0.0));
  return EE_GLTF_RASTER_COATED_EMISSION(surface, clearcoat_normal, outgoing_direction);
}
public
float EE_GLTF_RASTER_OPACITY(GltfRasterMaterial surface) {
  const float alpha = surface.alpha_mode == EE_GLTF_ALPHA_MODE_OPAQUE ? 1.0 : clamp(surface.base_color.a, 0.0, 1.0);
  const float transmission = clamp(max(surface.transmission, surface.diffuse_transmission_factor), 0.0, 1.0);
  return alpha * (1.0 - transmission);
}
public
bool EE_GLTF_RASTER_SHOULD_DISCARD(GltfRasterMaterial surface) {
  if (surface.alpha_mode == EE_GLTF_ALPHA_MODE_OPAQUE) {
    return false;
  }
  if (surface.alpha_mode == EE_GLTF_ALPHA_MODE_MASK) {
    return surface.base_color.a < surface.alpha_cutoff;
  }
  return EE_GLTF_RASTER_OPACITY(surface) <= 0.0;
}
public
float3 EE_GLTF_RASTER_FRESNEL(const float3 f0, const float3 f90, const float cos_theta) {
  return f0 + (f90 - f0) * pow(max(1.0 - clamp(cos_theta, 0.0, 1.0), 0.0), 5.0);
}
public
float EE_GLTF_RASTER_BASE_COLOR_ALPHA(uint material_index, GltfTexCoords tex_coords, float vertex_alpha) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  float base_color_alpha;
  if (material.pbr_model == EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS) {
    base_color_alpha = material.pbr_diffuse_factor.a;
    base_color_alpha *= EE_GLTF_SAMPLE_TEXTURE_SLOT(material.pbr_diffuse_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR,
                                                    tex_coords, float4(1.0))
                            .a;
  } else {
    base_color_alpha = material.pbr_base_color_factor.a;
    base_color_alpha *= EE_GLTF_SAMPLE_TEXTURE_SLOT(material.pbr_base_color_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR,
                                                    tex_coords, float4(1.0))
                            .a;
  }
  base_color_alpha *= vertex_alpha;
  return base_color_alpha;
}
public
bool EE_GLTF_RASTER_ALPHA_MASK_PASSES(uint material_index, GltfTexCoords tex_coords, float vertex_alpha) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  return material.alpha_mode != EE_GLTF_ALPHA_MODE_MASK ||
         EE_GLTF_RASTER_BASE_COLOR_ALPHA(material_index, tex_coords, vertex_alpha) >= material.alpha_cutoff;
}
public
float EE_GLTF_RASTER_BASE_COLOR_ALPHA_LOD0(uint material_index, GltfTexCoords tex_coords, float vertex_alpha) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  float base_color_alpha;
  if (material.pbr_model == EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS) {
    base_color_alpha = material.pbr_diffuse_factor.a;
    base_color_alpha *= EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(material.pbr_diffuse_texture,
                                                         EE_GLTF_RASTER_TEXTURE_BASE_COLOR, tex_coords, float4(1.0))
                            .a;
  } else {
    base_color_alpha = material.pbr_base_color_factor.a;
    base_color_alpha *= EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(material.pbr_base_color_texture,
                                                         EE_GLTF_RASTER_TEXTURE_BASE_COLOR, tex_coords, float4(1.0))
                            .a;
  }
  base_color_alpha *= vertex_alpha;
  return base_color_alpha;
}
public
bool EE_GLTF_RASTER_ALPHA_MASK_PASSES_LOD0(uint material_index, GltfTexCoords tex_coords, float vertex_alpha) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  return material.alpha_mode != EE_GLTF_ALPHA_MODE_MASK ||
         EE_GLTF_RASTER_BASE_COLOR_ALPHA_LOD0(material_index, tex_coords, vertex_alpha) >= material.alpha_cutoff;
}
public
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
public
float EE_GLTF_RASTER_OPACITY_LOD0(uint material_index, float2 tex_coord_0, float2 tex_coord_1, float vertex_alpha) {
  return EE_GLTF_RASTER_OPACITY_LOD0(material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1), vertex_alpha);
}
public
float EE_GLTF_RASTER_OPACITY_LOD0(uint material_index, float2 tex_coord_0, float2 tex_coord_1, float2 tex_coord_2,
                                  float2 tex_coord_3, float vertex_alpha) {
  return EE_GLTF_RASTER_OPACITY_LOD0(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, float4(0.0)),
      vertex_alpha);
}
public
float3 EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0(uint material_index, GltfTexCoords tex_coords, float3 vertex_color,
                                               float cos_theta, float segment_length, inout bool is_inside,
                                               float min_transmission) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  float3 base_color = material.pbr_base_color_factor.rgb * vertex_color;
  if (material.pbr_model == EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS) {
    base_color = material.pbr_diffuse_factor.rgb * vertex_color;
    base_color *= EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(material.pbr_diffuse_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR,
                                                   tex_coords, float4(1.0))
                      .rgb;
  } else {
    base_color *= EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(material.pbr_base_color_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR,
                                                   tex_coords, float4(1.0))
                      .rgb;
  }
  float specular_transmission = 0.0;
  specular_transmission = material.transmission_factor;
  specular_transmission *= EE_GLTF_SAMPLE_TEXTURE_LOD0(material.transmission_texture, tex_coords, float4(1.0)).r;
  float diffuse_transmission = 0.0;
  float3 diffuse_transmission_color = float3(1.0);
  diffuse_transmission = material.diffuse_transmission_factor;
  diffuse_transmission *= EE_GLTF_SAMPLE_TEXTURE_LOD0(material.diffuse_transmission_texture, tex_coords, float4(1.0)).a;
  diffuse_transmission_color = material.diffuse_transmission_color;
  diffuse_transmission_color *=
      EE_GLTF_SAMPLE_TEXTURE_LOD0(material.diffuse_transmission_color_texture, tex_coords, float4(1.0)).rgb;
  specular_transmission = clamp(specular_transmission, 0.0, 1.0);
  diffuse_transmission = clamp(diffuse_transmission, 0.0, 1.0);
  const float effective_diffuse_transmission = (1.0 - specular_transmission) * diffuse_transmission;
  if (max(specular_transmission, effective_diffuse_transmission) <= min_transmission) {
    return float3(0.0);
  }
  float ior = 1.5;
  ior = material.ior == 0.0 ? 0.0 : max(material.ior, 1.0);
  float ior_f0 = (ior - 1.0) / max(ior + 1.0, 0.000001);
  ior_f0 *= ior_f0;
  float specular_weight = 1.0;
  float3 specular_color = float3(1.0);
  specular_weight = material.specular_factor;
  specular_weight *= EE_GLTF_SAMPLE_TEXTURE_LOD0(material.specular_texture, tex_coords, float4(1.0)).a;
  specular_color = material.specular_color_factor;
  specular_color *= EE_GLTF_SAMPLE_TEXTURE_LOD0(material.specular_color_texture, tex_coords, float4(1.0)).rgb;
  specular_weight = clamp(specular_weight, 0.0, 1.0);
  const float3 specular_f0 =
      clamp(float3(ior_f0) * max(specular_color, float3(0.0)), float3(0.0), float3(1.0)) * specular_weight;
  const float3 fresnel = EE_GLTF_RASTER_FRESNEL(specular_f0, float3(specular_weight), cos_theta);
  const float remaining_energy = 1.0 - max(fresnel.r, max(fresnel.g, fresnel.b));
  float3 transmission = remaining_energy * (specular_transmission * base_color +
                                            effective_diffuse_transmission * diffuse_transmission_color);
  transmission = clamp(transmission, float3(0.0), float3(1.0));
  if (material.thickness_factor > 0.0) {
    if (is_inside) {
      const float3 absorption_coefficient =
          -log(max(material.attenuation_color, float3(0.001))) / max(material.attenuation_distance, 0.001);
      float3 scatter_coefficient = float3(0.0);
      scatter_coefficient = absorption_coefficient * EE_GLTF_MULTI_TO_SINGLE_SCATTER_ALBEDO(
                                                         max(material.multiscatter_color_factor, float3(0.0)));
      const float3 extinction = absorption_coefficient + scatter_coefficient;
      transmission *= exp(-segment_length * extinction);
      const float max_scatter = max(scatter_coefficient.x, max(scatter_coefficient.y, scatter_coefficient.z));
      if (max_scatter > 0.001) {
        const float max_extinction = max(extinction.x, max(extinction.y, extinction.z));
        transmission *= exp(-segment_length * max_extinction);
      }
    }
    is_inside = !is_inside;
  }
  float roughness = material.pbr_roughness_factor;
  float metallic = material.pbr_metallic_factor;
  const float4 metallic_roughness = EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(
      material.pbr_metallic_roughness_texture, EE_GLTF_RASTER_TEXTURE_METALLIC_ROUGHNESS, tex_coords, float4(1.0));
  roughness *= metallic_roughness.g;
  metallic *= metallic_roughness.b;
  const float roughness_effect = 1.0 - roughness * roughness;
  const float transmission_attenuation = (1.0 - metallic) * lerp(0.65, 1.0, roughness_effect);
  return transmission * transmission_attenuation;
}
public
float3 EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0(uint material_index, float2 tex_coord_0, float2 tex_coord_1,
                                               float3 vertex_color, float cos_theta, float segment_length,
                                               inout bool is_inside, float min_transmission) {
  return EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0(material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1),
                                                 vertex_color, cos_theta, segment_length, is_inside, min_transmission);
}
public
float3 EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0(uint material_index, float2 tex_coord_0, float2 tex_coord_1,
                                               float2 tex_coord_2, float2 tex_coord_3, float3 vertex_color,
                                               float cos_theta, float segment_length, inout bool is_inside,
                                               float min_transmission) {
  return EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, float4(0.0)),
      vertex_color, cos_theta, segment_length, is_inside, min_transmission);
}
