// @evoengine-dialect native
__exported import EvoEngine.CameraRayOutputs;
import EvoEngine.CameraRayConstants;
__exported import EvoEngine.CameraRayTraversal;
import EvoEngine.CameraRayTracingPayload;
import EvoEngine.EmissiveTriangleSampling;
import EvoEngine.GltfRayTracingBsdf;
import EvoEngine.Random;

struct CameraRayPushConstants {
  uint camera_index;
  uint frame_id;
  uint total_samples;
  uint frame_sample_size;
  uint shader_execution_reordering;
  uint max_directional_light_size;
};
[[vk::push_constant]] ConstantBuffer<CameraRayPushConstants> EE_CAMERA_CONSTANTS;
static CameraRayTracingPayload hit_value;

void EE_CAMERA_TRACE_SURFACE<T : ICameraRayTraversal>(inout T traversal, const float3 origin, const float3 direction,
                                                      const float min_distance, inout uint seed) {
  traversal.traceSurface(origin, direction, min_distance, seed, hit_value);
}
float3 EE_CAMERA_SHADOW_TRANSMISSION<T : ICameraRayTraversal>(inout T traversal, const float3 origin,
                                                              const float3 direction, const float max_distance,
                                                              inout uint seed, const bool initial_inside) {
  return traversal.shadowTransmission(origin, direction, max_distance, seed, initial_inside, hit_value);
}
import EvoEngine.CameraRayGeometry;
import EvoEngine.EmissiveTriangleSampling;
struct EE_CAMERA_DIRECTION_SAMPLE {
  float3 direction;
  float pdf;
};
struct EE_CAMERA_PRIMARY_RAY {
  float3 origin;
  float3 direction;
};
struct EE_CAMERA_SURFACE_HIT {
  float3 position;
  float3 shadow_position;
  float3 normal;
  float3 shading_normal;
  float3 geometric_normal;
  float3 tangent;
  float3 bitangent;
  float2 tex_coord_0;
  float2 tex_coord_1;
  float2 tex_coord_2;
  float2 tex_coord_3;
  float4 vertex_color;
  float4 tex_gradients;
  uint material_index;
  bool front_facing;
  GltfRasterMaterial surface;
  GltfRayTracingPbrMaterial pbr;
};
uint EE_CAMERA_RANDOM_STREAM(const uint sample_seed, const uint domain, const uint path_segment) {
  return EE_XXHASH32(uint3(sample_seed, domain, path_segment));
}
bool EE_CAMERA_IS_MATERIAL_DEBUG_VIEW(const uint view) {
  return view >= EE_CAMERA_DEBUG_MATERIAL_ID && view <= EE_CAMERA_DEBUG_IRIDESCENCE;
}
float3 EE_CAMERA_ENCODE_NORMAL(const float3 normal) {
  return clamp(normal * 0.5f + 0.5f, float3(0.0f), float3(1.0f));
}
float EE_CAMERA_ENCODE_PDF(const float pdf) {
  if (pdf == EE_CAMERA_DIRAC_PDF) {
    return 0.75f;
  }
  return pdf > 0.0f ? 0.05f + 0.45f * clamp(log2(1.0f + pdf) / 16.0f, 0.0f, 1.0f) : 0.0f;
}
float3 EE_CAMERA_MATERIAL_DEBUG_VALUE<let feature_mask : uint>(const uint view, const EE_CAMERA_SURFACE_HIT hit) {
  if (view == EE_CAMERA_DEBUG_MATERIAL_ID) {
    const uint hash = EE_XXHASH32(uint3(hit.material_index, 0x9e3779b9u, 0x85ebca6bu));
    return 0.2f + 0.8f * float3(float(hash & 255u), float((hash >> 8u) & 255u), float((hash >> 16u) & 255u)) / 255.0f;
  }
  if (view == EE_CAMERA_DEBUG_BASE_COLOR) {
    return hit.pbr.base_color;
  }
  if (view == EE_CAMERA_DEBUG_GEOMETRIC_NORMAL) {
    return EE_CAMERA_ENCODE_NORMAL(hit.geometric_normal);
  }
  if (view == EE_CAMERA_DEBUG_SHADING_NORMAL) {
    return EE_CAMERA_ENCODE_NORMAL(hit.pbr.normal);
  }
  if (view == EE_CAMERA_DEBUG_ROUGHNESS) {
    return float3(hit.surface.roughness, hit.pbr.roughness);
  }
  if (view == EE_CAMERA_DEBUG_METALLIC) {
    return float3(hit.pbr.metallic);
  }
  if (view == EE_CAMERA_DEBUG_SPECULAR_F0) {
    return clamp(hit.pbr.specular * hit.pbr.specular_f0, float3(0.0f), float3(1.0f));
  }
  if (view == EE_CAMERA_DEBUG_ALPHA_COVERAGE) {
    const float raw_alpha = clamp(hit.surface.base_color.a, 0.0f, 1.0f);
    const float transport_opacity = EE_GLTF_RASTER_OPACITY(hit.surface);
    const float accepted_coverage = hit.surface.alpha_mode == EE_GLTF_ALPHA_MODE_BLEND ? transport_opacity : 1.0f;
    return float3(raw_alpha, transport_opacity, accepted_coverage);
  }
  if (view == EE_CAMERA_DEBUG_TRANSMISSION) {
    const float combined = 1.0f - (1.0f - hit.pbr.transmission) * (1.0f - hit.pbr.diffuse_transmission_factor);
    return float3(hit.pbr.transmission, hit.pbr.diffuse_transmission_factor, combined);
  }
  if (view == EE_CAMERA_DEBUG_IRIDESCENCE) {
    return float3(hit.pbr.iridescence, clamp(hit.pbr.iridescence_thickness / 1200.0f, 0.0f, 1.0f),
                  clamp(hit.pbr.iridescence_ior / 3.0f, 0.0f, 1.0f));
  }
  if ((feature_mask & EE_GLTF_FEATURE_UNLIT) != 0u && EE_GLTF_MATERIALS[hit.material_index].unlit > 0) {
    return hit.surface.base_color.rgb;
  }
  return hit.pbr.emissive;
}
struct EE_CAMERA_VOLUME_MEDIUM {
  float3 extinction;
  float3 scatter_coefficient;
  float scatter_anisotropy;
};
float3 EE_CAMERA_SAFE_NORMALIZE(const float3 value, const float3 fallback) {
  const float length_squared = dot(value, value);
  return length_squared > 0.00000001f ? value * rsqrt(length_squared) : fallback;
}
float3 EE_CAMERA_POINT_OFFSET(const float3 position, const float3 v0_position, const float3 v1_position,
                              const float3 v2_position, const float3 v0_normal, const float3 v1_normal,
                              const float3 v2_normal, const float3 barycentrics) {
  float3 u = position - v0_position;
  float3 v = position - v1_position;
  float3 w = position - v2_position;
  u -= min(0.0f, dot(u, v0_normal)) * v0_normal;
  v -= min(0.0f, dot(v, v1_normal)) * v1_normal;
  w -= min(0.0f, dot(w, v2_normal)) * v2_normal;
  return position + u * barycentrics.x + v * barycentrics.y + w * barycentrics.z;
}
float3 EE_CAMERA_SAFE_OFFSET_RAY(const float3 world_position, const float3 offset_direction) {
  const float scale_value = 256.0f;
  const int3 scale_int = int3(scale_value * offset_direction);
  const float3 offset_position =
      float3(asfloat(asint(world_position.x) + (world_position.x < 0.0f ? -scale_int.x : scale_int.x)),
             asfloat(asint(world_position.y) + (world_position.y < 0.0f ? -scale_int.y : scale_int.y)),
             asfloat(asint(world_position.z) + (world_position.z < 0.0f ? -scale_int.z : scale_int.z)));
  const float origin = 1.0f / 32.0f;
  const float float_scale = 1.0f / 65536.0f;
  return float3(
      abs(world_position.x) < origin ? world_position.x + float_scale * offset_direction.x : offset_position.x,
      abs(world_position.y) < origin ? world_position.y + float_scale * offset_direction.y : offset_position.y,
      abs(world_position.z) < origin ? world_position.z + float_scale * offset_direction.z : offset_position.z);
}
float4 EE_CAMERA_TEXEL_DENSITY(const float4x4 model, const Vertex v0, const Vertex v1, const Vertex v2) {
  const float3 world_edge_1 = mul(float4(v1.position - v0.position, 0.0f), model).xyz;
  const float3 world_edge_2 = mul(float4(v2.position - v0.position, 0.0f), model).xyz;
  const float world_area = length(cross(world_edge_1, world_edge_2));
  const float2 uv0_edge_1 = v1.tex_coord - v0.tex_coord;
  const float2 uv0_edge_2 = v2.tex_coord - v0.tex_coord;
  const float2 uv1_edge_1 = v1.tex_coord_1 - v0.tex_coord_1;
  const float2 uv1_edge_2 = v2.tex_coord_1 - v0.tex_coord_1;
  const float2 uv2_edge_1 = v1.tex_coord_2 - v0.tex_coord_2;
  const float2 uv2_edge_2 = v2.tex_coord_2 - v0.tex_coord_2;
  const float2 uv3_edge_1 = v1.tex_coord_3 - v0.tex_coord_3;
  const float2 uv3_edge_2 = v2.tex_coord_3 - v0.tex_coord_3;
  const float4 uv_area = abs(float4(uv0_edge_1.x * uv0_edge_2.y - uv0_edge_2.x * uv0_edge_1.y,
                                    uv1_edge_1.x * uv1_edge_2.y - uv1_edge_2.x * uv1_edge_1.y,
                                    uv2_edge_1.x * uv2_edge_2.y - uv2_edge_2.x * uv2_edge_1.y,
                                    uv3_edge_1.x * uv3_edge_2.y - uv3_edge_2.x * uv3_edge_1.y));
  return sqrt(uv_area / max(world_area, 1e-20f));
}
float EE_CAMERA_RAY_SPREAD_ANGLE(const float resolution_y) {
  const Camera camera = EE_CAMERAS[EE_CAMERA_CONSTANTS.camera_index];
  return 2.0f * abs(camera.inverse_projection[1][1]) / max(resolution_y, 1.0f);
}
float EE_CAMERA_WORLD_FOOTPRINT(const float ray_cone_width, const float hit_t, const float3 geometric_normal,
                                const float3 ray_direction, const float ray_spread_angle) {
  return (ray_cone_width + hit_t * ray_spread_angle) / max(abs(dot(geometric_normal, -ray_direction)), 1e-3f);
}
float4 EE_CAMERA_TEXTURE_GRADIENTS(const float ray_cone_width, const float hit_t, const float3 geometric_normal,
                                   const float3 ray_direction, const float4x4 model, const Vertex v0, const Vertex v1,
                                   const Vertex v2, const float ray_spread_angle) {
  return EE_CAMERA_WORLD_FOOTPRINT(ray_cone_width, hit_t, geometric_normal, ray_direction, ray_spread_angle) *
         EE_CAMERA_TEXEL_DENSITY(model, v0, v1, v2);
}
float3 EE_CAMERA_SANITIZE_RADIANCE(const float3 value) {
  return float3(value.x >= 0.0f && value.x < 3.402823466e+38f ? value.x : 0.0f,
                value.y >= 0.0f && value.y < 3.402823466e+38f ? value.y : 0.0f,
                value.z >= 0.0f && value.z < 3.402823466e+38f ? value.z : 0.0f);
}
bool EE_CAMERA_HAS_NONFINITE_RADIANCE(const float3 radiance) {
  return any(isnan(radiance)) || any(isinf(radiance));
}
float3 EE_CAMERA_REJECT_INVALID_RADIANCE(const float3 radiance, inout uint invalid_rejections) {
  if (EE_CAMERA_HAS_NONFINITE_RADIANCE(radiance)) {
    invalid_rejections += 1u;
    return float3(0.0f);
  }
  return max(radiance, float3(0.0f));
}
float3 EE_CAMERA_APPLY_FIREFLY_CLAMP(const Camera camera, const float3 radiance, inout uint firefly_clamps) {
  const float threshold = max(camera.firefly_clamp_threshold, 0.0f);
  const float luminance = dot(radiance, float3(1.0f / 3.0f));
  if (luminance > threshold) {
    firefly_clamps += 1u;
    return radiance * (threshold / luminance);
  }
  return radiance;
}
float EE_CAMERA_PACK_SAMPLE_DIAGNOSTICS(const uint invalid_rejections, const uint firefly_clamps) {
  return float(min(invalid_rejections, 65535u)) + float(min(firefly_clamps, 65535u)) / 65536.0f;
}
float EE_CAMERA_LUMINANCE(const float3 radiance) {
  return dot(radiance, float3(0.2126f, 0.7152f, 0.0722f));
}
float EE_CAMERA_RELATIVE_LUMINANCE_DELTA(const float3 previous_radiance, const float3 current_radiance) {
  const float previous_luminance = EE_CAMERA_LUMINANCE(previous_radiance);
  const float current_luminance = EE_CAMERA_LUMINANCE(current_radiance);
  return abs(current_luminance - previous_luminance) / max(max(previous_luminance, current_luminance), 1e-3f);
}
float3 EE_CAMERA_SAMPLE_CUBEMAP_RADIANCE(const int cubemap_index, const float3 direction, const float lod) {
  float3 environment_color =
      EE_CUBEMAPS[cubemap_index].SampleLevel(normalize(EE_ENVIRONMENT_LOCAL_DIRECTION(direction)), lod).rgb;
  if (EE_ENVIRONMENT.gamma != 1.0f) {
    environment_color = pow(max(environment_color, float3(0.0f)), float3(1.0f / max(EE_ENVIRONMENT.gamma, 0.001f)));
  }
  return max(environment_color, float3(0.0f));
}
float2 EE_CAMERA_ENVIRONMENT_SPHERICAL_UV(const float3 direction) {
  const float3 dir = EE_CAMERA_SAFE_NORMALIZE(direction, float3(0.0f, 1.0f, 0.0f));
  return float2(atan2(dir.z, dir.x) / (2.0f * EE_CAMERA_PI) + 0.5f,
                asin(clamp(dir.y, -1.0f, 1.0f)) / EE_CAMERA_PI + 0.5f);
}
float3 EE_CAMERA_ENVIRONMENT_SPHERICAL_DIRECTION(const float2 uv) {
  const float azimuth = (uv.x - 0.5f) * 2.0f * EE_CAMERA_PI;
  const float elevation = (uv.y - 0.5f) * EE_CAMERA_PI;
  const float cos_elevation = cos(elevation);
  return float3(cos(azimuth) * cos_elevation, sin(elevation), sin(azimuth) * cos_elevation);
}
int EE_CAMERA_ENVIRONMENT_PDF_TEXTURE_INDEX() {
  return int(round(EE_ENVIRONMENT.environment_pdf_texture_index));
}
float3 EE_CAMERA_BACKGROUND_RADIANCE(const float3 ray_direction) {
  const Camera camera = EE_CAMERAS[EE_CAMERA_CONSTANTS.camera_index];
  if (camera.background_source == 1) {
    return max(camera.clear_color.xyz, float3(0.0f)) * max(camera.clear_color.w, 0.0f);
  }
  return EE_CAMERA_SAMPLE_CUBEMAP_RADIANCE(camera.skybox_tex_index, ray_direction, 0.0f) *
         max(camera.clear_color.w, 0.0f);
}
int EE_CAMERA_ENVIRONMENT_CUBEMAP_INDEX() {
  return int(round(EE_ENVIRONMENT.environment_cubemap_index));
}
int2 EE_CAMERA_TEXTURE_SIZE(const int texture_index) {
  uint width;
  uint height;
  uint mip_count;
  EE_TEXTURE_2DS[texture_index].GetDimensions(0, width, height, mip_count);
  return int2(width, height);
}
float4 EE_CAMERA_TEXEL_FETCH(const int texture_index, const int2 texel) {
  return EE_TEXTURE_2DS[texture_index].Load(int3(texel, 0));
}
float EE_CAMERA_ENVIRONMENT_MAP_PDF(const float3 direction) {
  const int pdf_texture_index = EE_CAMERA_ENVIRONMENT_PDF_TEXTURE_INDEX();
  if (pdf_texture_index < 0) {
    return 1.0f / (4.0f * EE_CAMERA_PI);
  }
  const int2 texture_size = EE_CAMERA_TEXTURE_SIZE(pdf_texture_index);
  if (texture_size.x <= 0 || texture_size.y <= 0) {
    return 1.0f / (4.0f * EE_CAMERA_PI);
  }
  float2 uv = EE_CAMERA_ENVIRONMENT_SPHERICAL_UV(EE_ENVIRONMENT_LOCAL_DIRECTION(direction));
  uv.x = frac(uv.x);
  const int2 texel = clamp(int2(uv * float2(texture_size)), int2(0), texture_size - int2(1));
  return max(EE_CAMERA_TEXEL_FETCH(pdf_texture_index, texel).b, 0.0f);
}
float3 EE_CAMERA_PATH_ENVIRONMENT_RADIANCE(const float3 ray_direction) {
  const Camera camera = EE_CAMERAS[EE_CAMERA_CONSTANTS.camera_index];
  if (EE_ENVIRONMENT.background_color.w == 1.0f) {
    return EE_CAMERA_SANITIZE_RADIANCE(max(EE_ENVIRONMENT.background_color.rgb, float3(0.0f)));
  }
  const int environment_cubemap_index = EE_CAMERA_ENVIRONMENT_CUBEMAP_INDEX();
  const int cubemap_index = environment_cubemap_index >= 0 ? environment_cubemap_index : camera.skybox_tex_index;
  return EE_CAMERA_SANITIZE_RADIANCE(EE_CAMERA_SAMPLE_CUBEMAP_RADIANCE(cubemap_index, ray_direction, 0.0f));
}
float EE_CAMERA_PATH_ENVIRONMENT_PDF(const float3 light_direction) {
  if (EE_ENVIRONMENT.environment_type != EE_CAMERA_ENVIRONMENT_TYPE_COLOR) {
    return EE_CAMERA_ENVIRONMENT_MAP_PDF(light_direction);
  }
  return 1.0f / (4.0f * EE_CAMERA_PI);
}
bool EE_CAMERA_HAS_ENVIRONMENT_LIGHTING() {
  if (EE_ENVIRONMENT.diffuse_sky_intensity <= 0.0f && EE_ENVIRONMENT.global_reflection_intensity <= 0.0f) {
    return false;
  }
  if (EE_ENVIRONMENT.background_color.w == 1.0f) {
    return dot(max(EE_ENVIRONMENT.background_color.rgb, float3(0.0f)), float3(1.0f)) > 0.0f;
  }
  return true;
}
float EE_CAMERA_ENVIRONMENT_INTENSITY_FOR_EVENT(const int event_type) {
  return (event_type & EE_GLTF_RT_BSDF_EVENT_DIFFUSE) != 0 ? max(EE_ENVIRONMENT.diffuse_sky_intensity, 0.0f)
                                                           : max(EE_ENVIRONMENT.global_reflection_intensity, 0.0f);
}
float EE_CAMERA_BALANCE_HEURISTIC(const float sampled_pdf, const float other_pdf) {
  if (sampled_pdf == EE_CAMERA_DIRAC_PDF) {
    return 1.0f;
  }
  const float safe_sampled_pdf = max(sampled_pdf, 0.0f);
  const float safe_other_pdf = max(other_pdf, 0.0f);
  const float pdf_sum = safe_sampled_pdf + safe_other_pdf;
  return pdf_sum > 0.0f ? safe_sampled_pdf / pdf_sum : 0.0f;
}
float EE_CAMERA_CONTRIBUTION_MIS_WEIGHT(const Camera camera, const float sampled_pdf, const float other_pdf) {
  return EE_CAMERA_BALANCE_HEURISTIC(sampled_pdf, other_pdf);
}
float EE_CAMERA_ENVIRONMENT_PDF(const float3 light_direction) {
  return EE_CAMERA_PATH_ENVIRONMENT_PDF(light_direction);
}
float EE_CAMERA_ENVIRONMENT_HIT_MIS_WEIGHT(const Camera camera, const float bsdf_pdf, const float environment_pdf,
                                           const float environment_weight) {
  if (bsdf_pdf == EE_CAMERA_DIRAC_PDF) {
    return 1.0f;
  }
  const float weighted_environment_pdf = max(environment_pdf, 0.0f) * max(environment_weight, 0.0f);
  const float safe_bsdf_pdf = max(bsdf_pdf, 0.0f);
  const float pdf_sum = safe_bsdf_pdf + weighted_environment_pdf;
  return pdf_sum > 0.0f ? safe_bsdf_pdf / pdf_sum : 1.0f;
}
float3x3 EE_CAMERA_TANGENT_SPACE(const float3 normal) {
  const float3 n = EE_CAMERA_SAFE_NORMALIZE(normal, float3(0.0f, 0.0f, 1.0f));
  if (n.z < -0.99998796f) {
    return float3x3(float3(0.0f, -1.0f, 0.0f), float3(-1.0f, 0.0f, 0.0f), n);
  }
  const float a = 1.0f / (1.0f + n.z);
  const float b = -n.x * n.y * a;
  const float3 tangent = float3(1.0f - n.x * n.x * a, b, -n.x);
  const float3 bitangent = float3(b, 1.0f - n.y * n.y * a, -n.y);
  return float3x3(tangent, bitangent, n);
}
float3 EE_CAMERA_TANGENT_TO_WORLD(const float3 normal, const float3 local_direction) {
  const float3x3 basis = EE_CAMERA_TANGENT_SPACE(normal);
  return mul(local_direction, basis);
}
EE_CAMERA_DIRECTION_SAMPLE EE_CAMERA_SAMPLE_SPHERE(inout uint seed) {
  EE_CAMERA_DIRECTION_SAMPLE ray_sample;
  const float z = 1.0f - 2.0f * EE_PCG_RANDOM(seed);
  const float radius = sqrt(max(0.0f, 1.0f - z * z));
  const float phi = 2.0f * EE_CAMERA_PI * EE_PCG_RANDOM(seed);
  ray_sample.direction = float3(cos(phi) * radius, z, sin(phi) * radius);
  ray_sample.pdf = 1.0f / (4.0f * EE_CAMERA_PI);
  return ray_sample;
}
int EE_CAMERA_FIND_ENVIRONMENT_MARGINAL_ROW(const int pdf_texture_index, const int2 texture_size,
                                            const float sample_value) {
  int low = 0;
  int high = texture_size.y - 1;
  for (int i = 0; i < 16; ++i) {
    if (low >= high) {
      break;
    }
    const int mid = (low + high) / 2;
    const float cdf = EE_CAMERA_TEXEL_FETCH(pdf_texture_index, int2(texture_size.x - 1, mid)).g;
    if (sample_value < cdf) {
      high = mid;
    } else {
      low = mid + 1;
    }
  }
  return low;
}
int EE_CAMERA_FIND_ENVIRONMENT_CONDITIONAL_COLUMN(const int pdf_texture_index, const int2 texture_size, const int row,
                                                  const float sample_value) {
  int low = 0;
  int high = texture_size.x - 1;
  for (int i = 0; i < 16; ++i) {
    if (low >= high) {
      break;
    }
    const int mid = (low + high) / 2;
    const float cdf = EE_CAMERA_TEXEL_FETCH(pdf_texture_index, int2(mid, row)).r;
    if (sample_value < cdf) {
      high = mid;
    } else {
      low = mid + 1;
    }
  }
  return low;
}
EE_CAMERA_DIRECTION_SAMPLE EE_CAMERA_SAMPLE_ENVIRONMENT_MAP(inout uint seed) {
  const int pdf_texture_index = EE_CAMERA_ENVIRONMENT_PDF_TEXTURE_INDEX();
  if (pdf_texture_index < 0) {
    return EE_CAMERA_SAMPLE_SPHERE(seed);
  }
  const int2 texture_size = EE_CAMERA_TEXTURE_SIZE(pdf_texture_index);
  if (texture_size.x <= 0 || texture_size.y <= 0) {
    return EE_CAMERA_SAMPLE_SPHERE(seed);
  }
  const float marginal_sample = min(EE_PCG_RANDOM(seed), 0.99999994f);
  const float conditional_sample = min(EE_PCG_RANDOM(seed), 0.99999994f);
  const int row = EE_CAMERA_FIND_ENVIRONMENT_MARGINAL_ROW(pdf_texture_index, texture_size, marginal_sample);
  const int column =
      EE_CAMERA_FIND_ENVIRONMENT_CONDITIONAL_COLUMN(pdf_texture_index, texture_size, row, conditional_sample);
  const float marginal_previous =
      row > 0 ? EE_CAMERA_TEXEL_FETCH(pdf_texture_index, int2(texture_size.x - 1, row - 1)).g : 0.0f;
  const float marginal_current = EE_CAMERA_TEXEL_FETCH(pdf_texture_index, int2(texture_size.x - 1, row)).g;
  const float conditional_previous =
      column > 0 ? EE_CAMERA_TEXEL_FETCH(pdf_texture_index, int2(column - 1, row)).r : 0.0f;
  const float conditional_current = EE_CAMERA_TEXEL_FETCH(pdf_texture_index, int2(column, row)).r;
  const float marginal_probability = marginal_current - marginal_previous;
  const float conditional_probability = conditional_current - conditional_previous;
  const float row_fraction = marginal_probability > 0.0f
                                 ? clamp((marginal_sample - marginal_previous) / marginal_probability, 0.0f, 1.0f)
                                 : 0.0f;
  const float column_fraction =
      conditional_probability > 0.0f
          ? clamp((conditional_sample - conditional_previous) / conditional_probability, 0.0f, 1.0f)
          : 0.0f;
  const float elevation_0 = (float(row) / float(texture_size.y) - 0.5f) * EE_CAMERA_PI;
  const float elevation_1 = (float(row + 1) / float(texture_size.y) - 0.5f) * EE_CAMERA_PI;
  const float sin_elevation = lerp(sin(elevation_0), sin(elevation_1), row_fraction);
  const float2 uv = float2((float(column) + column_fraction) / float(texture_size.x),
                           asin(clamp(sin_elevation, -1.0f, 1.0f)) / EE_CAMERA_PI + 0.5f);
  EE_CAMERA_DIRECTION_SAMPLE ray_sample;
  ray_sample.direction = EE_CAMERA_ENVIRONMENT_SPHERICAL_DIRECTION(uv);
  ray_sample.pdf = max(EE_CAMERA_TEXEL_FETCH(pdf_texture_index, int2(column, row)).b, 0.0f);
  return ray_sample;
}
float EE_CAMERA_PATH_ENVIRONMENT_HIT_PDF(const float3 light_direction) {
  if (EE_ENVIRONMENT.background_color.w != 1.0f) {
    return EE_CAMERA_ENVIRONMENT_MAP_PDF(light_direction);
  }
  return 1.0f / (4.0f * EE_CAMERA_PI);
}
EE_CAMERA_DIRECTION_SAMPLE EE_CAMERA_SAMPLE_PATH_ENVIRONMENT(inout uint seed) {
  EE_CAMERA_DIRECTION_SAMPLE ray_sample = EE_CAMERA_SAMPLE_ENVIRONMENT_MAP(seed);
  ray_sample.direction = EE_ENVIRONMENT_WORLD_DIRECTION(ray_sample.direction);
  return ray_sample;
}
float3 EE_CAMERA_EVALUATE_DIRECT_BSDF(const EE_CAMERA_SURFACE_HIT hit, const float3 view_direction,
                                      const float3 light_direction, const float3 light_radiance, const uint light_kind,
                                      const bool diffuse_indirect_path, const bool primary_surface, inout uint seed,
                                      out float bsdf_pdf) {
  GltfRayTracingBsdfEvaluateData eval_data;
  eval_data.k1 = view_direction;
  eval_data.k2 = light_direction;
  eval_data.xi = EE_PCG_RANDOM_3(seed);
  EE_GLTF_RT_BSDF_EVALUATE(eval_data, hit.pbr);
  bsdf_pdf = eval_data.pdf;
  const float indirect_scale = max(EE_RENDER_INFO.indirect_lighting_intensity, 0.0f);
  const float path_scale = diffuse_indirect_path ? indirect_scale : 1.0f;
  const float diffuse_scale =
      diffuse_indirect_path || !primary_surface || light_kind == EE_CAMERA_LIGHT_KIND_ENVIRONMENT ? indirect_scale
                                                                                                  : 1.0f;
  float3 bsdf = eval_data.bsdf_diffuse * diffuse_scale + eval_data.bsdf_glossy * path_scale;
  if (light_kind == EE_CAMERA_LIGHT_KIND_ENVIRONMENT) {
    bsdf = eval_data.bsdf_diffuse * diffuse_scale * max(EE_ENVIRONMENT.diffuse_sky_intensity, 0.0f) +
           eval_data.bsdf_glossy * path_scale * max(EE_ENVIRONMENT.global_reflection_intensity, 0.0f);
  }
  return EE_CAMERA_SANITIZE_RADIANCE(bsdf * max(light_radiance, float3(0.0f)));
}
void EE_CAMERA_RESET_PAYLOAD(const uint seed) {
  hit_value.shadow_transmission = float3(1.0f);
  hit_value.seed = seed;
  hit_value.barycentrics = float2(0.0f);
  hit_value.hit_t = EE_CAMERA_FAR(int(EE_CAMERA_CONSTANTS.camera_index));
  hit_value.instance_index = 0u;
  hit_value.shadow_previous_hit_t = 0.0f;
  hit_value.primitive_id = 0u;
  hit_value.type = EE_CAMERA_RAY_PAYLOAD_MISS;
  hit_value.shadow_is_inside = 0u;
  hit_value.camera_index = EE_CAMERA_CONSTANTS.camera_index;
}
struct EE_CAMERA_RADIANCE_SPLIT {
  float3 color;
  float intensity;
};
struct EE_CAMERA_GLTF_LIGHT {
  float3 direction;
  int type;
  float3 position;
  float radius;
  float3 color;
  float intensity;
  float angular_size_or_inv_range;
  float inner_angle;
  float outer_angle;
  float cast_shadow;
  float3 attenuation;
  float use_native_attenuation;
};
struct EE_CAMERA_LIGHT_CONTRIB {
  float3 incident_vector;
  float3 intensity;
  float distance;
  float pdf;
  float cast_shadow;
};
struct EE_CAMERA_DIRECT_LIGHT {
  float3 direction;
  float3 radiance_over_pdf;
  float distance;
  float pdf;
  float cast_shadow;
  uint kind;
};
struct EE_CAMERA_BOUNCE_SCRATCH {
  bool next_event_valid;
  float3 contribution;
  float3 shadow_ray_origin;
  float3 shadow_ray_direction;
  float shadow_ray_distance;
  float cast_shadow;
  float light_pdf;
  float bsdf_pdf;
  uint light_kind;
};
uint EE_CAMERA_EMISSIVE_TRIANGLE_COUNT() {
  return EE_EMISSIVE_TRIANGLE_COUNT();
}
bool EE_CAMERA_FIND_EMISSIVE_TRIANGLE(const uint instance_index, const uint primitive_id,
                                      out EmissiveTriangleInfo result) {
  return EE_FIND_EMISSIVE_TRIANGLE(instance_index, primitive_id, result);
}
void EE_CAMERA_SAMPLE_EMISSIVE_TRIANGLE<let feature_mask : uint>(const float3 shading_position,
                                                                 inout uint seed, out EE_CAMERA_DIRECT_LIGHT direct_light) {
  direct_light.direction = float3(0.0f, 1.0f, 0.0f);
  direct_light.radiance_over_pdf = float3(0.0f);
  direct_light.distance = 0.0f;
  direct_light.pdf = 0.0f;
  direct_light.cast_shadow = 1.0f;
  direct_light.kind = EE_CAMERA_LIGHT_KIND_EMISSIVE;
  if (EE_CAMERA_EMISSIVE_TRIANGLE_COUNT() == 0u) {
    return;
  }
  const float record_sample = EE_PCG_RANDOM(seed);
  const float2 barycentric_sample = EE_PCG_RANDOM_2(seed);
  EeEmissiveTriangleSample emissive_sample;
  if (!EE_SAMPLE_EMISSIVE_TRIANGLE_SPECIALIZED<feature_mask>(shading_position, record_sample, barycentric_sample,
                                                              false, emissive_sample)) {
    return;
  }
  direct_light.direction = emissive_sample.direction;
  direct_light.radiance_over_pdf = emissive_sample.radiance_over_pdf;
  direct_light.distance = emissive_sample.distance;
  direct_light.pdf = emissive_sample.solid_angle_pdf;
}
float EE_CAMERA_EMISSIVE_TRIANGLE_HIT_PDF(const EmissiveTriangleInfo record, const float3 previous_position,
                                          const float3 hit_position, const float3 hit_geometric_normal) {
  return EE_EMISSIVE_TRIANGLE_HIT_PDF(record, previous_position, hit_position, hit_geometric_normal);
}
struct EE_CAMERA_LIGHT_TECHNIQUE_PROBABILITIES {
  float light_weight;
  float environment_weight;
};
EE_CAMERA_RADIANCE_SPLIT EE_CAMERA_SPLIT_LIGHT_RADIANCE(const float3 radiance) {
  EE_CAMERA_RADIANCE_SPLIT split;
  split.intensity = max(max(radiance.x, radiance.y), radiance.z);
  split.color = split.intensity > EE_CAMERA_PDF_EPSILON ? radiance / split.intensity : float3(0.0f);
  return split;
}
float EE_CAMERA_REFERENCE_LIGHT_RADIUS(const float light_size) {
  return light_size > EE_CAMERA_NATIVE_DEFAULT_LIGHT_SIZE + EE_CAMERA_PDF_EPSILON ? light_size : 0.0f;
}
float EE_CAMERA_REFERENCE_INV_RANGE(const float range) {
  const float camera_far = EE_CAMERA_FAR(int(EE_CAMERA_CONSTANTS.camera_index));
  return range > EE_CAMERA_RAY_EPSILON && range < camera_far - EE_CAMERA_RAY_EPSILON ? 1.0f / range : 0.0f;
}
int EE_CAMERA_PUNCTUAL_LIGHT_COUNT() {
  return int(EE_RENDER_INFO.directional_light_size) + int(EE_RENDER_INFO.point_light_size) +
         int(EE_RENDER_INFO.spot_light_size);
}
EE_CAMERA_GLTF_LIGHT EE_CAMERA_DEFAULT_GLTF_LIGHT() {
  EE_CAMERA_GLTF_LIGHT light;
  light.direction = float3(0.0f, -1.0f, 0.0f);
  light.type = EE_CAMERA_LIGHT_TYPE_DIRECTIONAL;
  light.position = float3(0.0f);
  light.radius = 0.0f;
  light.color = float3(0.0f);
  light.intensity = 0.0f;
  light.angular_size_or_inv_range = 0.0f;
  light.inner_angle = 0.0f;
  light.outer_angle = 0.0f;
  light.cast_shadow = 0.0f;
  light.attenuation = float3(0.0f, 0.0f, 1.0f);
  light.use_native_attenuation = 0.0f;
  return light;
}
EE_CAMERA_GLTF_LIGHT EE_CAMERA_DIRECTIONAL_TO_GLTF_LIGHT(const DirectionalLight light) {
  EE_CAMERA_GLTF_LIGHT gltf_light = EE_CAMERA_DEFAULT_GLTF_LIGHT();
  const EE_CAMERA_RADIANCE_SPLIT radiance = EE_CAMERA_SPLIT_LIGHT_RADIANCE(max(light.diffuse.rgb, float3(0.0f)));
  gltf_light.direction = EE_CAMERA_SAFE_NORMALIZE(light.direction, float3(0.0f, -1.0f, 0.0f));
  gltf_light.type = EE_CAMERA_LIGHT_TYPE_DIRECTIONAL;
  gltf_light.color = radiance.color;
  gltf_light.intensity = radiance.intensity;
  gltf_light.radius = EE_CAMERA_REFERENCE_LIGHT_RADIUS(light.reserved_parameters.x);
  gltf_light.angular_size_or_inv_range = gltf_light.radius;
  gltf_light.cast_shadow = light.diffuse.w;
  return gltf_light;
}
EE_CAMERA_GLTF_LIGHT EE_CAMERA_POINT_TO_GLTF_LIGHT(const PointLight light) {
  EE_CAMERA_GLTF_LIGHT gltf_light = EE_CAMERA_DEFAULT_GLTF_LIGHT();
  const EE_CAMERA_RADIANCE_SPLIT radiance = EE_CAMERA_SPLIT_LIGHT_RADIANCE(max(light.diffuse.rgb, float3(0.0f)));
  gltf_light.type = EE_CAMERA_LIGHT_TYPE_POINT;
  gltf_light.position = light.position;
  gltf_light.radius = EE_CAMERA_REFERENCE_LIGHT_RADIUS(light.reserved_parameters.y);
  gltf_light.color = radiance.color;
  gltf_light.intensity = radiance.intensity;
  gltf_light.angular_size_or_inv_range = EE_CAMERA_REFERENCE_INV_RANGE(light.constant_linear_quadratic_far.w);
  gltf_light.cast_shadow = light.diffuse.w;
  gltf_light.attenuation = max(light.constant_linear_quadratic_far.xyz, float3(0.0f));
  gltf_light.use_native_attenuation = dot(gltf_light.attenuation, float3(1.0f)) > EE_CAMERA_PDF_EPSILON ? 1.0f : 0.0f;
  return gltf_light;
}
EE_CAMERA_GLTF_LIGHT EE_CAMERA_SPOT_TO_GLTF_LIGHT(const SpotLight light) {
  EE_CAMERA_GLTF_LIGHT gltf_light = EE_CAMERA_DEFAULT_GLTF_LIGHT();
  const EE_CAMERA_RADIANCE_SPLIT radiance = EE_CAMERA_SPLIT_LIGHT_RADIANCE(max(light.diffuse.rgb, float3(0.0f)));
  gltf_light.direction = EE_CAMERA_SAFE_NORMALIZE(light.direction, float3(0.0f, -1.0f, 0.0f));
  gltf_light.type = EE_CAMERA_LIGHT_TYPE_SPOT;
  gltf_light.position = light.position;
  gltf_light.radius = EE_CAMERA_REFERENCE_LIGHT_RADIUS(light.cutoff_outer_inner_size_bias.z);
  gltf_light.color = radiance.color;
  gltf_light.intensity = radiance.intensity;
  gltf_light.angular_size_or_inv_range = EE_CAMERA_REFERENCE_INV_RANGE(light.constant_linear_quadratic_far.w);
  gltf_light.inner_angle = acos(clamp(light.cutoff_outer_inner_size_bias.x, -1.0f, 1.0f));
  gltf_light.outer_angle = acos(clamp(light.cutoff_outer_inner_size_bias.y, -1.0f, 1.0f));
  gltf_light.cast_shadow = light.diffuse.w;
  gltf_light.attenuation = max(light.constant_linear_quadratic_far.xyz, float3(0.0f));
  gltf_light.use_native_attenuation = dot(gltf_light.attenuation, float3(1.0f)) > EE_CAMERA_PDF_EPSILON ? 1.0f : 0.0f;
  return gltf_light;
}
EE_CAMERA_GLTF_LIGHT EE_CAMERA_GET_GLTF_LIGHT(const int light_index) {
  const int directional_count = int(EE_RENDER_INFO.directional_light_size);
  const int point_count = int(EE_RENDER_INFO.point_light_size);
  if (light_index < directional_count) {
    const int directional_light_index =
        int(EE_CAMERA_CONSTANTS.camera_index) * int(EE_CAMERA_CONSTANTS.max_directional_light_size) + light_index;
    return EE_CAMERA_DIRECTIONAL_TO_GLTF_LIGHT(EE_DIRECTIONAL_LIGHTS[directional_light_index]);
  }
  const int point_index = light_index - directional_count;
  if (point_index < point_count) {
    return EE_CAMERA_POINT_TO_GLTF_LIGHT(EE_POINT_LIGHTS[point_index]);
  }
  const int spot_index = point_index - point_count;
  if (spot_index < int(EE_RENDER_INFO.spot_light_size)) {
    return EE_CAMERA_SPOT_TO_GLTF_LIGHT(EE_SPOT_LIGHTS[spot_index]);
  }
  return EE_CAMERA_DEFAULT_GLTF_LIGHT();
}
EE_CAMERA_LIGHT_TECHNIQUE_PROBABILITIES EE_CAMERA_GET_DIRECT_LIGHTING_TECHNIQUE_PROBABILITIES() {
  EE_CAMERA_LIGHT_TECHNIQUE_PROBABILITIES probabilities;
  probabilities.light_weight = EE_CAMERA_PUNCTUAL_LIGHT_COUNT() > 0 ? 0.5f : 0.0f;
  probabilities.environment_weight = EE_CAMERA_HAS_ENVIRONMENT_LIGHTING() ? 0.5f : 0.0f;
  const float total_weight = probabilities.light_weight + probabilities.environment_weight;
  if (total_weight > 0.0f) {
    probabilities.light_weight /= total_weight;
    probabilities.environment_weight /= total_weight;
  }
  return probabilities;
}
float EE_CAMERA_SPOT_ATTENUATION(const float3 point_to_light, const float3 spot_direction, const float outer_cone_cos,
                                 const float inner_cone_cos) {
  const float actual_cos = dot(EE_CAMERA_SAFE_NORMALIZE(spot_direction, float3(0.0f, -1.0f, 0.0f)),
                               EE_CAMERA_SAFE_NORMALIZE(-point_to_light, float3(0.0f, -1.0f, 0.0f)));
  if (actual_cos <= outer_cone_cos) {
    return 0.0f;
  }
  if (actual_cos >= inner_cone_cos) {
    return 1.0f;
  }
  const float angular_attenuation = (actual_cos - outer_cone_cos) / max(inner_cone_cos - outer_cone_cos, 0.001f);
  return angular_attenuation * angular_attenuation;
}
EE_CAMERA_LIGHT_CONTRIB EE_CAMERA_SINGLE_LIGHT_CONTRIBUTION(const EE_CAMERA_GLTF_LIGHT light, const float3 surface_pos,
                                                            const float3 surface_normal, const float2 random_value) {
  EE_CAMERA_LIGHT_CONTRIB contrib;
  contrib.incident_vector = float3(0.0f);
  contrib.intensity = float3(0.0f);
  contrib.distance = EE_CAMERA_MAX_TRACE_DISTANCE;
  contrib.pdf = EE_CAMERA_DIRAC_PDF;
  contrib.cast_shadow = light.cast_shadow;
  float irradiance = 0.0f;
  float one_minus_cos_half_angle = 0.0f;
  if (light.type == EE_CAMERA_LIGHT_TYPE_DIRECTIONAL) {
    if (dot(surface_normal, -light.direction) <= 0.0f) {
      return contrib;
    }
    contrib.incident_vector = light.direction;
    const float angular_size_squared = light.angular_size_or_inv_range * light.angular_size_or_inv_range;
    one_minus_cos_half_angle = (-1.0f / 384.0f * angular_size_squared + 0.125f) * angular_size_squared;
    irradiance = light.intensity;
  } else if (light.type == EE_CAMERA_LIGHT_TYPE_POINT || light.type == EE_CAMERA_LIGHT_TYPE_SPOT) {
    const float3 light_to_surface = surface_pos - light.position;
    const float distance = length(light_to_surface);
    if (distance <= EE_CAMERA_RAY_EPSILON) {
      return contrib;
    }
    const float reciprocal_distance = 1.0f / distance;
    contrib.distance = distance;
    contrib.incident_vector = light_to_surface * reciprocal_distance;
    float range_smoothing = 1.0f;
    if (light.angular_size_or_inv_range > 0.0f) {
      const float distance_over_range = distance * light.angular_size_or_inv_range;
      const float distance_over_range_2 = distance_over_range * distance_over_range;
      range_smoothing = clamp(1.0f - distance_over_range_2 * distance_over_range_2, 0.0f, 1.0f);
      if (range_smoothing <= 0.0f) {
        return contrib;
      }
    }
    float spotlight = 1.0f;
    if (light.type == EE_CAMERA_LIGHT_TYPE_SPOT) {
      spotlight = EE_CAMERA_SPOT_ATTENUATION(-contrib.incident_vector, light.direction, cos(light.outer_angle),
                                             cos(light.inner_angle));
      if (spotlight <= 0.0f) {
        return contrib;
      }
    }
    if (light.radius > 0.0f) {
      const float radius_over_distance = min(light.radius * reciprocal_distance, 1.0f);
      const float radius_over_distance_2_plus_1 = radius_over_distance * radius_over_distance + 1.0f;
      one_minus_cos_half_angle = radius_over_distance * radius_over_distance /
                                 (radius_over_distance_2_plus_1 + sqrt(radius_over_distance_2_plus_1));
    }
    if (light.use_native_attenuation > 0.5f) {
      const float native_attenuation =
          light.attenuation.x + light.attenuation.y * distance + light.attenuation.z * distance * distance;
      irradiance = light.intensity / max(native_attenuation, EE_CAMERA_PDF_EPSILON);
    } else if (light.radius > 0.0f) {
      irradiance =
          light.intensity / max(light.radius * light.radius, EE_CAMERA_PDF_EPSILON) * (2.0f * one_minus_cos_half_angle);
    } else {
      irradiance = light.intensity * reciprocal_distance * reciprocal_distance;
    }
    irradiance *= spotlight * range_smoothing;
  }
  contrib.intensity = irradiance * light.color;
  if (one_minus_cos_half_angle > 0.0f) {
    const float cone_z = 1.0f - random_value.y * one_minus_cos_half_angle;
    const float cone_sin = sqrt(max(0.0f, 1.0f - cone_z * cone_z));
    const float cone_phi = 2.0f * EE_CAMERA_PI * random_value.x;
    const float3 cone_direction = float3(cos(cone_phi) * cone_sin, sin(cone_phi) * cone_sin, cone_z);
    const float3 center_surface_to_light = -contrib.incident_vector;
    const float3 sampled_surface_to_light = EE_CAMERA_SAFE_NORMALIZE(
        EE_CAMERA_TANGENT_TO_WORLD(center_surface_to_light, cone_direction), center_surface_to_light);
    contrib.incident_vector = -sampled_surface_to_light;
    contrib.pdf = (1.0f / (2.0f * EE_CAMERA_PI)) / max(one_minus_cos_half_angle, 1e-10f);
  }
  return contrib;
}
void EE_CAMERA_SAMPLE_DIRECT_LIGHT(const EE_CAMERA_SURFACE_HIT hit, const float3 view_direction, inout uint seed,
                                   out EE_CAMERA_DIRECT_LIGHT direct_light) {
  direct_light.direction = hit.normal;
  direct_light.radiance_over_pdf = float3(0.0f);
  direct_light.distance = EE_CAMERA_MAX_TRACE_DISTANCE;
  direct_light.pdf = 0.0f;
  direct_light.cast_shadow = 0.0f;
  direct_light.kind = EE_CAMERA_LIGHT_KIND_NONE;
  const EE_CAMERA_LIGHT_TECHNIQUE_PROBABILITIES probabilities = EE_CAMERA_GET_DIRECT_LIGHTING_TECHNIQUE_PROBABILITIES();
  const float light_weight = probabilities.light_weight;
  const float environment_weight = probabilities.environment_weight;
  if (light_weight == 0.0f && environment_weight == 0.0f) {
    return;
  }
  float3 radiance = float3(0.0f);
  float environment_pdf = 0.0f;
  const bool sample_light = EE_PCG_RANDOM(seed) < light_weight;
  if (sample_light) {
    direct_light.kind = EE_CAMERA_LIGHT_KIND_PUNCTUAL;
    const int light_count = EE_CAMERA_PUNCTUAL_LIGHT_COUNT();
    const float selection_pdf = 1.0f / float(light_count);
    const int light_index = min(int(EE_PCG_RANDOM(seed) * float(light_count)), light_count - 1);
    const EE_CAMERA_GLTF_LIGHT light = EE_CAMERA_GET_GLTF_LIGHT(light_index);
    const EE_CAMERA_LIGHT_CONTRIB contrib =
        EE_CAMERA_SINGLE_LIGHT_CONTRIBUTION(light, hit.position, hit.normal, EE_PCG_RANDOM_2(seed));
    direct_light.direction = -contrib.incident_vector;
    direct_light.distance = contrib.distance;
    direct_light.cast_shadow = contrib.cast_shadow;
    radiance = contrib.intensity / max(selection_pdf * light_weight, EE_CAMERA_PDF_EPSILON);
    direct_light.pdf = contrib.pdf == EE_CAMERA_DIRAC_PDF ? EE_CAMERA_DIRAC_PDF : selection_pdf * contrib.pdf;
  }
  if (environment_weight > 0.0f && direct_light.pdf != EE_CAMERA_DIRAC_PDF) {
    if (!sample_light) {
      direct_light.kind = EE_CAMERA_LIGHT_KIND_ENVIRONMENT;
      const EE_CAMERA_DIRECTION_SAMPLE environment_sample = EE_CAMERA_SAMPLE_PATH_ENVIRONMENT(seed);
      direct_light.direction = environment_sample.direction;
      direct_light.distance = EE_CAMERA_MAX_TRACE_DISTANCE;
      direct_light.cast_shadow = 1.0f;
      environment_pdf = environment_sample.pdf;
      if (environment_pdf > 0.0f) {
        radiance = EE_CAMERA_PATH_ENVIRONMENT_RADIANCE(direct_light.direction) / (environment_pdf * environment_weight);
      }
    } else {
      environment_pdf = EE_CAMERA_ENVIRONMENT_PDF(direct_light.direction);
    }
  }
  float mis_weight = 1.0f;
  if (direct_light.pdf != EE_CAMERA_DIRAC_PDF) {
    const float pdf_sum = light_weight * direct_light.pdf + environment_weight * environment_pdf;
    if (pdf_sum > 0.0f) {
      mis_weight =
          sample_light ? light_weight * direct_light.pdf / pdf_sum : environment_weight * environment_pdf / pdf_sum;
    } else {
      mis_weight = 0.0f;
    }
    direct_light.pdf = pdf_sum;
  }
  direct_light.radiance_over_pdf = radiance * mis_weight;
}
EE_CAMERA_BOUNCE_SCRATCH EE_CAMERA_EMPTY_BOUNCE_SCRATCH() {
  EE_CAMERA_BOUNCE_SCRATCH bounce;
  bounce.next_event_valid = false;
  bounce.contribution = float3(0.0f);
  bounce.shadow_ray_origin = float3(0.0f);
  bounce.shadow_ray_direction = float3(0.0f, 1.0f, 0.0f);
  bounce.shadow_ray_distance = 0.0f;
  bounce.cast_shadow = 0.0f;
  bounce.light_pdf = 0.0f;
  bounce.bsdf_pdf = 0.0f;
  bounce.light_kind = EE_CAMERA_LIGHT_KIND_NONE;
  return bounce;
}
void EE_CAMERA_PREPARE_DIRECT_LIGHTING(const EE_CAMERA_SURFACE_HIT hit, const float3 view_direction,
                                       const float3 throughput, const bool diffuse_indirect_path,
                                       const bool primary_surface, inout uint light_seed, inout uint bsdf_seed,
                                       inout EE_CAMERA_BOUNCE_SCRATCH bounce) {
  EE_CAMERA_DIRECT_LIGHT direct_light;
  EE_CAMERA_SAMPLE_DIRECT_LIGHT(hit, view_direction, light_seed, direct_light);
  bounce.light_pdf = direct_light.pdf;
  bounce.light_kind = direct_light.kind;
  if (direct_light.pdf == 0.0f) {
    return;
  }
  if (dot(direct_light.direction, hit.shading_normal) <= 0.0f && hit.pbr.diffuse_transmission_factor <= 0.0f) {
    return;
  }
  const bool shadow_side_forward = dot(direct_light.direction, hit.shading_normal) > 0.0f;
  const float3 offset_direction = shadow_side_forward ? hit.geometric_normal : -hit.geometric_normal;
  const float3 shadow_base =
      shadow_side_forward && dot(hit.shadow_position - hit.position, hit.geometric_normal) >= -EE_CAMERA_RAY_EPSILON
          ? hit.shadow_position
          : hit.position;
  bounce.next_event_valid = true;
  bounce.shadow_ray_origin = EE_CAMERA_SAFE_OFFSET_RAY(shadow_base, offset_direction);
  bounce.shadow_ray_direction = direct_light.direction;
  bounce.shadow_ray_distance = max(direct_light.distance, 0.0f);
  bounce.cast_shadow = direct_light.cast_shadow;
  float bsdf_pdf = 0.0f;
  const float3 bsdf_radiance =
      EE_CAMERA_EVALUATE_DIRECT_BSDF(hit, view_direction, direct_light.direction, direct_light.radiance_over_pdf,
                                     direct_light.kind, diffuse_indirect_path, primary_surface, bsdf_seed, bsdf_pdf);
  bounce.bsdf_pdf = bsdf_pdf;
  const float mis_weight =
      EE_CAMERA_CONTRIBUTION_MIS_WEIGHT(EE_CAMERAS[EE_CAMERA_CONSTANTS.camera_index], direct_light.pdf, bsdf_pdf);
  const float3 contribution = EE_CAMERA_SANITIZE_RADIANCE(throughput * bsdf_radiance * mis_weight);
  if (max(contribution.x, max(contribution.y, contribution.z)) <= EE_CAMERA_PDF_EPSILON) {
    return;
  }
  bounce.contribution = contribution;
}
void EE_CAMERA_PREPARE_EMISSIVE_LIGHTING<let feature_mask : uint>(const EE_CAMERA_SURFACE_HIT hit,
                                                                  const float3 view_direction, const float3 throughput,
                                                                  const bool diffuse_indirect_path,
                                                                  const bool primary_surface, inout uint light_seed,
                                                                  inout uint bsdf_seed,
                                                                  inout EE_CAMERA_BOUNCE_SCRATCH bounce) {
  if (EE_CAMERA_EMISSIVE_TRIANGLE_COUNT() == 0u) {
    return;
  }
  EE_CAMERA_DIRECT_LIGHT direct_light;
  EE_CAMERA_SAMPLE_EMISSIVE_TRIANGLE<feature_mask>(hit.position, light_seed, direct_light);
  bounce.light_pdf = direct_light.pdf;
  bounce.light_kind = direct_light.kind;
  if (direct_light.pdf <= 0.0f ||
      (dot(direct_light.direction, hit.shading_normal) <= 0.0f && hit.pbr.diffuse_transmission_factor <= 0.0f)) {
    return;
  }
  const bool shadow_side_forward = dot(direct_light.direction, hit.shading_normal) > 0.0f;
  const float3 offset_direction = shadow_side_forward ? hit.geometric_normal : -hit.geometric_normal;
  const float3 shadow_base =
      shadow_side_forward && dot(hit.shadow_position - hit.position, hit.geometric_normal) >= -EE_CAMERA_RAY_EPSILON
          ? hit.shadow_position
          : hit.position;
  bounce.next_event_valid = true;
  bounce.shadow_ray_origin = EE_CAMERA_SAFE_OFFSET_RAY(shadow_base, offset_direction);
  bounce.shadow_ray_direction = direct_light.direction;
  const float origin_advance = max(dot(bounce.shadow_ray_origin - hit.position, direct_light.direction), 0.0f);
  bounce.shadow_ray_distance = max(direct_light.distance - origin_advance - EE_CAMERA_RAY_EPSILON, 0.0f);
  bounce.cast_shadow = 1.0f;
  float bsdf_pdf = 0.0f;
  const float3 bsdf_radiance =
      EE_CAMERA_EVALUATE_DIRECT_BSDF(hit, view_direction, direct_light.direction, direct_light.radiance_over_pdf,
                                     direct_light.kind, diffuse_indirect_path, primary_surface, bsdf_seed, bsdf_pdf);
  bounce.bsdf_pdf = bsdf_pdf;
  const float mis_weight =
      EE_CAMERA_CONTRIBUTION_MIS_WEIGHT(EE_CAMERAS[EE_CAMERA_CONSTANTS.camera_index], direct_light.pdf, bsdf_pdf);
  const float3 contribution = EE_CAMERA_SANITIZE_RADIANCE(throughput * bsdf_radiance * mis_weight);
  if (max(contribution.x, max(contribution.y, contribution.z)) <= EE_CAMERA_PDF_EPSILON) {
    bounce.next_event_valid = false;
    return;
  }
  bounce.contribution = contribution;
}
float3 EE_CAMERA_RESOLVE_DIRECT_LIGHTING<T : ICameraRayTraversal>(inout T traversal,
                                                                 const EE_CAMERA_BOUNCE_SCRATCH bounce,
                                                                 inout uint seed) {
  if (!bounce.next_event_valid) {
    return float3(0.0f);
  }
  if (bounce.cast_shadow != 1.0f) {
    return bounce.contribution;
  }
  const float3 shadow_transmission = EE_CAMERA_SHADOW_TRANSMISSION(
      traversal, bounce.shadow_ray_origin, bounce.shadow_ray_direction, bounce.shadow_ray_distance, seed, false);
  if (max(max(shadow_transmission.x, shadow_transmission.y), shadow_transmission.z) <= EE_CAMERA_PDF_EPSILON) {
    return float3(0.0f);
  }
  return EE_CAMERA_SANITIZE_RADIANCE(bounce.contribution * shadow_transmission);
}
EE_CAMERA_VOLUME_MEDIUM EE_CAMERA_EMPTY_VOLUME_MEDIUM() {
  EE_CAMERA_VOLUME_MEDIUM medium;
  medium.extinction = float3(0.0f);
  medium.scatter_coefficient = float3(0.0f);
  medium.scatter_anisotropy = 0.0f;
  return medium;
}
EE_CAMERA_VOLUME_MEDIUM EE_CAMERA_MAKE_VOLUME_MEDIUM(const GltfRayTracingPbrMaterial pbr) {
  EE_CAMERA_VOLUME_MEDIUM medium;
  medium.extinction = EE_GLTF_RT_VOLUME_EXTINCTION_COEFFICIENT(pbr);
  medium.scatter_coefficient = max(pbr.scatter_coefficient, float3(0.0f));
  medium.scatter_anisotropy = clamp(pbr.scatter_anisotropy, -0.99f, 0.99f);
  return medium;
}
EE_CAMERA_VOLUME_MEDIUM EE_CAMERA_MAKE_VOLUME_MEDIUM_FROM_MATERIAL<let feature_mask : uint>(const uint material_index) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  EE_CAMERA_VOLUME_MEDIUM medium;
  if ((feature_mask & EE_GLTF_FEATURE_VOLUME) == 0u || material.attenuation_distance <= EE_CAMERA_PDF_EPSILON) {
    medium.extinction = float3(0.0f);
  } else {
    medium.extinction =
        -log(max(material.attenuation_color, float3(0.001f))) / max(material.attenuation_distance, 0.001f);
  }
  if ((feature_mask & EE_GLTF_FEATURE_VOLUME_SCATTER) != 0u) {
    medium.scatter_anisotropy = clamp(material.scatter_anisotropy, -0.99f, 0.99f);
    if (max(max(material.multiscatter_color_factor.x, material.multiscatter_color_factor.y),
            material.multiscatter_color_factor.z) > 0.0f) {
      medium.scatter_coefficient =
          medium.extinction * EE_GLTF_RT_MULTI_TO_SINGLE_SCATTER_ALBEDO(material.multiscatter_color_factor);
    } else {
      medium.scatter_coefficient = float3(0.0f);
    }
  } else {
    medium.scatter_coefficient = float3(0.0f);
    medium.scatter_anisotropy = 0.0f;
  }
  return medium;
}
bool EE_CAMERA_HAS_VOLUME_MEDIUM(const EE_CAMERA_VOLUME_MEDIUM medium) {
  return max(max(medium.extinction.x, medium.extinction.y), medium.extinction.z) > 0.0f ||
         max(max(medium.scatter_coefficient.x, medium.scatter_coefficient.y), medium.scatter_coefficient.z) > 0.0f;
}
float EE_CAMERA_HENYEY_GREENSTEIN_PDF(const float cos_theta, const float anisotropy) {
  const float g = clamp(anisotropy, -0.99f, 0.99f);
  const float g2 = g * g;
  const float denominator = max(1.0f + g2 - 2.0f * g * cos_theta, EE_CAMERA_PDF_EPSILON);
  return (1.0f - g2) / max(4.0f * EE_CAMERA_PI * denominator * sqrt(denominator), EE_CAMERA_PDF_EPSILON);
}
float3 EE_CAMERA_SAMPLE_HENYEY_GREENSTEIN(const float2 xi, const float anisotropy, const float3 wi) {
  const float g = clamp(anisotropy, -0.99f, 0.99f);
  float cos_theta;
  if (abs(g) < 1.0e-3f) {
    cos_theta = 1.0f - 2.0f * xi.x;
  } else {
    const float s = (1.0f - g * g) / max(1.0f - g + 2.0f * g * xi.x, EE_CAMERA_PDF_EPSILON);
    cos_theta = (1.0f + g * g - s * s) / (2.0f * g);
  }
  const float sin_theta = sqrt(max(0.0f, 1.0f - cos_theta * cos_theta));
  const float phi = 2.0f * EE_CAMERA_PI * xi.y;
  const float3 tangent_direction = float3(cos(phi) * sin_theta, sin(phi) * sin_theta, cos_theta);
  return EE_CAMERA_SAFE_NORMALIZE(
      EE_CAMERA_TANGENT_TO_WORLD(EE_CAMERA_SAFE_NORMALIZE(wi, float3(0.0f, 0.0f, 1.0f)), tangent_direction), wi);
}
float3 EE_CAMERA_VOLUME_SCATTER_NEE<T : ICameraRayTraversal>(
    inout T traversal, const EE_CAMERA_VOLUME_MEDIUM medium, const float3 scatter_position,
    const float3 wi_before_scatter, const float3 throughput, inout uint light_seed, inout uint shadow_seed) {
  EE_CAMERA_DIRECT_LIGHT direct_light;
  direct_light.direction = wi_before_scatter;
  direct_light.radiance_over_pdf = float3(0.0f);
  direct_light.distance = EE_CAMERA_MAX_TRACE_DISTANCE;
  direct_light.pdf = 0.0f;
  direct_light.cast_shadow = 0.0f;
  direct_light.kind = EE_CAMERA_LIGHT_KIND_NONE;
  const EE_CAMERA_LIGHT_TECHNIQUE_PROBABILITIES probabilities = EE_CAMERA_GET_DIRECT_LIGHTING_TECHNIQUE_PROBABILITIES();
  const float light_weight = probabilities.light_weight;
  const float environment_weight = probabilities.environment_weight;
  if (light_weight == 0.0f && environment_weight == 0.0f) {
    return float3(0.0f);
  }
  float3 radiance = float3(0.0f);
  float environment_pdf = 0.0f;
  const bool sample_light = EE_PCG_RANDOM(light_seed) < light_weight;
  if (sample_light) {
    direct_light.kind = EE_CAMERA_LIGHT_KIND_PUNCTUAL;
    const int light_count = EE_CAMERA_PUNCTUAL_LIGHT_COUNT();
    const float selection_pdf = 1.0f / float(light_count);
    const int light_index = min(int(EE_PCG_RANDOM(light_seed) * float(light_count)), light_count - 1);
    const EE_CAMERA_GLTF_LIGHT light = EE_CAMERA_GET_GLTF_LIGHT(light_index);
    const EE_CAMERA_LIGHT_CONTRIB contrib =
        EE_CAMERA_SINGLE_LIGHT_CONTRIBUTION(light, scatter_position, wi_before_scatter, EE_PCG_RANDOM_2(light_seed));
    direct_light.direction = -contrib.incident_vector;
    direct_light.distance = contrib.distance;
    direct_light.cast_shadow = contrib.cast_shadow;
    radiance = contrib.intensity / max(selection_pdf * light_weight, EE_CAMERA_PDF_EPSILON);
    direct_light.pdf = contrib.pdf == EE_CAMERA_DIRAC_PDF ? EE_CAMERA_DIRAC_PDF : selection_pdf * contrib.pdf;
  }
  if (environment_weight > 0.0f && direct_light.pdf != EE_CAMERA_DIRAC_PDF) {
    if (!sample_light) {
      direct_light.kind = EE_CAMERA_LIGHT_KIND_ENVIRONMENT;
      const EE_CAMERA_DIRECTION_SAMPLE environment_sample = EE_CAMERA_SAMPLE_PATH_ENVIRONMENT(light_seed);
      direct_light.direction = environment_sample.direction;
      direct_light.distance = EE_CAMERA_MAX_TRACE_DISTANCE;
      direct_light.cast_shadow = 1.0f;
      environment_pdf = environment_sample.pdf;
      if (environment_pdf > 0.0f) {
        radiance = EE_CAMERA_PATH_ENVIRONMENT_RADIANCE(direct_light.direction) *
                   max(EE_ENVIRONMENT.diffuse_sky_intensity, 0.0f) / (environment_pdf * environment_weight);
      }
    } else {
      environment_pdf = EE_CAMERA_ENVIRONMENT_PDF(direct_light.direction);
    }
  }
  float light_mis_weight = 1.0f;
  if (direct_light.pdf != EE_CAMERA_DIRAC_PDF) {
    const float pdf_sum = light_weight * direct_light.pdf + environment_weight * environment_pdf;
    light_mis_weight =
        pdf_sum > 0.0f
            ? (sample_light ? light_weight * direct_light.pdf : environment_weight * environment_pdf) / pdf_sum
            : 0.0f;
    direct_light.pdf = pdf_sum;
  }
  direct_light.radiance_over_pdf = radiance * light_mis_weight;
  if (max(direct_light.radiance_over_pdf.x, max(direct_light.radiance_over_pdf.y, direct_light.radiance_over_pdf.z)) <=
      EE_CAMERA_PDF_EPSILON) {
    return float3(0.0f);
  }
  if (direct_light.cast_shadow == 1.0f) {
    const float3 shadow_origin = scatter_position + direct_light.direction * EE_CAMERA_RAY_EPSILON;
    const float3 shadow_transmission = EE_CAMERA_SHADOW_TRANSMISSION(
        traversal, shadow_origin, direct_light.direction, direct_light.distance - EE_CAMERA_RAY_EPSILON, shadow_seed,
        true);
    if (max(max(shadow_transmission.x, shadow_transmission.y), shadow_transmission.z) <= EE_CAMERA_PDF_EPSILON) {
      return float3(0.0f);
    }
    direct_light.radiance_over_pdf *= shadow_transmission;
  }
  const float phase_pdf =
      EE_CAMERA_HENYEY_GREENSTEIN_PDF(dot(wi_before_scatter, direct_light.direction), medium.scatter_anisotropy);
  const float mis_weight = EE_CAMERA_BALANCE_HEURISTIC(direct_light.pdf, phase_pdf);
  return EE_CAMERA_SANITIZE_RADIANCE(throughput * direct_light.radiance_over_pdf * mis_weight * phase_pdf *
                                     max(EE_RENDER_INFO.indirect_lighting_intensity, 0.0f));
}
float3 EE_CAMERA_VOLUME_EMISSIVE_NEE<T : ICameraRayTraversal, let feature_mask : uint>(
    inout T traversal, const EE_CAMERA_VOLUME_MEDIUM medium, const float3 scatter_position,
    const float3 wi_before_scatter, const float3 throughput, inout uint light_seed, inout uint shadow_seed) {
  if (EE_CAMERA_EMISSIVE_TRIANGLE_COUNT() == 0u) {
    return float3(0.0f);
  }
  EE_CAMERA_DIRECT_LIGHT direct_light;
  EE_CAMERA_SAMPLE_EMISSIVE_TRIANGLE<feature_mask>(scatter_position, light_seed, direct_light);
  if (direct_light.pdf <= 0.0f ||
      max(direct_light.radiance_over_pdf.x, max(direct_light.radiance_over_pdf.y, direct_light.radiance_over_pdf.z)) <=
          EE_CAMERA_PDF_EPSILON) {
    return float3(0.0f);
  }
  const float3 shadow_origin = scatter_position + direct_light.direction * EE_CAMERA_RAY_EPSILON;
  const float3 shadow_transmission = EE_CAMERA_SHADOW_TRANSMISSION(
      traversal, shadow_origin, direct_light.direction,
      max(direct_light.distance - 2.0f * EE_CAMERA_RAY_EPSILON, 0.0f), shadow_seed, true);
  if (max(shadow_transmission.x, max(shadow_transmission.y, shadow_transmission.z)) <= EE_CAMERA_PDF_EPSILON) {
    return float3(0.0f);
  }
  const float phase_pdf =
      EE_CAMERA_HENYEY_GREENSTEIN_PDF(dot(wi_before_scatter, direct_light.direction), medium.scatter_anisotropy);
  const float mis_weight = EE_CAMERA_BALANCE_HEURISTIC(direct_light.pdf, phase_pdf);
  return EE_CAMERA_SANITIZE_RADIANCE(throughput * direct_light.radiance_over_pdf * shadow_transmission * mis_weight *
                                     phase_pdf * max(EE_RENDER_INFO.indirect_lighting_intensity, 0.0f));
}
bool EE_CAMERA_PROCESS_VOLUME_SEGMENT<T : ICameraRayTraversal, let feature_mask : uint>(
    inout T traversal, const float hit_distance, inout float3 ray_origin, inout float3 ray_direction,
    inout float3 throughput,
    inout float3 radiance, inout float last_sample_pdf, inout float ray_cone_width, const float ray_spread_angle,
    const EE_CAMERA_VOLUME_MEDIUM medium, inout uint scatter_seed, inout uint light_seed,
    inout uint direct_shadow_seed, inout uint emissive_seed, inout uint emissive_shadow_seed,
    inout uint scatter_bounces) {
  if (!EE_CAMERA_HAS_VOLUME_MEDIUM(medium)) {
    return false;
  }
  const float3 extinction = max(medium.extinction, float3(0.0f));
  const float3 scatter_coefficient = min(max(medium.scatter_coefficient, float3(0.0f)), extinction);
  const float max_scatter = max(scatter_coefficient.x, max(scatter_coefficient.y, scatter_coefficient.z));
  if (max_scatter > EE_CAMERA_VOLUME_MIN_SCATTER) {
    const float max_extinction = max(extinction.x, max(extinction.y, extinction.z));
    const float scatter_distance = -log(max(EE_PCG_RANDOM(scatter_seed), EE_CAMERA_VOLUME_RAND_FLOOR)) /
                                   max(max_extinction, EE_CAMERA_PDF_EPSILON);
    if (scatter_distance < hit_distance) {
      throughput *=
          clamp(float3(1.0f) - (extinction - scatter_coefficient) / max(max_extinction, EE_CAMERA_PDF_EPSILON),
                float3(0.0f), float3(1.0f));
      const float3 wi_before_scatter = ray_direction;
      ray_origin += ray_direction * scatter_distance;
      ray_cone_width += scatter_distance * ray_spread_angle;
      ray_direction = EE_CAMERA_SAMPLE_HENYEY_GREENSTEIN(EE_PCG_RANDOM_2(scatter_seed), medium.scatter_anisotropy,
                                                         wi_before_scatter);
      last_sample_pdf =
          EE_CAMERA_HENYEY_GREENSTEIN_PDF(dot(wi_before_scatter, ray_direction), medium.scatter_anisotropy);
      radiance += EE_CAMERA_VOLUME_SCATTER_NEE(traversal, medium, ray_origin, wi_before_scatter, throughput, light_seed,
                                               direct_shadow_seed);
      radiance += EE_CAMERA_VOLUME_EMISSIVE_NEE<T, feature_mask>(
          traversal, medium, ray_origin, wi_before_scatter, throughput, emissive_seed, emissive_shadow_seed);
      scatter_bounces += 1u;
      return true;
    }
    throughput *= exp(hit_distance * (float3(max_extinction) - extinction));
    return false;
  }
  throughput *= exp(-hit_distance * extinction);
  return false;
}
EE_CAMERA_SURFACE_HIT EE_CAMERA_RECONSTRUCT_SURFACE_HIT<let feature_mask : uint>(const bool is_inside,
                                                                                 const float3 ray_direction,
                                                                                 const float ray_cone_width,
                                                                                 const float ray_spread_angle) {
  EE_CAMERA_SURFACE_HIT hit;
  const Instance instance = EE_INSTANCES[hit_value.instance_index];
  const int triangle_offset = instance.triangle_offset + int(hit_value.primitive_id);
  const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
  const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
  const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];
  const float3 barycentrics = float3(1.0f - hit_value.barycentrics.x - hit_value.barycentrics.y,
                                     hit_value.barycentrics.x, hit_value.barycentrics.y);
  const float3 object_position =
      v0.position * barycentrics.x + v1.position * barycentrics.y + v2.position * barycentrics.z;
  const float3 object_normal = v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z;
  const float3 object_shading_normal = EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(object_normal, float3(0.0f, 1.0f, 0.0f));
  const float3 object_geometric_normal =
      EE_CAMERA_GEOMETRIC_NORMAL(v1.position - v0.position, v2.position - v0.position, object_shading_normal);
  const float2 tex_coord_0 =
      v0.tex_coord * barycentrics.x + v1.tex_coord * barycentrics.y + v2.tex_coord * barycentrics.z;
  const float2 tex_coord_1 =
      v0.tex_coord_1 * barycentrics.x + v1.tex_coord_1 * barycentrics.y + v2.tex_coord_1 * barycentrics.z;
  const float2 tex_coord_2 =
      v0.tex_coord_2 * barycentrics.x + v1.tex_coord_2 * barycentrics.y + v2.tex_coord_2 * barycentrics.z;
  const float2 tex_coord_3 =
      v0.tex_coord_3 * barycentrics.x + v1.tex_coord_3 * barycentrics.y + v2.tex_coord_3 * barycentrics.z;
  const float4 vertex_color = v0.color * barycentrics.x + v1.color * barycentrics.y + v2.color * barycentrics.z;
  const float3 object_tangent = v0.tangent * barycentrics.x + v1.tangent * barycentrics.y + v2.tangent * barycentrics.z;
  const float tangent_handedness = (v0.vertex_info3 < 0.0f ? -1.0f : 1.0f) * EE_TRANSFORM_HANDEDNESS(instance.model);
  hit.material_index = uint(instance.material_index);
  hit.position = mul(float4(object_position, 1.0f), instance.model).xyz;
  const float3x3 world_normal_matrix = EE_RT_WORLD_NORMAL_MATRIX(instance.model);
  const float3 unflipped_shading_normal =
      EE_RT_WORLD_NORMAL(world_normal_matrix, object_shading_normal, float3(0.0f, 1.0f, 0.0f));
  const float3 unflipped_geometric_normal =
      EE_RT_WORLD_NORMAL(world_normal_matrix, object_geometric_normal, unflipped_shading_normal);
  hit.front_facing = dot(unflipped_geometric_normal, ray_direction) < 0.0f;
  const float side_flip = hit.front_facing ? 1.0f : -1.0f;
  hit.geometric_normal = unflipped_geometric_normal * side_flip;
  const float3 v0_shadow_normal = v0.normal * side_flip;
  const float3 v1_shadow_normal = v1.normal * side_flip;
  const float3 v2_shadow_normal = v2.normal * side_flip;
  const float3 object_shadow_position =
      EE_CAMERA_POINT_OFFSET(object_position, v0.position, v1.position, v2.position, v0_shadow_normal, v1_shadow_normal,
                             v2_shadow_normal, barycentrics);
  hit.shadow_position = mul(float4(object_shadow_position, 1.0f), instance.model).xyz;
  hit.tex_gradients = EE_CAMERA_TEXTURE_GRADIENTS(ray_cone_width, hit_value.hit_t, hit.geometric_normal, ray_direction,
                                                  instance.model, v0, v1, v2, ray_spread_angle);
  hit.surface = EE_EVALUATE_GLTF_RASTER_SURFACE_SPECIALIZED<feature_mask>(
      hit.material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color, hit.tex_gradients);
  hit.normal = unflipped_shading_normal;
  hit.tangent = EE_RT_WORLD_TANGENT(instance.model, object_tangent, hit.normal);
  hit.bitangent =
      EE_CAMERA_SAFE_NORMALIZE(cross(hit.normal, hit.tangent) * tangent_handedness, float3(0.0f, 0.0f, 1.0f));
  if (dot(hit.normal, hit.geometric_normal) < 0.0f) {
    hit.normal = -hit.normal;
    hit.tangent = -hit.tangent;
    hit.bitangent = -hit.bitangent;
  }
  const float3 reflected_direction =
      reflect(EE_CAMERA_SAFE_NORMALIZE(ray_direction, -hit.geometric_normal), hit.normal);
  if (dot(reflected_direction, hit.geometric_normal) < 0.0f) {
    hit.normal = hit.geometric_normal;
    hit.tangent =
        EE_CAMERA_SAFE_NORMALIZE(hit.tangent - hit.normal * dot(hit.tangent, hit.normal), float3(1.0f, 0.0f, 0.0f));
    hit.bitangent =
        EE_CAMERA_SAFE_NORMALIZE(cross(hit.normal, hit.tangent) * tangent_handedness, float3(0.0f, 0.0f, 1.0f));
  }
  hit.shading_normal = hit.normal;
  hit.tex_coord_0 = tex_coord_0;
  hit.tex_coord_1 = tex_coord_1;
  hit.tex_coord_2 = tex_coord_2;
  hit.tex_coord_3 = tex_coord_3;
  hit.vertex_color = vertex_color;
  hit.pbr = EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL_SPECIALIZED<feature_mask>(
      hit.material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color, hit.surface, hit.normal,
      hit.tangent, hit.bitangent, hit.geometric_normal, is_inside, hit.tex_gradients);
  hit.normal = hit.pbr.normal;
  return hit;
}
float3 EE_CAMERA_OFFSET_RAY_ORIGIN(const EE_CAMERA_SURFACE_HIT hit, const float3 direction) {
  const float3 offset_direction =
      dot(direction, hit.geometric_normal) > 0.0f ? hit.geometric_normal : -hit.geometric_normal;
  return EE_CAMERA_SAFE_OFFSET_RAY(hit.position, offset_direction);
}
float3 EE_CAMERA_TRACE_PATH<T : ICameraRayTraversal, let feature_mask : uint>(
    inout T traversal, const uint sample_seed, float3 ray_origin, float3 ray_direction, const float ray_spread_angle,
    const uint debug_view, out float primary_hit_distance, inout CameraRayOutputDiagnostics ray_outputs) {
  const Camera camera = EE_CAMERAS[EE_CAMERA_CONSTANTS.camera_index];
  const uint max_depth = max(camera.bounce, 1u);
  float3 radiance = float3(0.0f);
  float3 debug_radiance = float3(0.0f);
  float3 throughput = float3(1.0f);
  float3 previous_scattering_position = ray_origin;
  float2 max_roughness = float2(0.0f);
  float last_sample_pdf = EE_CAMERA_DIRAC_PDF;
  float last_environment_intensity = 1.0f;
  bool diffuse_indirect_path = false;
  float ray_cone_width = 0.0f;
  bool is_inside = false;
  EE_CAMERA_VOLUME_MEDIUM volume_medium = EE_CAMERA_EMPTY_VOLUME_MEDIUM();
  uint scatter_bounces = 0u;
  uint path_surface_count = 0u;
  primary_hit_distance = EE_CAMERA_FAR(int(EE_CAMERA_CONSTANTS.camera_index));
  uint surface_depth = 0u;
  uint path_segment = 0u;
  while (surface_depth < max_depth) {
    const uint current_segment = path_segment++;
    uint surface_alpha_seed =
        EE_CAMERA_RANDOM_STREAM(sample_seed, EE_CAMERA_RANDOM_DOMAIN_SURFACE_ALPHA, current_segment);
    EE_CAMERA_RESET_PAYLOAD(surface_alpha_seed);
    ray_outputs.ray_count += 1u;
    EE_CAMERA_TRACE_SURFACE(traversal, ray_origin, ray_direction,
                            surface_depth == 0u ? 0.0f : EE_CAMERA_RAY_EPSILON, surface_alpha_seed);
    if (hit_value.type == EE_CAMERA_RAY_PAYLOAD_MISS) {
      const float3 miss_direction = EE_CAMERA_SAFE_NORMALIZE(ray_direction, float3(0.0f, 1.0f, 0.0f));
      if (surface_depth == 0u) {
        const float3 background_contribution =
            throughput * EE_CAMERA_BACKGROUND_RADIANCE(miss_direction);
        radiance += background_contribution;
        if (debug_view == EE_CAMERA_DEBUG_DIRECT_ENVIRONMENT) {
          debug_radiance += background_contribution;
        }
        break;
      }
      const float environment_pdf = EE_CAMERA_PATH_ENVIRONMENT_HIT_PDF(miss_direction);
      const float environment_weight = EE_CAMERA_GET_DIRECT_LIGHTING_TECHNIQUE_PROBABILITIES().environment_weight;
      const float mis_weight =
          EE_CAMERA_ENVIRONMENT_HIT_MIS_WEIGHT(camera, last_sample_pdf, environment_pdf, environment_weight);
      const float indirect_scale = diffuse_indirect_path ? max(EE_RENDER_INFO.indirect_lighting_intensity, 0.0f) : 1.0f;
      const float3 environment_irradiance =
          EE_CAMERA_PATH_ENVIRONMENT_RADIANCE(miss_direction) * last_environment_intensity * indirect_scale;
      const float3 environment_contribution = throughput * mis_weight * environment_irradiance;
      radiance += environment_contribution;
      if (debug_view == EE_CAMERA_DEBUG_INDIRECT_RADIANCE) {
        debug_radiance += environment_contribution;
      }
      break;
    }
    if (surface_depth == 0u) {
      primary_hit_distance = hit_value.hit_t;
    }
    EE_CAMERA_SURFACE_HIT surface_hit = EE_CAMERA_RECONSTRUCT_SURFACE_HIT<feature_mask>(
        is_inside, ray_direction, ray_cone_width, ray_spread_angle);
    ray_outputs.path_length = max(ray_outputs.path_length, surface_depth + 1u);
    path_surface_count = max(path_surface_count, surface_depth + 1u);
    if (surface_depth == 0u) {
      ray_outputs.primary_hit = 1u;
      ray_outputs.albedo = surface_hit.pbr.base_color;
      ray_outputs.normal = surface_hit.normal;
      if (EE_CAMERA_IS_MATERIAL_DEBUG_VIEW(debug_view)) {
        return EE_CAMERA_SANITIZE_RADIANCE(EE_CAMERA_MATERIAL_DEBUG_VALUE<feature_mask>(debug_view, surface_hit));
      }
    }
    max_roughness = max(max_roughness, surface_hit.pbr.roughness);
    surface_hit.pbr.roughness = max_roughness;
    const float3 view_direction = EE_CAMERA_SAFE_NORMALIZE(-ray_direction, surface_hit.normal);
    const float3 pre_volume_radiance = radiance;
    bool volume_scattered = false;
    uint volume_roulette_seed = 0u;
    if ((feature_mask & EE_GLTF_FEATURE_VOLUME) != 0u && is_inside) {
      uint volume_scatter_seed =
          EE_CAMERA_RANDOM_STREAM(sample_seed, EE_CAMERA_RANDOM_DOMAIN_VOLUME_SCATTER, current_segment);
      uint volume_light_seed =
          EE_CAMERA_RANDOM_STREAM(sample_seed, EE_CAMERA_RANDOM_DOMAIN_VOLUME_LIGHT, current_segment);
      uint volume_direct_shadow_seed =
          EE_CAMERA_RANDOM_STREAM(sample_seed, EE_CAMERA_RANDOM_DOMAIN_VOLUME_DIRECT_SHADOW_ALPHA, current_segment);
      uint volume_emissive_seed =
          EE_CAMERA_RANDOM_STREAM(sample_seed, EE_CAMERA_RANDOM_DOMAIN_VOLUME_EMISSIVE, current_segment);
      uint volume_emissive_shadow_seed =
          EE_CAMERA_RANDOM_STREAM(sample_seed, EE_CAMERA_RANDOM_DOMAIN_VOLUME_EMISSIVE_SHADOW_ALPHA, current_segment);
      volume_roulette_seed =
          EE_CAMERA_RANDOM_STREAM(sample_seed, EE_CAMERA_RANDOM_DOMAIN_VOLUME_ROULETTE, current_segment);
      volume_scattered = EE_CAMERA_PROCESS_VOLUME_SEGMENT<T, feature_mask>(
          traversal, hit_value.hit_t, ray_origin, ray_direction, throughput, radiance, last_sample_pdf, ray_cone_width,
          ray_spread_angle, volume_medium, volume_scatter_seed, volume_light_seed,
          volume_direct_shadow_seed, volume_emissive_seed, volume_emissive_shadow_seed, scatter_bounces);
    }
    if (volume_scattered) {
      last_environment_intensity = max(EE_ENVIRONMENT.diffuse_sky_intensity, 0.0f);
      diffuse_indirect_path = true;
      if (debug_view == EE_CAMERA_DEBUG_INDIRECT_RADIANCE && surface_depth > 0u) {
        debug_radiance += radiance - pre_volume_radiance;
      }
      previous_scattering_position = ray_origin;
      throughput = EE_CAMERA_SANITIZE_RADIANCE(throughput);
      if (max(throughput.x, max(throughput.y, throughput.z)) <= EE_CAMERA_PDF_EPSILON) {
        break;
      }
      if (scatter_bounces >= EE_CAMERA_VOLUME_FREE_BUDGET) {
        const float continue_probability =
            clamp(max(throughput.x, max(throughput.y, throughput.z)) + 0.001f, 0.001f, 0.95f);
        if (EE_PCG_RANDOM(volume_roulette_seed) >= continue_probability) {
          break;
        }
        throughput /= continue_probability;
      }
      continue;
    }
    throughput = EE_CAMERA_SANITIZE_RADIANCE(throughput);
    if (max(throughput.x, max(throughput.y, throughput.z)) <= EE_CAMERA_PDF_EPSILON) {
      break;
    }
    if ((feature_mask & EE_GLTF_FEATURE_UNLIT) != 0u && EE_GLTF_MATERIALS[surface_hit.material_index].unlit > 0) {
      const float indirect_scale = diffuse_indirect_path ? max(EE_RENDER_INFO.indirect_lighting_intensity, 0.0f) : 1.0f;
      const float3 unlit_contribution = throughput * surface_hit.surface.base_color.rgb * indirect_scale;
      radiance += unlit_contribution;
      if (debug_view == EE_CAMERA_DEBUG_EMISSION && surface_depth == 0u) {
        debug_radiance += unlit_contribution;
      } else if (debug_view == EE_CAMERA_DEBUG_INDIRECT_RADIANCE && surface_depth > 0u) {
        debug_radiance += unlit_contribution;
      }
      break;
    }
    const GltfShadeMaterial surface_material = EE_GLTF_MATERIALS[surface_hit.material_index];
    EmissiveTriangleInfo emissive_record;
    emissive_record.instance_index = 0u;
    emissive_record.primitive_id = 0u;
    emissive_record.alias_probability = 1.0f;
    emissive_record.alias_index = 0u;
    emissive_record.area_pdf = 0.0f;
    const bool surface_emissive_visible = true;
    const bool emissive_triangle_hit =
        surface_emissive_visible && EE_CAMERA_EMISSIVE_TRIANGLE_COUNT() != 0u &&
        max(surface_material.emissive_factor.x,
            max(surface_material.emissive_factor.y, surface_material.emissive_factor.z)) > 0.0f &&
        EE_CAMERA_FIND_EMISSIVE_TRIANGLE(hit_value.instance_index, hit_value.primitive_id, emissive_record);
    float3 emissive_radiance = float3(0.0f);
    if (surface_emissive_visible) {
      emissive_radiance =
          emissive_triangle_hit
              ? EE_RT_COATED_EMISSION_LOD0_SPECIALIZED<feature_mask>(
                    surface_hit.material_index,
                    EE_GLTF_MAKE_TEX_COORDS(surface_hit.tex_coord_0, surface_hit.tex_coord_1, surface_hit.tex_coord_2,
                                            surface_hit.tex_coord_3, float4(0.0f)),
                    surface_hit.shading_normal, surface_hit.tangent, surface_hit.bitangent,
                    surface_hit.geometric_normal, view_direction)
              : EE_GLTF_RT_COATED_EMISSION(surface_hit.pbr, view_direction);
    }
    const float emissive_indirect_scale =
        diffuse_indirect_path ? max(EE_RENDER_INFO.indirect_lighting_intensity, 0.0f) : 1.0f;
    float emissive_hit_pdf = 0.0f;
    float emissive_hit_mis_weight = 1.0f;
    if (last_sample_pdf != EE_CAMERA_DIRAC_PDF && emissive_triangle_hit) {
      emissive_hit_pdf = EE_CAMERA_EMISSIVE_TRIANGLE_HIT_PDF(
          emissive_record, previous_scattering_position, surface_hit.position, surface_hit.geometric_normal);
      if (emissive_hit_pdf > 0.0f) {
        emissive_hit_mis_weight = EE_CAMERA_CONTRIBUTION_MIS_WEIGHT(camera, last_sample_pdf, emissive_hit_pdf);
      }
    }
    const float3 emissive_contribution =
        throughput * emissive_hit_mis_weight * emissive_radiance * emissive_indirect_scale;
    radiance += emissive_contribution;
    if (debug_view == EE_CAMERA_DEBUG_EMISSION && surface_depth == 0u) {
      debug_radiance += emissive_contribution;
    } else if (debug_view == EE_CAMERA_DEBUG_INDIRECT_RADIANCE && surface_depth > 0u) {
      debug_radiance += emissive_contribution;
    }
    ray_cone_width = EE_CAMERA_WORLD_FOOTPRINT(ray_cone_width, hit_value.hit_t, surface_hit.geometric_normal,
                                               ray_direction, ray_spread_angle);
    EE_CAMERA_BOUNCE_SCRATCH bounce = EE_CAMERA_EMPTY_BOUNCE_SCRATCH();
    EE_CAMERA_BOUNCE_SCRATCH emissive_bounce = EE_CAMERA_EMPTY_BOUNCE_SCRATCH();
    uint surface_light_seed =
        EE_CAMERA_RANDOM_STREAM(sample_seed, EE_CAMERA_RANDOM_DOMAIN_SURFACE_LIGHT, current_segment);
    uint surface_direct_bsdf_seed =
        EE_CAMERA_RANDOM_STREAM(sample_seed, EE_CAMERA_RANDOM_DOMAIN_SURFACE_DIRECT_BSDF, current_segment);
    uint surface_emissive_seed =
        EE_CAMERA_RANDOM_STREAM(sample_seed, EE_CAMERA_RANDOM_DOMAIN_SURFACE_EMISSIVE, current_segment);
    uint surface_emissive_bsdf_seed =
        EE_CAMERA_RANDOM_STREAM(sample_seed, EE_CAMERA_RANDOM_DOMAIN_SURFACE_EMISSIVE_BSDF, current_segment);
    uint surface_bsdf_seed =
        EE_CAMERA_RANDOM_STREAM(sample_seed, EE_CAMERA_RANDOM_DOMAIN_SURFACE_BSDF, current_segment);
    uint surface_direct_shadow_seed =
        EE_CAMERA_RANDOM_STREAM(sample_seed, EE_CAMERA_RANDOM_DOMAIN_SURFACE_DIRECT_SHADOW_ALPHA, current_segment);
    uint surface_emissive_shadow_seed =
        EE_CAMERA_RANDOM_STREAM(sample_seed, EE_CAMERA_RANDOM_DOMAIN_SURFACE_EMISSIVE_SHADOW_ALPHA, current_segment);
    uint surface_roulette_seed =
        EE_CAMERA_RANDOM_STREAM(sample_seed, EE_CAMERA_RANDOM_DOMAIN_SURFACE_ROULETTE, current_segment);
    EE_CAMERA_PREPARE_DIRECT_LIGHTING(surface_hit, view_direction, throughput, diffuse_indirect_path,
                                      surface_depth == 0u, surface_light_seed, surface_direct_bsdf_seed, bounce);
    EE_CAMERA_PREPARE_EMISSIVE_LIGHTING<feature_mask>(surface_hit, view_direction, throughput, diffuse_indirect_path,
                                                      surface_depth == 0u, surface_emissive_seed,
                                                      surface_emissive_bsdf_seed, emissive_bounce);
    GltfRayTracingBsdfSampleData sample_data;
    sample_data.k1 = view_direction;
    sample_data.xi = EE_PCG_RANDOM_3(surface_bsdf_seed);
    EE_GLTF_RT_BSDF_SAMPLE(sample_data, surface_hit.pbr);
    bool terminate_path = (sample_data.pdf <= EE_CAMERA_PDF_EPSILON && sample_data.pdf != EE_GLTF_RT_BSDF_DIRAC_PDF) ||
                          sample_data.event_type == EE_GLTF_RT_BSDF_EVENT_ABSORB;
    if (!terminate_path) {
      throughput *= sample_data.bsdf_over_pdf;
      throughput = EE_CAMERA_SANITIZE_RADIANCE(throughput);
      if (max(throughput.x, max(throughput.y, throughput.z)) <= EE_CAMERA_PDF_EPSILON) {
        terminate_path = true;
      } else {
        previous_scattering_position = surface_hit.position;
        ray_origin = EE_CAMERA_OFFSET_RAY_ORIGIN(surface_hit, sample_data.k2);
        ray_direction = sample_data.k2;
        last_sample_pdf = sample_data.pdf;
        last_environment_intensity = EE_CAMERA_ENVIRONMENT_INTENSITY_FOR_EVENT(sample_data.event_type);
        diffuse_indirect_path = diffuse_indirect_path || (sample_data.event_type & EE_GLTF_RT_BSDF_EVENT_DIFFUSE) != 0;
        if ((feature_mask & EE_GLTF_FEATURE_VOLUME) != 0u &&
            (sample_data.event_type & EE_GLTF_RT_BSDF_EVENT_TRANSMISSION) != 0 && surface_hit.pbr.thickness > 0.0f) {
          const bool entered_volume = !is_inside;
          is_inside = !is_inside;
          volume_medium = entered_volume ? EE_CAMERA_MAKE_VOLUME_MEDIUM(surface_hit.pbr)
                                         : EE_CAMERA_EMPTY_VOLUME_MEDIUM();
        }
      }
    }
    const float3 direct_contribution =
        EE_CAMERA_RESOLVE_DIRECT_LIGHTING(traversal, bounce, surface_direct_shadow_seed);
    const float3 direct_emissive_contribution =
        EE_CAMERA_RESOLVE_DIRECT_LIGHTING(traversal, emissive_bounce, surface_emissive_shadow_seed);
    radiance += direct_contribution + direct_emissive_contribution;
    if (surface_depth == 0u) {
      if (debug_view == EE_CAMERA_DEBUG_DIRECT_PUNCTUAL && bounce.light_kind == EE_CAMERA_LIGHT_KIND_PUNCTUAL) {
        debug_radiance += direct_contribution;
      } else if (debug_view == EE_CAMERA_DEBUG_DIRECT_ENVIRONMENT &&
                 bounce.light_kind == EE_CAMERA_LIGHT_KIND_ENVIRONMENT) {
        debug_radiance += direct_contribution;
      } else if (debug_view == EE_CAMERA_DEBUG_DIRECT_EMISSIVE) {
        debug_radiance += direct_emissive_contribution;
      } else if (debug_view == EE_CAMERA_DEBUG_BSDF_PDF) {
        debug_radiance = float3(EE_CAMERA_ENCODE_PDF(sample_data.pdf), EE_CAMERA_ENCODE_PDF(bounce.bsdf_pdf),
                                EE_CAMERA_ENCODE_PDF(emissive_bounce.bsdf_pdf));
      } else if (debug_view == EE_CAMERA_DEBUG_LIGHT_PDF) {
        debug_radiance = float3(EE_CAMERA_ENCODE_PDF(bounce.light_pdf),
                                bounce.light_kind == EE_CAMERA_LIGHT_KIND_PUNCTUAL ? 1.0f : 0.0f,
                                bounce.light_kind == EE_CAMERA_LIGHT_KIND_ENVIRONMENT ? 1.0f : 0.0f);
      } else if (debug_view == EE_CAMERA_DEBUG_EMISSIVE_PDF) {
        debug_radiance = float3(EE_CAMERA_ENCODE_PDF(emissive_bounce.light_pdf),
                                emissive_bounce.light_pdf > 0.0f ? 1.0f : 0.0f, 0.0f);
      }
    } else if (debug_view == EE_CAMERA_DEBUG_INDIRECT_RADIANCE) {
      debug_radiance += direct_contribution + direct_emissive_contribution;
    }
    if (terminate_path) {
      break;
    }
    if (surface_depth >= EE_CAMERA_RUSSIAN_ROULETTE_MIN_DEPTH) {
      const float continue_probability =
          clamp(max(throughput.x, max(throughput.y, throughput.z)) + 0.001f, 0.001f, 0.95f);
      if (EE_PCG_RANDOM(surface_roulette_seed) >= continue_probability) {
        break;
      }
      throughput /= continue_probability;
    }
    surface_depth += 1u;
  }
  if (debug_view != EE_CAMERA_DEBUG_BEAUTY) {
    if (debug_view == EE_CAMERA_DEBUG_PATH_DEPTH) {
      ray_outputs.debug = float3(float(path_surface_count) / float(max_depth));
      return ray_outputs.debug;
    }
    ray_outputs.debug = EE_CAMERA_SANITIZE_RADIANCE(debug_radiance);
    return EE_CAMERA_SANITIZE_RADIANCE(debug_radiance);
  }
  ray_outputs.debug = EE_CAMERA_SANITIZE_RADIANCE(radiance);
  return EE_CAMERA_SANITIZE_RADIANCE(radiance);
}
EE_CAMERA_PRIMARY_RAY EE_CAMERA_CREATE_PRIMARY_RAY(const Camera camera, const float2 sample_position,
                                                   const float2 sample_offset, const float2 image_size) {
  const float2 clip_coords = (sample_position + sample_offset) / image_size * 2.0f - 1.0f;
  float4 view_position = mul(float4(clip_coords, -1.0f, 1.0f), camera.inverse_projection);
  view_position /= view_position.w;
  const float3 origin = camera.inverse_view[3].xyz;
  float4 world_position = mul(view_position, camera.inverse_view);
  world_position /= world_position.w;
  EE_CAMERA_PRIMARY_RAY ray;
  ray.origin = origin;
  ray.direction =
      EE_CAMERA_SAFE_NORMALIZE(world_position.xyz - origin,
                               EE_CAMERA_SAFE_NORMALIZE(mul(float4(0.0f, 0.0f, -1.0f, 0.0f), camera.inverse_view).xyz,
                                                        float3(0.0f, 0.0f, -1.0f)));
  return ray;
}
float2 EE_CAMERA_SAMPLE_GAUSSIAN(const float2 value) {
  const float radius = sqrt(-2.0f * log(max(1e-38f, value.x)));
  const float theta = 2.0f * EE_CAMERA_PI * value.y;
  return radius * float2(cos(theta), sin(theta));
}
public
void EE_CAMERA_RENDER_PIXEL<T : ICameraRayTraversal, let feature_mask : uint>(const uint2 pixel_coordinate,
                                                                              const uint2 image_size) {
  T traversal;
  traversal.initialize(EE_CAMERA_CONSTANTS.shader_execution_reordering != 0u);
  const Camera camera = EE_CAMERAS[EE_CAMERA_CONSTANTS.camera_index];
  uint debug_view = camera.ray_debug_view;
  const bool auto_spp_enabled = camera.auto_spp_enabled != 0u;
  const float4 previous_history =
      EE_CAMERA_CONSTANTS.total_samples > 0u ? radiance_history_image[int2(pixel_coordinate)] : float4(0.0f);
  const float4 previous_auto_metadata = auto_spp_enabled && EE_CAMERA_CONSTANTS.total_samples > 0u
                                            ? convergence_history_image[int2(pixel_coordinate)]
                                            : float4(0.0f);
  const uint auto_spp_min_samples = max(camera.auto_spp_min_samples, 1u);
  const uint auto_spp_max_samples = max(camera.auto_spp_max_samples, auto_spp_min_samples);
  const uint previous_auto_samples = uint(max(previous_auto_metadata.x, 0.0f) + 0.5f);
  const bool auto_history_valid =
      auto_spp_enabled && EE_CAMERA_CONSTANTS.total_samples > 0u && previous_auto_samples > 0u;
  const bool auto_pixel_converged =
      auto_history_valid && previous_auto_samples >= auto_spp_min_samples && previous_auto_metadata.z > 0.5f;
  if (auto_spp_enabled && auto_history_valid &&
      (auto_pixel_converged || previous_auto_samples >= auto_spp_max_samples)) {
    result_image[int2(pixel_coordinate)] = float4(previous_history.xyz, 1.0f);
    radiance_history_image[int2(pixel_coordinate)] = previous_history;
    convergence_history_image[int2(pixel_coordinate)] =
        float4(float(previous_auto_samples), previous_auto_metadata.y, 1.0f, 0.0f);
    return;
  }
  float3 linear_radiance = float3(0.0f);
  const uint manual_sample_size = max(camera.sample_size, 1u);
  const uint remaining_auto_samples =
      auto_spp_enabled && auto_history_valid ? auto_spp_max_samples - previous_auto_samples : auto_spp_max_samples;
  const uint sample_size =
      auto_spp_enabled ? min(manual_sample_size, max(remaining_auto_samples, 1u)) : manual_sample_size;
  const uint previous_accumulated_samples =
      auto_spp_enabled && auto_history_valid ? previous_auto_samples : EE_CAMERA_CONSTANTS.total_samples;
  const uint frame_sample_size = auto_spp_enabled ? sample_size : max(EE_CAMERA_CONSTANTS.frame_sample_size, 1u);
  uint invalid_radiance_rejections = 0u;
  uint firefly_clamp_count = 0u;
  float primary_hit_distance = EE_CAMERA_FAR(int(EE_CAMERA_CONSTANTS.camera_index));
  CameraRayOutputDiagnostics frame_outputs = EE_CAMERA_INIT_RAY_OUTPUT_DIAGNOSTICS();
  const float ray_spread_angle = EE_CAMERA_RAY_SPREAD_ANGLE(float(image_size.y));
  [[unroll]] for (uint i = 0u; i < sample_size; ++i) {
    const uint global_sample_index = previous_accumulated_samples + i;
    const uint sample_seed = EE_XXHASH32(uint3(pixel_coordinate, global_sample_index));
    uint camera_seed = EE_CAMERA_RANDOM_STREAM(sample_seed, EE_CAMERA_RANDOM_DOMAIN_CAMERA, 0u);
    const float2 sample_offset = global_sample_index == 0u
                                     ? float2(0.5f) + EE_CAMERA_ANTIALIASING_STANDARD_DEVIATION *
                                                          EE_CAMERA_SAMPLE_GAUSSIAN(EE_PCG_RANDOM_2(camera_seed))
                                     : EE_PCG_RANDOM_2(camera_seed);
    const EE_CAMERA_PRIMARY_RAY primary_ray =
        EE_CAMERA_CREATE_PRIMARY_RAY(camera, float2(pixel_coordinate), sample_offset, float2(image_size));
    float sample_hit_distance = EE_CAMERA_FAR(int(EE_CAMERA_CONSTANTS.camera_index));
    CameraRayOutputDiagnostics sample_outputs = EE_CAMERA_INIT_RAY_OUTPUT_DIAGNOSTICS();
    float3 sample_radiance = EE_CAMERA_REJECT_INVALID_RADIANCE(
        EE_CAMERA_TRACE_PATH<T, feature_mask>(traversal, sample_seed, primary_ray.origin, primary_ray.direction,
                                              ray_spread_angle, debug_view, sample_hit_distance, sample_outputs),
        invalid_radiance_rejections);
    sample_radiance = EE_CAMERA_APPLY_FIREFLY_CLAMP(camera, sample_radiance, firefly_clamp_count);
    linear_radiance += sample_radiance;
    primary_hit_distance = min(primary_hit_distance, sample_hit_distance);
    frame_outputs.ray_count += sample_outputs.ray_count;
    frame_outputs.path_length = max(frame_outputs.path_length, sample_outputs.path_length);
    frame_outputs.debug += sample_outputs.debug;
    if (frame_outputs.primary_hit == 0u && sample_outputs.primary_hit != 0u) {
      frame_outputs.primary_hit = 1u;
      frame_outputs.albedo = sample_outputs.albedo;
      frame_outputs.normal = sample_outputs.normal;
    }
  }
  frame_outputs.debug /= float(sample_size);
  linear_radiance /= float(sample_size);
  if (previous_accumulated_samples > 0u) {
    const float3 previous_linear_radiance = previous_history.xyz;
    linear_radiance =
        (previous_linear_radiance * float(previous_accumulated_samples) + linear_radiance * float(frame_sample_size)) /
        float(previous_accumulated_samples + frame_sample_size);
  }
  const uint accumulated_sample_count = previous_accumulated_samples + frame_sample_size;
  const float convergence_delta = previous_accumulated_samples > 0u
                                      ? EE_CAMERA_RELATIVE_LUMINANCE_DELTA(previous_history.xyz, linear_radiance)
                                      : 3.402823466e+38f;
  const bool auto_sample_converged = auto_spp_enabled && accumulated_sample_count >= auto_spp_min_samples &&
                                     (convergence_delta <= max(camera.auto_spp_convergence_threshold, 0.0f) ||
                                      accumulated_sample_count >= auto_spp_max_samples);
  result_image[int2(pixel_coordinate)] = float4(linear_radiance, 1.0f);
  radiance_history_image[int2(pixel_coordinate)] =
      float4(linear_radiance, EE_CAMERA_PACK_SAMPLE_DIAGNOSTICS(invalid_radiance_rejections, firefly_clamp_count));
  convergence_history_image[int2(pixel_coordinate)] =
      auto_spp_enabled
          ? float4(float(accumulated_sample_count), convergence_delta, auto_sample_converged ? 1.0f : 0.0f, 0.0f)
          : float4(0.0f);
  ray_hit_distance_image[int2(pixel_coordinate)] = primary_hit_distance;
  EE_CAMERA_STORE_RAY_OUTPUT_DIAGNOSTICS(camera, int2(pixel_coordinate), frame_outputs);
}
