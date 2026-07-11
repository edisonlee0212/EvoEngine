#ifndef EE_CAMERA_RAY_INTEGRATOR_GLSL
#define EE_CAMERA_RAY_INTEGRATOR_GLSL

const float EE_CAMERA_PI = 3.14159265359f;
const float EE_CAMERA_RAY_EPSILON = 1e-3f;
const float EE_CAMERA_PDF_EPSILON = 1e-6f;
const float EE_CAMERA_DIRAC_PDF = -1.0f;
const uint EE_CAMERA_RUSSIAN_ROULETTE_MIN_DEPTH = 3u;
const uint EE_CAMERA_RAY_PAYLOAD_SURFACE = 0u;
const uint EE_CAMERA_RAY_PAYLOAD_SHADOW = 1u;
const uint EE_CAMERA_RAY_MASK_SHADOW = 0x02u;
const float EE_CAMERA_ENVIRONMENT_TYPE_COLOR = 1.0f;
const float EE_CAMERA_ANTIALIASING_STANDARD_DEVIATION = 0.4246609f;
const float EE_CAMERA_MIN_SHADOW_TRANSMISSION = 0.01f;
const float EE_CAMERA_VOLUME_MIN_SCATTER = 0.001f;
const float EE_CAMERA_VOLUME_RAND_FLOOR = 1.0e-10f;
const uint EE_CAMERA_VOLUME_FREE_BUDGET = 64u;

struct EE_CAMERA_DIRECTION_SAMPLE {
  vec3 direction;
  float pdf;
};

struct EE_CAMERA_PRIMARY_RAY {
  vec3 origin;
  vec3 direction;
};

struct EE_CAMERA_SURFACE_HIT {
  vec3 position;
  vec3 shadow_position;
  vec3 normal;
  vec3 shading_normal;
  vec3 geometric_normal;
  vec3 tangent;
  vec3 bitangent;
  vec2 tex_coord_0;
  vec2 tex_coord_1;
  vec4 vertex_color;
  vec2 tex_gradients;
  uint material_index;
  GltfRasterMaterial surface;
  GltfRayTracingPbrMaterial pbr;
};

struct EE_CAMERA_VOLUME_MEDIUM {
  vec3 extinction;
  vec3 scatter_coefficient;
  float scatter_anisotropy;
};

vec3 EE_CAMERA_SAFE_NORMALIZE(const vec3 value, const vec3 fallback) {
  const float length_squared = dot(value, value);
  return length_squared > 0.00000001f ? value * inversesqrt(length_squared) : fallback;
}

vec3 EE_CAMERA_POINT_OFFSET(const vec3 position, const vec3 v0_position, const vec3 v1_position,
                            const vec3 v2_position, const vec3 v0_normal, const vec3 v1_normal,
                            const vec3 v2_normal, const vec3 barycentrics) {
  vec3 u = position - v0_position;
  vec3 v = position - v1_position;
  vec3 w = position - v2_position;

  u -= min(0.0f, dot(u, v0_normal)) * v0_normal;
  v -= min(0.0f, dot(v, v1_normal)) * v1_normal;
  w -= min(0.0f, dot(w, v2_normal)) * v2_normal;
  return position + u * barycentrics.x + v * barycentrics.y + w * barycentrics.z;
}

vec3 EE_CAMERA_SAFE_OFFSET_RAY(const vec3 world_position, const vec3 offset_direction) {
  const float scale_value = 256.0f;
  const ivec3 scale_int = ivec3(scale_value * offset_direction);
  const vec3 offset_position = vec3(
      intBitsToFloat(floatBitsToInt(world_position.x) + (world_position.x < 0.0f ? -scale_int.x : scale_int.x)),
      intBitsToFloat(floatBitsToInt(world_position.y) + (world_position.y < 0.0f ? -scale_int.y : scale_int.y)),
      intBitsToFloat(floatBitsToInt(world_position.z) + (world_position.z < 0.0f ? -scale_int.z : scale_int.z)));

  const float origin = 1.0f / 32.0f;
  const float float_scale = 1.0f / 65536.0f;
  return vec3(abs(world_position.x) < origin ? world_position.x + float_scale * offset_direction.x
                                             : offset_position.x,
              abs(world_position.y) < origin ? world_position.y + float_scale * offset_direction.y
                                             : offset_position.y,
              abs(world_position.z) < origin ? world_position.z + float_scale * offset_direction.z
                                              : offset_position.z);
}

vec2 EE_CAMERA_TEXEL_DENSITY(const mat4 model, const Vertex v0, const Vertex v1, const Vertex v2) {
  const vec3 world_edge_1 = vec3(model * vec4(v1.position - v0.position, 0.0f));
  const vec3 world_edge_2 = vec3(model * vec4(v2.position - v0.position, 0.0f));
  const float world_area = length(cross(world_edge_1, world_edge_2));
  const vec2 uv0_edge_1 = v1.tex_coord - v0.tex_coord;
  const vec2 uv0_edge_2 = v2.tex_coord - v0.tex_coord;
  const vec2 uv1_edge_1 = v1.tex_coord_1 - v0.tex_coord_1;
  const vec2 uv1_edge_2 = v2.tex_coord_1 - v0.tex_coord_1;
  const vec2 uv_area = abs(vec2(uv0_edge_1.x * uv0_edge_2.y - uv0_edge_2.x * uv0_edge_1.y,
                                uv1_edge_1.x * uv1_edge_2.y - uv1_edge_2.x * uv1_edge_1.y));
  return sqrt(uv_area / max(world_area, 1e-20f));
}

float EE_CAMERA_WORLD_FOOTPRINT(const float ray_cone_width, const float hit_t, const vec3 geometric_normal,
                                const vec3 ray_direction) {
  const Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
  const float pixel_angle = 2.0f * abs(camera.inverse_projection[1][1]) / max(camera.resolution_y, 1.0f);
  return (ray_cone_width + hit_t * pixel_angle) / max(abs(dot(geometric_normal, -ray_direction)), 1e-3f);
}

vec2 EE_CAMERA_TEXTURE_GRADIENTS(const float ray_cone_width, const float hit_t, const vec3 geometric_normal,
                                 const vec3 ray_direction, const mat4 model, const Vertex v0, const Vertex v1,
                                 const Vertex v2) {
  return EE_CAMERA_WORLD_FOOTPRINT(ray_cone_width, hit_t, geometric_normal, ray_direction) *
         EE_CAMERA_TEXEL_DENSITY(model, v0, v1, v2);
}

vec3 EE_CAMERA_SANITIZE_RADIANCE(const vec3 value) {
  return vec3(value.x >= 0.0f && value.x < 3.402823466e+38f ? value.x : 0.0f,
              value.y >= 0.0f && value.y < 3.402823466e+38f ? value.y : 0.0f,
              value.z >= 0.0f && value.z < 3.402823466e+38f ? value.z : 0.0f);
}

bool EE_CAMERA_HAS_NONFINITE_RADIANCE(const vec3 radiance) {
  return any(isnan(radiance)) || any(isinf(radiance));
}

vec3 EE_CAMERA_REJECT_INVALID_RADIANCE(const vec3 radiance, inout uint invalid_rejections) {
  if (EE_CAMERA_HAS_NONFINITE_RADIANCE(radiance)) {
    invalid_rejections += 1u;
    return vec3(0.0f);
  }
  return max(radiance, vec3(0.0f));
}

vec3 EE_CAMERA_APPLY_FIREFLY_CLAMP(const Camera camera, const vec3 radiance, inout uint firefly_clamps) {
  if (camera.firefly_clamp_enabled == 0u) {
    return radiance;
  }
  const float threshold = max(camera.firefly_clamp_threshold, 0.0f);
  if (threshold <= 0.0f) {
    return radiance;
  }
  const float luminance = dot(radiance, vec3(1.0f / 3.0f));
  if (luminance > threshold) {
    firefly_clamps += 1u;
    return radiance * (threshold / luminance);
  }
  return radiance;
}

float EE_CAMERA_PACK_SAMPLE_DIAGNOSTICS(const uint invalid_rejections, const uint firefly_clamps) {
  return float(min(invalid_rejections, 65535u)) + float(min(firefly_clamps, 65535u)) / 65536.0f;
}

float EE_CAMERA_LUMINANCE(const vec3 radiance) {
  return dot(radiance, vec3(0.2126f, 0.7152f, 0.0722f));
}

float EE_CAMERA_RELATIVE_LUMINANCE_DELTA(const vec3 previous_radiance, const vec3 current_radiance) {
  const float previous_luminance = EE_CAMERA_LUMINANCE(previous_radiance);
  const float current_luminance = EE_CAMERA_LUMINANCE(current_radiance);
  return abs(current_luminance - previous_luminance) / max(max(previous_luminance, current_luminance), 1e-3f);
}

vec3 EE_CAMERA_SAMPLE_CUBEMAP_RADIANCE(const int cubemap_index, const vec3 direction, const float lod) {
  vec3 environment_color = textureLod(EE_CUBEMAPS[cubemap_index], normalize(direction), lod).rgb;
  if (EE_ENVIRONMENT.gamma != 1.0f) {
    environment_color = pow(max(environment_color, vec3(0.0f)), vec3(1.0f / max(EE_ENVIRONMENT.gamma, 0.001f)));
  }
  return max(environment_color, vec3(0.0f));
}

vec2 EE_CAMERA_ENVIRONMENT_SPHERICAL_UV(const vec3 direction) {
  const vec3 dir = EE_CAMERA_SAFE_NORMALIZE(direction, vec3(0.0f, 1.0f, 0.0f));
  return vec2(atan(dir.z, dir.x) / (2.0f * EE_CAMERA_PI) + 0.5f, asin(clamp(dir.y, -1.0f, 1.0f)) / EE_CAMERA_PI + 0.5f);
}

vec3 EE_CAMERA_ENVIRONMENT_SPHERICAL_DIRECTION(const vec2 uv) {
  const float azimuth = (uv.x - 0.5f) * 2.0f * EE_CAMERA_PI;
  const float elevation = (uv.y - 0.5f) * EE_CAMERA_PI;
  const float cos_elevation = cos(elevation);
  return vec3(cos(azimuth) * cos_elevation, sin(elevation), sin(azimuth) * cos_elevation);
}

int EE_CAMERA_ENVIRONMENT_PDF_TEXTURE_INDEX() {
  return int(round(EE_ENVIRONMENT.environment_pdf_texture_index));
}

bool EE_CAMERA_HAS_ENVIRONMENT_PDF_TEXTURE() {
  return EE_CAMERA_ENVIRONMENT_PDF_TEXTURE_INDEX() >= 0;
}

float EE_CAMERA_ENVIRONMENT_MAP_PDF(const vec3 direction) {
  const int pdf_texture_index = EE_CAMERA_ENVIRONMENT_PDF_TEXTURE_INDEX();
  if (pdf_texture_index < 0) {
    return 1.0f / (4.0f * EE_CAMERA_PI);
  }
  const ivec2 texture_size = textureSize(EE_TEXTURE_2DS[pdf_texture_index], 0);
  if (texture_size.x <= 0 || texture_size.y <= 0) {
    return 1.0f / (4.0f * EE_CAMERA_PI);
  }
  const vec2 uv = EE_CAMERA_ENVIRONMENT_SPHERICAL_UV(direction);
  const ivec2 texel = clamp(ivec2(uv * vec2(texture_size)), ivec2(0), texture_size - ivec2(1));
  return max(texelFetch(EE_TEXTURE_2DS[pdf_texture_index], texel, 0).b, 0.0f);
}

vec3 EE_CAMERA_BACKGROUND_LIGHT_RADIANCE(const vec3 ray_direction) {
  const Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
  if (camera.use_clear_color == 1) {
    return EE_CAMERA_SANITIZE_RADIANCE(max(camera.clear_color.xyz, vec3(0.0f)) *
                                       max(camera.clear_color.w, 0.0f));
  }
  if (EE_ENVIRONMENT.light_intensity <= 0.0f) {
    return vec3(0.0f);
  }
  if (EE_ENVIRONMENT.background_color.w == 1.0f) {
    return EE_CAMERA_SANITIZE_RADIANCE(max(EE_ENVIRONMENT.background_color.rgb, vec3(0.0f)) *
                                       EE_ENVIRONMENT.light_intensity);
  }

  return EE_CAMERA_SANITIZE_RADIANCE(EE_CAMERA_SAMPLE_CUBEMAP_RADIANCE(camera.skybox_tex_index, ray_direction, 0.0f) *
                                     max(camera.clear_color.w, 0.0f) * EE_ENVIRONMENT.light_intensity);
}

vec3 EE_CAMERA_PATH_ENVIRONMENT_RADIANCE(const vec3 ray_direction) {
  return EE_CAMERA_BACKGROUND_LIGHT_RADIANCE(ray_direction);
}

float EE_CAMERA_PATH_ENVIRONMENT_PDF(const vec3 light_direction) {
  if (EE_ENVIRONMENT.environment_type != EE_CAMERA_ENVIRONMENT_TYPE_COLOR &&
      EE_CAMERAS[EE_CAMERA_INDEX].use_clear_color != 1) {
    return EE_CAMERA_ENVIRONMENT_MAP_PDF(light_direction);
  }
  return 1.0f / (4.0f * EE_CAMERA_PI);
}

bool EE_CAMERA_HAS_ENVIRONMENT_LIGHTING() {
  const Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
  if (camera.use_clear_color == 1) {
    return dot(max(camera.clear_color.xyz, vec3(0.0f)), vec3(1.0f)) * max(camera.clear_color.w, 0.0f) > 0.0f;
  }
  if (EE_ENVIRONMENT.light_intensity <= 0.0f) {
    return false;
  }
  if (EE_ENVIRONMENT.background_color.w == 1.0f) {
    return dot(max(EE_ENVIRONMENT.background_color.rgb, vec3(0.0f)), vec3(1.0f)) > 0.0f;
  }
  return camera.clear_color.w > 0.0f;
}

float EE_CAMERA_BALANCE_HEURISTIC(const float sampled_pdf, const float other_pdf) {
  if (sampled_pdf == EE_CAMERA_DIRAC_PDF) {
    return 1.0f;
  }
  const float safe_sampled_pdf = max(sampled_pdf, 0.0f);
  const float safe_other_pdf = max(other_pdf, 0.0f);
  const float pdf_sum = safe_sampled_pdf + safe_other_pdf;
  return pdf_sum > EE_CAMERA_PDF_EPSILON ? safe_sampled_pdf / pdf_sum : 0.0f;
}

float EE_CAMERA_ENVIRONMENT_PDF(const vec3 light_direction) {
  if (!EE_CAMERA_HAS_ENVIRONMENT_LIGHTING()) {
    return 0.0f;
  }
  return EE_CAMERA_PATH_ENVIRONMENT_PDF(light_direction);
}

float EE_CAMERA_ENVIRONMENT_HIT_MIS_WEIGHT(const float bsdf_pdf, const float environment_pdf,
                                           const float environment_weight) {
  if (bsdf_pdf == EE_CAMERA_DIRAC_PDF) {
    return 1.0f;
  }
  const float safe_bsdf_pdf = max(bsdf_pdf, 0.0f);
  const float safe_environment_pdf = max(environment_pdf, 0.0f) * max(environment_weight, 0.0f);
  const float pdf_sum = safe_bsdf_pdf + safe_environment_pdf;
  return pdf_sum > EE_CAMERA_PDF_EPSILON ? safe_bsdf_pdf / pdf_sum : 1.0f;
}

mat3 EE_CAMERA_TANGENT_SPACE(const vec3 normal) {
  const vec3 n = EE_CAMERA_SAFE_NORMALIZE(normal, vec3(0.0f, 0.0f, 1.0f));
  if (n.z < -0.99998796f) {
    return mat3(vec3(0.0f, -1.0f, 0.0f), vec3(-1.0f, 0.0f, 0.0f), n);
  }
  const float a = 1.0f / (1.0f + n.z);
  const float b = -n.x * n.y * a;
  const vec3 tangent = vec3(1.0f - n.x * n.x * a, b, -n.x);
  const vec3 bitangent = vec3(b, 1.0f - n.y * n.y * a, -n.y);
  return mat3(tangent, bitangent, n);
}

float EE_CAMERA_HEMISPHERE_PDF(const float alpha, const vec3 normal, const vec3 direction) {
  const float clamped_alpha = clamp(alpha, 0.0f, 1.0f);
  const float cone_range = max((1.0f - clamped_alpha) * (1.0f - clamped_alpha), EE_CAMERA_PDF_EPSILON);
  const float cone_cos_min = 1.0f - cone_range;
  return dot(EE_CAMERA_SAFE_NORMALIZE(normal, vec3(0.0f, 1.0f, 0.0f)),
             EE_CAMERA_SAFE_NORMALIZE(direction, vec3(0.0f, 1.0f, 0.0f))) >= cone_cos_min
             ? 1.0f / (2.0f * EE_CAMERA_PI * cone_range)
             : 0.0f;
}

EE_CAMERA_DIRECTION_SAMPLE EE_CAMERA_SAMPLE_HEMISPHERE(inout uint seed, const vec3 normal, const float alpha) {
  EE_CAMERA_DIRECTION_SAMPLE ray_sample;
  const float clamped_alpha = clamp(alpha, 0.0f, 1.0f);
  const float cone_range = max((1.0f - clamped_alpha) * (1.0f - clamped_alpha), EE_CAMERA_PDF_EPSILON);
  const float cos_theta = 1.0f - EE_RANDOM(seed) * cone_range;
  const float sin_theta = sqrt(max(0.0f, 1.0f - cos_theta * cos_theta));
  const float phi = 2.0f * EE_CAMERA_PI * EE_RANDOM(seed);
  const vec3 tangent_space_direction = vec3(cos(phi) * sin_theta, sin(phi) * sin_theta, cos_theta);
  ray_sample.direction = EE_CAMERA_SAFE_NORMALIZE(EE_CAMERA_TANGENT_SPACE(normal) * tangent_space_direction, normal);
  ray_sample.pdf = 1.0f / (2.0f * EE_CAMERA_PI * cone_range);
  return ray_sample;
}

EE_CAMERA_DIRECTION_SAMPLE EE_CAMERA_SAMPLE_SPHERE(inout uint seed) {
  EE_CAMERA_DIRECTION_SAMPLE ray_sample;
  const float z = 1.0f - 2.0f * EE_RANDOM(seed);
  const float radius = sqrt(max(0.0f, 1.0f - z * z));
  const float phi = 2.0f * EE_CAMERA_PI * EE_RANDOM(seed);
  ray_sample.direction = vec3(cos(phi) * radius, z, sin(phi) * radius);
  ray_sample.pdf = 1.0f / (4.0f * EE_CAMERA_PI);
  return ray_sample;
}

int EE_CAMERA_FIND_ENVIRONMENT_MARGINAL_ROW(const int pdf_texture_index, const ivec2 texture_size, const float sample_value) {
  int low = 0;
  int high = texture_size.y - 1;
  for (int i = 0; i < 16; ++i) {
    if (low >= high) {
      break;
    }
    const int mid = (low + high) / 2;
    const float cdf = texelFetch(EE_TEXTURE_2DS[pdf_texture_index], ivec2(texture_size.x - 1, mid), 0).g;
    if (sample_value <= cdf) {
      high = mid;
    } else {
      low = mid + 1;
    }
  }
  return low;
}

int EE_CAMERA_FIND_ENVIRONMENT_CONDITIONAL_COLUMN(const int pdf_texture_index, const ivec2 texture_size,
                                                  const int row, const float sample_value) {
  int low = 0;
  int high = texture_size.x - 1;
  for (int i = 0; i < 16; ++i) {
    if (low >= high) {
      break;
    }
    const int mid = (low + high) / 2;
    const float cdf = texelFetch(EE_TEXTURE_2DS[pdf_texture_index], ivec2(mid, row), 0).r;
    if (sample_value <= cdf) {
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
  const ivec2 texture_size = textureSize(EE_TEXTURE_2DS[pdf_texture_index], 0);
  if (texture_size.x <= 0 || texture_size.y <= 0) {
    return EE_CAMERA_SAMPLE_SPHERE(seed);
  }

  const float marginal_sample = EE_RANDOM(seed);
  const float conditional_sample = EE_RANDOM(seed);
  const int row = EE_CAMERA_FIND_ENVIRONMENT_MARGINAL_ROW(pdf_texture_index, texture_size, marginal_sample);
  const int column = EE_CAMERA_FIND_ENVIRONMENT_CONDITIONAL_COLUMN(pdf_texture_index, texture_size, row, conditional_sample);
  const vec2 uv = (vec2(column, row) + vec2(0.5f)) / vec2(texture_size);
  EE_CAMERA_DIRECTION_SAMPLE ray_sample;
  ray_sample.direction = EE_CAMERA_ENVIRONMENT_SPHERICAL_DIRECTION(uv);
  ray_sample.pdf = max(texelFetch(EE_TEXTURE_2DS[pdf_texture_index], ivec2(column, row), 0).b, 0.0f);
  return ray_sample.pdf > EE_CAMERA_PDF_EPSILON ? ray_sample : EE_CAMERA_SAMPLE_SPHERE(seed);
}

EE_CAMERA_DIRECTION_SAMPLE EE_CAMERA_SAMPLE_PATH_ENVIRONMENT(inout uint seed) {
  return EE_CAMERA_SAMPLE_ENVIRONMENT_MAP(seed);
}

float EE_CAMERA_COSINE_HEMISPHERE_PDF(const vec3 normal, const vec3 direction) {
  return max(dot(EE_CAMERA_SAFE_NORMALIZE(normal, vec3(0.0f, 1.0f, 0.0f)),
                 EE_CAMERA_SAFE_NORMALIZE(direction, vec3(0.0f, 1.0f, 0.0f))),
             0.0f) /
         EE_CAMERA_PI;
}

EE_CAMERA_DIRECTION_SAMPLE EE_CAMERA_SAMPLE_COSINE_HEMISPHERE(inout uint seed, const vec3 normal) {
  EE_CAMERA_DIRECTION_SAMPLE ray_sample;
  const float u0 = EE_RANDOM(seed);
  const float u1 = EE_RANDOM(seed);
  const float r = sqrt(u0);
  const float phi = 2.0f * EE_CAMERA_PI * u1;
  const vec3 tangent_space_direction = vec3(cos(phi) * r, sin(phi) * r, sqrt(max(0.0f, 1.0f - u0)));
  ray_sample.direction = EE_CAMERA_SAFE_NORMALIZE(EE_CAMERA_TANGENT_SPACE(normal) * tangent_space_direction, normal);
  ray_sample.pdf = EE_CAMERA_COSINE_HEMISPHERE_PDF(normal, ray_sample.direction);
  return ray_sample;
}

float EE_CAMERA_DISTRIBUTION_GGX(const vec3 normal, const vec3 half_vector, const float roughness) {
  const float a = roughness * roughness;
  const float a2 = a * a;
  const float n_dot_h = max(dot(normal, half_vector), 0.0f);
  const float n_dot_h2 = n_dot_h * n_dot_h;
  const float denominator = EE_CAMERA_PI * pow(n_dot_h2 * (a2 - 1.0f) + 1.0f, 2.0f);
  return a2 / max(denominator, 0.001f);
}

float EE_CAMERA_GEOMETRY_SCHLICK_GGX(const float n_dot_v, const float roughness) {
  const float r = roughness + 1.0f;
  const float k = (r * r) / 8.0f;
  return n_dot_v / max(n_dot_v * (1.0f - k) + k, 0.001f);
}

float EE_CAMERA_GEOMETRY_SMITH(const vec3 normal, const vec3 view_direction, const vec3 light_direction,
                               const float roughness) {
  const float n_dot_v = max(dot(normal, view_direction), 0.0f);
  const float n_dot_l = max(dot(normal, light_direction), 0.0f);
  return EE_CAMERA_GEOMETRY_SCHLICK_GGX(n_dot_v, roughness) *
         EE_CAMERA_GEOMETRY_SCHLICK_GGX(n_dot_l, roughness);
}

vec3 EE_CAMERA_FRESNEL_SCHLICK(const float cos_theta, const vec3 f0) {
  return f0 + (1.0f - f0) * pow(max(1.0f - cos_theta, 0.0f), 5.0f);
}

vec3 EE_CAMERA_FRESNEL_SCHLICK_ROUGHNESS(const float cos_theta, const vec3 f0, const float roughness) {
  return f0 + (max(vec3(1.0f - roughness), f0) - f0) * pow(max(1.0f - cos_theta, 0.0f), 5.0f);
}

float EE_CAMERA_BSDF_PDF(const EE_CAMERA_SURFACE_HIT hit, const vec3 view_direction, const vec3 light_direction) {
  GltfRayTracingBsdfEvaluateData eval_data;
  eval_data.k1 = view_direction;
  eval_data.k2 = light_direction;
  eval_data.xi = vec3(0.0f);
  EE_GLTF_RT_BSDF_EVALUATE(eval_data, hit.pbr);
  return eval_data.pdf;
}

vec3 EE_CAMERA_EVALUATE_DIRECT_BSDF(const EE_CAMERA_SURFACE_HIT hit, const vec3 view_direction,
                                    const vec3 light_direction, const vec3 light_radiance,
                                    inout uint seed, out float bsdf_pdf) {
  GltfRayTracingBsdfEvaluateData eval_data;
  eval_data.k1 = view_direction;
  eval_data.k2 = light_direction;
  eval_data.xi = vec3(EE_RANDOM(seed), EE_RANDOM(seed), EE_RANDOM(seed));
  EE_GLTF_RT_BSDF_EVALUATE(eval_data, hit.pbr);
  bsdf_pdf = eval_data.pdf;
  return EE_CAMERA_SANITIZE_RADIANCE((eval_data.bsdf_diffuse + eval_data.bsdf_glossy) *
                                     max(light_radiance, vec3(0.0f)));
}

void EE_CAMERA_RESET_PAYLOAD(const uint seed, const float last_sample_pdf) {
  hit_value.color = vec3(0.0f);
  hit_value.seed = seed;
  hit_value.position = vec3(0.0f);
  hit_value.type = EE_CAMERA_RAY_PAYLOAD_SURFACE;
  hit_value.normal = vec3(0.0f);
  hit_value.hit_count = 0u;
  hit_value.geometric_normal = vec3(0.0f);
  hit_value.hit_t = EE_CAMERA_FAR(int(EE_CAMERA_INDEX));
  hit_value.initial_position = vec3(0.0f);
  hit_value.last_sample_pdf = last_sample_pdf;
  hit_value.initial_normal = vec3(0.0f);
  hit_value.instance_index = 0u;
  hit_value.barycentrics = vec2(0.0f);
  hit_value.primitive_id = 0u;
  hit_value.material_index = 0u;
  hit_value.environment_radiance = vec3(0.0f);
  hit_value.environment_pdf = 0.0f;
  hit_value.shadow_transmission = vec3(1.0f);
  hit_value.shadow_previous_hit_t = 0.0f;
  hit_value.shadow_is_inside = 0u;
}

#if defined(EE_CAMERA_RAY_QUERY_TRAVERSAL)
#include "CameraRayQueryTraversal.glsl"
#elif defined(EE_CAMERA_RAY_TRACING_TRAVERSAL)
#include "CameraRayTracingTraversal.glsl"
#else
#error "A camera ray traversal adapter must be selected."
#endif

const int EE_CAMERA_LIGHT_TYPE_DIRECTIONAL = 0;
const int EE_CAMERA_LIGHT_TYPE_POINT = 1;
const int EE_CAMERA_LIGHT_TYPE_SPOT = 2;
const float EE_CAMERA_NATIVE_DEFAULT_LIGHT_SIZE = 0.01f;

struct EE_CAMERA_RADIANCE_SPLIT {
  vec3 color;
  float intensity;
};

struct EE_CAMERA_GLTF_LIGHT {
  vec3 direction;
  int type;
  vec3 position;
  float radius;
  vec3 color;
  float intensity;
  float angular_size_or_inv_range;
  float inner_angle;
  float outer_angle;
  float cast_shadow;
  vec3 attenuation;
  float use_native_attenuation;
};

struct EE_CAMERA_LIGHT_CONTRIB {
  vec3 incident_vector;
  vec3 intensity;
  float distance;
  float pdf;
  float cast_shadow;
};

struct EE_CAMERA_DIRECT_LIGHT {
  vec3 direction;
  vec3 radiance_over_pdf;
  float distance;
  float pdf;
  float cast_shadow;
};

struct EE_CAMERA_BOUNCE_SCRATCH {
  bool next_event_valid;
  vec3 contribution;
  vec3 shadow_ray_origin;
  vec3 shadow_ray_direction;
  float shadow_ray_distance;
  float cast_shadow;
};

struct EE_CAMERA_LIGHT_TECHNIQUE_PROBABILITIES {
  float light_weight;
  float environment_weight;
};

EE_CAMERA_RADIANCE_SPLIT EE_CAMERA_SPLIT_LIGHT_RADIANCE(const vec3 radiance) {
  EE_CAMERA_RADIANCE_SPLIT split;
  split.intensity = max(max(radiance.x, radiance.y), radiance.z);
  split.color = split.intensity > EE_CAMERA_PDF_EPSILON ? radiance / split.intensity : vec3(0.0f);
  return split;
}

float EE_CAMERA_REFERENCE_LIGHT_RADIUS(const float light_size) {
  return light_size > EE_CAMERA_NATIVE_DEFAULT_LIGHT_SIZE + EE_CAMERA_PDF_EPSILON ? light_size : 0.0f;
}

float EE_CAMERA_REFERENCE_INV_RANGE(const float range) {
  const float camera_far = EE_CAMERA_FAR(int(EE_CAMERA_INDEX));
  return range > EE_CAMERA_RAY_EPSILON && range < camera_far - EE_CAMERA_RAY_EPSILON ? 1.0f / range : 0.0f;
}

int EE_CAMERA_PUNCTUAL_LIGHT_COUNT() {
  return int(EE_RENDER_INFO.directional_light_size) + int(EE_RENDER_INFO.point_light_size) +
         int(EE_RENDER_INFO.spot_light_size);
}

EE_CAMERA_GLTF_LIGHT EE_CAMERA_DEFAULT_GLTF_LIGHT() {
  EE_CAMERA_GLTF_LIGHT light;
  light.direction = vec3(0.0f, -1.0f, 0.0f);
  light.type = EE_CAMERA_LIGHT_TYPE_DIRECTIONAL;
  light.position = vec3(0.0f);
  light.radius = 0.0f;
  light.color = vec3(0.0f);
  light.intensity = 0.0f;
  light.angular_size_or_inv_range = 0.0f;
  light.inner_angle = 0.0f;
  light.outer_angle = 0.0f;
  light.cast_shadow = 0.0f;
  light.attenuation = vec3(0.0f, 0.0f, 1.0f);
  light.use_native_attenuation = 0.0f;
  return light;
}

EE_CAMERA_GLTF_LIGHT EE_CAMERA_DIRECTIONAL_TO_GLTF_LIGHT(const DirectionalLight light) {
  EE_CAMERA_GLTF_LIGHT gltf_light = EE_CAMERA_DEFAULT_GLTF_LIGHT();
  const EE_CAMERA_RADIANCE_SPLIT radiance = EE_CAMERA_SPLIT_LIGHT_RADIANCE(max(light.diffuse.rgb, vec3(0.0f)));
  gltf_light.direction = EE_CAMERA_SAFE_NORMALIZE(light.direction, vec3(0.0f, -1.0f, 0.0f));
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
  const EE_CAMERA_RADIANCE_SPLIT radiance = EE_CAMERA_SPLIT_LIGHT_RADIANCE(max(light.diffuse.rgb, vec3(0.0f)));
  gltf_light.type = EE_CAMERA_LIGHT_TYPE_POINT;
  gltf_light.position = light.position;
  gltf_light.radius = EE_CAMERA_REFERENCE_LIGHT_RADIUS(light.reserved_parameters.y);
  gltf_light.color = radiance.color;
  gltf_light.intensity = radiance.intensity;
  gltf_light.angular_size_or_inv_range = EE_CAMERA_REFERENCE_INV_RANGE(light.constant_linear_quadratic_far.w);
  gltf_light.cast_shadow = light.diffuse.w;
  gltf_light.attenuation = max(light.constant_linear_quadratic_far.xyz, vec3(0.0f));
  gltf_light.use_native_attenuation =
      dot(gltf_light.attenuation, vec3(1.0f)) > EE_CAMERA_PDF_EPSILON ? 1.0f : 0.0f;
  return gltf_light;
}

EE_CAMERA_GLTF_LIGHT EE_CAMERA_SPOT_TO_GLTF_LIGHT(const SpotLight light) {
  EE_CAMERA_GLTF_LIGHT gltf_light = EE_CAMERA_DEFAULT_GLTF_LIGHT();
  const EE_CAMERA_RADIANCE_SPLIT radiance = EE_CAMERA_SPLIT_LIGHT_RADIANCE(max(light.diffuse.rgb, vec3(0.0f)));
  gltf_light.direction = EE_CAMERA_SAFE_NORMALIZE(light.direction, vec3(0.0f, -1.0f, 0.0f));
  gltf_light.type = EE_CAMERA_LIGHT_TYPE_SPOT;
  gltf_light.position = light.position;
  gltf_light.radius = EE_CAMERA_REFERENCE_LIGHT_RADIUS(light.cutoff_outer_inner_size_bias.z);
  gltf_light.color = radiance.color;
  gltf_light.intensity = radiance.intensity;
  gltf_light.angular_size_or_inv_range = EE_CAMERA_REFERENCE_INV_RANGE(light.constant_linear_quadratic_far.w);
  gltf_light.inner_angle = acos(clamp(light.cutoff_outer_inner_size_bias.x, -1.0f, 1.0f));
  gltf_light.outer_angle = acos(clamp(light.cutoff_outer_inner_size_bias.y, -1.0f, 1.0f));
  gltf_light.cast_shadow = light.diffuse.w;
  gltf_light.attenuation = max(light.constant_linear_quadratic_far.xyz, vec3(0.0f));
  gltf_light.use_native_attenuation =
      dot(gltf_light.attenuation, vec3(1.0f)) > EE_CAMERA_PDF_EPSILON ? 1.0f : 0.0f;
  return gltf_light;
}

EE_CAMERA_GLTF_LIGHT EE_CAMERA_GET_GLTF_LIGHT(const int light_index) {
  const int directional_count = int(EE_RENDER_INFO.directional_light_size);
  const int point_count = int(EE_RENDER_INFO.point_light_size);
  if (light_index < directional_count) {
    const int directional_light_index = int(EE_CAMERA_INDEX) * MAX_DIRECTIONAL_LIGHT_SIZE + light_index;
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

float EE_CAMERA_SPOT_ATTENUATION(const vec3 point_to_light, const vec3 spot_direction, const float outer_cone_cos,
                                 const float inner_cone_cos) {
  const float actual_cos = dot(EE_CAMERA_SAFE_NORMALIZE(spot_direction, vec3(0.0f, -1.0f, 0.0f)),
                               EE_CAMERA_SAFE_NORMALIZE(-point_to_light, vec3(0.0f, -1.0f, 0.0f)));
  if (actual_cos <= outer_cone_cos) {
    return 0.0f;
  }
  if (actual_cos >= inner_cone_cos) {
    return 1.0f;
  }
  const float angular_attenuation = (actual_cos - outer_cone_cos) / max(inner_cone_cos - outer_cone_cos, 0.001f);
  return angular_attenuation * angular_attenuation;
}

EE_CAMERA_LIGHT_CONTRIB EE_CAMERA_SINGLE_LIGHT_CONTRIBUTION(const EE_CAMERA_GLTF_LIGHT light, const vec3 surface_pos,
                                                            const vec3 surface_normal, const vec2 random_value) {
  EE_CAMERA_LIGHT_CONTRIB contrib;
  contrib.incident_vector = vec3(0.0f);
  contrib.intensity = vec3(0.0f);
  contrib.distance = EE_CAMERA_FAR(int(EE_CAMERA_INDEX));
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
    const vec3 light_to_surface = surface_pos - light.position;
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
      one_minus_cos_half_angle =
          radius_over_distance * radius_over_distance /
          (radius_over_distance_2_plus_1 + sqrt(radius_over_distance_2_plus_1));
    }

    if (light.use_native_attenuation > 0.5f) {
      const float native_attenuation =
          light.attenuation.x + light.attenuation.y * distance + light.attenuation.z * distance * distance;
      irradiance = light.intensity / max(native_attenuation, EE_CAMERA_PDF_EPSILON);
    } else if (light.radius > 0.0f) {
      irradiance = light.intensity / max(light.radius * light.radius, EE_CAMERA_PDF_EPSILON) *
                   (2.0f * one_minus_cos_half_angle);
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
    const vec3 cone_direction = vec3(cos(cone_phi) * cone_sin, sin(cone_phi) * cone_sin, cone_z);
    const vec3 center_surface_to_light = -contrib.incident_vector;
    const vec3 sampled_surface_to_light =
        EE_CAMERA_SAFE_NORMALIZE(EE_CAMERA_TANGENT_SPACE(center_surface_to_light) * cone_direction,
                                 center_surface_to_light);
    contrib.incident_vector = -sampled_surface_to_light;
    contrib.pdf = (1.0f / (2.0f * EE_CAMERA_PI)) / max(one_minus_cos_half_angle, 1e-10f);
  }
  return contrib;
}

void EE_CAMERA_SAMPLE_DIRECT_LIGHT(const EE_CAMERA_SURFACE_HIT hit, const vec3 view_direction, inout uint seed,
                                   out EE_CAMERA_DIRECT_LIGHT direct_light) {
  direct_light.direction = hit.normal;
  direct_light.radiance_over_pdf = vec3(0.0f);
  direct_light.distance = EE_CAMERA_FAR(int(EE_CAMERA_INDEX));
  direct_light.pdf = 0.0f;
  direct_light.cast_shadow = 0.0f;

  const EE_CAMERA_LIGHT_TECHNIQUE_PROBABILITIES probabilities =
      EE_CAMERA_GET_DIRECT_LIGHTING_TECHNIQUE_PROBABILITIES();
  const float light_weight = probabilities.light_weight;
  const float environment_weight = probabilities.environment_weight;
  if (light_weight == 0.0f && environment_weight == 0.0f) {
    return;
  }

  vec3 radiance = vec3(0.0f);
  float environment_pdf = 0.0f;
  const bool sample_light = EE_RANDOM(seed) < light_weight;
  if (sample_light) {
    const int light_count = EE_CAMERA_PUNCTUAL_LIGHT_COUNT();
    const float selection_pdf = 1.0f / float(light_count);
    const int light_index = min(int(EE_RANDOM(seed) * float(light_count)), light_count - 1);
    const EE_CAMERA_GLTF_LIGHT light = EE_CAMERA_GET_GLTF_LIGHT(light_index);
    const EE_CAMERA_LIGHT_CONTRIB contrib =
        EE_CAMERA_SINGLE_LIGHT_CONTRIBUTION(light, hit.position, hit.normal, vec2(EE_RANDOM(seed), EE_RANDOM(seed)));

    direct_light.direction = -contrib.incident_vector;
    direct_light.distance = contrib.distance;
    direct_light.cast_shadow = contrib.cast_shadow;
    radiance = contrib.intensity / max(selection_pdf * light_weight, EE_CAMERA_PDF_EPSILON);
    direct_light.pdf = contrib.pdf == EE_CAMERA_DIRAC_PDF ? EE_CAMERA_DIRAC_PDF : selection_pdf * contrib.pdf;
  }

  if (environment_weight > 0.0f && direct_light.pdf != EE_CAMERA_DIRAC_PDF) {
    if (!sample_light) {
      const EE_CAMERA_DIRECTION_SAMPLE environment_sample = EE_CAMERA_SAMPLE_PATH_ENVIRONMENT(seed);
      direct_light.direction = environment_sample.direction;
      direct_light.distance = EE_CAMERA_FAR(int(EE_CAMERA_INDEX));
      direct_light.cast_shadow = 1.0f;
      environment_pdf = environment_sample.pdf;
      if (environment_pdf > EE_CAMERA_PDF_EPSILON) {
        radiance = EE_CAMERA_PATH_ENVIRONMENT_RADIANCE(direct_light.direction) /
                   max(environment_pdf * environment_weight, EE_CAMERA_PDF_EPSILON);
      }
    } else {
      environment_pdf = EE_CAMERA_ENVIRONMENT_PDF(direct_light.direction);
    }
  }

  float mis_weight = 1.0f;
  if (direct_light.pdf != EE_CAMERA_DIRAC_PDF) {
    const float pdf_sum = light_weight * direct_light.pdf + environment_weight * environment_pdf;
    if (pdf_sum > EE_CAMERA_PDF_EPSILON) {
      mis_weight = sample_light ? light_weight * direct_light.pdf / pdf_sum : environment_weight * environment_pdf /
                                                                                 pdf_sum;
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
  bounce.contribution = vec3(0.0f);
  bounce.shadow_ray_origin = vec3(0.0f);
  bounce.shadow_ray_direction = vec3(0.0f, 1.0f, 0.0f);
  bounce.shadow_ray_distance = 0.0f;
  bounce.cast_shadow = 0.0f;
  return bounce;
}

void EE_CAMERA_PREPARE_DIRECT_LIGHTING(const EE_CAMERA_SURFACE_HIT hit, const vec3 view_direction,
                                       const vec3 throughput, inout uint seed,
                                       inout EE_CAMERA_BOUNCE_SCRATCH bounce) {
  EE_CAMERA_DIRECT_LIGHT direct_light;
  EE_CAMERA_SAMPLE_DIRECT_LIGHT(hit, view_direction, seed, direct_light);
  if (direct_light.pdf == 0.0f) {
    return;
  }
  if (dot(direct_light.direction, hit.shading_normal) <= 0.0f && hit.pbr.diffuse_transmission_factor <= 0.0f) {
    return;
  }

  const bool shadow_side_forward = dot(direct_light.direction, hit.shading_normal) > 0.0f;
  const vec3 offset_direction = shadow_side_forward ? hit.geometric_normal : -hit.geometric_normal;
  const vec3 shadow_base = shadow_side_forward &&
                                   dot(hit.shadow_position - hit.position, hit.geometric_normal) >= -EE_CAMERA_RAY_EPSILON
                               ? hit.shadow_position
                               : hit.position;
  bounce.next_event_valid = true;
  bounce.shadow_ray_origin = EE_CAMERA_SAFE_OFFSET_RAY(shadow_base, offset_direction);
  bounce.shadow_ray_direction = direct_light.direction;
  bounce.shadow_ray_distance = max(direct_light.distance, 0.0f);
  bounce.cast_shadow = direct_light.cast_shadow;

  float bsdf_pdf = 0.0f;
  const vec3 bsdf_radiance = EE_CAMERA_EVALUATE_DIRECT_BSDF(
      hit, view_direction, direct_light.direction, direct_light.radiance_over_pdf, seed, bsdf_pdf);
  const float mis_weight = EE_CAMERA_BALANCE_HEURISTIC(direct_light.pdf, bsdf_pdf);
  const vec3 contribution = EE_CAMERA_SANITIZE_RADIANCE(throughput * bsdf_radiance * mis_weight);
  if (max(contribution.x, max(contribution.y, contribution.z)) <= EE_CAMERA_PDF_EPSILON) {
    return;
  }
  bounce.contribution = contribution;
}

vec3 EE_CAMERA_RESOLVE_DIRECT_LIGHTING(const EE_CAMERA_BOUNCE_SCRATCH bounce, inout uint seed) {
  if (!bounce.next_event_valid) {
    return vec3(0.0f);
  }
  if (bounce.cast_shadow != 1.0f) {
    return bounce.contribution;
  }

  const vec3 shadow_transmission = EE_CAMERA_SHADOW_TRANSMISSION(
      bounce.shadow_ray_origin, bounce.shadow_ray_direction, bounce.shadow_ray_distance, seed, false);
  if (max(max(shadow_transmission.x, shadow_transmission.y), shadow_transmission.z) <= EE_CAMERA_PDF_EPSILON) {
    return vec3(0.0f);
  }
  return EE_CAMERA_SANITIZE_RADIANCE(bounce.contribution * shadow_transmission);
}

vec3 EE_CAMERA_DIRECT_LIGHTING(const EE_CAMERA_SURFACE_HIT hit, const vec3 view_direction, inout uint seed) {
  EE_CAMERA_BOUNCE_SCRATCH bounce = EE_CAMERA_EMPTY_BOUNCE_SCRATCH();
  EE_CAMERA_PREPARE_DIRECT_LIGHTING(hit, view_direction, vec3(1.0f), seed, bounce);
  return EE_CAMERA_RESOLVE_DIRECT_LIGHTING(bounce, seed);
}

EE_CAMERA_VOLUME_MEDIUM EE_CAMERA_EMPTY_VOLUME_MEDIUM() {
  EE_CAMERA_VOLUME_MEDIUM medium;
  medium.extinction = vec3(0.0f);
  medium.scatter_coefficient = vec3(0.0f);
  medium.scatter_anisotropy = 0.0f;
  return medium;
}

EE_CAMERA_VOLUME_MEDIUM EE_CAMERA_MAKE_VOLUME_MEDIUM(const GltfRayTracingPbrMaterial pbr) {
  EE_CAMERA_VOLUME_MEDIUM medium;
  medium.extinction = EE_GLTF_RT_VOLUME_EXTINCTION_COEFFICIENT(pbr);
  medium.scatter_coefficient = max(pbr.scatter_coefficient, vec3(0.0f));
  medium.scatter_anisotropy = clamp(pbr.scatter_anisotropy, -0.99f, 0.99f);
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

vec3 EE_CAMERA_SAMPLE_HENYEY_GREENSTEIN(const vec2 xi, const float anisotropy, const vec3 wi) {
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
  const vec3 tangent_direction = vec3(cos(phi) * sin_theta, sin(phi) * sin_theta, cos_theta);
  return EE_CAMERA_SAFE_NORMALIZE(EE_CAMERA_TANGENT_SPACE(EE_CAMERA_SAFE_NORMALIZE(wi, vec3(0.0f, 0.0f, 1.0f))) *
                                      tangent_direction,
                                  wi);
}

vec3 EE_CAMERA_VOLUME_SCATTER_NEE(const EE_CAMERA_VOLUME_MEDIUM medium, const vec3 scatter_position,
                                  const vec3 wi_before_scatter, const vec3 throughput, inout uint seed) {
  EE_CAMERA_DIRECT_LIGHT direct_light;
  direct_light.direction = wi_before_scatter;
  direct_light.radiance_over_pdf = vec3(0.0f);
  direct_light.distance = EE_CAMERA_FAR(int(EE_CAMERA_INDEX));
  direct_light.pdf = 0.0f;
  direct_light.cast_shadow = 0.0f;

  const EE_CAMERA_LIGHT_TECHNIQUE_PROBABILITIES probabilities =
      EE_CAMERA_GET_DIRECT_LIGHTING_TECHNIQUE_PROBABILITIES();
  const float light_weight = probabilities.light_weight;
  const float environment_weight = probabilities.environment_weight;
  if (light_weight == 0.0f && environment_weight == 0.0f) {
    return vec3(0.0f);
  }

  vec3 radiance = vec3(0.0f);
  float environment_pdf = 0.0f;
  const bool sample_light = EE_RANDOM(seed) < light_weight;
  if (sample_light) {
    const int light_count = EE_CAMERA_PUNCTUAL_LIGHT_COUNT();
    const float selection_pdf = 1.0f / float(light_count);
    const int light_index = min(int(EE_RANDOM(seed) * float(light_count)), light_count - 1);
    const EE_CAMERA_GLTF_LIGHT light = EE_CAMERA_GET_GLTF_LIGHT(light_index);
    const EE_CAMERA_LIGHT_CONTRIB contrib = EE_CAMERA_SINGLE_LIGHT_CONTRIBUTION(
        light, scatter_position, wi_before_scatter, vec2(EE_RANDOM(seed), EE_RANDOM(seed)));

    direct_light.direction = -contrib.incident_vector;
    direct_light.distance = contrib.distance;
    direct_light.cast_shadow = contrib.cast_shadow;
    radiance = contrib.intensity / max(selection_pdf * light_weight, EE_CAMERA_PDF_EPSILON);
    direct_light.pdf = contrib.pdf == EE_CAMERA_DIRAC_PDF ? EE_CAMERA_DIRAC_PDF : selection_pdf * contrib.pdf;
  }

  if (environment_weight > 0.0f && direct_light.pdf != EE_CAMERA_DIRAC_PDF) {
    if (!sample_light) {
      const EE_CAMERA_DIRECTION_SAMPLE environment_sample = EE_CAMERA_SAMPLE_PATH_ENVIRONMENT(seed);
      direct_light.direction = environment_sample.direction;
      direct_light.distance = EE_CAMERA_FAR(int(EE_CAMERA_INDEX));
      direct_light.cast_shadow = 1.0f;
      environment_pdf = environment_sample.pdf;
      if (environment_pdf > EE_CAMERA_PDF_EPSILON) {
        radiance = EE_CAMERA_PATH_ENVIRONMENT_RADIANCE(direct_light.direction) /
                   max(environment_pdf * environment_weight, EE_CAMERA_PDF_EPSILON);
      }
    } else {
      environment_pdf = EE_CAMERA_ENVIRONMENT_PDF(direct_light.direction);
    }
  }

  float light_mis_weight = 1.0f;
  if (direct_light.pdf != EE_CAMERA_DIRAC_PDF) {
    const float pdf_sum = light_weight * direct_light.pdf + environment_weight * environment_pdf;
    light_mis_weight = pdf_sum > EE_CAMERA_PDF_EPSILON
                           ? (sample_light ? light_weight * direct_light.pdf : environment_weight * environment_pdf) /
                                 pdf_sum
                           : 0.0f;
    direct_light.pdf = pdf_sum;
  }
  direct_light.radiance_over_pdf = radiance * light_mis_weight;

  if (max(direct_light.radiance_over_pdf.x, max(direct_light.radiance_over_pdf.y,
                                                direct_light.radiance_over_pdf.z)) <= EE_CAMERA_PDF_EPSILON) {
    return vec3(0.0f);
  }

  if (direct_light.cast_shadow == 1.0f) {
    const vec3 shadow_origin = scatter_position + direct_light.direction * EE_CAMERA_RAY_EPSILON;
    const vec3 shadow_transmission = EE_CAMERA_SHADOW_TRANSMISSION(
        shadow_origin, direct_light.direction, direct_light.distance - EE_CAMERA_RAY_EPSILON, seed, true);
    if (max(max(shadow_transmission.x, shadow_transmission.y), shadow_transmission.z) <= EE_CAMERA_PDF_EPSILON) {
      return vec3(0.0f);
    }
    direct_light.radiance_over_pdf *= shadow_transmission;
  }

  const float phase_pdf = EE_CAMERA_HENYEY_GREENSTEIN_PDF(dot(wi_before_scatter, direct_light.direction),
                                                          medium.scatter_anisotropy);
  const float mis_weight = EE_CAMERA_BALANCE_HEURISTIC(direct_light.pdf, phase_pdf);
  return EE_CAMERA_SANITIZE_RADIANCE(throughput * direct_light.radiance_over_pdf * mis_weight * phase_pdf);
}

bool EE_CAMERA_PROCESS_VOLUME_SEGMENT(const float hit_distance, inout vec3 ray_origin, inout vec3 ray_direction,
                                      inout vec3 throughput, inout vec3 radiance, inout float last_sample_pdf,
                                      const EE_CAMERA_VOLUME_MEDIUM medium, inout uint seed,
                                      inout uint scatter_bounces) {
  if (!EE_CAMERA_HAS_VOLUME_MEDIUM(medium)) {
    return false;
  }

  const vec3 extinction = max(medium.extinction, vec3(0.0f));
  const vec3 scatter_coefficient = min(max(medium.scatter_coefficient, vec3(0.0f)), extinction);
  const float max_scatter =
      max(scatter_coefficient.x, max(scatter_coefficient.y, scatter_coefficient.z));
  if (max_scatter > EE_CAMERA_VOLUME_MIN_SCATTER) {
    const float max_extinction = max(extinction.x, max(extinction.y, extinction.z));
    const float scatter_distance =
        -log(max(EE_RANDOM(seed), EE_CAMERA_VOLUME_RAND_FLOOR)) / max(max_extinction, EE_CAMERA_PDF_EPSILON);
    if (scatter_distance < hit_distance) {
      throughput *=
          clamp(vec3(1.0f) - (extinction - scatter_coefficient) / max(max_extinction, EE_CAMERA_PDF_EPSILON),
                vec3(0.0f), vec3(1.0f));

      const vec3 wi_before_scatter = ray_direction;
      ray_origin += ray_direction * scatter_distance;
      ray_direction = EE_CAMERA_SAMPLE_HENYEY_GREENSTEIN(vec2(EE_RANDOM(seed), EE_RANDOM(seed)),
                                                         medium.scatter_anisotropy, wi_before_scatter);
      last_sample_pdf = EE_CAMERA_HENYEY_GREENSTEIN_PDF(dot(wi_before_scatter, ray_direction),
                                                        medium.scatter_anisotropy);
      radiance += EE_CAMERA_VOLUME_SCATTER_NEE(medium, ray_origin, wi_before_scatter, throughput, seed);
      scatter_bounces += 1u;
      return true;
    }

    throughput *= exp(hit_distance * (vec3(max_extinction) - extinction));
    return false;
  }

  throughput *= exp(-hit_distance * extinction);
  return false;
}

EE_CAMERA_SURFACE_HIT EE_CAMERA_RECONSTRUCT_SURFACE_HIT(const bool is_inside, const vec3 ray_direction,
                                                        const float ray_cone_width) {
  EE_CAMERA_SURFACE_HIT hit;
  const Instance instance = EE_INSTANCES[hit_value.instance_index];
  const int triangle_offset = instance.triangle_offset + int(hit_value.primitive_id);

  const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
  const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
  const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];
  const vec3 barycentrics =
      vec3(1.0f - hit_value.barycentrics.x - hit_value.barycentrics.y, hit_value.barycentrics.x,
           hit_value.barycentrics.y);

  const vec3 object_position = v0.position * barycentrics.x + v1.position * barycentrics.y +
                               v2.position * barycentrics.z;
  const vec3 object_geometric_normal = EE_CAMERA_SAFE_NORMALIZE(cross(v1.position - v0.position,
                                                                      v2.position - v0.position),
                                                                vec3(0.0f, 1.0f, 0.0f));
  const vec2 tex_coord_0 = v0.tex_coord * barycentrics.x + v1.tex_coord * barycentrics.y +
                           v2.tex_coord * barycentrics.z;
  const vec2 tex_coord_1 = v0.tex_coord_1 * barycentrics.x + v1.tex_coord_1 * barycentrics.y +
                           v2.tex_coord_1 * barycentrics.z;
  const vec4 vertex_color = v0.color * barycentrics.x + v1.color * barycentrics.y + v2.color * barycentrics.z;
  const vec3 object_normal = v0.normal * barycentrics.x + v1.normal * barycentrics.y +
                             v2.normal * barycentrics.z;
  const vec3 object_tangent = v0.tangent * barycentrics.x + v1.tangent * barycentrics.y +
                              v2.tangent * barycentrics.z;
  const float tangent_handedness =
      (v0.vertex_info3 < 0.0f ? -1.0f : 1.0f) * EE_TRANSFORM_HANDEDNESS(instance.model);
  const mat3 normal_matrix = transpose(inverse(mat3(instance.model)));

  hit.material_index = hit_value.material_index;
  hit.position = vec3(instance.model * vec4(object_position, 1.0f));
  hit.geometric_normal = EE_CAMERA_SAFE_NORMALIZE(hit_value.geometric_normal, vec3(0.0f, 1.0f, 0.0f));
  const vec3 unflipped_geometric_normal = EE_CAMERA_SAFE_NORMALIZE(normal_matrix * object_geometric_normal,
                                                                   hit.geometric_normal);
  const float side_flip = dot(unflipped_geometric_normal, ray_direction) < 0.0f ? 1.0f : -1.0f;
  const vec3 v0_shadow_normal = v0.normal * side_flip;
  const vec3 v1_shadow_normal = v1.normal * side_flip;
  const vec3 v2_shadow_normal = v2.normal * side_flip;
  const vec3 object_shadow_position = EE_CAMERA_POINT_OFFSET(object_position, v0.position, v1.position, v2.position,
                                                             v0_shadow_normal, v1_shadow_normal, v2_shadow_normal,
                                                             barycentrics);
  hit.shadow_position = vec3(instance.model * vec4(object_shadow_position, 1.0f));
  hit.tex_gradients = EE_CAMERA_TEXTURE_GRADIENTS(ray_cone_width, hit_value.hit_t, hit.geometric_normal,
                                                  ray_direction, instance.model, v0, v1, v2);
  hit.surface = EE_EVALUATE_GLTF_RASTER_SURFACE(hit.material_index, tex_coord_0, tex_coord_1, vertex_color,
                                                hit.tex_gradients);
  hit.normal = EE_CAMERA_SAFE_NORMALIZE(normal_matrix * object_normal, hit.geometric_normal);
  const vec3 world_tangent = mat3(instance.model) * object_tangent;
  hit.tangent = EE_CAMERA_SAFE_NORMALIZE(world_tangent - hit.normal * dot(world_tangent, hit.normal),
                                          vec3(1.0f, 0.0f, 0.0f));
  hit.bitangent = EE_CAMERA_SAFE_NORMALIZE(cross(hit.normal, hit.tangent) * tangent_handedness,
                                           vec3(0.0f, 0.0f, 1.0f));
  if (dot(hit.normal, hit.geometric_normal) < 0.0f) {
    hit.normal = -hit.normal;
    hit.tangent = -hit.tangent;
    hit.bitangent = -hit.bitangent;
  }
  const vec3 reflected_direction = reflect(EE_CAMERA_SAFE_NORMALIZE(ray_direction, -hit.geometric_normal), hit.normal);
  if (dot(reflected_direction, hit.geometric_normal) < 0.0f) {
    hit.normal = hit.geometric_normal;
  }
  hit.shading_normal = hit.normal;
  hit.tex_coord_0 = tex_coord_0;
  hit.tex_coord_1 = tex_coord_1;
  hit.vertex_color = vertex_color;
  hit.pbr = EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(
      hit.material_index, tex_coord_0, tex_coord_1, vertex_color, hit.normal, hit.tangent, hit.bitangent,
      hit.geometric_normal, is_inside, hit.tex_gradients);
  hit.normal = hit.pbr.normal;
  hit.tangent = hit.pbr.tangent;
  hit.bitangent = hit.pbr.bitangent;
  return hit;
}

vec3 EE_CAMERA_OFFSET_RAY_ORIGIN(const EE_CAMERA_SURFACE_HIT hit, const vec3 direction) {
  const vec3 offset_direction = dot(direction, hit.geometric_normal) > 0.0f ? hit.geometric_normal
                                                                            : -hit.geometric_normal;
  return EE_CAMERA_SAFE_OFFSET_RAY(hit.position, offset_direction);
}

vec3 EE_CAMERA_TRACE_PATH(inout uint seed, vec3 ray_origin, vec3 ray_direction, out float primary_hit_distance) {
  const Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
  const uint max_depth = max(camera.bounce, 1u);
  vec3 radiance = vec3(0.0f);
  vec3 throughput = vec3(1.0f);
  vec2 max_roughness = vec2(0.0f);
  float last_sample_pdf = EE_CAMERA_DIRAC_PDF;
  float ray_cone_width = 0.0f;
  bool is_inside = false;
  EE_CAMERA_VOLUME_MEDIUM volume_medium = EE_CAMERA_EMPTY_VOLUME_MEDIUM();
  uint scatter_bounces = 0u;
  primary_hit_distance = EE_CAMERA_FAR(int(EE_CAMERA_INDEX));

  uint surface_depth = 0u;
  while (surface_depth < max_depth) {
    EE_CAMERA_RESET_PAYLOAD(seed, last_sample_pdf);
    EE_CAMERA_TRACE_SURFACE(ray_origin, ray_direction,
                            surface_depth == 0u ? 0.0f : EE_CAMERA_RAY_EPSILON, seed);
    seed = hit_value.seed;

    if (hit_value.hit_count == 0u) {
      if (surface_depth == 0u) {
        radiance += throughput * hit_value.color;
        break;
      }
      const float environment_pdf =
          hit_value.environment_pdf > 0.0f ? hit_value.environment_pdf : 1.0f / (4.0f * EE_CAMERA_PI);
      const float environment_weight = EE_CAMERA_GET_DIRECT_LIGHTING_TECHNIQUE_PROBABILITIES().environment_weight;
      const float mis_weight =
          EE_CAMERA_ENVIRONMENT_HIT_MIS_WEIGHT(last_sample_pdf, environment_pdf, environment_weight);
      radiance += throughput * mis_weight * hit_value.environment_radiance;
      break;
    }

    if (surface_depth == 0u) {
      primary_hit_distance = hit_value.hit_t;
      hit_value.initial_position = hit_value.position;
    }

    EE_CAMERA_SURFACE_HIT surface_hit =
        EE_CAMERA_RECONSTRUCT_SURFACE_HIT(is_inside, ray_direction, ray_cone_width);
    if (surface_depth == 0u) {
      hit_value.initial_normal = surface_hit.normal;
    }
    max_roughness = max(max_roughness, surface_hit.pbr.roughness);
    surface_hit.pbr.roughness = max_roughness;
    const vec3 view_direction = EE_CAMERA_SAFE_NORMALIZE(-ray_direction, surface_hit.normal);
    radiance += throughput * surface_hit.pbr.emissive;

#if MAT_EXT_UNLIT
    if (EE_GLTF_MATERIALS[surface_hit.material_index].unlit > 0) {
      radiance += throughput * surface_hit.surface.base_color.rgb;
      break;
    }
#endif

    if (is_inside && EE_CAMERA_PROCESS_VOLUME_SEGMENT(hit_value.hit_t, ray_origin, ray_direction, throughput,
                                                       radiance, last_sample_pdf, volume_medium, seed,
                                                       scatter_bounces)) {
      throughput = EE_CAMERA_SANITIZE_RADIANCE(throughput);
      if (max(throughput.x, max(throughput.y, throughput.z)) <= EE_CAMERA_PDF_EPSILON) {
        break;
      }
      if (scatter_bounces >= EE_CAMERA_VOLUME_FREE_BUDGET) {
        const float continue_probability = clamp(max(throughput.x, max(throughput.y, throughput.z)) + 0.001f,
                                                 0.001f, 0.95f);
        if (EE_RANDOM(seed) >= continue_probability) {
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
    ray_cone_width =
        EE_CAMERA_WORLD_FOOTPRINT(ray_cone_width, hit_value.hit_t, surface_hit.geometric_normal, ray_direction);

    EE_CAMERA_BOUNCE_SCRATCH bounce = EE_CAMERA_EMPTY_BOUNCE_SCRATCH();
    EE_CAMERA_PREPARE_DIRECT_LIGHTING(surface_hit, view_direction, throughput, seed, bounce);

    GltfRayTracingBsdfSampleData sample_data;
    sample_data.k1 = view_direction;
    sample_data.xi = vec3(EE_RANDOM(seed), EE_RANDOM(seed), EE_RANDOM(seed));
    EE_GLTF_RT_BSDF_SAMPLE(sample_data, surface_hit.pbr);
    bool terminate_path = (sample_data.pdf <= EE_CAMERA_PDF_EPSILON &&
                           sample_data.pdf != EE_GLTF_RT_BSDF_DIRAC_PDF) ||
                          sample_data.event_type == EE_GLTF_RT_BSDF_EVENT_ABSORB;
    if (!terminate_path) {
      throughput *= sample_data.bsdf_over_pdf;
      throughput = EE_CAMERA_SANITIZE_RADIANCE(throughput);
      if (max(throughput.x, max(throughput.y, throughput.z)) <= EE_CAMERA_PDF_EPSILON) {
        terminate_path = true;
      } else {
        ray_origin = EE_CAMERA_OFFSET_RAY_ORIGIN(surface_hit, sample_data.k2);
        ray_direction = sample_data.k2;
        last_sample_pdf = sample_data.pdf;
        if (sample_data.event_type == EE_GLTF_RT_BSDF_EVENT_GLOSSY_TRANSMISSION &&
            surface_hit.pbr.thickness > 0.0f) {
          const bool entered_volume = !is_inside;
          is_inside = !is_inside;
          volume_medium = entered_volume ? EE_CAMERA_MAKE_VOLUME_MEDIUM(surface_hit.pbr)
                                         : EE_CAMERA_EMPTY_VOLUME_MEDIUM();
        }
      }
    } else {
      surface_depth = max_depth;
    }

    radiance += EE_CAMERA_RESOLVE_DIRECT_LIGHTING(bounce, seed);
    if (terminate_path) {
      break;
    }

    if (surface_depth >= EE_CAMERA_RUSSIAN_ROULETTE_MIN_DEPTH) {
      const float continue_probability = clamp(max(throughput.x, max(throughput.y, throughput.z)) + 0.001f,
                                               0.001f, 0.95f);
      if (EE_RANDOM(seed) >= continue_probability) {
        break;
      }
      throughput /= continue_probability;
    }
    surface_depth += 1u;
  }

  return EE_CAMERA_SANITIZE_RADIANCE(radiance);
}

EE_CAMERA_PRIMARY_RAY EE_CAMERA_CREATE_PRIMARY_RAY(const Camera camera, const vec2 sample_position,
                                                   const vec2 sample_offset, const vec2 image_size) {
  const vec2 clip_coords = (sample_position + sample_offset) / image_size * 2.0f - 1.0f;
  vec4 view_position = camera.inverse_projection * vec4(clip_coords, -1.0f, 1.0f);
  view_position /= view_position.w;

  const vec3 origin = camera.inverse_view[3].xyz;
  vec4 world_position = camera.inverse_view * view_position;
  world_position /= world_position.w;

  EE_CAMERA_PRIMARY_RAY ray;
  ray.origin = origin;
  ray.direction =
      EE_CAMERA_SAFE_NORMALIZE(world_position.xyz - origin,
                               EE_CAMERA_SAFE_NORMALIZE((camera.inverse_view * vec4(0.0f, 0.0f, -1.0f, 0.0f)).xyz,
                                                        vec3(0.0f, 0.0f, -1.0f)));
  return ray;
}

vec2 EE_CAMERA_SAMPLE_GAUSSIAN(const vec2 value) {
  const float radius = sqrt(-2.0f * log(max(1e-38f, value.x)));
  const float theta = 2.0f * EE_CAMERA_PI * value.y;
  return radius * vec2(cos(theta), sin(theta));
}


void EE_CAMERA_RENDER_PIXEL(const uvec2 pixel_coordinate, const uvec2 image_size) {
  const Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
  uint seed = EE_XXHASH32(uvec3(pixel_coordinate, EE_FRAME_ID));

  const bool auto_spp_enabled = camera.auto_spp_enabled != 0u;
  const vec4 previous_history = EE_TOTAL_SAMPLES > 0u ? imageLoad(radiance_history_image, ivec2(pixel_coordinate))
                                                      : vec4(0.0f);
  const vec4 previous_auto_metadata =
      auto_spp_enabled && EE_TOTAL_SAMPLES > 0u ? imageLoad(convergence_history_image, ivec2(pixel_coordinate))
                                                : vec4(0.0f);
  const uint auto_spp_min_samples = max(camera.auto_spp_min_samples, 1u);
  const uint auto_spp_max_samples = max(camera.auto_spp_max_samples, auto_spp_min_samples);
  const uint previous_auto_samples = uint(max(previous_auto_metadata.x, 0.0f) + 0.5f);
  const bool auto_history_valid = auto_spp_enabled && EE_TOTAL_SAMPLES > 0u && previous_auto_samples > 0u;
  const bool auto_pixel_converged =
      auto_history_valid && previous_auto_samples >= auto_spp_min_samples && previous_auto_metadata.z > 0.5f;
  if (auto_spp_enabled && auto_history_valid &&
      (auto_pixel_converged || previous_auto_samples >= auto_spp_max_samples)) {
    imageStore(result_image, ivec2(pixel_coordinate), vec4(previous_history.xyz, 1.0f));
    imageStore(radiance_history_image, ivec2(pixel_coordinate), previous_history);
    imageStore(convergence_history_image, ivec2(pixel_coordinate),
               vec4(float(previous_auto_samples), previous_auto_metadata.y, 1.0f, 0.0f));
    return;
  }

  vec3 linear_radiance = vec3(0.0f);
  const uint manual_sample_size = max(camera.sample_size, 1u);
  const uint remaining_auto_samples =
      auto_spp_enabled && auto_history_valid ? auto_spp_max_samples - previous_auto_samples : auto_spp_max_samples;
  const uint sample_size = auto_spp_enabled ? min(manual_sample_size, max(remaining_auto_samples, 1u))
                                            : manual_sample_size;
  const uint previous_accumulated_samples = auto_spp_enabled && auto_history_valid ? previous_auto_samples
                                                                                  : EE_TOTAL_SAMPLES;
  const uint frame_sample_size = auto_spp_enabled ? sample_size : max(EE_FRAME_SAMPLE_SIZE, 1u);
  uint invalid_radiance_rejections = 0u;
  uint firefly_clamp_count = 0u;
  float primary_hit_distance = EE_CAMERA_FAR(int(EE_CAMERA_INDEX));
  vec2 sample_offset = vec2(0.5f) + EE_CAMERA_ANTIALIASING_STANDARD_DEVIATION *
                                         EE_CAMERA_SAMPLE_GAUSSIAN(
                                             vec2(EE_REFERENCE_RANDOM(seed), EE_REFERENCE_RANDOM(seed)));

  [[unroll]]
  for (uint i = 0u; i < sample_size; ++i) {
    const EE_CAMERA_PRIMARY_RAY primary_ray =
        EE_CAMERA_CREATE_PRIMARY_RAY(camera, vec2(pixel_coordinate), sample_offset, vec2(image_size));

    float sample_hit_distance = EE_CAMERA_FAR(int(EE_CAMERA_INDEX));
    vec3 sample_radiance =
        EE_CAMERA_REJECT_INVALID_RADIANCE(EE_CAMERA_TRACE_PATH(seed, primary_ray.origin, primary_ray.direction,
                                                               sample_hit_distance),
                                          invalid_radiance_rejections);
    sample_radiance = EE_CAMERA_APPLY_FIREFLY_CLAMP(camera, sample_radiance, firefly_clamp_count);
    linear_radiance += sample_radiance;
    primary_hit_distance = min(primary_hit_distance, sample_hit_distance);
    if (i + 1u < sample_size) {
      sample_offset = vec2(EE_REFERENCE_RANDOM(seed), EE_REFERENCE_RANDOM(seed));
    }
  }

  linear_radiance /= float(sample_size);
  if (previous_accumulated_samples > 0u) {
    const vec3 previous_linear_radiance = previous_history.xyz;
    linear_radiance = (previous_linear_radiance * float(previous_accumulated_samples) +
                       linear_radiance * float(frame_sample_size)) /
                      float(previous_accumulated_samples + frame_sample_size);
  }
  const uint accumulated_sample_count = previous_accumulated_samples + frame_sample_size;
  const float convergence_delta =
      previous_accumulated_samples > 0u ? EE_CAMERA_RELATIVE_LUMINANCE_DELTA(previous_history.xyz, linear_radiance)
                                        : 3.402823466e+38f;
  const bool auto_sample_converged =
      auto_spp_enabled && accumulated_sample_count >= auto_spp_min_samples &&
      (convergence_delta <= max(camera.auto_spp_convergence_threshold, 0.0f) ||
       accumulated_sample_count >= auto_spp_max_samples);

  imageStore(result_image, ivec2(pixel_coordinate), vec4(linear_radiance, 1.0f));
  imageStore(radiance_history_image, ivec2(pixel_coordinate),
             vec4(linear_radiance,
                  EE_CAMERA_PACK_SAMPLE_DIAGNOSTICS(invalid_radiance_rejections, firefly_clamp_count)));
  imageStore(convergence_history_image, ivec2(pixel_coordinate),
             auto_spp_enabled ? vec4(float(accumulated_sample_count), convergence_delta,
                                     auto_sample_converged ? 1.0f : 0.0f, 0.0f)
                              : vec4(0.0f));
  imageStore(ray_hit_distance_image, ivec2(pixel_coordinate), vec4(primary_hit_distance));
}

#endif
