#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_ray_tracing : require

#define EE_GLTF_USE_EXPLICIT_TEXTURE_LOD
#define EE_GLTF_TEXTURE_LOD 0.0
#include "CameraRayTracingPayload.glsl"
#include "Random.glsl"
#include "RayTracingBasic.glsl"
#include "GltfRasterMaterial.glsl"
layout(location = 0) rayPayloadInEXT CameraRayTracingPayload hit_value;

hitAttributeEXT vec2 attribs;

vec3 Reflect(in vec3 incident, in vec3 normal) {
  return incident - 2.0f * dot(incident, normal) * normal;
}

layout(push_constant) uniform EE_CAMERA_CONSTANTS {
  uint EE_CAMERA_INDEX;
  uint EE_FRAME_ID;
};

const float EE_CAMERA_PI = 3.14159265359f;
const float EE_CAMERA_RAY_EPSILON = 1e-3f;
const float EE_CAMERA_DIRECT_LIGHT_EXPOSURE = 0.001f;
const float EE_CAMERA_PDF_EPSILON = 1e-6f;
const float EE_CAMERA_DIRAC_PDF = -1.0f;
const uint EE_CAMERA_RAY_PAYLOAD_SURFACE = 0u;
const uint EE_CAMERA_RAY_PAYLOAD_SHADOW = 1u;
const uint EE_CAMERA_RAY_MASK_SHADOW = 0x02u;

struct EE_CAMERA_DIRECTION_SAMPLE {
  vec3 direction;
  float pdf;
};

vec3 EE_CAMERA_SAFE_NORMALIZE(const vec3 value, const vec3 fallback) {
  const float length_squared = dot(value, value);
  return length_squared > 0.00000001f ? value * inversesqrt(length_squared) : fallback;
}

vec3 EE_SKY_COLOR(vec3 direction) {
  Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
  return camera.use_clear_color == 1 ? camera.clear_color.xyz * camera.clear_color.w
                                     : pow(texture(EE_CUBEMAPS[camera.skybox_tex_index], normalize(direction)).rgb,
                                           vec3(1.0f / EE_ENVIRONMENT.gamma)) *
                                           camera.clear_color.w;
}

float EE_CAMERA_SANITIZE_FLOAT(const float value) {
  return value >= 0.0f && value < 3.402823466e+38f ? value : 0.0f;
}

vec3 EE_CAMERA_SANITIZE_RADIANCE(const vec3 value) {
  return vec3(EE_CAMERA_SANITIZE_FLOAT(value.x), EE_CAMERA_SANITIZE_FLOAT(value.y),
              EE_CAMERA_SANITIZE_FLOAT(value.z));
}

vec3 EE_CAMERA_PATH_ENVIRONMENT_RADIANCE(const vec3 ray_direction) {
  if (EE_ENVIRONMENT.light_intensity <= 0.0f) {
    return vec3(0.0f);
  }
  if (EE_ENVIRONMENT.background_color.w == 1.0f) {
    return EE_CAMERA_SANITIZE_RADIANCE(max(EE_ENVIRONMENT.background_color.rgb, vec3(0.0f)) *
                                       EE_ENVIRONMENT.light_intensity);
  }

  const int environment_index = EE_CAMERAS[EE_CAMERA_INDEX].prefiltered_map_index;
  vec3 environment_color = textureLod(EE_CUBEMAPS[environment_index], normalize(ray_direction), 0.0f).rgb;
  if (EE_ENVIRONMENT.gamma != 1.0f) {
    environment_color = pow(max(environment_color, vec3(0.0f)), vec3(1.0f / EE_ENVIRONMENT.gamma));
  }
  return EE_CAMERA_SANITIZE_RADIANCE(environment_color * EE_ENVIRONMENT.light_intensity);
}

bool EE_CAMERA_HAS_ENVIRONMENT_LIGHTING() {
  if (EE_ENVIRONMENT.light_intensity <= 0.0f) {
    return false;
  }
  if (EE_ENVIRONMENT.background_color.w == 1.0f) {
    return dot(max(EE_ENVIRONMENT.background_color.rgb, vec3(0.0f)), vec3(1.0f)) > 0.0f;
  }
  return true;
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

float EE_CAMERA_ENVIRONMENT_PDF(const vec3 normal, const vec3 light_direction) {
  if (!EE_CAMERA_HAS_ENVIRONMENT_LIGHTING() || dot(normal, light_direction) <= 0.0f) {
    return 0.0f;
  }
  return 1.0f / (2.0f * EE_CAMERA_PI);
}

mat3 GetTangentSpace(in vec3 normal) {
  // Choose a helper vector for the cross product
  vec3 helper = vec3(1.0f, 0.0f, 0.0f);
  if (abs(normal.x) > 0.99f)
    helper = vec3(0.0f, 0.0f, 1.0f);
  // Generate vectors
  const vec3 tangent = normalize(cross(normal, helper));
  const vec3 binormal = normalize(cross(normal, tangent));
  return mat3(tangent, binormal, normal);
}

float EE_CAMERA_HEMISPHERE_PDF(const float alpha, const vec3 normal, const vec3 direction) {
  const float cone_range = max((1.0f - clamp(alpha, 0.0f, 1.0f)) * (1.0f - clamp(alpha, 0.0f, 1.0f)),
                               EE_CAMERA_PDF_EPSILON);
  const float cone_cos_min = 1.0f - cone_range;
  return dot(EE_CAMERA_SAFE_NORMALIZE(normal, vec3(0.0f, 1.0f, 0.0f)),
             EE_CAMERA_SAFE_NORMALIZE(direction, vec3(0.0f, 1.0f, 0.0f))) >= cone_cos_min
             ? 1.0f / (2.0f * EE_CAMERA_PI * cone_range)
             : 0.0f;
}

EE_CAMERA_DIRECTION_SAMPLE EE_CAMERA_SAMPLE_HEMISPHERE(inout uint seed, in vec3 normal, in float alpha) {
  EE_CAMERA_DIRECTION_SAMPLE ray_sample;
  // Uniformly sample hemisphere direction
  const float cone_range = max((1.0f - clamp(alpha, 0.0f, 1.0f)) * (1.0f - clamp(alpha, 0.0f, 1.0f)),
                               EE_CAMERA_PDF_EPSILON);
  const float cosTheta = 1.0f - EE_RANDOM(seed) * cone_range;
  const float sinTheta = sqrt(max(0.0f, 1.0f - cosTheta * cosTheta));
  const float phi = 2.0f * 3.1415926f * EE_RANDOM(seed);
  const vec3 tangentSpaceDir = vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
  // Transform direction to world space
  ray_sample.direction = EE_CAMERA_SAFE_NORMALIZE(GetTangentSpace(normal) * tangentSpaceDir, normal);
  ray_sample.pdf = 1.0f / (2.0f * EE_CAMERA_PI * cone_range);
  return ray_sample;
}

EE_CAMERA_DIRECTION_SAMPLE EE_CAMERA_SAMPLE_BSDF(in float metallic, inout uint seed, in vec3 inDirection,
                                                 in vec3 inNormal) {
  const vec3 reflected = Reflect(inDirection, inNormal);
  return EE_CAMERA_SAMPLE_HEMISPHERE(seed, reflected, metallic);
}

float EE_CAMERA_BSDF_PDF(in float metallic, in vec3 inDirection, in vec3 inNormal, in vec3 sampleDirection) {
  const vec3 reflected = Reflect(inDirection, inNormal);
  return EE_CAMERA_HEMISPHERE_PDF(metallic, reflected, sampleDirection);
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

vec3 EE_CAMERA_EVALUATE_DIRECT_BRDF(const vec3 albedo, const float metallic, const float roughness,
                                    const float occlusion, const vec3 normal, const vec3 view_direction,
                                    const vec3 light_direction, const vec3 light_radiance) {
  const float n_dot_l = max(dot(normal, light_direction), 0.0f);
  const float n_dot_v = max(dot(normal, view_direction), 0.0f);
  if (n_dot_l <= 0.0f || n_dot_v <= 0.0f) {
    return vec3(0.0f);
  }

  const float clamped_roughness = clamp(roughness, 0.04f, 1.0f);
  const vec3 half_vector = EE_CAMERA_SAFE_NORMALIZE(view_direction + light_direction, normal);
  const vec3 f0 = mix(vec3(0.04f), albedo, clamp(metallic, 0.0f, 1.0f));
  const vec3 fresnel = EE_CAMERA_FRESNEL_SCHLICK(max(dot(half_vector, view_direction), 0.0f), f0);
  const float distribution = EE_CAMERA_DISTRIBUTION_GGX(normal, half_vector, clamped_roughness);
  const float geometry = EE_CAMERA_GEOMETRY_SMITH(normal, view_direction, light_direction, clamped_roughness);
  const vec3 specular = distribution * geometry * fresnel / max(4.0f * n_dot_v * n_dot_l, 0.001f);
  const vec3 diffuse = (vec3(1.0f) - fresnel) * (1.0f - clamp(metallic, 0.0f, 1.0f)) * albedo / EE_CAMERA_PI;
  return (diffuse + specular) * light_radiance * n_dot_l * occlusion;
}

bool EE_CAMERA_SHADOW_VISIBLE(const vec3 origin, const vec3 direction, const float max_distance) {
  if (max_distance <= EE_CAMERA_RAY_EPSILON) {
    return true;
  }
  const CameraRayTracingPayload primary_hit = hit_value;
  hit_value.type = EE_CAMERA_RAY_PAYLOAD_SHADOW;
  hit_value.hit_count = 1u;
  traceRayEXT(EE_TLAS, gl_RayFlagsOpaqueEXT | gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsSkipClosestHitShaderEXT,
              EE_CAMERA_RAY_MASK_SHADOW, 0, 0, 0, origin, EE_CAMERA_RAY_EPSILON, direction, max_distance, 0);
  const bool occluded = hit_value.hit_count != 0u;
  hit_value = primary_hit;
  return !occluded;
}

float EE_CAMERA_DISTANCE_LIGHT_ATTENUATION(const vec4 constant_linear_quadratic_far, const float light_distance) {
  if (light_distance <= EE_CAMERA_RAY_EPSILON || light_distance >= constant_linear_quadratic_far.w) {
    return 0.0f;
  }
  return 1.0f / max(constant_linear_quadratic_far.x + constant_linear_quadratic_far.y * light_distance +
                       constant_linear_quadratic_far.z * light_distance * light_distance,
                   0.001f);
}

float EE_CAMERA_LIGHT_BSDF_MIS_WEIGHT(const float light_pdf, const float bsdf_pdf) {
  return EE_CAMERA_BALANCE_HEURISTIC(light_pdf, bsdf_pdf);
}

vec3 EE_CAMERA_DIRECT_ENVIRONMENT_LIGHT(const vec3 albedo, const float metallic, const float roughness,
                                        const float occlusion, const vec3 normal, const vec3 position,
                                        const vec3 view_direction, inout uint seed) {
  if (!EE_CAMERA_HAS_ENVIRONMENT_LIGHTING()) {
    return vec3(0.0f);
  }

  const EE_CAMERA_DIRECTION_SAMPLE environment_sample = EE_CAMERA_SAMPLE_HEMISPHERE(seed, normal, 0.0f);
  const float environment_pdf = EE_CAMERA_ENVIRONMENT_PDF(normal, environment_sample.direction);
  if (environment_pdf <= EE_CAMERA_PDF_EPSILON) {
    return vec3(0.0f);
  }

  if (!EE_CAMERA_SHADOW_VISIBLE(position + normal * EE_CAMERA_RAY_EPSILON, environment_sample.direction,
                                EE_CAMERA_FAR(int(EE_CAMERA_INDEX)))) {
    return vec3(0.0f);
  }

  const float bsdf_pdf = EE_CAMERA_BSDF_PDF(metallic, -view_direction, normal, environment_sample.direction);
  const float mis_weight = EE_CAMERA_LIGHT_BSDF_MIS_WEIGHT(environment_pdf, bsdf_pdf);
  const vec3 environment_radiance = EE_CAMERA_PATH_ENVIRONMENT_RADIANCE(environment_sample.direction);
  return EE_CAMERA_EVALUATE_DIRECT_BRDF(albedo, metallic, roughness, occlusion, normal, view_direction,
                                        environment_sample.direction, environment_radiance) *
         (mis_weight / max(environment_pdf, EE_CAMERA_PDF_EPSILON));
}

vec3 EE_CAMERA_DIRECT_DIRECTIONAL_LIGHT(const DirectionalLight light, const vec3 albedo, const float metallic,
                                        const float roughness, const float occlusion, const vec3 normal,
                                        const vec3 position, const vec3 view_direction) {
  const vec3 light_direction = EE_CAMERA_SAFE_NORMALIZE(-light.direction, vec3(0.0f, 1.0f, 0.0f));
  if (dot(normal, light_direction) <= 0.0f) {
    return vec3(0.0f);
  }
  if (light.diffuse.w == 1.0f &&
      !EE_CAMERA_SHADOW_VISIBLE(position + normal * EE_CAMERA_RAY_EPSILON, light_direction,
                                EE_CAMERA_FAR(int(EE_CAMERA_INDEX)))) {
    return vec3(0.0f);
  }
  const float bsdf_pdf = EE_CAMERA_BSDF_PDF(metallic, -view_direction, normal, light_direction);
  const float mis_weight = EE_CAMERA_LIGHT_BSDF_MIS_WEIGHT(EE_CAMERA_DIRAC_PDF, bsdf_pdf);
  return EE_CAMERA_EVALUATE_DIRECT_BRDF(albedo, metallic, roughness, occlusion, normal, view_direction,
                                        light_direction, max(light.diffuse.rgb, vec3(0.0f))) *
         mis_weight;
}

vec3 EE_CAMERA_DIRECT_POINT_LIGHT(const PointLight light, const vec3 albedo, const float metallic,
                                  const float roughness, const float occlusion, const vec3 normal, const vec3 position,
                                  const vec3 view_direction) {
  const vec3 light_delta = light.position - position;
  const float light_distance = length(light_delta);
  const float attenuation = EE_CAMERA_DISTANCE_LIGHT_ATTENUATION(light.constant_linear_quadratic_far, light_distance);
  if (attenuation <= 0.0f) {
    return vec3(0.0f);
  }
  const vec3 light_direction = light_delta / light_distance;
  if (dot(normal, light_direction) <= 0.0f) {
    return vec3(0.0f);
  }
  if (light.diffuse.w == 1.0f &&
      !EE_CAMERA_SHADOW_VISIBLE(position + normal * EE_CAMERA_RAY_EPSILON, light_direction,
                                light_distance - EE_CAMERA_RAY_EPSILON)) {
    return vec3(0.0f);
  }
  const float bsdf_pdf = EE_CAMERA_BSDF_PDF(metallic, -view_direction, normal, light_direction);
  const float mis_weight = EE_CAMERA_LIGHT_BSDF_MIS_WEIGHT(EE_CAMERA_DIRAC_PDF, bsdf_pdf);
  return EE_CAMERA_EVALUATE_DIRECT_BRDF(albedo, metallic, roughness, occlusion, normal, view_direction,
                                        light_direction, max(light.diffuse.rgb, vec3(0.0f)) * attenuation) *
         mis_weight;
}

vec3 EE_CAMERA_DIRECT_SPOT_LIGHT(const SpotLight light, const vec3 albedo, const float metallic, const float roughness,
                                 const float occlusion, const vec3 normal, const vec3 position,
                                 const vec3 view_direction) {
  const vec3 light_delta = light.position - position;
  const float light_distance = length(light_delta);
  const float attenuation = EE_CAMERA_DISTANCE_LIGHT_ATTENUATION(light.constant_linear_quadratic_far, light_distance);
  if (attenuation <= 0.0f) {
    return vec3(0.0f);
  }
  const vec3 light_direction = light_delta / light_distance;
  if (dot(normal, light_direction) <= 0.0f) {
    return vec3(0.0f);
  }
  const float theta = dot(light_direction, EE_CAMERA_SAFE_NORMALIZE(-light.direction, vec3(0.0f, 0.0f, -1.0f)));
  const float epsilon = max(light.cutoff_outer_inner_size_bias.x - light.cutoff_outer_inner_size_bias.y, 0.001f);
  const float spot_intensity = clamp((theta - light.cutoff_outer_inner_size_bias.y) / epsilon, 0.0f, 1.0f);
  if (spot_intensity <= 0.0f) {
    return vec3(0.0f);
  }
  if (light.diffuse.w == 1.0f &&
      !EE_CAMERA_SHADOW_VISIBLE(position + normal * EE_CAMERA_RAY_EPSILON, light_direction,
                                light_distance - EE_CAMERA_RAY_EPSILON)) {
    return vec3(0.0f);
  }
  const float bsdf_pdf = EE_CAMERA_BSDF_PDF(metallic, -view_direction, normal, light_direction);
  const float mis_weight = EE_CAMERA_LIGHT_BSDF_MIS_WEIGHT(EE_CAMERA_DIRAC_PDF, bsdf_pdf);
  return EE_CAMERA_EVALUATE_DIRECT_BRDF(albedo, metallic, roughness, occlusion, normal, view_direction,
                                        light_direction,
                                        max(light.diffuse.rgb, vec3(0.0f)) * attenuation * spot_intensity) *
         mis_weight;
}

vec3 EE_CAMERA_DIRECT_LIGHTING(const vec3 albedo, const float metallic, const float roughness, const float occlusion,
                               const vec3 normal, const vec3 position, const vec3 view_direction, inout uint seed) {
  vec3 direct_lighting = vec3(0.0f);
  for (int i = 0; i < EE_RENDER_INFO.directional_light_size; ++i) {
    direct_lighting += EE_CAMERA_DIRECT_DIRECTIONAL_LIGHT(EE_DIRECTIONAL_LIGHTS[i], albedo, metallic, roughness,
                                                          occlusion, normal, position, view_direction);
  }
  for (int i = 0; i < EE_RENDER_INFO.point_light_size; ++i) {
    direct_lighting +=
        EE_CAMERA_DIRECT_POINT_LIGHT(EE_POINT_LIGHTS[i], albedo, metallic, roughness, occlusion, normal, position,
                                     view_direction);
  }
  for (int i = 0; i < EE_RENDER_INFO.spot_light_size; ++i) {
    direct_lighting += EE_CAMERA_DIRECT_SPOT_LIGHT(EE_SPOT_LIGHTS[i], albedo, metallic, roughness, occlusion, normal,
                                                   position, view_direction);
  }
  direct_lighting *= EE_CAMERA_DIRECT_LIGHT_EXPOSURE;
  direct_lighting += EE_CAMERA_DIRECT_ENVIRONMENT_LIGHT(albedo, metallic, roughness, occlusion, normal, position,
                                                       view_direction, seed);
  return EE_CAMERA_SANITIZE_RADIANCE(direct_lighting);
}

void main() {
  const int instance_index = int(gl_InstanceCustomIndexEXT);
  const Instance instance = EE_INSTANCES[instance_index];
  const uint material_index = uint(instance.material_index);

  const int triangle_offset = instance.triangle_offset + gl_PrimitiveID;

  // Vertex of the triangle
  const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
  const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
  const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];

  const vec3 barycentrics = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

  const vec3 position = v0.position * barycentrics.x + v1.position * barycentrics.y + v2.position * barycentrics.z;
  const vec2 tex_coord = v0.tex_coord * barycentrics.x + v1.tex_coord * barycentrics.y + v2.tex_coord * barycentrics.z;
  vec3 normal = v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z;
  const vec3 tangent = v0.tangent * barycentrics.x + v1.tangent * barycentrics.y + v2.tangent * barycentrics.z;
  const GltfRasterMaterial surface = EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, tex_coord, tex_coord);
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  normal = EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coord, tex_coord, normal, tangent);

  const vec3 worldPosition =
      vec3(gl_ObjectToWorldEXT * vec4(position, 1.0));  // Transforming the position to world space
  const vec3 worldNormal = EE_CAMERA_SAFE_NORMALIZE(vec3(normal * gl_WorldToObjectEXT),
                                                    vec3(0.0f, 1.0f, 0.0f));  // Transforming the normal to world space

  const float roughness = surface.roughness;
  const float metallic = surface.metallic;
  const vec4 albedo = surface.base_color;
  float deferred_bsdf_extension_weight = 0.0;
#if MAT_EXT_TRANSMISSION
  deferred_bsdf_extension_weight += material.transmission_factor;
#endif
#if MAT_EXT_IOR
  deferred_bsdf_extension_weight += material.ior;
#endif
#if MAT_EXT_CLEARCOAT
  deferred_bsdf_extension_weight += material.clearcoat_factor + material.clearcoat_roughness;
#endif
#if MAT_EXT_SHEEN
  deferred_bsdf_extension_weight += dot(material.sheen_color_factor, vec3(1.0)) + material.sheen_roughness_factor;
#endif

  // Proceed...
  float f = 1.0f;
  if (metallic >= 0.0f) {
    f = (metallic + 2) / (metallic + 1);
  }

  hit_value.hit_count += 1;
  hit_value.position = worldPosition;
  hit_value.normal = worldNormal;
  const Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
  const vec3 view_direction = EE_CAMERA_SAFE_NORMALIZE(-gl_WorldRayDirectionEXT, worldNormal);
  const vec3 direct_lighting =
      EE_CAMERA_DIRECT_LIGHTING(albedo.xyz, metallic, roughness, surface.occlusion, worldNormal, worldPosition,
                                view_direction, hit_value.seed);

  vec3 combined_color = vec3(0.0f, 0.0f, 0.0f);
  if (hit_value.hit_count <= camera.bounce) {
    const EE_CAMERA_DIRECTION_SAMPLE bsdf_sample =
        EE_CAMERA_SAMPLE_BSDF(metallic, hit_value.seed, gl_WorldRayDirectionEXT, worldNormal);
    hit_value.type = EE_CAMERA_RAY_PAYLOAD_SURFACE;
    hit_value.last_sample_pdf = bsdf_sample.pdf;
    traceRayEXT(EE_TLAS, gl_RayFlagsOpaqueEXT, 0xff, 0, 0, 0, worldPosition, 1e-3f, bsdf_sample.direction, 1e20f, 0);
    const vec3 received_color = hit_value.color;
    combined_color = albedo.xyz * surface.occlusion *
                     clamp(abs(dot(worldNormal, bsdf_sample.direction)) * roughness + (1.f - roughness) * f, 0.0f,
                           1.0f) *
                     received_color;
  } else {
    combined_color = EE_SKY_COLOR(worldNormal) * 1e-3f;
  }
  hit_value.color = EE_CAMERA_SANITIZE_RADIANCE(combined_color + direct_lighting + surface.emissive +
                                                vec3(0.0f) * deferred_bsdf_extension_weight);

  hit_value.initial_normal = worldNormal;
  hit_value.initial_position = worldPosition;
}
