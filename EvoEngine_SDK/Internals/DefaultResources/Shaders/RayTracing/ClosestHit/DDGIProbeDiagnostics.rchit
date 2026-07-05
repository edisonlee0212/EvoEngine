#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_ray_tracing : require

#define EE_GLTF_USE_EXPLICIT_TEXTURE_LOD
#define EE_GLTF_TEXTURE_LOD 0.0
#include "PointCloudRayTracingPayload.glsl"
#include "RayTracingBasic.glsl"
#include "GltfRasterMaterial.glsl"
#include "DDGI.glsl"

layout(location = 0) rayPayloadInEXT PointCloudRayTracingPayload hit_value;

hitAttributeEXT vec2 attribs;

layout(set = 2, binding = 17) uniform sampler2D EE_DDGI_IRRADIANCE_ATLAS;
layout(set = 2, binding = 18) uniform sampler2D EE_DDGI_VISIBILITY_ATLAS;
layout(set = 2, binding = 1) readonly buffer EE_DDGI_PROBE_STATE_BLOCK {
  vec4 EE_DDGI_PROBE_STATE[];
};

layout(push_constant) uniform EE_DDGI_PROBE_RAY_CONSTANTS {
  vec4 first_probe;
  vec4 probe_step_x;
  vec4 probe_step_y;
  vec4 probe_step_z;
  uvec4 probe_counts_and_ray_count;
  uvec4 probe_offset_and_update_count;
  vec4 trace_parameters;
};

vec3 EE_DDGI_RECURSIVE_IRRADIANCE(const vec3 albedo, const vec3 normal, const vec3 position) {
  const float intensity = EE_RENDER_INFO.ddgi_indirect_intensity;
  if (intensity <= 0.0f) {
    return vec3(0.0f);
  }

  const uvec3 probe_counts = max(uvec3(EE_RENDER_INFO.ddgi_probe_counts.xyz), uvec3(1u));
  const vec3 view_direction = -normalize(gl_WorldRayDirectionEXT);
  const vec3 biased_position =
      EE_DDGI_SURFACE_BIASED_POSITION(position, normal, view_direction, EE_RENDER_INFO.ddgi_volume_parameters.z,
                                      EE_RENDER_INFO.ddgi_volume_parameters.w);
  const vec3 volume_probe_coordinate =
      EE_DDGI_PROBE_COORDINATE(position, EE_RENDER_INFO.ddgi_first_probe.xyz,
                               EE_RENDER_INFO.ddgi_probe_step_x.xyz, EE_RENDER_INFO.ddgi_probe_step_y.xyz,
                               EE_RENDER_INFO.ddgi_probe_step_z.xyz);
  const float volume_blend_weight =
      EE_DDGI_VOLUME_BLEND_WEIGHT(volume_probe_coordinate, probe_counts, EE_RENDER_INFO.ddgi_probe_step_x.xyz,
                                  EE_RENDER_INFO.ddgi_probe_step_y.xyz, EE_RENDER_INFO.ddgi_probe_step_z.xyz);
  if (volume_blend_weight <= 0.0f) {
    return vec3(0.0f);
  }

  const vec3 biased_probe_coordinate =
      EE_DDGI_PROBE_COORDINATE(biased_position, EE_RENDER_INFO.ddgi_first_probe.xyz,
                               EE_RENDER_INFO.ddgi_probe_step_x.xyz, EE_RENDER_INFO.ddgi_probe_step_y.xyz,
                               EE_RENDER_INFO.ddgi_probe_step_z.xyz);

  const uint irradiance_tile_size = max(uint(EE_RENDER_INFO.ddgi_atlas_parameters.x), 1u);
  const uint irradiance_atlas_columns = max(uint(EE_RENDER_INFO.ddgi_atlas_parameters.y), 1u);
  const uint visibility_tile_size = max(uint(EE_RENDER_INFO.ddgi_atlas_parameters.z), 1u);
  const uint visibility_atlas_columns = max(uint(EE_RENDER_INFO.ddgi_atlas_parameters.w), 1u);
  const vec2 irradiance_atlas_size = vec2(textureSize(EE_DDGI_IRRADIANCE_ATLAS, 0));
  const vec2 visibility_atlas_size = vec2(textureSize(EE_DDGI_VISIBILITY_ATLAS, 0));
  const ivec3 probe_scroll_offset = ivec3(round(EE_RENDER_INFO.ddgi_probe_scroll_offset.xyz));
  const float irradiance_gamma = max(EE_RENDER_INFO.ddgi_probe_counts.w, 1.0f);
  const float visibility_bias = max(EE_RENDER_INFO.ddgi_volume_parameters.y, 0.0f);
  const vec3 max_probe_grid = vec3(probe_counts - uvec3(1u));
  const vec3 base_probe_grid = clamp(floor(biased_probe_coordinate), vec3(0.0f), max_probe_grid);
  const vec3 base_probe_world_position =
      EE_DDGI_PROBE_WORLD_POSITION(base_probe_grid, EE_RENDER_INFO.ddgi_first_probe.xyz,
                                   EE_RENDER_INFO.ddgi_probe_step_x.xyz, EE_RENDER_INFO.ddgi_probe_step_y.xyz,
                                   EE_RENDER_INFO.ddgi_probe_step_z.xyz);
  const vec3 base_probe_to_biased_position = biased_position - base_probe_world_position;
  const vec3 probe_fraction =
      clamp(vec3(EE_DDGI_AXIS_COORDINATE(base_probe_to_biased_position, EE_RENDER_INFO.ddgi_probe_step_x.xyz),
                 EE_DDGI_AXIS_COORDINATE(base_probe_to_biased_position, EE_RENDER_INFO.ddgi_probe_step_y.xyz),
                 EE_DDGI_AXIS_COORDINATE(base_probe_to_biased_position, EE_RENDER_INFO.ddgi_probe_step_z.xyz)),
            vec3(0.0f), vec3(1.0f));

  vec3 diffuse = vec3(0.0f);
  float weight_sum = 0.0f;
  for (uint z = 0u; z < 2u; ++z) {
    for (uint y = 0u; y < 2u; ++y) {
      for (uint x = 0u; x < 2u; ++x) {
        const uvec3 corner = uvec3(x, y, z);
        const vec3 corner_weight = max(vec3(0.001f), mix(vec3(1.0f) - probe_fraction, probe_fraction, vec3(corner)));
        const float trilinear_weight = corner_weight.x * corner_weight.y * corner_weight.z;

        const vec3 probe_grid = clamp(base_probe_grid + vec3(corner), vec3(0.0f), max_probe_grid);
        const uvec3 probe_index_3d = uvec3(probe_grid);
        const uint probe_index = EE_DDGI_SCROLL_PROBE_INDEX(probe_index_3d, probe_scroll_offset, probe_counts);
        const vec4 probe_state = EE_DDGI_PROBE_STATE[probe_index];
        const float probe_active = 1.0f - clamp(probe_state.w, 0.0f, 1.0f);
        if (probe_active <= 0.0f) {
          continue;
        }
        const vec3 probe_position =
            EE_DDGI_PROBE_WORLD_POSITION(probe_grid, EE_RENDER_INFO.ddgi_first_probe.xyz,
                                         EE_RENDER_INFO.ddgi_probe_step_x.xyz, EE_RENDER_INFO.ddgi_probe_step_y.xyz,
                                         EE_RENDER_INFO.ddgi_probe_step_z.xyz) +
            probe_state.xyz;

        const vec3 surface_to_probe = probe_position - position;
        const float probe_distance = length(surface_to_probe);
        const vec3 biased_surface_to_probe = probe_position - biased_position;
        const float biased_probe_distance = length(biased_surface_to_probe);
        const vec3 surface_to_probe_direction = probe_distance > 0.001f ? surface_to_probe / probe_distance : normal;
        const vec3 biased_surface_to_probe_direction =
            biased_probe_distance > 0.001f ? biased_surface_to_probe / biased_probe_distance : normal;
        const vec3 probe_to_surface = -biased_surface_to_probe_direction;
        const vec2 irradiance_atlas_uv =
            EE_DDGI_ATLAS_UV(probe_index, irradiance_atlas_columns, irradiance_tile_size, normal, irradiance_atlas_size);
        const vec2 visibility_atlas_uv = EE_DDGI_ATLAS_UV(probe_index, visibility_atlas_columns, visibility_tile_size,
                                                          probe_to_surface, visibility_atlas_size);
        const vec4 irradiance = texture(EE_DDGI_IRRADIANCE_ATLAS, irradiance_atlas_uv);
        const vec4 visibility_sample = texture(EE_DDGI_VISIBILITY_ATLAS, visibility_atlas_uv);
        const float wrap_shading = (dot(surface_to_probe_direction, normal) + 1.0f) * 0.5f;
        float visibility_weight = wrap_shading * wrap_shading + 0.2f;
        const float distance_visibility =
            EE_DDGI_CHEBYSHEV_VISIBILITY(visibility_sample.rg, biased_probe_distance, visibility_bias);
        visibility_weight *= max(0.05f, distance_visibility);
        visibility_weight = max(0.000001f, visibility_weight);
        visibility_weight = EE_DDGI_CRUSH_LOW_WEIGHT(visibility_weight);
        const float sample_weight = trilinear_weight * visibility_weight;
        const vec3 decoded_irradiance = pow(max(irradiance.rgb, vec3(0.0f)), vec3(irradiance_gamma * 0.5f));
        diffuse += decoded_irradiance * sample_weight;
        weight_sum += sample_weight;
      }
    }
  }
  if (weight_sum <= 0.0f) {
    return vec3(0.0f);
  }
  diffuse /= weight_sum;
  diffuse *= diffuse * (2.0f * EE_DDGI_PI);
  return albedo / EE_DDGI_PI * diffuse * intensity * volume_blend_weight;
}

bool EE_DDGI_OCCLUDED(const vec3 origin, const vec3 direction, const float max_distance) {
  if (max_distance <= trace_parameters.y) {
    return false;
  }
  const PointCloudRayTracingPayload primary_hit = hit_value;
  hit_value.hit_count = 1u;
  traceRayEXT(EE_TLAS, gl_RayFlagsOpaqueEXT | gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsSkipClosestHitShaderEXT,
              EE_DDGI_RAY_MASK_SHADOW, 0, 0, 0, origin, trace_parameters.y, direction, max_distance, 0);
  const bool occluded = hit_value.hit_count != 0u;
  hit_value = primary_hit;
  return occluded;
}

float EE_DDGI_SHADOW_VISIBILITY(const vec3 origin, const vec3 direction, const float max_distance) {
  return EE_DDGI_OCCLUDED(origin, direction, max_distance) ? 0.0f : 1.0f;
}

vec3 EE_DDGI_LAMBERT_IRRADIANCE(const vec3 albedo, const vec3 light_radiance, const vec3 normal,
                                const vec3 light_direction) {
  const float diffuse_weight = max(dot(normal, light_direction), 0.0f);
  return albedo / EE_DDGI_PI * light_radiance * diffuse_weight;
}

float EE_DDGI_DISTANCE_LIGHT_ATTENUATION(const vec4 constant_linear_quadratic_far, const float light_distance) {
  if (light_distance <= trace_parameters.y || light_distance >= constant_linear_quadratic_far.w) {
    return 0.0f;
  }
  return 1.0f / max(constant_linear_quadratic_far.x + constant_linear_quadratic_far.y * light_distance +
                       constant_linear_quadratic_far.z * light_distance * light_distance,
                   0.001f);
}

vec3 EE_DDGI_EXPLICIT_DIRECTIONAL_LIGHT_IRRADIANCE(const DirectionalLight light, const vec3 albedo, const vec3 normal,
                                                   const vec3 position) {
  const vec3 light_direction = normalize(-light.direction);
  const float diffuse_weight = max(dot(normal, light_direction), 0.0f);
  if (diffuse_weight <= 0.0f) {
    return vec3(0.0f);
  }
  const float visibility = light.diffuse.w == 1.0f ? EE_DDGI_SHADOW_VISIBILITY(position + normal * trace_parameters.y,
                                                                               light_direction, trace_parameters.x)
                                                   : 1.0f;
  return EE_DDGI_LAMBERT_IRRADIANCE(albedo, light.diffuse.rgb, normal, light_direction) * visibility;
}

vec3 EE_DDGI_EXPLICIT_POINT_LIGHT_IRRADIANCE(const PointLight light, const vec3 albedo, const vec3 normal,
                                             const vec3 position) {
  const vec3 light_delta = light.position - position;
  const float light_distance = length(light_delta);
  const float attenuation = EE_DDGI_DISTANCE_LIGHT_ATTENUATION(light.constant_linear_quadratic_far, light_distance);
  if (attenuation <= 0.0f) {
    return vec3(0.0f);
  }
  const vec3 light_direction = light_delta / light_distance;
  const float diffuse_weight = max(dot(normal, light_direction), 0.0f);
  if (diffuse_weight <= 0.0f) {
    return vec3(0.0f);
  }
  const float visibility = light.diffuse.w == 1.0f
                               ? EE_DDGI_SHADOW_VISIBILITY(position + normal * trace_parameters.y, light_direction,
                                                           light_distance - trace_parameters.y)
                               : 1.0f;
  return EE_DDGI_LAMBERT_IRRADIANCE(albedo, light.diffuse.rgb * attenuation, normal, light_direction) * visibility;
}

vec3 EE_DDGI_EXPLICIT_SPOT_LIGHT_IRRADIANCE(const SpotLight light, const vec3 albedo, const vec3 normal,
                                            const vec3 position) {
  const vec3 light_delta = light.position - position;
  const float light_distance = length(light_delta);
  const float attenuation = EE_DDGI_DISTANCE_LIGHT_ATTENUATION(light.constant_linear_quadratic_far, light_distance);
  if (attenuation <= 0.0f) {
    return vec3(0.0f);
  }
  const vec3 light_direction = light_delta / light_distance;
  const float diffuse_weight = max(dot(normal, light_direction), 0.0f);
  if (diffuse_weight <= 0.0f) {
    return vec3(0.0f);
  }
  const float theta = dot(light_direction, normalize(-light.direction));
  const float epsilon = max(light.cutoff_outer_inner_size_bias.x - light.cutoff_outer_inner_size_bias.y, 0.001f);
  const float spot_intensity = clamp((theta - light.cutoff_outer_inner_size_bias.y) / epsilon, 0.0f, 1.0f);
  const float visibility = light.diffuse.w == 1.0f
                               ? EE_DDGI_SHADOW_VISIBILITY(position + normal * trace_parameters.y, light_direction,
                                                           light_distance - trace_parameters.y)
                               : 1.0f;
  return EE_DDGI_LAMBERT_IRRADIANCE(albedo, light.diffuse.rgb * attenuation * spot_intensity, normal,
                                    light_direction) *
         visibility;
}

vec3 EE_DDGI_DIRECT_IRRADIANCE(const vec3 albedo, const vec3 normal, const vec3 position) {
  vec3 irradiance = vec3(0.0f);
  for (int i = 0; i < EE_RENDER_INFO.directional_light_size; ++i) {
    irradiance += EE_DDGI_EXPLICIT_DIRECTIONAL_LIGHT_IRRADIANCE(EE_DIRECTIONAL_LIGHTS[i], albedo, normal, position);
  }
  for (int i = 0; i < EE_RENDER_INFO.point_light_size; ++i) {
    irradiance += EE_DDGI_EXPLICIT_POINT_LIGHT_IRRADIANCE(EE_POINT_LIGHTS[i], albedo, normal, position);
  }
  for (int i = 0; i < EE_RENDER_INFO.spot_light_size; ++i) {
    irradiance += EE_DDGI_EXPLICIT_SPOT_LIGHT_IRRADIANCE(EE_SPOT_LIGHTS[i], albedo, normal, position);
  }
  return irradiance;
}

void main() {
  const int instance_index = int(gl_InstanceCustomIndexEXT);
  const Instance instance = EE_INSTANCES[instance_index];
  const int triangle_offset = instance.triangle_offset + gl_PrimitiveID;

  const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
  const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
  const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];
  const vec3 barycentrics = vec3(1.0f - attribs.x - attribs.y, attribs.x, attribs.y);
  const uint material_index = uint(instance.material_index);
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];

  const vec3 position = v0.position * barycentrics.x + v1.position * barycentrics.y + v2.position * barycentrics.z;
  const vec2 tex_coord = v0.tex_coord * barycentrics.x + v1.tex_coord * barycentrics.y + v2.tex_coord * barycentrics.z;
  vec3 normal = v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z;
  const vec3 tangent = v0.tangent * barycentrics.x + v1.tangent * barycentrics.y + v2.tangent * barycentrics.z;
  const GltfRasterMaterial surface = EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, tex_coord, tex_coord);
  normal = EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coord, tex_coord, normal, tangent);
  const vec3 world_position = vec3(gl_ObjectToWorldEXT * vec4(position, 1.0f));
  const vec3 triangle_world_normal =
      EE_DDGI_SAFE_NORMALIZE(vec3(normal * gl_WorldToObjectEXT), vec3(0.0f, 1.0f, 0.0f));
  const vec3 world_tangent = EE_DDGI_SAFE_NORMALIZE(vec3(tangent * gl_WorldToObjectEXT), vec3(1.0f, 0.0f, 0.0f));
  const bool ray_backface_hit = gl_HitKindEXT == gl_HitKindBackFacingTriangleEXT;
  const bool hit_face_is_culled = ray_backface_hit && material.double_sided == 0;
  const bool visible_backface = ray_backface_hit && material.double_sided != 0;
  const vec3 world_normal = visible_backface ? -triangle_world_normal : triangle_world_normal;
  const bool backface_hit = ray_backface_hit || hit_face_is_culled;
  const bool fixed_probe_ray = hit_value.seed == EE_DDGI_FIXED_RAY_PAYLOAD_FLAG;

  hit_value.hit_count = 1u;
  hit_value.handle = instance.renderer_handle;
  hit_value.hit_info.position = world_position;
  hit_value.hit_info.normal = world_normal;
  hit_value.hit_info.tangent = world_tangent;
  const vec3 albedo = max(surface.base_color.rgb, vec3(0.0f));
  const vec3 recursive_albedo = min(albedo, vec3(0.9f));
  const vec3 emissive_radiance = surface.emissive;
  const bool skip_recursive_ddgi = trace_parameters.w > 0.5f;
  const vec3 recursive_irradiance =
      skip_recursive_ddgi ? vec3(0.0f) : EE_DDGI_RECURSIVE_IRRADIANCE(recursive_albedo, world_normal, world_position);
  const vec3 frontface_radiance =
      backface_hit || fixed_probe_ray
          ? vec3(0.0f)
          : emissive_radiance + EE_DDGI_DIRECT_IRRADIANCE(albedo, world_normal, world_position) +
                recursive_irradiance;
  hit_value.hit_info.color = vec4(max(frontface_radiance, vec3(0.0f)), backface_hit ? -1.0f : 1.0f);
  hit_value.hit_info.tex_coord = tex_coord;
  hit_value.hit_info.vertex_info1 = gl_HitTEXT;
  hit_value.hit_info.vertex_info2 = float(instance_index);
  hit_value.hit_info.vertex_info3 = float(gl_PrimitiveID);
  hit_value.hit_info.vertex_info4 = vec2(attribs);
}
