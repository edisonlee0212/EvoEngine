// @evoengine-dialect native
import EvoEngine.DDGI;
import EvoEngine.DDGIGatherSingle;
import EvoEngine.DDGIProbeRayConstants;
import EvoEngine.EmissiveTriangleSampling;
import EvoEngine.PointCloudRayTracingPayload;
import EvoEngine.Random;
import EvoEngine.RayTracingBasic;
import EvoEngine.RayTracingMaterial;
void EE_DDGI_TRACE_RAY(const uint ray_flags, const uint instance_inclusion_mask, const float3 origin,
                       const float t_min, const float3 direction, const float t_max,
                       inout PointCloudRayTracingPayload hit_value) {
  RayDesc ray;
  ray.Origin = origin;
  ray.Direction = direction;
  ray.TMin = t_min;
  ray.TMax = t_max;
  TraceRay(EE_TLAS, ray_flags, instance_inclusion_mask, 0, 0, 0, ray, hit_value);
}

bool EE_DDGI_OCCLUDED(const float3 origin, const float3 direction, const float max_distance,
                      const uint shadow_seed, inout PointCloudRayTracingPayload hit_value) {
  if (max_distance <= EE_DDGI_PROBE_RAY_CONSTANTS.trace_parameters.y) {
    return false;
  }
  const PointCloudRayTracingPayload primary_hit = hit_value;
  hit_value.hit_count = 1u;
  hit_value.seed = shadow_seed;
  EE_DDGI_TRACE_RAY(RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER,
                    EE_DDGI_RAY_MASK_SHADOW, origin, EE_DDGI_PROBE_RAY_CONSTANTS.trace_parameters.y, direction, max_distance, hit_value);
  const bool occluded = hit_value.hit_count != 0u;
  hit_value = primary_hit;
  return occluded;
}

float EE_DDGI_SHADOW_VISIBILITY(const float3 origin, const float3 direction, const float max_distance,
                                const uint shadow_seed, inout PointCloudRayTracingPayload hit_value) {
  return EE_DDGI_OCCLUDED(origin, direction, max_distance, shadow_seed, hit_value) ? 0.0f : 1.0f;
}

float3 EE_DDGI_LAMBERT_IRRADIANCE(const float3 albedo, const float3 light_radiance, const float3 normal,
                                const float3 light_direction) {
  return albedo / EE_DDGI_PI * light_radiance * max(dot(normal, light_direction), 0.0f);
}

float EE_DDGI_DISTANCE_LIGHT_ATTENUATION(const float4 constant_linear_quadratic_far, const float light_distance) {
  if (light_distance <= EE_DDGI_PROBE_RAY_CONSTANTS.trace_parameters.y || light_distance >= constant_linear_quadratic_far.w) {
    return 0.0f;
  }
  return 1.0f / max(constant_linear_quadratic_far.x + constant_linear_quadratic_far.y * light_distance +
                       constant_linear_quadratic_far.z * light_distance * light_distance,
                   0.001f);
}

float3 EE_DDGI_EXPLICIT_DIRECTIONAL_LIGHT_IRRADIANCE(const DirectionalLight light, const float3 albedo,
                                                   const float3 normal, const float3 geometric_normal,
                                                   const float3 position,
                                                   inout PointCloudRayTracingPayload hit_value) {
  const float3 light_direction = normalize(-light.direction);
  if (dot(normal, light_direction) <= 0.0f) {
    return float3(0.0f);
  }
  const float3 shadow_origin = EE_RT_OFFSET_RAY_ORIGIN(position, geometric_normal, light_direction);
  const float visibility = light.diffuse.w == 1.0f
                               ? EE_DDGI_SHADOW_VISIBILITY(
                                     shadow_origin, light_direction, EE_DDGI_PROBE_RAY_CONSTANTS.trace_parameters.x, hit_value.seed, hit_value)
                               : 1.0f;
  return EE_DDGI_LAMBERT_IRRADIANCE(albedo, light.diffuse.rgb, normal, light_direction) * visibility;
}

float3 EE_DDGI_EXPLICIT_POINT_LIGHT_IRRADIANCE(const PointLight light, const float3 albedo, const float3 normal,
                                             const float3 geometric_normal, const float3 position,
                                             inout PointCloudRayTracingPayload hit_value) {
  const float3 light_delta = light.position - position;
  const float light_distance = length(light_delta);
  const float attenuation = EE_DDGI_DISTANCE_LIGHT_ATTENUATION(light.constant_linear_quadratic_far, light_distance);
  if (attenuation <= 0.0f) {
    return float3(0.0f);
  }
  const float3 light_direction = light_delta / light_distance;
  if (dot(normal, light_direction) <= 0.0f) {
    return float3(0.0f);
  }
  const float3 shadow_origin = EE_RT_OFFSET_RAY_ORIGIN(position, geometric_normal, light_direction);
  const float visibility = light.diffuse.w == 1.0f
                               ? EE_DDGI_SHADOW_VISIBILITY(shadow_origin, light_direction,
                                                           light_distance - EE_DDGI_PROBE_RAY_CONSTANTS.trace_parameters.y,
                                                           hit_value.seed, hit_value)
                               : 1.0f;
  return EE_DDGI_LAMBERT_IRRADIANCE(albedo, light.diffuse.rgb * attenuation, normal, light_direction) * visibility;
}

float3 EE_DDGI_EXPLICIT_SPOT_LIGHT_IRRADIANCE(const SpotLight light, const float3 albedo, const float3 normal,
                                            const float3 geometric_normal, const float3 position,
                                            inout PointCloudRayTracingPayload hit_value) {
  const float3 light_delta = light.position - position;
  const float light_distance = length(light_delta);
  const float attenuation = EE_DDGI_DISTANCE_LIGHT_ATTENUATION(light.constant_linear_quadratic_far, light_distance);
  if (attenuation <= 0.0f) {
    return float3(0.0f);
  }
  const float3 light_direction = light_delta / light_distance;
  if (dot(normal, light_direction) <= 0.0f) {
    return float3(0.0f);
  }
  const float theta = dot(light_direction, normalize(-light.direction));
  const float epsilon = max(light.cutoff_outer_inner_size_bias.x - light.cutoff_outer_inner_size_bias.y, 0.001f);
  const float spot_intensity = clamp((theta - light.cutoff_outer_inner_size_bias.y) / epsilon, 0.0f, 1.0f);
  const float3 shadow_origin = EE_RT_OFFSET_RAY_ORIGIN(position, geometric_normal, light_direction);
  const float visibility = light.diffuse.w == 1.0f
                               ? EE_DDGI_SHADOW_VISIBILITY(shadow_origin, light_direction,
                                                           light_distance - EE_DDGI_PROBE_RAY_CONSTANTS.trace_parameters.y,
                                                           hit_value.seed, hit_value)
                               : 1.0f;
  return EE_DDGI_LAMBERT_IRRADIANCE(albedo, light.diffuse.rgb * attenuation * spot_intensity, normal,
                                    light_direction) *
         visibility;
}

float3 EE_DDGI_DIRECT_IRRADIANCE(const float3 albedo, const float3 normal, const float3 geometric_normal,
                               const float3 position, inout PointCloudRayTracingPayload hit_value) {
  float3 irradiance = float3(0.0f);
  for (int i = 0; i < EE_RENDER_INFO.directional_light_size; ++i) {
    irradiance += EE_DDGI_EXPLICIT_DIRECTIONAL_LIGHT_IRRADIANCE(
        EE_DIRECTIONAL_LIGHTS[i], albedo, normal, geometric_normal, position, hit_value);
  }
  for (int i = 0; i < EE_RENDER_INFO.point_light_size; ++i) {
    irradiance += EE_DDGI_EXPLICIT_POINT_LIGHT_IRRADIANCE(
        EE_POINT_LIGHTS[i], albedo, normal, geometric_normal, position, hit_value);
  }
  for (int i = 0; i < EE_RENDER_INFO.spot_light_size; ++i) {
    irradiance += EE_DDGI_EXPLICIT_SPOT_LIGHT_IRRADIANCE(
        EE_SPOT_LIGHTS[i], albedo, normal, geometric_normal, position, hit_value);
  }
  return irradiance;
}

float3 EE_DDGI_EMISSIVE_MESH_IRRADIANCE(const float3 diffuse_albedo, const float3 normal,
                                      const float3 geometric_normal, const float3 position,
                                      const uint primary_ray_seed, inout PointCloudRayTracingPayload hit_value) {
  if (EE_EMISSIVE_TRIANGLE_COUNT() == 0u || max(max(diffuse_albedo.x, diffuse_albedo.y), diffuse_albedo.z) <= 0.0f) {
    return float3(0.0f);
  }
  uint selection_seed = EE_XXHASH32(uint3(primary_ray_seed, 0x68bc21ebu, 0x02e5be93u));
  uint barycentric_seed = EE_XXHASH32(uint3(primary_ray_seed, 0x967a889bu, 0x368cc8b7u));
  const uint shadow_seed = EE_XXHASH32(uint3(primary_ray_seed, 0x1b56c4e9u, 0xa54ff53au));
  EeEmissiveTriangleSample emissive_sample;
  if (!EE_SAMPLE_EMISSIVE_TRIANGLE(position, EE_PCG_RANDOM(selection_seed),
                                   EE_PCG_RANDOM_2(barycentric_seed), emissive_sample)) {
    return float3(0.0f);
  }
  const float receiver_cosine = max(dot(normal, emissive_sample.direction), 0.0f);
  if (receiver_cosine <= 0.0f) {
    return float3(0.0f);
  }
  const float3 shadow_origin = EE_RT_OFFSET_RAY_ORIGIN(position, geometric_normal, emissive_sample.direction);
  const float origin_advance = max(dot(shadow_origin - position, emissive_sample.direction), 0.0f);
  const float shadow_distance =
      max(emissive_sample.distance - origin_advance - EE_EMISSIVE_TRIANGLE_RAY_EPSILON, 0.0f);
  const float visibility =
      EE_DDGI_SHADOW_VISIBILITY(shadow_origin, emissive_sample.direction, shadow_distance, shadow_seed, hit_value);
  return diffuse_albedo / EE_DDGI_PI * emissive_sample.radiance_over_pdf * receiver_cosine * visibility;
}

[shader("closesthit")]
void main(inout PointCloudRayTracingPayload hit_value, in BuiltInTriangleIntersectionAttributes attribs) {
  const int instance_index = int(InstanceID());
  const Instance instance = EE_INSTANCES[instance_index];
  const uint primitive_index = PrimitiveIndex();
  const int triangle_offset = instance.triangle_offset + primitive_index;
  const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
  const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
  const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];
  const float2 hit_barycentrics = attribs.barycentrics;
  const float3 barycentrics = float3(1.0f - hit_barycentrics.x - hit_barycentrics.y, hit_barycentrics.x,
                                 hit_barycentrics.y);
  const float3 object_shading_normal = EE_RT_SAFE_NORMALIZE(
      v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z,
      float3(0.0f, 1.0f, 0.0f));
  const float3 object_geometric_normal =
      EE_RT_GEOMETRIC_NORMAL(v1.position - v0.position, v2.position - v0.position, object_shading_normal);
  const float3 unflipped_world_geometric_normal =
      EE_RT_WORLD_NORMAL(instance.model, object_geometric_normal, float3(0.0f, 1.0f, 0.0f));
  const float3 ray_direction = normalize(WorldRayDirection());
  const float4 tex_gradients = EE_RT_TEXTURE_GRADIENTS(
      0.0f, RayTCurrent(), unflipped_world_geometric_normal, ray_direction, instance.model, v0, v1, v2,
      EE_RT_SPHERICAL_RAY_SPREAD(EE_DDGI_PROBE_RAY_CONSTANTS.probe_counts_and_ray_count.w));
  const EeRayTracingSurfaceAttributes attributes =
      EE_RT_INTERPOLATE_SURFACE_ATTRIBUTES(v0, v1, v2, barycentrics, tex_gradients);
  const uint material_index = uint(instance.material_index);
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  const GltfRasterMaterial surface =
      EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, attributes.tex_coords, attributes.vertex_color);
  const float3 unflipped_world_shading_normal =
      EE_RT_WORLD_NORMAL(instance.model, attributes.normal, unflipped_world_geometric_normal);
  const float3 unflipped_world_tangent =
      EE_RT_WORLD_TANGENT(instance.model, attributes.tangent, unflipped_world_shading_normal);
  const float world_tangent_handedness =
      EE_RT_WORLD_TANGENT_HANDEDNESS(attributes.tangent_handedness, instance.model);
  const float3 unflipped_world_surface_normal = EE_EVALUATE_GLTF_RASTER_NORMAL(
      material_index, attributes.tex_coords, unflipped_world_shading_normal, unflipped_world_tangent,
      world_tangent_handedness);
  const bool ray_backface_hit = HitKind() == HIT_KIND_TRIANGLE_BACK_FACE;
  const bool hit_face_is_culled = ray_backface_hit && material.double_sided == 0;
  const bool visible_backface = ray_backface_hit && material.double_sided != 0;
  const float facing_sign = visible_backface ? -1.0f : 1.0f;
  const float3 world_geometric_normal = facing_sign * unflipped_world_geometric_normal;
  float3 world_normal = facing_sign * unflipped_world_surface_normal;
  if (dot(world_normal, world_geometric_normal) < 0.0f) {
    world_normal = world_geometric_normal;
  }
  const float3 facing_world_tangent = facing_sign * unflipped_world_tangent;
  const float3 world_tangent = EE_RT_SAFE_NORMALIZE(
      facing_world_tangent - world_normal * dot(facing_world_tangent, world_normal),
      EE_GLTF_FALLBACK_TANGENT(world_normal));
  const float3 world_position = mul(float4(attributes.position, 1.0f), instance.model).xyz;
  const bool fixed_probe_ray = hit_value.seed == EE_DDGI_FIXED_RAY_PAYLOAD_FLAG;
  const bool signed_backface_hit =
      ray_backface_hit && (material.double_sided == 0 || fixed_probe_ray);

  hit_value.hit_count = 1u;
  hit_value.handle = instance.renderer_handle;
  hit_value.hit_info.position = world_position;
  hit_value.hit_info.normal = world_normal;
  hit_value.hit_info.tangent = world_tangent;
  const float3 albedo = max(surface.base_color.rgb, float3(0.0f));
  const float3 surface_fresnel = EE_GLTF_RASTER_FRESNEL(
      surface.specular_f0, float3(surface.specular_f90), max(dot(world_normal, -ray_direction), 0.0f));
  const float3 diffuse_albedo =
      albedo * (float3(1.0f) - surface_fresnel) * (1.0f - clamp(surface.metallic, 0.0f, 1.0f));
  const bool skip_recursive_ddgi = EE_DDGI_PROBE_RAY_CONSTANTS.trace_parameters.w > 0.5f;
  float3 shaded_radiance = float3(0.0f);
  if (!hit_face_is_culled && !fixed_probe_ray) {
    const float3 emissive_radiance = EE_RT_COATED_EMISSION(
        material_index, surface, attributes.tex_coords, unflipped_world_shading_normal,
        unflipped_world_tangent, world_tangent_handedness, facing_sign, world_geometric_normal,
        -ray_direction);
    float3 recursive_irradiance = float3(0.0f);
    if (!skip_recursive_ddgi) {
      const EeDdgiGatherResult recursive_gather =
          EE_DDGI_GATHER_IRRADIANCE(EE_DDGI_PROBE_RAY_CONSTANTS.selected_probe_volume_flags_environment.y, world_normal, -ray_direction, world_position);
      recursive_irradiance =
          EE_DDGI_WEIGHTED_DIFFUSE(recursive_gather, min(diffuse_albedo, float3(0.9f)));
    }
    float3 emissive_mesh_irradiance = float3(0.0f);
    if ((EE_DDGI_PROBE_RAY_CONSTANTS.selected_probe_volume_flags_environment.z & (1u << 1u)) != 0u) {
      emissive_mesh_irradiance = EE_DDGI_EMISSIVE_MESH_IRRADIANCE(
          diffuse_albedo, world_normal, world_geometric_normal, world_position, hit_value.seed, hit_value);
    }
    shaded_radiance =
        emissive_radiance +
        EE_DDGI_DIRECT_IRRADIANCE(diffuse_albedo, world_normal, world_geometric_normal, world_position, hit_value) +
        emissive_mesh_irradiance +
        recursive_irradiance;
  }
  hit_value.hit_info.color =
      float4(max(shaded_radiance, float3(0.0f)), signed_backface_hit ? -1.0f : 1.0f);
  hit_value.hit_info.tex_coord = attributes.tex_coords.uv0;
  hit_value.hit_info.vertex_info1 = RayTCurrent();
  hit_value.hit_info.vertex_info2 = float(instance_index);
  hit_value.hit_info.vertex_info3 = float(primitive_index);
  hit_value.hit_info.vertex_info4 = hit_barycentrics;
}
