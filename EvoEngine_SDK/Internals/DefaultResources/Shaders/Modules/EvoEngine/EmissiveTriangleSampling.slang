__exported import EvoEngine.RayTracingMaterial;
__exported import EvoEngine.RayTracingBasic;
public
static const float EE_EMISSIVE_TRIANGLE_RAY_EPSILON = 1e-3f;
public
static const float EE_EMISSIVE_TRIANGLE_PDF_EPSILON = 1e-6f;
public
static const uint EE_EMISSIVE_SAMPLER_UNIFORM = 0u;
public
static const uint EE_EMISSIVE_SAMPLER_POWER = 1u;
public
static const uint EE_DDGI_EMISSIVE_SAMPLE_REJECT_NONE = 0u;
public
static const uint EE_DDGI_EMISSIVE_SAMPLE_REJECT_ZERO_PDF = 1u;
public
static const uint EE_DDGI_EMISSIVE_SAMPLE_REJECT_EMITTER_BACKFACE = 2u;
public
static const uint EE_DDGI_EMISSIVE_SAMPLE_REJECT_ALPHA_MASK = 3u;
public
static const uint EE_DDGI_EMISSIVE_SAMPLE_REJECT_INVALID = 4u;
public
struct EeEmissiveTriangleSample {
 public
  float3 position;
 public
  float distance;
 public
  float3 direction;
 public
  float solid_angle_pdf;
 public
  float3 radiance_over_pdf;
 public
  uint instance_index;
 public
  uint primitive_id;
 public
  uint material_index;
 public
  uint rejection_reason;
};
public
uint EE_EMISSIVE_TRIANGLE_COUNT() {
  return EE_RENDER_INFO.emissive_triangle_parameters.x;
}
public
bool EE_FIND_EMISSIVE_INSTANCE_INDEX(const uint instance_index, out uint record_index,
                                     out EmissiveInstanceInfo result) {
  record_index = 0u;
  const uint count = EE_EMISSIVE_TRIANGLE_COUNT();
  if (count == 0u) {
    return false;
  }
  uint low = 0u;
  uint high = count;
  for (int iteration = 0; iteration < 32 && low < high; ++iteration) {
    const uint mid = low + (high - low) / 2u;
    const EmissiveInstanceInfo candidate = EE_EMISSIVE_INSTANCES[mid];
    if (candidate.instance_index < instance_index) {
      low = mid + 1u;
    } else {
      high = mid;
    }
  }
  if (low >= count) {
    return false;
  }
  record_index = low;
  result = EE_EMISSIVE_INSTANCES[low];
  return result.instance_index == instance_index;
}
public
EmissiveInstanceInfo EE_SAMPLE_EMISSIVE_INSTANCE_RECORD(const float sample_value, const bool uniform_sampling) {
  const uint count = EE_EMISSIVE_TRIANGLE_COUNT();
  const float table_sample = clamp(sample_value, 0.0f, 0.99999994f) * float(count);
  const uint record_index = min(uint(table_sample), count - 1u);
  const EmissiveInstanceInfo record = EE_EMISSIVE_INSTANCES[record_index];
  const float alias_probability =
      uniform_sampling ? record.uniform_alias_probability : record.power_alias_probability;
  const uint alias_index = uniform_sampling ? record.uniform_alias_index : record.power_alias_index;
  if (frac(table_sample) >= alias_probability) {
    return EE_EMISSIVE_INSTANCES[min(alias_index, count - 1u)];
  }
  return record;
}
public
EmissiveTriangleDistributionEntry EE_SAMPLE_EMISSIVE_DISTRIBUTION_RECORD(
    const EmissiveTriangleDistributionInfo distribution, const float sample_value, const bool uniform_sampling) {
  const float table_sample = clamp(sample_value, 0.0f, 0.99999994f) * float(distribution.triangle_count);
  const uint local_index = min(uint(table_sample), distribution.triangle_count - 1u);
  const EmissiveTriangleDistributionEntry record = EE_EMISSIVE_TRIANGLES[distribution.triangle_offset + local_index];
  if (!uniform_sampling && frac(table_sample) >= record.alias_probability) {
    return EE_EMISSIVE_TRIANGLES[distribution.triangle_offset + min(record.alias_index,
                                                                     distribution.triangle_count - 1u)];
  }
  return record;
}
public
bool EE_FIND_EMISSIVE_TRIANGLE(const uint instance_index, const uint primitive_id, out EmissiveTriangleInfo result) {
  result.instance_index = instance_index;
  result.primitive_id = primitive_id;
  result.power_selection_probability = 0.0f;
  result.uniform_selection_probability = 0.0f;
  uint instance_record_index;
  EmissiveInstanceInfo instance_record;
  if (!EE_FIND_EMISSIVE_INSTANCE_INDEX(instance_index, instance_record_index, instance_record)) {
    return false;
  }
  const EmissiveTriangleDistributionInfo distribution = EE_EMISSIVE_DISTRIBUTIONS[instance_record.distribution_index];
  uint low = 0u;
  uint high = distribution.triangle_count;
  for (int iteration = 0; iteration < 32 && low < high; ++iteration) {
    const uint mid = low + (high - low) / 2u;
    const EmissiveTriangleDistributionEntry candidate = EE_EMISSIVE_TRIANGLES[distribution.triangle_offset + mid];
    if (candidate.primitive_id < primitive_id) {
      low = mid + 1u;
    } else {
      high = mid;
    }
  }
  if (low >= distribution.triangle_count) {
    return false;
  }
  const EmissiveTriangleDistributionEntry triangle_record = EE_EMISSIVE_TRIANGLES[distribution.triangle_offset + low];
  if (triangle_record.primitive_id != primitive_id) {
    return false;
  }
  result.power_selection_probability =
      instance_record.power_selection_probability * triangle_record.selection_probability;
  result.uniform_selection_probability =
      instance_record.uniform_selection_probability / float(distribution.triangle_count);
  return true;
}
public
float EE_EMISSIVE_TRIANGLE_AREA(const uint instance_index, const uint primitive_id) {
  const Instance instance = EE_INSTANCES[instance_index];
  const int triangle_offset = instance.triangle_offset + int(primitive_id);
  const float3 p0 = mul(float4(EE_VERTICES[EE_INDICES[triangle_offset * 3]].position, 1.0f), instance.model).xyz;
  const float3 p1 = mul(float4(EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]].position, 1.0f), instance.model).xyz;
  const float3 p2 = mul(float4(EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]].position, 1.0f), instance.model).xyz;
  const float twice_area = length(cross(p1 - p0, p2 - p0));
  return 0.5f * twice_area;
}
public
bool EE_SAMPLE_EMISSIVE_TRIANGLE_SPECIALIZED_MODE<let feature_mask : uint>(
    const float3 shading_position, const float3 shading_normal, const bool upper_hemisphere, const float2 record_sample,
    const float2 barycentric_sample, const uint sampling_mode, out EeEmissiveTriangleSample result) {
  result.position = shading_position;
  result.distance = 0.0f;
  result.direction = float3(0.0f, 1.0f, 0.0f);
  result.solid_angle_pdf = 0.0f;
  result.radiance_over_pdf = float3(0.0f);
  result.instance_index = 0u;
  result.primitive_id = 0u;
  result.material_index = 0u;
  result.rejection_reason = EE_DDGI_EMISSIVE_SAMPLE_REJECT_NONE;
  if (EE_EMISSIVE_TRIANGLE_COUNT() == 0u) {
    result.rejection_reason = EE_DDGI_EMISSIVE_SAMPLE_REJECT_INVALID;
    return false;
  }
  const bool uniform_sampling = sampling_mode == EE_EMISSIVE_SAMPLER_UNIFORM;
  const EmissiveInstanceInfo instance_record = EE_SAMPLE_EMISSIVE_INSTANCE_RECORD(record_sample.x, uniform_sampling);
  const EmissiveTriangleDistributionInfo distribution = EE_EMISSIVE_DISTRIBUTIONS[instance_record.distribution_index];
  if (distribution.triangle_count == 0u) {
    result.rejection_reason = EE_DDGI_EMISSIVE_SAMPLE_REJECT_INVALID;
    return false;
  }
  const EmissiveTriangleDistributionEntry triangle_record =
      EE_SAMPLE_EMISSIVE_DISTRIBUTION_RECORD(distribution, record_sample.y, uniform_sampling);
  const float instance_probability = uniform_sampling ? instance_record.uniform_selection_probability
                                                       : instance_record.power_selection_probability;
  const float triangle_probability =
      uniform_sampling ? rcp(float(distribution.triangle_count)) : triangle_record.selection_probability;
  const float selection_pdf = instance_probability * triangle_probability;
  const Instance instance = EE_INSTANCES[instance_record.instance_index];
  const int triangle_offset = instance.triangle_offset + int(triangle_record.primitive_id);
  const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
  const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
  const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];
  const float barycentric_sqrt = sqrt(clamp(barycentric_sample.x, 0.0f, 1.0f));
  const float barycentric_y = clamp(barycentric_sample.y, 0.0f, 1.0f);
  const float3 barycentrics =
      float3(1.0f - barycentric_sqrt, barycentric_sqrt * (1.0f - barycentric_y), barycentric_sqrt * barycentric_y);
  const float3 p0 = mul(float4(v0.position, 1.0f), instance.model).xyz;
  const float3 p1 = mul(float4(v1.position, 1.0f), instance.model).xyz;
  const float3 p2 = mul(float4(v2.position, 1.0f), instance.model).xyz;
  const float3 light_position = p0 * barycentrics.x + p1 * barycentrics.y + p2 * barycentrics.z;
  const float3 to_light = light_position - shading_position;
  const float distance_squared = dot(to_light, to_light);
  if (distance_squared <= EE_EMISSIVE_TRIANGLE_RAY_EPSILON * EE_EMISSIVE_TRIANGLE_RAY_EPSILON) {
    result.rejection_reason = EE_DDGI_EMISSIVE_SAMPLE_REJECT_INVALID;
    return false;
  }
  const float distance = sqrt(distance_squared);
  const float3 direction = to_light / distance;
  const float3 object_shading_normal = EE_RT_SAFE_NORMALIZE(
      v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z, float3(0.0f, 1.0f, 0.0f));
  const float3 object_geometric_normal =
      EE_RT_GEOMETRIC_NORMAL(v1.position - v0.position, v2.position - v0.position, object_shading_normal);
  const float3x3 world_normal_matrix = EE_RT_WORLD_NORMAL_MATRIX(instance.model);
  const float3 light_geometric_normal =
      EE_RT_WORLD_NORMAL(world_normal_matrix, object_geometric_normal, float3(0.0f, 1.0f, 0.0f));
  const uint material_index = uint(instance.material_index);
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  const float light_cosine = material.double_sided != 0 ? abs(dot(light_geometric_normal, -direction))
                                                        : max(dot(light_geometric_normal, -direction), 0.0f);
  if (light_cosine <= EE_EMISSIVE_TRIANGLE_PDF_EPSILON) {
    result.rejection_reason = EE_DDGI_EMISSIVE_SAMPLE_REJECT_EMITTER_BACKFACE;
    return false;
  }
  const float area = EE_EMISSIVE_TRIANGLE_AREA(instance_record.instance_index, triangle_record.primitive_id);
  const float area_pdf = area > EE_EMISSIVE_TRIANGLE_PDF_EPSILON ? selection_pdf / area : 0.0f;
  const float solid_angle_pdf = area_pdf * distance_squared / light_cosine;
  if (solid_angle_pdf <= 0.0f || isnan(solid_angle_pdf) || isinf(solid_angle_pdf)) {
    result.rejection_reason = EE_DDGI_EMISSIVE_SAMPLE_REJECT_ZERO_PDF;
    return false;
  }
  const EeRayTracingSurfaceAttributes attributes =
      EE_RT_INTERPOLATE_SURFACE_ATTRIBUTES(v0, v1, v2, barycentrics, float4(0.0f));
  if (!EE_GLTF_RASTER_ALPHA_MASK_PASSES_LOD0_SPECIALIZED<feature_mask>(material_index, attributes.tex_coords,
                                                                       attributes.vertex_color.a)) {
    result.rejection_reason = EE_DDGI_EMISSIVE_SAMPLE_REJECT_ALPHA_MASK;
    return false;
  }
  const float emitter_opacity = EE_GLTF_RASTER_OPACITY_LOD0_SPECIALIZED<feature_mask>(
      material_index, attributes.tex_coords, attributes.vertex_color.a);
  const float3 world_shading_normal =
      EE_RT_WORLD_NORMAL(world_normal_matrix, attributes.normal, light_geometric_normal);
  const float3 world_tangent = EE_RT_WORLD_TANGENT(instance.model, attributes.tangent, world_shading_normal);
  const float tangent_handedness = EE_RT_WORLD_TANGENT_HANDEDNESS(attributes.tangent_handedness, instance.model);
  const float facing_sign = dot(light_geometric_normal, -direction) >= 0.0f ? 1.0f : -1.0f;
  const float3 facing_geometric_normal = facing_sign * light_geometric_normal;
  float3 facing_shading_normal = facing_sign * world_shading_normal;
  float3 facing_tangent = facing_sign * world_tangent;
  if (dot(facing_shading_normal, facing_geometric_normal) < 0.0f) {
    facing_shading_normal = -facing_shading_normal;
    facing_tangent = -facing_tangent;
  }
  if (dot(reflect(direction, facing_shading_normal), facing_geometric_normal) < 0.0f) {
    facing_shading_normal = facing_geometric_normal;
    facing_tangent =
        EE_RT_SAFE_NORMALIZE(facing_tangent - facing_shading_normal * dot(facing_tangent, facing_shading_normal),
                             EE_GLTF_FALLBACK_TANGENT(facing_shading_normal));
  }
  const float3 radiance =
      EE_RT_COATED_EMISSION_LOD0_SPECIALIZED<feature_mask>(
          material_index, attributes.tex_coords, facing_sign * facing_shading_normal, facing_sign * facing_tangent,
          tangent_handedness, facing_sign, facing_geometric_normal, -direction) *
      emitter_opacity;
  if (any(isnan(radiance)) || any(isinf(radiance))) {
    result.rejection_reason = EE_DDGI_EMISSIVE_SAMPLE_REJECT_INVALID;
    return false;
  }
  const float3 radiance_over_pdf = max(radiance, float3(0.0f)) / solid_angle_pdf;
  if (any(isnan(radiance_over_pdf)) || any(isinf(radiance_over_pdf))) {
    result.rejection_reason = EE_DDGI_EMISSIVE_SAMPLE_REJECT_INVALID;
    return false;
  }
  result.position = light_position;
  result.distance = distance;
  result.direction = direction;
  result.solid_angle_pdf = solid_angle_pdf;
  result.radiance_over_pdf = radiance_over_pdf;
  result.instance_index = instance_record.instance_index;
  result.primitive_id = triangle_record.primitive_id;
  result.material_index = material_index;
  return true;
}
public
bool EE_SAMPLE_EMISSIVE_TRIANGLE_SPECIALIZED<let feature_mask : uint>(const float3 shading_position,
                                                                      const float2 record_sample,
                                                                      const float2 barycentric_sample,
                                                                      const bool uniform_sampling,
                                                                      out EeEmissiveTriangleSample result) {
  return EE_SAMPLE_EMISSIVE_TRIANGLE_SPECIALIZED_MODE<feature_mask>(
      shading_position, float3(0.0f, 1.0f, 0.0f), false, record_sample, barycentric_sample,
      uniform_sampling ? EE_EMISSIVE_SAMPLER_UNIFORM : EE_EMISSIVE_SAMPLER_POWER, result);
}
public
bool EE_SAMPLE_EMISSIVE_TRIANGLE(const float3 shading_position, const float2 record_sample,
                                 const float2 barycentric_sample, out EeEmissiveTriangleSample result) {
  return EE_SAMPLE_EMISSIVE_TRIANGLE_SPECIALIZED<EE_GLTF_ALL_FEATURES>(shading_position, record_sample,
                                                                       barycentric_sample, false, result);
}
public
float EE_EMISSIVE_TRIANGLE_HIT_PDF_SAMPLER(const EmissiveTriangleInfo record, const float3 previous_position,
                                           const float3 previous_normal, const bool upper_hemisphere,
                                           const float3 hit_position, const float3 hit_geometric_normal,
                                           const uint sampling_mode) {
  EmissiveTriangleInfo found_record;
  if (!EE_FIND_EMISSIVE_TRIANGLE(record.instance_index, record.primitive_id, found_record)) {
    return 0.0f;
  }
  const bool uniform_sampling = sampling_mode == EE_EMISSIVE_SAMPLER_UNIFORM;
  const float area = EE_EMISSIVE_TRIANGLE_AREA(record.instance_index, record.primitive_id);
  const float selection_pdf = uniform_sampling ? found_record.uniform_selection_probability
                                                : found_record.power_selection_probability;
  const float area_pdf = area > EE_EMISSIVE_TRIANGLE_PDF_EPSILON ? selection_pdf / area : 0.0f;
  const float3 to_hit = hit_position - previous_position;
  const float distance_squared = dot(to_hit, to_hit);
  if (distance_squared <= EE_EMISSIVE_TRIANGLE_RAY_EPSILON * EE_EMISSIVE_TRIANGLE_RAY_EPSILON) {
    return 0.0f;
  }
  const float light_cosine =
      abs(dot(EE_RT_SAFE_NORMALIZE(hit_geometric_normal, float3(0.0f, 1.0f, 0.0f)), -to_hit * rsqrt(distance_squared)));
  return light_cosine > EE_EMISSIVE_TRIANGLE_PDF_EPSILON ? area_pdf * distance_squared / light_cosine : 0.0f;
}
public
float EE_EMISSIVE_TRIANGLE_HIT_PDF_MODE(const EmissiveTriangleInfo record, const float3 previous_position,
                                        const float3 hit_position, const float3 hit_geometric_normal,
                                        const bool uniform_sampling) {
  return EE_EMISSIVE_TRIANGLE_HIT_PDF_SAMPLER(
      record, previous_position, float3(0.0f, 1.0f, 0.0f), false, hit_position, hit_geometric_normal,
      uniform_sampling ? EE_EMISSIVE_SAMPLER_UNIFORM : EE_EMISSIVE_SAMPLER_POWER);
}
public
float EE_EMISSIVE_TRIANGLE_HIT_PDF(const EmissiveTriangleInfo record, const float3 previous_position,
                                   const float3 hit_position, const float3 hit_geometric_normal) {
  return EE_EMISSIVE_TRIANGLE_HIT_PDF_MODE(record, previous_position, hit_position, hit_geometric_normal, false);
}
