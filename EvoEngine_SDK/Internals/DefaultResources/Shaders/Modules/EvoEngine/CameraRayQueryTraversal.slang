__exported import EvoEngine.CameraRayConstants;
__exported import EvoEngine.CameraRayGeometry;
__exported import EvoEngine.CameraRayTracingPayload;
__exported import EvoEngine.RayTracingBasic;
__exported import EvoEngine.RayTracingMaterial;
import EvoEngine.Random;

void EE_CAMERA_RQ_APPLY_MISS(inout CameraRayTracingPayload hit_value) {
  hit_value.type = EE_CAMERA_RAY_PAYLOAD_MISS;
}

void EE_CAMERA_RQ_FILL_TRIANGLE_SURFACE_PAYLOAD<let Flags : uint>(inout RayQuery<Flags> ray_query,
                                                                  inout CameraRayTracingPayload hit_value) {
  hit_value.type = EE_CAMERA_RAY_PAYLOAD_SURFACE;
  hit_value.hit_t = ray_query.CommittedRayT();
  hit_value.instance_index = uint(ray_query.CommittedRayInstanceCustomIndex());
  hit_value.primitive_id = uint(ray_query.CommittedRayPrimitiveIndex());
  hit_value.barycentrics = ray_query.CommittedTriangleBarycentrics();
  hit_value.object_geometric_normal = float3(0.0f);
  hit_value.geometry_type = EE_CAMERA_GEOMETRY_TRIANGLE;
}

#if EE_RAY_TRACING_LINEAR_SWEPT_SPHERES_SUPPORTED
void EE_CAMERA_RQ_FILL_LSS_SURFACE_PAYLOAD<let Flags : uint>(inout RayQuery<Flags> ray_query,
                                                             inout CameraRayTracingPayload hit_value) {
  hit_value.type = EE_CAMERA_RAY_PAYLOAD_SURFACE;
  hit_value.hit_t = ray_query.CommittedRayT();
  hit_value.instance_index = uint(ray_query.CommittedRayInstanceCustomIndex());
  hit_value.primitive_id = uint(ray_query.CommittedRayPrimitiveIndex());
  const Instance instance = EE_INSTANCES[hit_value.instance_index];
  RayTracingStrandPoint point_0;
  RayTracingStrandPoint point_1;
  EE_CAMERA_LSS_LOAD_POINTS(instance, hit_value.primitive_id, point_0, point_1);
  const float parameter = clamp(ray_query.CommittedLssHitParameterNV(), 0.0f, 1.0f);
  const float2x4 endpoints = ray_query.CommittedLssObjectPositionsAndRadiiNV();
  const float3 object_hit =
      ray_query.CommittedRayObjectRayOrigin() + ray_query.CommittedRayObjectRayDirection() * hit_value.hit_t;
  hit_value.barycentrics = float2(parameter, 0.0f);
  hit_value.object_geometric_normal = EE_CAMERA_LSS_OBJECT_GEOMETRIC_NORMAL(
      endpoints[0].xyz, endpoints[1].xyz, parameter, object_hit, ray_query.CommittedRayObjectRayDirection());
  hit_value.geometry_type = EE_CAMERA_GEOMETRY_LINEAR_SWEPT_SPHERE;
}
#endif

void EE_CAMERA_RQ_CANDIDATE_SURFACE<let Flags : uint>(inout RayQuery<Flags> ray_query, out uint material_index,
                                                      out float2 tex_coord_0, out float2 tex_coord_1,
                                                      out float2 tex_coord_2, out float2 tex_coord_3,
                                                      out float4 vertex_color) {
  const int instance_index = int(ray_query.CandidateRayInstanceCustomIndex());
  const Instance instance = EE_INSTANCES[instance_index];
  material_index = uint(instance.material_index);
  const int triangle_offset = instance.triangle_offset + int(ray_query.CandidatePrimitiveIndex());

  const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
  const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
  const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];
  const float2 bary = ray_query.CandidateTriangleBarycentrics();
  const float3 barycentrics = float3(1.0f - bary.x - bary.y, bary.x, bary.y);
  tex_coord_0 = v0.tex_coord * barycentrics.x + v1.tex_coord * barycentrics.y + v2.tex_coord * barycentrics.z;
  tex_coord_1 = v0.tex_coord_1 * barycentrics.x + v1.tex_coord_1 * barycentrics.y + v2.tex_coord_1 * barycentrics.z;
  tex_coord_2 = v0.tex_coord_2 * barycentrics.x + v1.tex_coord_2 * barycentrics.y + v2.tex_coord_2 * barycentrics.z;
  tex_coord_3 = v0.tex_coord_3 * barycentrics.x + v1.tex_coord_3 * barycentrics.y + v2.tex_coord_3 * barycentrics.z;
  vertex_color = v0.color * barycentrics.x + v1.color * barycentrics.y + v2.color * barycentrics.z;
}

#if EE_RAY_TRACING_LINEAR_SWEPT_SPHERES_SUPPORTED
void EE_CAMERA_RQ_CANDIDATE_LSS_SURFACE<let Flags : uint>(
    inout RayQuery<Flags> ray_query, out uint material_index, out float2 tex_coord_0, out float2 tex_coord_1,
    out float2 tex_coord_2, out float2 tex_coord_3, out float4 vertex_color, out float parameter,
    out float3 object_geometric_normal, out float3 world_geometric_normal) {
  const uint instance_index = uint(ray_query.CandidateRayInstanceCustomIndex());
  const Instance instance = EE_INSTANCES[instance_index];
  material_index = uint(instance.material_index);
  RayTracingStrandPoint point_0;
  RayTracingStrandPoint point_1;
  EE_CAMERA_LSS_LOAD_POINTS(instance, uint(ray_query.CandidateRayPrimitiveIndex()), point_0, point_1);
  parameter = clamp(ray_query.CandidateLssHitParameterNV(), 0.0f, 1.0f);
  tex_coord_0 = float2(lerp(point_0.tex_coord, point_1.tex_coord, parameter), 0.0f);
  tex_coord_1 = tex_coord_0;
  tex_coord_2 = tex_coord_0;
  tex_coord_3 = tex_coord_0;
  vertex_color = lerp(point_0.color, point_1.color, parameter);
  const float2x4 endpoints = ray_query.CandidateLssObjectPositionsAndRadiiNV();
  const float candidate_t = ray_query.CandidateTriangleRayT();
  const float3 object_hit =
      ray_query.CandidateRayObjectRayOrigin() + ray_query.CandidateRayObjectRayDirection() * candidate_t;
  object_geometric_normal = EE_CAMERA_LSS_OBJECT_GEOMETRIC_NORMAL(
      endpoints[0].xyz, endpoints[1].xyz, parameter, object_hit, ray_query.CandidateRayObjectRayDirection());
  world_geometric_normal =
      EE_RT_WORLD_NORMAL(instance.model, object_geometric_normal, -ray_query.WorldRayDirection());
}
#endif

public
void EE_CAMERA_RQ_TRACE_SURFACE<let feature_mask : uint>(const float3 origin, const float3 direction,
                                                         const float min_distance, inout uint seed,
                                                         inout CameraRayTracingPayload hit_value) {
  RayDesc ray = {origin, min_distance, direction, EE_CAMERA_MAX_TRACE_DISTANCE};
  RayQuery<RAY_FLAG_CULL_BACK_FACING_TRIANGLES> triangle_query;
  triangle_query.TraceRayInline(EE_TLAS, RAY_FLAG_CULL_BACK_FACING_TRIANGLES, EE_CAMERA_TRIANGLE_RAY_MASK_GEOMETRY,
                                ray);
  while (triangle_query.Proceed()) {
    if (triangle_query.CandidateType() != CANDIDATE_NON_OPAQUE_TRIANGLE) {
      continue;
    }
    uint material_index;
    float2 tex_coord_0;
    float2 tex_coord_1;
    float2 tex_coord_2;
    float2 tex_coord_3;
    float4 vertex_color;
    EE_CAMERA_RQ_CANDIDATE_SURFACE(triangle_query, material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3,
                                   vertex_color);
    const float opacity = EE_GLTF_RASTER_OPACITY_LOD0_SPECIALIZED<feature_mask>(
        material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color.a);
    if (EE_PCG_RANDOM(seed) <= opacity) {
      triangle_query.CommitNonOpaqueTriangleHit();
    }
  }

#if EE_RAY_TRACING_LINEAR_SWEPT_SPHERES_SUPPORTED
  RayQuery<RAY_FLAG_NONE> lss_query;
  lss_query.TraceRayInline(EE_TLAS, RAY_FLAG_NONE, EE_CAMERA_LSS_RAY_MASK_GEOMETRY, ray);
  while (lss_query.Proceed()) {
    uint material_index;
    float2 tex_coord_0;
    float2 tex_coord_1;
    float2 tex_coord_2;
    float2 tex_coord_3;
    float4 vertex_color;
    float parameter;
    float3 object_geometric_normal;
    float3 world_geometric_normal;
    EE_CAMERA_RQ_CANDIDATE_LSS_SURFACE(lss_query, material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3,
                                       vertex_color, parameter, object_geometric_normal, world_geometric_normal);
    const float opacity = EE_GLTF_RASTER_OPACITY_LOD0_SPECIALIZED<feature_mask>(
        material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color.a);
    if (EE_PCG_RANDOM(seed) <= opacity) {
      lss_query.CommitNonOpaqueTriangleHit();
    }
  }
  const bool lss_hit = lss_query.CommittedIsLssNV();
  if (lss_hit && (triangle_query.CommittedStatus() != COMMITTED_TRIANGLE_HIT ||
                  lss_query.CommittedRayT() < triangle_query.CommittedRayT())) {
    EE_CAMERA_RQ_FILL_LSS_SURFACE_PAYLOAD(lss_query, hit_value);
    return;
  }
#endif
  if (triangle_query.CommittedStatus() == COMMITTED_TRIANGLE_HIT) {
    EE_CAMERA_RQ_FILL_TRIANGLE_SURFACE_PAYLOAD(triangle_query, hit_value);
    return;
  }
  EE_CAMERA_RQ_APPLY_MISS(hit_value);
}

float3 EE_CAMERA_RQ_CANDIDATE_SHADOW_TRANSMISSION<let Flags : uint, let feature_mask : uint>(
    inout RayQuery<Flags> ray_query, const uint material_index, const float2 tex_coord_0, const float2 tex_coord_1,
    const float2 tex_coord_2, const float2 tex_coord_3, const float3 vertex_color, const float3 ray_direction,
    inout float previous_hit_t, inout uint shadow_is_inside) {
  const int instance_index = int(ray_query.CandidateRayInstanceCustomIndex());
  const Instance instance = EE_INSTANCES[instance_index];
  const int triangle_offset = instance.triangle_offset + int(ray_query.CandidatePrimitiveIndex());
  const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
  const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
  const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];
  const float2 bary = ray_query.CandidateTriangleBarycentrics();
  const float3 barycentrics = float3(1.0f - bary.x - bary.y, bary.x, bary.y);
  const float3 object_shading_normal = EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(
      v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z, float3(0.0f, 1.0f, 0.0f));
  const float3 object_geometric_normal =
      EE_CAMERA_GEOMETRIC_NORMAL(v1.position - v0.position, v2.position - v0.position, object_shading_normal);
  const float3x3 world_normal_matrix = EE_RT_WORLD_NORMAL_MATRIX(instance.model);
  const float3 world_shading_normal = EE_RT_WORLD_NORMAL(world_normal_matrix, object_shading_normal, -ray_direction);
  const float3 world_geometric_normal =
      EE_RT_WORLD_NORMAL(world_normal_matrix, object_geometric_normal, world_shading_normal);

  bool is_inside = shadow_is_inside != 0u;
  const float cos_theta =
      abs(dot(normalize(ray_direction), EE_RT_SAFE_NORMALIZE(world_geometric_normal, -ray_direction)));
  const float hit_t = ray_query.CandidateTriangleRayT();
  const float segment_length = max(0.0f, hit_t - previous_hit_t);
  const float3 transmission = EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0_SPECIALIZED<feature_mask>(
      material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color, cos_theta, segment_length,
      is_inside, EE_CAMERA_MIN_SHADOW_TRANSMISSION);
  shadow_is_inside = is_inside ? 1u : 0u;
  previous_hit_t = hit_t;
  return max(transmission, float3(0.0f));
}

#if EE_RAY_TRACING_LINEAR_SWEPT_SPHERES_SUPPORTED
float3 EE_CAMERA_RQ_CANDIDATE_LSS_SHADOW_TRANSMISSION<let Flags : uint, let feature_mask : uint>(
    inout RayQuery<Flags> ray_query, const uint material_index, const float2 tex_coord_0, const float2 tex_coord_1,
    const float2 tex_coord_2, const float2 tex_coord_3, const float3 vertex_color,
    const float3 world_geometric_normal, const float3 ray_direction, inout float previous_hit_t,
    inout uint shadow_is_inside) {
  bool is_inside = shadow_is_inside != 0u;
  const float cos_theta =
      abs(dot(normalize(ray_direction), EE_RT_SAFE_NORMALIZE(world_geometric_normal, -ray_direction)));
  const float hit_t = ray_query.CandidateTriangleRayT();
  const float segment_length = max(0.0f, hit_t - previous_hit_t);
  const float3 transmission = EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0_SPECIALIZED<feature_mask>(
      material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color, cos_theta, segment_length,
      is_inside, EE_CAMERA_MIN_SHADOW_TRANSMISSION);
  shadow_is_inside = is_inside ? 1u : 0u;
  previous_hit_t = hit_t;
  return max(transmission, float3(0.0f));
}
#endif

public
float3 EE_CAMERA_RQ_SHADOW_TRANSMISSION<let feature_mask : uint>(const float3 origin, const float3 direction,
                                                                 const float max_distance, inout uint seed,
                                                                 const bool initial_inside) {
  if (max_distance <= EE_CAMERA_RAY_EPSILON) {
    return float3(1.0f);
  }
  float3 shadow_transmission = float3(1.0f);
  float previous_hit_t = 0.0f;
  uint shadow_is_inside = initial_inside ? 1u : 0u;
  RayDesc ray = {origin, EE_CAMERA_RAY_EPSILON, direction, max_distance};
  RayQuery<RAY_FLAG_NONE> triangle_query;
  triangle_query.TraceRayInline(EE_TLAS, RAY_FLAG_NONE, EE_CAMERA_TRIANGLE_RAY_MASK_SHADOW, ray);
  while (triangle_query.Proceed()) {
    if (triangle_query.CandidateType() != CANDIDATE_NON_OPAQUE_TRIANGLE) {
      continue;
    }
    uint material_index;
    float2 tex_coord_0;
    float2 tex_coord_1;
    float2 tex_coord_2;
    float2 tex_coord_3;
    float4 vertex_color;
    EE_CAMERA_RQ_CANDIDATE_SURFACE(triangle_query, material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3,
                                   vertex_color);
    const float opacity = EE_GLTF_RASTER_OPACITY_LOD0_SPECIALIZED<feature_mask>(
        material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color.a);
    if (EE_PCG_RANDOM(seed) > opacity) {
      continue;
    }
    shadow_transmission *= EE_CAMERA_RQ_CANDIDATE_SHADOW_TRANSMISSION<RAY_FLAG_NONE, feature_mask>(
        triangle_query, material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color.rgb,
        direction, previous_hit_t, shadow_is_inside);
    if (max(max(shadow_transmission.x, shadow_transmission.y), shadow_transmission.z) <=
        EE_CAMERA_MIN_SHADOW_TRANSMISSION) {
      return float3(0.0f);
    }
  }
  if (triangle_query.CommittedStatus() == COMMITTED_TRIANGLE_HIT) {
    return float3(0.0f);
  }

#if EE_RAY_TRACING_LINEAR_SWEPT_SPHERES_SUPPORTED
  float lss_previous_hit_t = 0.0f;
  uint lss_shadow_is_inside = initial_inside ? 1u : 0u;
  RayQuery<RAY_FLAG_NONE> lss_query;
  lss_query.TraceRayInline(EE_TLAS, RAY_FLAG_NONE, EE_CAMERA_LSS_RAY_MASK_SHADOW, ray);
  while (lss_query.Proceed()) {
    uint material_index;
    float2 tex_coord_0;
    float2 tex_coord_1;
    float2 tex_coord_2;
    float2 tex_coord_3;
    float4 vertex_color;
    float parameter;
    float3 object_geometric_normal;
    float3 world_geometric_normal;
    EE_CAMERA_RQ_CANDIDATE_LSS_SURFACE(lss_query, material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3,
                                       vertex_color, parameter, object_geometric_normal, world_geometric_normal);
    const float opacity = EE_GLTF_RASTER_OPACITY_LOD0_SPECIALIZED<feature_mask>(
        material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color.a);
    if (EE_PCG_RANDOM(seed) > opacity) {
      continue;
    }
    shadow_transmission *= EE_CAMERA_RQ_CANDIDATE_LSS_SHADOW_TRANSMISSION<RAY_FLAG_NONE, feature_mask>(
        lss_query, material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color.rgb,
        world_geometric_normal, direction, lss_previous_hit_t, lss_shadow_is_inside);
    if (max(max(shadow_transmission.x, shadow_transmission.y), shadow_transmission.z) <=
        EE_CAMERA_MIN_SHADOW_TRANSMISSION) {
      return float3(0.0f);
    }
  }
  if (lss_query.CommittedIsLssNV()) {
    return float3(0.0f);
  }
#endif
  return shadow_transmission;
}
