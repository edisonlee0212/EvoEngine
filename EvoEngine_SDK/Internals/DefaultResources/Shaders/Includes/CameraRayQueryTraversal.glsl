#ifndef EE_CAMERA_RAY_QUERY_TRAVERSAL_GLSL
#define EE_CAMERA_RAY_QUERY_TRAVERSAL_GLSL

void EE_CAMERA_RAY_QUERY_APPLY_MISS() {
  hit_value.type = EE_CAMERA_RAY_PAYLOAD_MISS;
}

vec3 EE_CAMERA_RAY_QUERY_WORLD_NORMAL(const mat4 model, const vec3 object_normal, const vec3 fallback) {
  const mat3 normal_matrix = transpose(inverse(mat3(model)));
  return EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(normal_matrix * object_normal, fallback);
}

void EE_CAMERA_RAY_QUERY_FILL_SURFACE_PAYLOAD(const rayQueryEXT ray_query) {
  const int instance_index = rayQueryGetIntersectionInstanceCustomIndexEXT(ray_query, true);
  const int primitive_id = rayQueryGetIntersectionPrimitiveIndexEXT(ray_query, true);
  const vec2 bary = rayQueryGetIntersectionBarycentricsEXT(ray_query, true);

  hit_value.type = EE_CAMERA_RAY_PAYLOAD_SURFACE;
  hit_value.hit_t = rayQueryGetIntersectionTEXT(ray_query, true);
  hit_value.instance_index = uint(instance_index);
  hit_value.primitive_id = uint(primitive_id);
  hit_value.barycentrics = bary;
}

void EE_CAMERA_RAY_QUERY_CANDIDATE_SURFACE(const rayQueryEXT ray_query, out uint material_index,
                                           out vec2 tex_coord_0, out vec2 tex_coord_1, out vec2 tex_coord_2,
                                           out vec2 tex_coord_3, out vec4 vertex_color) {
  const int instance_index = rayQueryGetIntersectionInstanceCustomIndexEXT(ray_query, false);
  const Instance instance = EE_INSTANCES[instance_index];
  material_index = uint(instance.material_index);
  const int primitive_id = rayQueryGetIntersectionPrimitiveIndexEXT(ray_query, false);
  const int triangle_offset = instance.triangle_offset + primitive_id;

  const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
  const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
  const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];
  const vec2 bary = rayQueryGetIntersectionBarycentricsEXT(ray_query, false);
  const vec3 barycentrics = vec3(1.0f - bary.x - bary.y, bary.x, bary.y);
  tex_coord_0 = v0.tex_coord * barycentrics.x + v1.tex_coord * barycentrics.y +
                v2.tex_coord * barycentrics.z;
  tex_coord_1 = v0.tex_coord_1 * barycentrics.x + v1.tex_coord_1 * barycentrics.y +
                v2.tex_coord_1 * barycentrics.z;
  tex_coord_2 = v0.tex_coord_2 * barycentrics.x + v1.tex_coord_2 * barycentrics.y +
                v2.tex_coord_2 * barycentrics.z;
  tex_coord_3 = v0.tex_coord_3 * barycentrics.x + v1.tex_coord_3 * barycentrics.y +
                v2.tex_coord_3 * barycentrics.z;
  vertex_color = v0.color * barycentrics.x + v1.color * barycentrics.y + v2.color * barycentrics.z;
}

void EE_CAMERA_TRACE_SURFACE(const vec3 origin, const vec3 direction, const float min_distance, inout uint seed) {
  rayQueryEXT ray_query;
  rayQueryInitializeEXT(ray_query, EE_TLAS, gl_RayFlagsCullBackFacingTrianglesEXT, 0xff, origin, min_distance, direction,
                        EE_CAMERA_MAX_TRACE_DISTANCE);
  while (rayQueryProceedEXT(ray_query)) {
    if (rayQueryGetIntersectionTypeEXT(ray_query, false) != gl_RayQueryCandidateIntersectionTriangleEXT) {
      continue;
    }
    uint material_index;
    vec2 tex_coord_0;
    vec2 tex_coord_1;
    vec2 tex_coord_2;
    vec2 tex_coord_3;
    vec4 vertex_color;
    EE_CAMERA_RAY_QUERY_CANDIDATE_SURFACE(ray_query, material_index, tex_coord_0, tex_coord_1, tex_coord_2,
                                          tex_coord_3, vertex_color);
    if (EE_PCG_RANDOM(seed) <=
        EE_GLTF_RASTER_OPACITY_LOD0(
            material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color.a)) {
      rayQueryConfirmIntersectionEXT(ray_query);
    }
  }

  if (rayQueryGetIntersectionTypeEXT(ray_query, true) == gl_RayQueryCommittedIntersectionTriangleEXT) {
    EE_CAMERA_RAY_QUERY_FILL_SURFACE_PAYLOAD(ray_query);
  } else {
    EE_CAMERA_RAY_QUERY_APPLY_MISS();
  }
}

vec3 EE_CAMERA_RAY_QUERY_SHADOW_TRANSMISSION(const rayQueryEXT ray_query, const uint material_index,
                                              const vec2 tex_coord_0, const vec2 tex_coord_1,
                                              const vec2 tex_coord_2, const vec2 tex_coord_3,
                                              const vec3 vertex_color, const vec3 ray_direction,
                                              inout float previous_hit_t, inout uint shadow_is_inside) {
  const int instance_index = rayQueryGetIntersectionInstanceCustomIndexEXT(ray_query, false);
  const Instance instance = EE_INSTANCES[instance_index];
  const int primitive_id = rayQueryGetIntersectionPrimitiveIndexEXT(ray_query, false);
  const int triangle_offset = instance.triangle_offset + primitive_id;
  const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
  const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
  const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];
  const vec2 bary = rayQueryGetIntersectionBarycentricsEXT(ray_query, false);
  const vec3 barycentrics = vec3(1.0f - bary.x - bary.y, bary.x, bary.y);
  const vec3 object_shading_normal = EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(
      v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z,
      vec3(0.0f, 1.0f, 0.0f));
  const vec3 object_geometric_normal =
      EE_CAMERA_GEOMETRIC_NORMAL(v1.position - v0.position, v2.position - v0.position, object_shading_normal);
  const vec3 world_shading_normal =
      EE_CAMERA_RAY_QUERY_WORLD_NORMAL(instance.model, object_shading_normal, -ray_direction);
  const vec3 world_geometric_normal = EE_CAMERA_RAY_QUERY_WORLD_NORMAL(instance.model, object_geometric_normal,
                                                                       world_shading_normal);

  bool is_inside = shadow_is_inside != 0u;
  const float cos_theta = abs(dot(normalize(ray_direction),
                                  EE_CAMERA_SAFE_NORMALIZE(world_geometric_normal, -ray_direction)));
  const float hit_t = rayQueryGetIntersectionTEXT(ray_query, false);
  const float segment_length = max(0.0f, hit_t - previous_hit_t);
  const vec3 transmission = EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0(
      material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color, cos_theta,
      segment_length, is_inside, EE_CAMERA_MIN_SHADOW_TRANSMISSION);
  shadow_is_inside = is_inside ? 1u : 0u;
  previous_hit_t = hit_t;
  return max(transmission, vec3(0.0f));
}

vec3 EE_CAMERA_SHADOW_TRANSMISSION(const vec3 origin, const vec3 direction, const float max_distance,
                                    inout uint seed, const bool initial_inside) {
  if (max_distance <= EE_CAMERA_RAY_EPSILON) {
    return vec3(1.0f);
  }
  vec3 shadow_transmission = vec3(1.0f);
  float previous_hit_t = 0.0f;
  uint shadow_is_inside = initial_inside ? 1u : 0u;
  rayQueryEXT ray_query;
  rayQueryInitializeEXT(ray_query, EE_TLAS, 0, EE_CAMERA_RAY_MASK_SHADOW, origin, EE_CAMERA_RAY_EPSILON, direction,
                        max_distance);
  while (rayQueryProceedEXT(ray_query)) {
    if (rayQueryGetIntersectionTypeEXT(ray_query, false) != gl_RayQueryCandidateIntersectionTriangleEXT) {
      continue;
    }
    uint material_index;
    vec2 tex_coord_0;
    vec2 tex_coord_1;
    vec2 tex_coord_2;
    vec2 tex_coord_3;
    vec4 vertex_color;
    EE_CAMERA_RAY_QUERY_CANDIDATE_SURFACE(ray_query, material_index, tex_coord_0, tex_coord_1, tex_coord_2,
                                          tex_coord_3, vertex_color);
    if (EE_PCG_RANDOM(seed) >
        EE_GLTF_RASTER_OPACITY_LOD0(
            material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color.a)) {
      continue;
    }
    shadow_transmission *= EE_CAMERA_RAY_QUERY_SHADOW_TRANSMISSION(
        ray_query, material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color.rgb,
        direction, previous_hit_t, shadow_is_inside);
    if (max(max(shadow_transmission.x, shadow_transmission.y), shadow_transmission.z) <=
        EE_CAMERA_MIN_SHADOW_TRANSMISSION) {
      return vec3(0.0f);
    }
  }
  if (rayQueryGetIntersectionTypeEXT(ray_query, true) == gl_RayQueryCommittedIntersectionTriangleEXT) {
    return vec3(0.0f);
  }
  return shadow_transmission;
}

#endif
