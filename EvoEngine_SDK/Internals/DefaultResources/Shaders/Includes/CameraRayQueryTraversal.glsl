#ifndef EE_CAMERA_RAY_QUERY_TRAVERSAL_GLSL
#define EE_CAMERA_RAY_QUERY_TRAVERSAL_GLSL

const uint EE_CAMERA_RAY_PAYLOAD_MISS = 2u;

vec3 EE_CAMERA_RAY_QUERY_SKY_RADIANCE(const vec3 ray_direction) {
  const Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
  if (camera.use_clear_color == 1) {
    return max(camera.clear_color.xyz, vec3(0.0f)) * max(camera.clear_color.w, 0.0f);
  }
  return EE_CAMERA_SAMPLE_CUBEMAP_RADIANCE(camera.skybox_tex_index, ray_direction, 0.0f) *
         max(camera.clear_color.w, 0.0f);
}

float EE_CAMERA_RAY_QUERY_ENVIRONMENT_PDF(const vec3 ray_direction) {
  if (EE_ENVIRONMENT.light_intensity <= 0.0f && EE_CAMERAS[EE_CAMERA_INDEX].use_clear_color != 1) {
    return 0.0f;
  }
  if (EE_ENVIRONMENT.background_color.w != 1.0f && EE_CAMERAS[EE_CAMERA_INDEX].use_clear_color != 1) {
    return EE_CAMERA_ENVIRONMENT_MAP_PDF(normalize(ray_direction));
  }
  return 1.0f / (4.0f * EE_CAMERA_PI);
}

void EE_CAMERA_RAY_QUERY_APPLY_MISS(const vec3 ray_direction) {
  const vec3 direction = normalize(ray_direction);
  hit_value.type = EE_CAMERA_RAY_PAYLOAD_MISS;
  hit_value.hit_count = 0u;
  hit_value.hit_t = EE_CAMERA_FAR(int(EE_CAMERA_INDEX));
  hit_value.position = vec3(0.0f);
  hit_value.normal = -direction;
  hit_value.geometric_normal = -direction;
  hit_value.environment_radiance = EE_CAMERA_PATH_ENVIRONMENT_RADIANCE(direction);
  hit_value.environment_pdf = EE_CAMERA_RAY_QUERY_ENVIRONMENT_PDF(direction);
  hit_value.color = EE_CAMERA_RAY_QUERY_SKY_RADIANCE(direction);
}

vec3 EE_CAMERA_RAY_QUERY_WORLD_NORMAL(const mat4 model, const vec3 object_normal, const vec3 fallback) {
  const mat3 normal_matrix = transpose(inverse(mat3(model)));
  return EE_CAMERA_SAFE_NORMALIZE(normal_matrix * object_normal, fallback);
}

void EE_CAMERA_RAY_QUERY_FILL_SURFACE_PAYLOAD(const rayQueryEXT ray_query, const vec3 ray_direction) {
  const int instance_index = rayQueryGetIntersectionInstanceCustomIndexEXT(ray_query, true);
  const Instance instance = EE_INSTANCES[instance_index];
  const uint material_index = uint(instance.material_index);
  const int primitive_id = rayQueryGetIntersectionPrimitiveIndexEXT(ray_query, true);
  const int triangle_offset = instance.triangle_offset + primitive_id;

  const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
  const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
  const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];

  const vec2 bary = rayQueryGetIntersectionBarycentricsEXT(ray_query, true);
  const vec3 barycentrics = vec3(1.0f - bary.x - bary.y, bary.x, bary.y);
  const vec3 object_position = v0.position * barycentrics.x + v1.position * barycentrics.y +
                               v2.position * barycentrics.z;
  const vec3 object_shading_normal = EE_CAMERA_SAFE_NORMALIZE(
      v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z,
      vec3(0.0f, 1.0f, 0.0f));
  const vec3 object_geometric_normal = EE_CAMERA_SAFE_NORMALIZE(cross(v1.position - v0.position,
                                                                      v2.position - v0.position),
                                                                object_shading_normal);
  const vec3 world_position = vec3(instance.model * vec4(object_position, 1.0f));
  vec3 world_geometric_normal = EE_CAMERA_RAY_QUERY_WORLD_NORMAL(instance.model, object_geometric_normal,
                                                                 vec3(0.0f, 1.0f, 0.0f));
  vec3 world_shading_normal = EE_CAMERA_RAY_QUERY_WORLD_NORMAL(instance.model, object_shading_normal,
                                                               world_geometric_normal);
  const bool front_face = dot(world_geometric_normal, ray_direction) < 0.0f;
  if (!front_face) {
    world_geometric_normal = -world_geometric_normal;
    world_shading_normal = -world_shading_normal;
  }
  if (dot(world_shading_normal, world_geometric_normal) < 0.0f) {
    world_shading_normal = -world_shading_normal;
  }

  hit_value.type = EE_CAMERA_RAY_PAYLOAD_SURFACE;
  hit_value.hit_count = 1u;
  hit_value.hit_t = rayQueryGetIntersectionTEXT(ray_query, true);
  hit_value.position = world_position;
  hit_value.normal = world_shading_normal;
  hit_value.geometric_normal = world_geometric_normal;
  hit_value.initial_position = world_position;
  hit_value.initial_normal = world_shading_normal;
  hit_value.instance_index = uint(instance_index);
  hit_value.primitive_id = uint(primitive_id);
  hit_value.material_index = material_index;
  hit_value.barycentrics = bary;
  hit_value.environment_radiance = vec3(0.0f);
  hit_value.environment_pdf = 0.0f;
}

bool EE_CAMERA_RAY_QUERY_CANDIDATE_SURFACE(const rayQueryEXT ray_query, const vec3 ray_direction,
                                           out uint material_index, out vec2 tex_coord_0, out vec2 tex_coord_1,
                                           out vec4 vertex_color) {
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
  vertex_color = v0.color * barycentrics.x + v1.color * barycentrics.y + v2.color * barycentrics.z;
  return true;
}

void EE_CAMERA_TRACE_SURFACE(const vec3 origin, const vec3 direction, const float min_distance, inout uint seed) {
  rayQueryEXT ray_query;
  rayQueryInitializeEXT(ray_query, EE_TLAS, gl_RayFlagsCullBackFacingTrianglesEXT, 0xff, origin, min_distance, direction,
                        1e20f);
  while (rayQueryProceedEXT(ray_query)) {
    if (rayQueryGetIntersectionTypeEXT(ray_query, false) != gl_RayQueryCandidateIntersectionTriangleEXT) {
      continue;
    }
    uint material_index;
    vec2 tex_coord_0;
    vec2 tex_coord_1;
    vec4 vertex_color;
    EE_CAMERA_RAY_QUERY_CANDIDATE_SURFACE(ray_query, direction, material_index, tex_coord_0, tex_coord_1,
                                          vertex_color);
    if (EE_RANDOM(seed) <=
        EE_GLTF_RASTER_OPACITY_LOD0(material_index, tex_coord_0, tex_coord_1, vertex_color.a)) {
      rayQueryConfirmIntersectionEXT(ray_query);
    }
  }

  if (rayQueryGetIntersectionTypeEXT(ray_query, true) == gl_RayQueryCommittedIntersectionTriangleEXT) {
    EE_CAMERA_RAY_QUERY_FILL_SURFACE_PAYLOAD(ray_query, direction);
  } else {
    EE_CAMERA_RAY_QUERY_APPLY_MISS(direction);
  }
  hit_value.seed = seed;
}

vec3 EE_CAMERA_RAY_QUERY_SHADOW_TRANSMISSION(const rayQueryEXT ray_query, const uint material_index,
                                              const vec2 tex_coord_0, const vec2 tex_coord_1,
                                              const vec3 vertex_color, const vec3 ray_direction,
                                              inout float previous_hit_t, inout uint shadow_is_inside) {
  const int instance_index = rayQueryGetIntersectionInstanceCustomIndexEXT(ray_query, false);
  const Instance instance = EE_INSTANCES[instance_index];
  const int primitive_id = rayQueryGetIntersectionPrimitiveIndexEXT(ray_query, false);
  const int triangle_offset = instance.triangle_offset + primitive_id;
  const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
  const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
  const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];
  const vec3 object_geometric_normal = EE_CAMERA_SAFE_NORMALIZE(cross(v1.position - v0.position,
                                                                      v2.position - v0.position),
                                                                vec3(0.0f, 1.0f, 0.0f));
  const vec3 world_geometric_normal = EE_CAMERA_RAY_QUERY_WORLD_NORMAL(instance.model, object_geometric_normal,
                                                                       -ray_direction);

  bool is_inside = shadow_is_inside != 0u;
  const float cos_theta = abs(dot(normalize(ray_direction),
                                  EE_CAMERA_SAFE_NORMALIZE(world_geometric_normal, -ray_direction)));
  const float hit_t = rayQueryGetIntersectionTEXT(ray_query, false);
  const float segment_length = max(0.0f, hit_t - previous_hit_t);
  const vec3 transmission = EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0(
      material_index, tex_coord_0, tex_coord_1, vertex_color, cos_theta, segment_length, is_inside,
      EE_CAMERA_MIN_SHADOW_TRANSMISSION);
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
    vec4 vertex_color;
    EE_CAMERA_RAY_QUERY_CANDIDATE_SURFACE(ray_query, direction, material_index, tex_coord_0, tex_coord_1,
                                          vertex_color);
    if (EE_RANDOM(seed) >
        EE_GLTF_RASTER_OPACITY_LOD0(material_index, tex_coord_0, tex_coord_1, vertex_color.a)) {
      continue;
    }
    shadow_transmission *= EE_CAMERA_RAY_QUERY_SHADOW_TRANSMISSION(
        ray_query, material_index, tex_coord_0, tex_coord_1, vertex_color.rgb, direction, previous_hit_t,
        shadow_is_inside);
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
