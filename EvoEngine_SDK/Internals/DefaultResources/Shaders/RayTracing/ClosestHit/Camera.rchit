#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_ray_tracing : require

#include "CameraRayTracingPayload.glsl"
#include "RayTracingBasic.glsl"

layout(location = 0) rayPayloadInEXT CameraRayTracingPayload hit_value;

hitAttributeEXT vec2 attribs;

layout(push_constant) uniform EE_CAMERA_CONSTANTS {
  uint EE_CAMERA_INDEX;
  uint EE_FRAME_ID;
};

const uint EE_CAMERA_RAY_PAYLOAD_SURFACE = 0u;
const uint EE_CAMERA_RAY_PAYLOAD_SHADOW = 1u;

vec3 EE_CAMERA_SAFE_NORMALIZE(const vec3 value, const vec3 fallback) {
  const float length_squared = dot(value, value);
  return length_squared > 0.00000001f ? value * inversesqrt(length_squared) : fallback;
}

vec3 EE_CAMERA_WORLD_NORMAL(const vec3 object_normal, const vec3 fallback) {
  return EE_CAMERA_SAFE_NORMALIZE(vec3(object_normal * gl_WorldToObjectEXT), fallback);
}

void main() {
  if (hit_value.type == EE_CAMERA_RAY_PAYLOAD_SHADOW) {
    hit_value.hit_count = 1u;
    hit_value.shadow_transmission = vec3(0.0f);
    return;
  }

  const int instance_index = int(gl_InstanceCustomIndexEXT);
  const Instance instance = EE_INSTANCES[instance_index];
  const uint material_index = uint(instance.material_index);
  const int triangle_offset = instance.triangle_offset + gl_PrimitiveID;

  const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
  const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
  const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];

  const vec3 barycentrics = vec3(1.0f - attribs.x - attribs.y, attribs.x, attribs.y);
  const vec3 object_position = v0.position * barycentrics.x + v1.position * barycentrics.y +
                               v2.position * barycentrics.z;
  const vec3 object_shading_normal = EE_CAMERA_SAFE_NORMALIZE(
      v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z,
      vec3(0.0f, 1.0f, 0.0f));
  const vec3 object_geometric_normal = EE_CAMERA_SAFE_NORMALIZE(cross(v1.position - v0.position,
                                                                      v2.position - v0.position),
                                                                object_shading_normal);
  const vec3 world_position = vec3(gl_ObjectToWorldEXT * vec4(object_position, 1.0f));
  vec3 world_geometric_normal = EE_CAMERA_WORLD_NORMAL(object_geometric_normal, vec3(0.0f, 1.0f, 0.0f));
  vec3 world_shading_normal = EE_CAMERA_WORLD_NORMAL(object_shading_normal, world_geometric_normal);
  const bool front_face = dot(world_geometric_normal, gl_WorldRayDirectionEXT) < 0.0f;
  if (!front_face) {
    world_geometric_normal = -world_geometric_normal;
    world_shading_normal = -world_shading_normal;
  }
  if (dot(world_shading_normal, world_geometric_normal) < 0.0f) {
    world_shading_normal = -world_shading_normal;
  }

  hit_value.type = EE_CAMERA_RAY_PAYLOAD_SURFACE;
  hit_value.hit_count = 1u;
  hit_value.hit_t = gl_HitTEXT;
  hit_value.position = world_position;
  hit_value.normal = world_shading_normal;
  hit_value.geometric_normal = world_geometric_normal;
  hit_value.initial_position = world_position;
  hit_value.initial_normal = world_shading_normal;
  hit_value.instance_index = uint(instance_index);
  hit_value.primitive_id = uint(gl_PrimitiveID);
  hit_value.material_index = material_index;
  hit_value.barycentrics = attribs;
  hit_value.environment_radiance = vec3(0.0f);
  hit_value.environment_pdf = 0.0f;
}
