#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_ray_tracing : require

#define EE_GLTF_USE_EXPLICIT_TEXTURE_LOD
#define EE_GLTF_TEXTURE_LOD 0.0
#include "PointCloudRayTracingPayload.slangh"
#include "RayTracingBasic.slangh"
#include "RayTracingMaterial.slangh"

layout(push_constant) uniform EE_DDGI_PROBE_RAY_CONSTANTS {
  vec4 first_probe;
  vec4 probe_step_x;
  vec4 probe_step_y;
  vec4 probe_step_z;
  uvec4 probe_counts_and_ray_count;
  uvec4 selected_probe_volume_flags_environment;
  vec4 trace_parameters;
  ivec4 probe_scroll_offset;
};

[shader("anyhit")]
void main(inout PointCloudRayTracingPayload hit_value, in BuiltInTriangleIntersectionAttributes attribs) {
  const int instance_index = int(InstanceID());
  const Instance instance = EE_INSTANCES[instance_index];
  const uint material_index = uint(instance.material_index);
  if (EE_GLTF_MATERIALS[material_index].alpha_mode != EE_GLTF_ALPHA_MODE_MASK) {
    return;
  }

  const int triangle_offset = instance.triangle_offset + PrimitiveIndex();
  const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
  const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
  const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];
  const vec2 hit_barycentrics = attribs.barycentrics;
  const vec3 barycentrics = vec3(1.0f - hit_barycentrics.x - hit_barycentrics.y, hit_barycentrics.x,
                                 hit_barycentrics.y);
  const vec3 object_normal = EE_RT_SAFE_NORMALIZE(
      v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z,
      vec3(0.0f, 1.0f, 0.0f));
  const vec3 object_geometric_normal =
      EE_RT_GEOMETRIC_NORMAL(v1.position - v0.position, v2.position - v0.position, object_normal);
  const vec3 world_geometric_normal =
      EE_RT_WORLD_NORMAL(instance.model, object_geometric_normal, -normalize(WorldRayDirection()));
  const bool shadow_ray = hit_value.hit_count != 0u;
  const vec4 tex_gradients =
      shadow_ray ? vec4(0.0f)
                 : EE_RT_TEXTURE_GRADIENTS(
                       0.0f, RayTCurrent(), world_geometric_normal, normalize(WorldRayDirection()), instance.model,
                       v0, v1, v2, EE_RT_SPHERICAL_RAY_SPREAD(probe_counts_and_ray_count.w));
  const EeRayTracingSurfaceAttributes attributes =
      EE_RT_INTERPOLATE_SURFACE_ATTRIBUTES(v0, v1, v2, barycentrics, tex_gradients);
  const bool alpha_mask_passes =
      shadow_ray ? EE_GLTF_RASTER_ALPHA_MASK_PASSES_LOD0(
                       material_index, attributes.tex_coords, attributes.vertex_color.a)
                 : EE_GLTF_RASTER_ALPHA_MASK_PASSES(
                       material_index, attributes.tex_coords, attributes.vertex_color.a);
  if (!alpha_mask_passes) {
    IgnoreHit();
  }
}
