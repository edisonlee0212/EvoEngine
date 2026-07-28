#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_ray_tracing : require

#ifndef EE_SHADER_EXECUTION_REORDERING_SUPPORTED
#define EE_SHADER_EXECUTION_REORDERING_SUPPORTED 0
#endif
#include "RayTracingBasic.slangh"
#include "Random.slangh"
#define EE_GLTF_USE_EXPLICIT_TEXTURE_LOD
#define EE_GLTF_TEXTURE_LOD 0.0
#include "GltfRayTracingBsdf.slangh"
#include "CameraRayTracingPayload.slangh"

layout(set = 2, binding = 0, rgba32f) uniform image2D result_image;
layout(set = 2, binding = 1, r32f) uniform image2D ray_hit_distance_image;
layout(set = 2, binding = 2, rgba32f) uniform image2D radiance_history_image;
layout(set = 2, binding = 3, rgba32f) uniform image2D convergence_history_image;

layout(push_constant) uniform EE_CAMERA_CONSTANTS {
  uint EE_CAMERA_INDEX;
  uint EE_FRAME_ID;
  uint EE_TOTAL_SAMPLES;
  uint EE_FRAME_SAMPLE_SIZE;
  uint EE_SHADER_EXECUTION_REORDERING;
};

CameraRayTracingPayload hit_value;

void EE_CAMERA_TRACE_RAY(const accelerationStructureEXT acceleration_structure, const uint ray_flags,
                         const uint instance_inclusion_mask, const uint ray_contribution_to_hit_group_index,
                         const uint multiplier_for_geometry_contribution_to_hit_group_index,
                         const uint miss_shader_index, const vec3 origin, const float t_min,
                         const vec3 direction, const float t_max) {
  RayDesc ray;
  ray.Origin = origin;
  ray.Direction = direction;
  ray.TMin = t_min;
  ray.TMax = t_max;
  TraceRay(acceleration_structure, ray_flags, instance_inclusion_mask, ray_contribution_to_hit_group_index,
           multiplier_for_geometry_contribution_to_hit_group_index, miss_shader_index, ray, hit_value);
}

#define EE_TRACE_RAY EE_CAMERA_TRACE_RAY

#define EE_CAMERA_RAY_TRACING_TRAVERSAL
#include "CameraRayIntegrator.slangh"

[shader("raygeneration")]
void main() {
  EE_CAMERA_RENDER_PIXEL(uvec2(DispatchRaysIndex().xy), uvec2(DispatchRaysDimensions().xy));
}
