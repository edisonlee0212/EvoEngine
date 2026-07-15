#ifndef EE_CAMERA_RAY_TRACING_TRAVERSAL_GLSL
#define EE_CAMERA_RAY_TRACING_TRAVERSAL_GLSL

vec3 EE_CAMERA_SHADOW_TRANSMISSION(const vec3 origin, const vec3 direction, const float max_distance,
                                    inout uint seed, const bool initial_inside) {
  if (max_distance <= EE_CAMERA_RAY_EPSILON) {
    return vec3(1.0f);
  }
  hit_value.type = EE_CAMERA_RAY_PAYLOAD_SHADOW;
  hit_value.seed = seed;
  hit_value.shadow_transmission = vec3(1.0f);
  hit_value.shadow_previous_hit_t = 0.0f;
  hit_value.shadow_is_inside = initial_inside ? 1u : 0u;
  traceRayEXT(EE_TLAS, gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsSkipClosestHitShaderEXT,
              EE_CAMERA_RAY_MASK_SHADOW, 0, 0, 0, origin, EE_CAMERA_RAY_EPSILON, direction, max_distance, 0);
  const vec3 shadow_transmission = hit_value.type == EE_CAMERA_RAY_PAYLOAD_MISS
                                       ? max(hit_value.shadow_transmission, vec3(0.0f))
                                       : vec3(0.0f);
  seed = hit_value.seed;
  return max(max(shadow_transmission.x, shadow_transmission.y), shadow_transmission.z) <=
                 EE_CAMERA_MIN_SHADOW_TRANSMISSION
             ? vec3(0.0f)
             : shadow_transmission;
}

void EE_CAMERA_TRACE_SURFACE(const vec3 origin, const vec3 direction, const float min_distance,
                             inout uint seed) {
  hit_value.seed = seed;
#if EE_SHADER_EXECUTION_REORDERING_SUPPORTED
  if (EE_SHADER_EXECUTION_REORDERING != 0u) {
    hitObjectNV hit_object;
    hitObjectTraceRayNV(hit_object, EE_TLAS, gl_RayFlagsCullBackFacingTrianglesEXT, 0xff, 0, 0, 0, origin,
                        min_distance, direction, EE_CAMERA_MAX_TRACE_DISTANCE, 0);
    reorderThreadNV(hit_object);
    hitObjectExecuteShaderNV(hit_object, 0);
    return;
  }
#endif
  traceRayEXT(EE_TLAS, gl_RayFlagsCullBackFacingTrianglesEXT, 0xff, 0, 0, 0, origin, min_distance, direction,
              EE_CAMERA_MAX_TRACE_DISTANCE, 0);
}

#endif
