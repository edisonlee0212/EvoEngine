// @evoengine-dialect native
__exported import EvoEngine.CameraRayConstants;
__exported import EvoEngine.CameraRayTracingPayload;
__exported import EvoEngine.RayTracingBasic;

public float3 EE_CAMERA_RT_SHADOW_TRANSMISSION(const float3 origin, const float3 direction,
                                               const float max_distance, inout uint seed,
                                               const bool initial_inside,
                                               inout CameraRayTracingPayload hit_value) {
  if (max_distance <= EE_CAMERA_RAY_EPSILON) {
    return float3(1.0f);
  }
  hit_value.type = EE_CAMERA_RAY_PAYLOAD_SHADOW;
  hit_value.seed = seed;
  hit_value.shadow_transmission = float3(1.0f);
  hit_value.shadow_previous_hit_t = 0.0f;
  hit_value.shadow_is_inside = initial_inside ? 1u : 0u;
  RayDesc ray = {origin, EE_CAMERA_RAY_EPSILON, direction, max_distance};
  TraceRay(EE_TLAS, RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER,
           EE_CAMERA_RAY_MASK_SHADOW, 0, 0, 0, ray, hit_value);
  const float3 shadow_transmission = hit_value.type == EE_CAMERA_RAY_PAYLOAD_MISS
                                         ? max(hit_value.shadow_transmission, float3(0.0f))
                                         : float3(0.0f);
  seed = hit_value.seed;
  return max(max(shadow_transmission.x, shadow_transmission.y), shadow_transmission.z) <=
                 EE_CAMERA_MIN_SHADOW_TRANSMISSION
             ? float3(0.0f)
             : shadow_transmission;
}

bool EE_CAMERA_RT_TRACE_SURFACE_SER(const RayDesc ray, const bool shader_execution_reordering, inout uint seed,
                                    inout CameraRayTracingPayload hit_value) {
  __target_switch {
    case spvShaderInvocationReorderEXT:
      if (shader_execution_reordering) {
        HitObject hit_object = HitObject::TraceRay(EE_TLAS, RAY_FLAG_CULL_BACK_FACING_TRIANGLES, 0xff, 0, 0, 0, ray,
                                                   hit_value);
        ReorderThread(hit_object);
        HitObject::Invoke(EE_TLAS, hit_object, hit_value);
        seed = hit_value.seed;
        return true;
      }
      return false;
    default:
      return false;
  }
}

public void EE_CAMERA_RT_TRACE_SURFACE(const float3 origin, const float3 direction, const float min_distance,
                                       const bool shader_execution_reordering, inout uint seed,
                                       inout CameraRayTracingPayload hit_value) {
  hit_value.seed = seed;
  RayDesc ray = {origin, min_distance, direction, EE_CAMERA_MAX_TRACE_DISTANCE};
  if (EE_CAMERA_RT_TRACE_SURFACE_SER(ray, shader_execution_reordering, seed, hit_value)) {
    return;
  }
  TraceRay(EE_TLAS, RAY_FLAG_CULL_BACK_FACING_TRIANGLES, 0xff, 0, 0, 0, ray, hit_value);
  seed = hit_value.seed;
}
