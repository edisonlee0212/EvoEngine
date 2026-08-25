__exported import EvoEngine.CameraRayTraversal;
import EvoEngine.CameraRayQueryTraversal;
import EvoEngine.CameraRayTracingTraversal;

public struct CameraRayTracingTraversalPolicy : ICameraRayTraversal {
  bool shader_execution_reordering;

  [mutating] void initialize(const bool enabled) {
    shader_execution_reordering = enabled;
  }

  [mutating] void traceSurface(const float3 origin, const float3 direction, const float min_distance,
                               inout uint seed, inout CameraRayTracingPayload payload) {
    EE_CAMERA_RT_TRACE_SURFACE(origin, direction, min_distance, shader_execution_reordering, seed, payload);
  }

  [mutating] float3 shadowTransmission(const float3 origin, const float3 direction, const float max_distance,
                                       inout uint seed, const bool initial_inside,
                                       inout CameraRayTracingPayload payload) {
    return EE_CAMERA_RT_SHADOW_TRANSMISSION(origin, direction, max_distance, seed, initial_inside, payload);
  }
}

public struct CameraRayQueryTraversalPolicy<let feature_mask : uint> : ICameraRayTraversal {
  [mutating] void initialize(const bool shader_execution_reordering) {}

  [mutating] void traceSurface(const float3 origin, const float3 direction, const float min_distance,
                               inout uint seed, inout CameraRayTracingPayload payload) {
    EE_CAMERA_RQ_TRACE_SURFACE<feature_mask>(origin, direction, min_distance, seed, payload);
  }

  [mutating] float3 shadowTransmission(const float3 origin, const float3 direction, const float max_distance,
                                       inout uint seed, const bool initial_inside,
                                       inout CameraRayTracingPayload payload) {
    return EE_CAMERA_RQ_SHADOW_TRANSMISSION<feature_mask>(origin, direction, max_distance, seed, initial_inside);
  }
}
