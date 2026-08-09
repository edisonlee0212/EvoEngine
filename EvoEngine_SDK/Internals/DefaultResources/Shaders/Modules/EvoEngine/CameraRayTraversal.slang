// @evoengine-dialect native
__exported import EvoEngine.CameraRayTracingPayload;

public interface ICameraRayTraversal {
  [mutating] void initialize(const bool shader_execution_reordering);
  [mutating] void traceSurface(const float3 origin, const float3 direction, const float min_distance,
                               inout uint seed, inout CameraRayTracingPayload payload);
  [mutating] float3 shadowTransmission(const float3 origin, const float3 direction, const float max_distance,
                                       inout uint seed, const bool initial_inside,
                                       inout CameraRayTracingPayload payload);
}
