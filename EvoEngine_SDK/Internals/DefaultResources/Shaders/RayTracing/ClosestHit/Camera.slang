// @evoengine-dialect native
import EvoEngine.CameraRayConstants;
import EvoEngine.CameraRayGeometry;
import EvoEngine.CameraRayTracingPayload;

[shader("closesthit")]
void main(inout CameraRayTracingPayload hit_value, in BuiltInTriangleIntersectionAttributes attributes) {
  if (hit_value.type == EE_CAMERA_RAY_PAYLOAD_SHADOW) {
    hit_value.shadow_transmission = float3(0.0f);
    return;
  }

  hit_value.type = EE_CAMERA_RAY_PAYLOAD_SURFACE;
  hit_value.hit_t = RayTCurrent();
  hit_value.instance_index = InstanceID();
  hit_value.primitive_id = PrimitiveIndex();
#if EE_RAY_TRACING_LINEAR_SWEPT_SPHERES_SUPPORTED
  if (IsLssHit()) {
    const float2x4 endpoints = GetLssPositionsAndRadii();
    const float3 segment = endpoints[1].xyz - endpoints[0].xyz;
    const float segment_length_squared = dot(segment, segment);
    const float3 object_hit = ObjectRayOrigin() + ObjectRayDirection() * RayTCurrent();
    const float parameter = segment_length_squared > 0.0f
                                ? clamp(dot(object_hit - endpoints[0].xyz, segment) / segment_length_squared, 0.0f, 1.0f)
                                : 0.0f;
    const float3 center = lerp(endpoints[0].xyz, endpoints[1].xyz, parameter);
    const float3 fallback = EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(
        cross(segment, ObjectRayDirection()), float3(0.0f, 1.0f, 0.0f));
    hit_value.barycentrics = float2(parameter, 0.0f);
    hit_value.object_geometric_normal =
        EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(object_hit - center, fallback);
    hit_value.geometry_type = EE_CAMERA_GEOMETRY_LINEAR_SWEPT_SPHERE;
    return;
  }
#endif
  hit_value.barycentrics = attributes.barycentrics;
  hit_value.object_geometric_normal = float3(0.0f);
  hit_value.geometry_type = EE_CAMERA_GEOMETRY_TRIANGLE;
}
