// @evoengine-dialect native
import EvoEngine.CameraRayConstants;
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
  hit_value.barycentrics = attributes.barycentrics;
}
