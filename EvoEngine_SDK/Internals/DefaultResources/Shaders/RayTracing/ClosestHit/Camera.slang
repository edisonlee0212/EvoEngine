#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_ray_tracing : require

#include "CameraRayTracingPayload.slangh"

const uint EE_CAMERA_RAY_PAYLOAD_SURFACE = 0u;
const uint EE_CAMERA_RAY_PAYLOAD_SHADOW = 1u;

[shader("closesthit")]
void main(inout CameraRayTracingPayload hit_value, in BuiltInTriangleIntersectionAttributes attribs) {
  if (hit_value.type == EE_CAMERA_RAY_PAYLOAD_SHADOW) {
    hit_value.shadow_transmission = vec3(0.0f);
    return;
  }

  hit_value.type = EE_CAMERA_RAY_PAYLOAD_SURFACE;
  hit_value.hit_t = RayTCurrent();
  hit_value.instance_index = InstanceID();
  hit_value.primitive_id = PrimitiveIndex();
  hit_value.barycentrics = attribs.barycentrics;
}
