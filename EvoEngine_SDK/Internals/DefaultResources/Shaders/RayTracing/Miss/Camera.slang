#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_ray_tracing : require

#include "CameraRayTracingPayload.slangh"

const uint EE_CAMERA_RAY_PAYLOAD_MISS = 2u;

[shader("miss")]
void main(inout CameraRayTracingPayload hit_value) {
  hit_value.type = EE_CAMERA_RAY_PAYLOAD_MISS;
}
