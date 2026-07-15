#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_ray_tracing : require

#include "CameraRayTracingPayload.glsl"

layout(location = 0) rayPayloadInEXT CameraRayTracingPayload hit_value;

const uint EE_CAMERA_RAY_PAYLOAD_MISS = 2u;

void main() {
  hit_value.type = EE_CAMERA_RAY_PAYLOAD_MISS;
}
