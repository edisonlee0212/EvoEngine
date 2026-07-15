#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_ray_tracing : require

#include "CameraRayTracingPayload.glsl"

layout(location = 0) rayPayloadInEXT CameraRayTracingPayload hit_value;

hitAttributeEXT vec2 attribs;

const uint EE_CAMERA_RAY_PAYLOAD_SURFACE = 0u;
const uint EE_CAMERA_RAY_PAYLOAD_SHADOW = 1u;

void main() {
  if (hit_value.type == EE_CAMERA_RAY_PAYLOAD_SHADOW) {
    hit_value.shadow_transmission = vec3(0.0f);
    return;
  }

  hit_value.type = EE_CAMERA_RAY_PAYLOAD_SURFACE;
  hit_value.hit_t = gl_HitTEXT;
  hit_value.instance_index = uint(gl_InstanceCustomIndexEXT);
  hit_value.primitive_id = uint(gl_PrimitiveID);
  hit_value.barycentrics = attribs;
}
