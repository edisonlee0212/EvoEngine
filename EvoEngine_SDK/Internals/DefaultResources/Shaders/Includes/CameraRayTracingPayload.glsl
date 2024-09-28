struct CameraRayTracingPayload {
  vec3 color;
  uint seed;

  vec3 intersection;
  uint type;

  vec3 normal;
  uint hit_count;
};

layout(location = 0) rayPayloadEXT CameraRayTracingPayload hit_value;