struct CameraRayTracingPayload {
  vec3 color;
  uint seed;

  vec3 position;
  uint type;

  vec3 normal;
  uint hit_count;

  vec3 initial_position;
  uint padding0;

  vec3 initial_normal;
  uint padding1;
};

