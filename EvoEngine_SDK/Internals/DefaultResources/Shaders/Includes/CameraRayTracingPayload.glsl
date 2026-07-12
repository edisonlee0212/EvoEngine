#ifdef EE_CAMERA_COMPACT_PAYLOAD
struct CameraRayTracingPayload {
  vec3 shadow_transmission;
  uint seed;

  vec2 barycentrics;
  float hit_t;
  uint instance_index;

  float shadow_previous_hit_t;
  uint primitive_id;
  uint type;
  uint shadow_is_inside;
};
#else
struct CameraRayTracingPayload {
  vec3 color;
  uint seed;

  vec3 position;
  uint type;

  vec3 normal;
  uint hit_count;

  vec3 geometric_normal;
  float hit_t;

  vec3 initial_position;
  float last_sample_pdf;

  vec3 initial_normal;
  uint instance_index;

  vec2 barycentrics;
  uint primitive_id;
  uint material_index;

  vec3 environment_radiance;
  float environment_pdf;

  vec3 shadow_transmission;
  float shadow_previous_hit_t;

  uint shadow_is_inside;
};
#endif
