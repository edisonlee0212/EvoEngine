
struct Environment {
  vec4 background_color;
  float gamma;
  float diffuse_sky_intensity;
  float global_reflection_intensity;
  float environment_type;
  float environment_pdf_texture_index;
  float environment_cubemap_index;
  float environment_rotation;
  float diffuse_fallback_intensity;
  float specular_fallback_intensity;
};

layout(set = EE_ENVIRONMENT_BLOCK_SET, binding = EE_ENVIRONMENT_BLOCK_BINDING) uniform EE_ENVIRONMENT_BLOCK {
  Environment EE_ENVIRONMENT;
};

vec3 EE_ENVIRONMENT_ROTATE_Y(const vec3 direction, const float angle) {
  const float c = cos(angle);
  const float s = sin(angle);
  return vec3(c * direction.x + s * direction.z, direction.y, -s * direction.x + c * direction.z);
}

vec3 EE_ENVIRONMENT_LOCAL_DIRECTION(const vec3 world_direction) {
  return EE_ENVIRONMENT_ROTATE_Y(world_direction, -EE_ENVIRONMENT.environment_rotation);
}

vec3 EE_ENVIRONMENT_WORLD_DIRECTION(const vec3 local_direction) {
  return EE_ENVIRONMENT_ROTATE_Y(local_direction, EE_ENVIRONMENT.environment_rotation);
}
