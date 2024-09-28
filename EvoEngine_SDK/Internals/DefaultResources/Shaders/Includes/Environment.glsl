
struct Environment {
  vec4 background_color;
  float gamma;
  float light_intensity;
  float padding1;
  float padding2;
};

layout(set = EE_ENVIRONMENT_BLOCK_SET, binding = EE_ENVIRONMENT_BLOCK_BINDING) uniform EE_ENVIRONMENT_BLOCK {
  Environment EE_ENVIRONMENT;
};