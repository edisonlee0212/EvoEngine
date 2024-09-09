
struct Environment {
  vec4 background_color;
  float gamma;
  float light_intensity;
  float padding1;
  float padding2;
};

layout(set = EE_PER_FRAME_SET, binding = EE_ENVIRONMENTAL_BLOCK_BINDING) uniform EE_ENVIRONMENTAL_BLOCK {
  Environment EE_ENVIRONMENT;
};