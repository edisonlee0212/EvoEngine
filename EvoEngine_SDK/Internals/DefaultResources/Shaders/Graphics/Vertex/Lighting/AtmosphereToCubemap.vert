
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inTangent;
layout(location = 3) in vec2 inTexCoord;
layout(location = 4) in vec2 inColor;

layout(location = 0) out vec3 outWorldPos;

struct Atmosphere {
  float earth_radius;       // In the paper this is usually Rg or Re (radius ground, eart)
  float atmosphere_radius;  // In the paper this is usually R or Ra (radius atmosphere)
  float hr;                 // Thickness of the atmosphere if density was uniform (Hr) for Rayleigh scattering
  float hm;                 // Same as above but for Mie scattering (Hm)

  float g;  // Mean cosine for Mie scattering
  int num_samples;
  int num_samples_light;
  float intensity;
};

layout(push_constant) uniform EE_PUSH_CONSTANTS {
  mat4 projection_view;
  Atmosphere atmosphere;

  vec3 sun_direction;
  float padding;
};

void main() {
  outWorldPos = inPosition;
  gl_Position = projection_view * vec4(inPosition, 1.0);
}