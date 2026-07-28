#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.slangh"
#include "Basic.slangh"
#include "Lighting.slangh"

precision highp float;

layout(location = 0) in VS_OUT {
  vec2 TexCoord;
}
fs_in;

layout(set = EE_PER_PASS_SET, binding = 17) uniform sampler2D inDepth;
layout(set = EE_PER_PASS_SET, binding = 20) uniform sampler2D inBaseColorAO;
layout(set = EE_PER_PASS_SET, binding = 21) uniform sampler2D inNormalRoughness;

layout(location = 0) out vec4 FragColor;

void main() {
  const float ndc_depth = texture(inDepth, fs_in.TexCoord).x;
  if (ndc_depth == 1.0f) {
    FragColor = vec4(0.0f);
    return;
  }
  const vec3 frag_pos = EE_DEPTH_TO_WORLD_POS(EE_CAMERA_INDEX, fs_in.TexCoord, ndc_depth);
  const vec3 camera_position = EE_CAMERA_POSITION(EE_CAMERA_INDEX);
  const vec3 albedo = texture(inBaseColorAO, fs_in.TexCoord).rgb;
  const vec3 normal = normalize(texture(inNormalRoughness, fs_in.TexCoord).xyz);
  const vec3 view_direction = normalize(camera_position - frag_pos);
  FragColor = vec4(EE_FUNC_CALCULATE_DDGI_DIFFUSE(albedo, normal, view_direction, frag_pos), 1.0f);
}
