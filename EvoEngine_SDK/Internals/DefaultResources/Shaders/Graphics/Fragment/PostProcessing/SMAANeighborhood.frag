#extension GL_ARB_shading_language_include : enable

layout(push_constant) uniform EE_SMAA_CONSTANTS {
  vec4 metrics;
  vec4 subsampleIndices;
  int toneMapped;
  int debugMode;
  int padding0;
  int padding1;
} pc;

#define SMAA_GLSL_4
#define SMAA_PRESET_ULTRA
#define SMAA_RT_METRICS pc.metrics
#define SMAA_INCLUDE_VS 0
#define SMAA_INCLUDE_PS 1
#include "SMAA/SMAA.hlsl"

layout(set = 0, binding = 0) uniform sampler2D colorTex;
layout(set = 0, binding = 1) uniform sampler2D blendTex;
layout(set = 0, binding = 2) uniform sampler2D edgesTex;
layout(location = 0) in vec2 inTexcoord;
layout(location = 1) in vec4 inOffset;
layout(location = 0) out vec4 outColor;

vec3 LinearToSrgb(vec3 color) {
  vec3 low = color * 12.92f;
  vec3 high = 1.055f * pow(max(color, vec3(0.0f)), vec3(1.0f / 2.4f)) - 0.055f;
  return mix(low, high, greaterThan(color, vec3(0.0031308f)));
}

void main() {
  if (pc.debugMode == 1) {
    vec2 edges = texture(edgesTex, inTexcoord).rg;
    outColor = vec4(edges.r, edges.g, 0.0f, 1.0f);
    return;
  }
  if (pc.debugMode == 2) {
    outColor = vec4(texture(blendTex, inTexcoord).rgb, 1.0f);
    return;
  }
  outColor = SMAANeighborhoodBlendingPS(inTexcoord, inOffset, colorTex, blendTex);
  if (pc.toneMapped != 0) {
    outColor.rgb = LinearToSrgb(max(outColor.rgb, vec3(0.0f)));
  }
}
