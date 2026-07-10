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
#define SMAA_RT_METRICS pc.metrics
#define SMAA_INCLUDE_VS 0
#define SMAA_INCLUDE_PS 1
#include "SMAA/SMAA.hlsl"

layout(set = 0, binding = 0) uniform sampler2D edgesTex;
layout(set = 0, binding = 1) uniform sampler2D areaTex;
layout(set = 0, binding = 2) uniform sampler2D searchTex;
layout(location = 0) in vec2 inTexcoord;
layout(location = 1) in vec2 inPixcoord;
layout(location = 2) in vec4 inOffset[3];
layout(location = 0) out vec4 outBlendWeights;

void main() {
  outBlendWeights = SMAABlendingWeightCalculationPS(inTexcoord, inPixcoord, inOffset, edgesTex, areaTex, searchTex,
                                                     pc.subsampleIndices);
}
