#extension GL_ARB_shading_language_include : enable

layout(push_constant) uniform EE_SMAA_CONSTANTS {
  vec4 metrics;
  vec4 subsampleIndices;
  int toneMapped;
  int debugMode;
  int padding0;
  int padding1;
} pc;

#define SMAA_SLANGH_4
#define SMAA_RT_METRICS pc.metrics
#define SMAA_INCLUDE_VS 1
#define SMAA_INCLUDE_PS 0
#include "SMAA/SMAA.hlsl"

layout(location = 0) out vec2 outTexcoord;
layout(location = 1) out vec2 outPixcoord;
layout(location = 2) out vec4 outOffset[3];

void main() {
  vec2 uv = vec2(float((gl_VertexIndex << 1) & 2), float(gl_VertexIndex & 2));
  outTexcoord = uv;
  SMAABlendingWeightCalculationVS(uv, outPixcoord, outOffset);
  gl_Position = vec4(uv * 2.0f - 1.0f, 0.0f, 1.0f);
}
