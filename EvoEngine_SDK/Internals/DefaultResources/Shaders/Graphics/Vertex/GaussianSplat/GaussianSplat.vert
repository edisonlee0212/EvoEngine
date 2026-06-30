#extension GL_ARB_shading_language_include : enable

#define EE_GAUSSIAN_SPLAT_ENABLE_DRAW_COMMAND 0
#include "GaussianSplat.glsl"

layout(location = 0) out VS_OUT {
  vec3 Color;
  vec2 LocalCoord;
  flat float Opacity;
  flat float CutoffRadiusSquared;
} vs_out;

void main() {
  vec4 clip_position;
  EE_GAUSSIAN_SPLAT_BUILD_VERTEX(gl_InstanceIndex, EE_GAUSSIAN_SPLAT_TRIANGLE_CORNERS[gl_VertexIndex],
                                 clip_position, vs_out.Color, vs_out.LocalCoord, vs_out.Opacity,
                                 vs_out.CutoffRadiusSquared);
  gl_Position = clip_position;
}
