#extension GL_ARB_shading_language_include : enable

#include "PerFrame.glsl"

layout(location = 0) out vec4 outSrcColor;
layout(location = 1) out vec4 outResultColor;

layout(location = 0) in VS_OUT {
  vec2 TexCoord;
}
fs_in;

layout(set = 1, binding = 0) uniform sampler2D inColor;

void main() {
  vec2 texCoord = fs_in.TexCoord;
  outSrcColor = texture(inColor, texCoord);

  outResultColor = vec4(max(vec3(0.00001f), texture(inColor, texCoord).xyz - vec3(1.0f)), 1.0f);
}