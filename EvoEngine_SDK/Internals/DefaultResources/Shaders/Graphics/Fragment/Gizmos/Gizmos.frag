#extension GL_ARB_shading_language_include : enable

#include "Basic.glsl"
#include "GizmosConstants.glsl"

layout(location = 0) out vec4 FragColor;

void main() {
  FragColor = EE_GIZMO_COLOR;
}