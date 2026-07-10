#extension GL_ARB_shader_draw_parameters : enable
#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"

layout(location = 0) in vec3 inPosition;
layout(location = 3) in vec2 inTexCoord;

struct PreviousInstance {
  mat4 previous_model;
  uvec4 flags;
};

layout(set = 2, binding = 1) readonly buffer EE_PREVIOUS_INSTANCE_BLOCK {
  PreviousInstance EE_PREVIOUS_INSTANCES[];
};

layout(location = 0) out vec2 outTexCoord;
layout(location = 1) out vec4 outCurrentClip;
layout(location = 2) out vec4 outPreviousClip;
layout(location = 3) out flat uint outInstanceIndex;
layout(location = 4) out flat uint outPreviousTransformValid;

void main() {
  uint instanceIndex = gl_DrawID + uint(EE_INSTANCE_INDEX);
  vec4 currentWorld = EE_INSTANCES[instanceIndex].model * vec4(inPosition, 1.0f);
  vec4 previousWorld = EE_PREVIOUS_INSTANCES[instanceIndex].previous_model * vec4(inPosition, 1.0f);
  outTexCoord = inTexCoord;
  outCurrentClip = EE_CAMERAS[EE_CAMERA_INDEX].unjittered_projection_view * currentWorld;
  outPreviousClip = EE_CAMERAS[EE_CAMERA_INDEX].previous_unjittered_projection_view * previousWorld;
  outInstanceIndex = instanceIndex;
  outPreviousTransformValid = EE_PREVIOUS_INSTANCES[instanceIndex].flags.x;
  gl_Position = EE_CAMERAS[EE_CAMERA_INDEX].projection_view * currentWorld;
}
