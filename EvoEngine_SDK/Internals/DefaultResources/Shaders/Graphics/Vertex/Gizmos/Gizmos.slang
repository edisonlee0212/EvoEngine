#extension GL_ARB_shading_language_include : enable

#include "GizmosConstants.slangh"
#include "Basic.slangh"
#define EE_PER_FRAME_SET 0
#define EE_PER_PASS_SET 1
#define EE_PER_GROUP_SET 2
#define EE_PER_COMMAND_SET 3

layout (location = 0) in vec3 inPosition;

void main()
{
	mat4 scaleMatrix = EE_GET_SCALE_MATRIX();
	gl_Position = EE_CAMERAS[EE_CAMERA_INDEX].projection_view * vec4(vec3(EE_MODEL_MATRIX * scaleMatrix * vec4(inPosition, 1.0)), 1.0);
}
