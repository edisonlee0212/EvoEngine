#define EE_PER_FRAME_SET 0
#define EE_PER_PASS_SET 1
#define EE_PER_GROUP_SET 2
#define EE_PER_COMMAND_SET 3

layout (location = 0) in vec3 inPosition;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec3 inTangent;
layout (location = 3) in vec2 inTexCoord;
layout (location = 4) in vec4 inColor;

void main()
{
	mat4 scaleMatrix = EE_GET_SCALE_MATRIX();
	gl_Position = EE_CAMERAS[EE_CAMERA_INDEX].EE_CAMERA_PROJECTION_VIEW * vec4(vec3(EE_MODEL_MATRIX * scaleMatrix * vec4(inPosition, 1.0)), 1.0);
}