
layout (location = 0) in vec3 inPosition;

layout (location = 0) out vec3 outWorldPos;

layout(push_constant) uniform EE_PUSH_CONSTANTS{
	mat4 PROJECTION_VIEW;
	float PRESET_VALUE;
};

void main()
{
    outWorldPos = inPosition;
    gl_Position = PROJECTION_VIEW * vec4(inPosition, 1.0);
}
