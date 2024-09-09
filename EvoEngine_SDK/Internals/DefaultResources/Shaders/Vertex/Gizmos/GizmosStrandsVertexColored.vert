#extension GL_ARB_shading_language_include : enable

#include "GizmosConstants.glsl"
#include "Basic.glsl"

#define EE_PER_FRAME_SET 0
#define EE_PER_PASS_SET 1
#define EE_PER_GROUP_SET 2
#define EE_PER_COMMAND_SET 3

layout (location = 0) in vec3 inPosition;
layout (location = 1) in float inThickness;
layout (location = 2) in vec3 inNormal;
layout (location = 3) in float inTexCoord;
layout (location = 4) in vec4 inColor;

layout(location = 0) out VS_OUT {
	vec3 FragPos;
	float Thickness;
	vec3 Normal;
	vec4 Color;
} vs_out;

void main()
{
	mat4 scaleMatrix = EE_GET_SCALE_MATRIX();
	mat4 matrix = EE_MODEL_MATRIX * scaleMatrix;
	vs_out.FragPos = vec3(matrix * vec4(inPosition, 1.0));
	vs_out.Thickness = inThickness;
	vs_out.Normal = vec3(matrix * vec4(inNormal, 0.0));
	vs_out.Color = inColor;
}