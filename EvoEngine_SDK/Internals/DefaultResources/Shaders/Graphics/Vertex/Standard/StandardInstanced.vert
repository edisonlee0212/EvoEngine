#extension GL_ARB_shader_draw_parameters : enable

#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"

layout (location = 0) in vec3 inPosition;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec3 inTangent;
layout (location = 3) in vec2 inTexCoord;
layout (location = 4) in vec4 inColor;
layout (location = 9) in float inTangentHandedness;
layout (location = 10) in vec2 inTexCoord1;

layout(location = 0) out VS_OUT {
	vec3 FragPos;
	vec3 Normal;
	vec3 Tangent;
	flat float TangentHandedness;
	vec2 TexCoord;
	vec2 TexCoord1;
	vec4 Color;
} vs_out;

layout(location = 7) out flat uint currentInstanceIndex;
layout(location = 8) out flat float transformHandedness;

void main()
{
	currentInstanceIndex = gl_DrawID + EE_INSTANCE_INDEX;
	mat4 matrix = EE_INSTANCES[currentInstanceIndex].model * EE_INSTANCED_DATA[gl_InstanceIndex].instance_matrix;
	vs_out.FragPos = vec3(matrix * vec4(inPosition, 1.0));
	vec3 N = normalize(transpose(inverse(mat3(matrix))) * inNormal);
	vec3 T = mat3(matrix) * inTangent;
	vs_out.Normal = N;
	vs_out.Tangent = T;
	transformHandedness = EE_TRANSFORM_HANDEDNESS(matrix);
	vs_out.TangentHandedness = (inTangentHandedness < 0.0 ? -1.0 : 1.0) * transformHandedness;
	vs_out.TexCoord = inTexCoord;
	vs_out.TexCoord1 = inTexCoord1;
	vs_out.Color = inColor;
	gl_Position = EE_CAMERAS[EE_CAMERA_INDEX].projection_view * vec4(vs_out.FragPos, 1.0);
}
