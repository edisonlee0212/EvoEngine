#extension GL_ARB_shader_draw_parameters : enable
#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.slangh"
#include "Basic.slangh"


layout (location = 0) in vec3 inPosition;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec3 inTangent;
layout (location = 3) in vec2 inTexCoord;
layout (location = 4) in vec4 inColor;
layout (location = 9) in float inTangentHandedness;
layout (location = 10) in vec2 inTexCoord1;
layout (location = 11) in vec2 inTexCoord2;
layout (location = 12) in vec2 inTexCoord3;

layout(location = 0) out VS_OUT {
	vec3 FragPos;
	vec3 Normal;
	vec3 Tangent;
	flat float TangentHandedness;
	vec4 TexCoord01;
	vec4 TexCoord23;
	vec4 Color;
} vs_out;

layout(location = 7) out flat uint currentInstanceIndex;
layout(location = 8) out flat float transformHandedness;

void main()
{
	currentInstanceIndex = gl_DrawID + EE_INSTANCE_INDEX;
	mat4 model = EE_INSTANCES[currentInstanceIndex].model;
	vs_out.FragPos = vec3(model * vec4(inPosition, 1.0));
	vec3 N = normalize(transpose(inverse(mat3(model))) * inNormal);
	vec3 T = mat3(model) * inTangent;
	vs_out.Normal = N;
	vs_out.Tangent = T;
	transformHandedness = EE_TRANSFORM_HANDEDNESS(model);
	vs_out.TangentHandedness = (inTangentHandedness < 0.0 ? -1.0 : 1.0) * transformHandedness;
	vs_out.TexCoord01 = vec4(inTexCoord, inTexCoord1);
	vs_out.TexCoord23 = vec4(inTexCoord2, inTexCoord3);
	vs_out.Color = inColor;
	gl_Position = EE_CAMERAS[EE_CAMERA_INDEX].projection_view * vec4(vs_out.FragPos, 1.0);
}
