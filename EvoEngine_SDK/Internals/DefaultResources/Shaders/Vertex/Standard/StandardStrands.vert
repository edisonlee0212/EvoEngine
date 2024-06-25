#extension GL_ARB_shader_draw_parameters : enable
layout (location = 0) in vec3 inPosition;
layout (location = 1) in float inThickness;
layout (location = 2) in vec3 inNormal;
layout (location = 3) in float inTexCoord;
layout (location = 4) in vec4 inColor;


layout(location = 0) out VS_OUT {
	vec3 FragPos;
	float Thickness;
	vec3 Normal;
	float TexCoord;
} vs_out;

layout(location = 5) out flat uint currentInstanceIndex;

void main()
{
	currentInstanceIndex = gl_DrawID + EE_INSTANCE_INDEX;
	vs_out.FragPos = vec3(EE_INSTANCES[currentInstanceIndex].model * vec4(inPosition, 1.0));
	vs_out.Thickness = inThickness;
	vs_out.TexCoord = inTexCoord;
	vs_out.Normal = vec3(EE_CAMERAS[EE_CAMERA_INDEX].projection_view * vec4(inNormal, 0.0));
}