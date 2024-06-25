#extension GL_ARB_shader_draw_parameters : enable
layout (location = 0) in vec3 inPosition;
layout (location = 1) in float inThickness;
layout (location = 2) in vec3 inNormal;
layout (location = 3) in float inTexCoord;
layout (location = 4) in vec4 inColor;

layout (location = 0) out VS_OUT {
    vec3 FragPos;
	float Thickness;
	vec3 Normal;
} vs_out;

void main()
{
	uint currentInstanceIndex = gl_DrawID + EE_INSTANCE_INDEX;
	vs_out.FragPos = vec3(EE_INSTANCES[currentInstanceIndex].model * vec4(inPosition, 1.0));
	vs_out.Thickness = inThickness;
	vs_out.Normal = vec3(EE_POINT_LIGHTS[EE_CAMERA_INDEX].light_space_matrix[EE_LIGHT_SPLIT_INDEX] * vec4(inNormal, 0.0));
}