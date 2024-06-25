#extension GL_ARB_shader_draw_parameters : enable
layout (location = 0) in vec3 inPosition;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec3 inTangent;
layout (location = 3) in vec2 inTexCoord;
layout (location = 4) in vec2 inColor;
layout(location = 0) out VS_OUT {
	vec2 TexCoord;
} vs_out;

layout(location = 5) out flat uint currentInstanceIndex;

void main()
{
    currentInstanceIndex = gl_DrawID + EE_INSTANCE_INDEX;
    vs_out.TexCoord = inTexCoord;
    gl_Position = EE_SPOT_LIGHTS[EE_CAMERA_INDEX].light_space_matrix * EE_INSTANCES[currentInstanceIndex].model * EE_INSTANCED_DATA[gl_InstanceIndex].instance_matrix * vec4(inPosition, 1.0);
}