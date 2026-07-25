#extension GL_ARB_shader_draw_parameters : enable
#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"

layout (location = 0) in vec3 inPosition;

void main()
{
    const uint current_instance_index = gl_DrawID + EE_INSTANCE_INDEX;
    gl_Position = EE_SPOT_LIGHTS[EE_CAMERA_INDEX].light_space_matrix * EE_INSTANCES[current_instance_index].model * EE_INSTANCED_DATA[gl_InstanceIndex].instance_matrix * vec4(inPosition, 1.0);
}
