#extension GL_ARB_shader_draw_parameters : enable
#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"

layout (location = 0) in vec3 inPosition;

layout (location = 5) in ivec4 inBoneIds; 
layout (location = 6) in vec4 inWeights;
layout (location = 7) in ivec4 inBoneIds2; 
layout (location = 8) in vec4 inWeights2;

void main()
{
	mat4 boneTransform = EE_ANIM_BONES[inBoneIds[0]] * inWeights[0];
    if(inBoneIds[1] != -1){
		boneTransform += EE_ANIM_BONES[inBoneIds[1]] * inWeights[1];
	}
    if(inBoneIds[2] != -1){
		boneTransform += EE_ANIM_BONES[inBoneIds[2]] * inWeights[2];
	}
	if(inBoneIds[3] != -1){
		boneTransform += EE_ANIM_BONES[inBoneIds[3]] * inWeights[3];
	}
	if(inBoneIds2[0] != -1){
		boneTransform += EE_ANIM_BONES[inBoneIds2[0]] * inWeights2[0];
	}
    if(inBoneIds2[1] != -1){
		boneTransform += EE_ANIM_BONES[inBoneIds2[1]] * inWeights2[1];
	}
	if(inBoneIds2[2] != -1){
		boneTransform += EE_ANIM_BONES[inBoneIds2[2]] * inWeights2[2];
	}
	if(inBoneIds2[3] != -1){
		boneTransform += EE_ANIM_BONES[inBoneIds2[3]] * inWeights2[3];
	}
	const uint current_instance_index = gl_DrawID + EE_INSTANCE_INDEX;
    gl_Position = EE_SPOT_LIGHTS[EE_CAMERA_INDEX].light_space_matrix * EE_INSTANCES[current_instance_index].model * boneTransform * vec4(inPosition, 1.0);
}
