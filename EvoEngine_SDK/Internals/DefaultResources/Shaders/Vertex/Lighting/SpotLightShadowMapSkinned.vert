#extension GL_ARB_shader_draw_parameters : enable
layout (location = 0) in vec3 inPosition;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec3 inTangent;
layout (location = 3) in vec2 inTexCoord;
layout (location = 4) in vec2 inColor;

layout (location = 5) in ivec4 inBoneIds; 
layout (location = 6) in vec4 inWeights;
layout (location = 7) in ivec4 inBoneIds2; 
layout (location = 8) in vec4 inWeights2;
layout(location = 0) out VS_OUT {
	vec2 TexCoord;
} vs_out;

layout(location = 5) out flat uint currentInstanceIndex;

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
	currentInstanceIndex = gl_DrawID + EE_INSTANCE_INDEX;
    vs_out.TexCoord = inTexCoord;
    gl_Position = EE_SPOT_LIGHTS[EE_CAMERA_INDEX].light_space_matrix * EE_INSTANCES[currentInstanceIndex].model * boneTransform * vec4(inPosition, 1.0);
}