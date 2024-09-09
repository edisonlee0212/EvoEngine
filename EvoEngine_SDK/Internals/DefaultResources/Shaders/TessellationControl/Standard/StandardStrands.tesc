#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"

layout (vertices = 4) out;

layout (location = 0) in VS_OUT {
    vec3 FragPos;
	float Thickness;
	vec3 Normal;
	float TexCoord;
} vs_in[];

layout (location = 0) out TCS_OUT {
	vec3 FragPos;
	float Thickness;
	vec3 Normal;
	float TexCoord;
} tcs_out[];

layout(location = 5) in uint currentInstanceIndexIn[];
layout(location = 5) patch out uint currentInstanceIndexOut;

void main(){
	currentInstanceIndexOut = currentInstanceIndexIn[gl_InvocationID];
	if(gl_InvocationID == 0){
		gl_TessLevelOuter[0] = 1;
		gl_TessLevelOuter[1] = EE_STRANDS_SEGMENT_SUBDIVISION(vs_in[0].FragPos, vs_in[3].FragPos);
	}

	tcs_out[gl_InvocationID].FragPos = vs_in[gl_InvocationID].FragPos;
	tcs_out[gl_InvocationID].Thickness = vs_in[gl_InvocationID].Thickness;
	tcs_out[gl_InvocationID].Normal = vs_in[gl_InvocationID].Normal;
	tcs_out[gl_InvocationID].TexCoord = vs_in[gl_InvocationID].TexCoord;
}