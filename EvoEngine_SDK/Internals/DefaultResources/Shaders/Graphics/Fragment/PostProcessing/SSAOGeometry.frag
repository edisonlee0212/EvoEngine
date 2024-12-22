#extension GL_ARB_shading_language_include : enable

#include "PerFrame.glsl"
#include "VogelDisk.glsl"

layout (location = 0) out vec4 outSrcColor;
layout (location = 1) out vec4 proximity;

layout (location = 0) in VS_OUT {
	vec2 TexCoord;
} fs_in;

layout(set = 1, binding = 17) uniform sampler2D inDepth;
layout(set = 1, binding = 18) uniform sampler2D inNormal;
layout(set = 1, binding = 19) uniform sampler2D inMaterial;
layout(set = 2, binding = 0) uniform sampler2D inColor;

layout(push_constant) uniform EE_SSAO_CONSTANTS{
	int EE_CAMERA_INDEX;
	int kernelSize;
	float radius;
	float bias;
	float factor;
	float intensity;
};

vec3 GetNormal(in vec2 texCoord){
	return texture(inNormal, texCoord).rgb;
}

vec3 GetViewNormal(in vec2 texCoord){
	return normalize((EE_CAMERAS[EE_CAMERA_INDEX].view * vec4(GetNormal(texCoord), 0.0f)).xyz);
}

bool GetViewPosition(in vec2 texCoord, out vec3 position){
	float ndcDepth = texture(inDepth, texCoord).x;
	if(ndcDepth == 1.0f) return false;
	position = EE_DEPTH_TO_VIEW_POS(EE_CAMERA_INDEX, texCoord, ndcDepth);
	return true;
}

void main()
{
	vec2 texSize  = textureSize(inColor, 0).xy;
	vec2 texCoord = fs_in.TexCoord;
	outSrcColor = texture(inColor, texCoord);
	vec2 uv = vec2(0.0f);
	vec3 viewPos;
	
	if(!GetViewPosition(texCoord, viewPos)){
		proximity = vec4(1, 1, 1, 1);
		return;
	}
	vec3 normal           = GetViewNormal(texCoord);

	vec3 randomVec = EE_UNIFORM_KERNEL[int(EE_INTERLEAVED_GRADIENT_NOISE(viewPos * 3141) * MAX_KERNEL_AMOUNT) % MAX_KERNEL_AMOUNT].xyz;
	
	mat4 projection = EE_CAMERAS[EE_CAMERA_INDEX].projection;

	// create TBN change-of-basis matrix: from tangent-space to view-space
	vec3 tangent = normalize(randomVec - normal * dot(randomVec, normal));
	vec3 bitangent = cross(normal, tangent);
	mat3 TBN = mat3(tangent, bitangent, normal);
	// iterate over the sample kernel and calculate occlusion factor
	float occlusion = 0.0f;
	for(int i = 0; i < kernelSize; ++i)
	{
		vec3 point = EE_UNIFORM_KERNEL[i].xyz;
		point.z = abs(point.z);
		// get sample position
		vec3 samplePos = TBN * point; // from tangent to view-space
		samplePos = viewPos + samplePos * radius;
		// project sample position (to sample texture) (to get position on screen/texture)
		vec4 offset = vec4(samplePos, 1.0f);
		offset = projection * offset; // from view to clip-space
		offset.xyz /= offset.w; // perspective divide
		offset.xyz = offset.xyz * 0.5f + 0.5f; // transform to range 0.0 - 1.0
		// get sample depth
		vec3 sampleViewPos = EE_DEPTH_TO_VIEW_POS(EE_CAMERA_INDEX, offset.xy, texture(inDepth, offset.xy).r);
		float sampleDepth = sampleViewPos.z;
		// range check & accumulate
		float rangeCheck = smoothstep(0.0f, 1.0f, pow((radius - distance(viewPos, sampleViewPos)) / radius, factor));
		occlusion += (sampleDepth >= samplePos.z + bias ? 1.0f : 0.0f) * rangeCheck;           
	}
	proximity = vec4(max(0.0f, 1.0f - occlusion / kernelSize * intensity));
}