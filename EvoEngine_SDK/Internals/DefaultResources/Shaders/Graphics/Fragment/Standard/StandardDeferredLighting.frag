#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"
#include "Lighting.glsl"

precision highp float;

layout (location = 0) in VS_OUT {
	vec2 TexCoord;
} fs_in;

layout(set = EE_PER_PASS_SET, binding = 17) uniform sampler2D inDepth;
layout(set = EE_PER_PASS_SET, binding = 20) uniform sampler2D inBaseColorAO;
layout(set = EE_PER_PASS_SET, binding = 21) uniform sampler2D inNormalRoughness;
layout(set = EE_PER_PASS_SET, binding = 22) uniform sampler2D inPbrFlags;
layout(set = EE_PER_PASS_SET, binding = 23) uniform sampler2D inEmissive;
layout(set = EE_RASTER_FIXED_LIGHTING_TEXTURE_SET, binding = 4) uniform sampler2D inAmbientOcclusion;

layout (location = 0) out vec4 FragColor;

void main()
{
	const int indirectLightingDebugView = EE_INDIRECT_LIGHTING_DEBUG_VIEW();
	float ndcDepth = 	texture(inDepth, fs_in.TexCoord).x;
	vec3 fragPos = EE_DEPTH_TO_WORLD_POS(EE_CAMERA_INDEX, fs_in.TexCoord, ndcDepth);
	vec3 cameraPosition = EE_CAMERA_POSITION(EE_CAMERA_INDEX);
	vec3 skyColor = EE_SKY_COLOR(fragPos - cameraPosition);
	if(ndcDepth == 1.0f) {
		FragColor = vec4(indirectLightingDebugView == 0 ? skyColor : vec3(0.0f), 1.0f);
		return;
	}

	vec4 baseColorAO = texture(inBaseColorAO, fs_in.TexCoord);
	vec4 normalRoughness = texture(inNormalRoughness, fs_in.TexCoord);
	vec4 pbrFlags = texture(inPbrFlags, fs_in.TexCoord);
	vec4 emissiveSample = texture(inEmissive, fs_in.TexCoord);
	vec3 emissive = emissiveSample.rgb;
	if (emissiveSample.a < 0.0f) {
		FragColor = vec4(indirectLightingDebugView == 0 ? baseColorAO.rgb : vec3(0.0f), 1.0f);
		return;
	}

	vec3 normal = normalize(normalRoughness.xyz);
	float depth = EE_LINEARIZE_DEPTH(EE_CAMERA_INDEX, ndcDepth);
	if (indirectLightingDebugView == 0) {
		vec4 shadowDebugColor = EE_FUNC_DIRECTIONAL_SHADOW_DEBUG(depth, fragPos);
		if (shadowDebugColor.a > 0.0f) {
			FragColor = shadowDebugColor;
			return;
		}
	}

	float roughness = normalRoughness.a;
	float metallic = pbrFlags.x;
	float materialOcclusion = baseColorAO.a;
	float screenSpaceVisibility = texture(inAmbientOcclusion, fs_in.TexCoord).r;
	vec4 albedo = vec4(baseColorAO.rgb, 1.0);

	vec3 viewDir = normalize(cameraPosition - fragPos);
	bool receiveShadow = true;
	vec3 F0 = pbrFlags.yzw;
	float F90 = emissiveSample.a;
	vec3 result = EE_FUNC_CALCULATE_LIGHTS(receiveShadow, albedo.xyz, 1.0, depth, normal, viewDir, fragPos, metallic, roughness, F0, F90);
	vec3 ambient = EE_FUNC_CALCULATE_DDGI_ENVIRONMENTAL_LIGHT(
		albedo.xyz, normal, viewDir, fragPos, metallic, roughness, F0, F90, materialOcclusion,
		screenSpaceVisibility);
	if (indirectLightingDebugView != 0) {
		FragColor = vec4(ambient, 1.0f);
		return;
	}
	vec3 outputColor = result + emissive + ambient;

	float fade_ratio = EE_CAMERA_FADE_RATIO(EE_CAMERA_INDEX);
	if(depth > EE_CAMERA_FAR(EE_CAMERA_INDEX) * fade_ratio){
		outputColor.xyz = mix(outputColor.xyz, skyColor, (depth - EE_CAMERA_FAR(EE_CAMERA_INDEX) * fade_ratio) / (EE_CAMERA_FAR(EE_CAMERA_INDEX) * (1.f - fade_ratio)));
	}
	FragColor = vec4(outputColor, 1.0f);
}
