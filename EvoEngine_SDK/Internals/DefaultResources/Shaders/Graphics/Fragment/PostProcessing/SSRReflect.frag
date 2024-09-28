#extension GL_ARB_shading_language_include : enable

#include "SSRConstants.glsl"
#include "Basic.glsl"

precision highp float;
layout (location = 0) out vec3 outOriginalColor;
layout (location = 1) out vec4 outOriginalColorVisibility;

layout (location = 0) in VS_OUT {
	vec2 TexCoord;
} fs_in;


layout(set = 1, binding = 17) uniform sampler2D inDepth;
layout(set = 1, binding = 18) uniform sampler2D inNormal;
layout(set = 1, binding = 19) uniform sampler2D inMaterial;
layout(set = 1, binding = 20) uniform sampler2D inColor;

#define Scale vec3(.8, .8, .8)
#define K 19.19


vec3 BinarySearch(inout vec3 dir, inout vec3 hitCoord, inout float dDepth);
vec4 RayMarch(vec3 dir, inout vec3 hitCoord, out float dDepth);
vec3 hash(vec3 a);

vec3 EE_FUNC_FRESNEL_SCHLICK_ROUGHNESS(float cosTheta, vec3 F0, float roughness)
{
	return F0 + (max(vec3(1.0 - roughness), F0) - F0) * pow(max(1.0 - cosTheta, 0.0), 5.0);
}


bool rayIsOutofScreen(vec2 ray){
	return (ray.x > 1 || ray.y > 1 || ray.x < 0 || ray.y < 0) ? true : false;
}


vec3 BinarySearch(inout vec3 dir, inout vec3 hitCoord, inout float dDepth)
{
	float depth;
	vec4 projectedCoord;
	for(int i = 0; i < numBinarySearchSteps; i++)
	{
		float actualDepth = texture(inDepth, hitCoord.xy).x;
		dir *= 0.5;
		if(hitCoord.z > actualDepth) hitCoord += dir;
		else hitCoord -= dir;
	}
	return vec3(hitCoord.xy, depth);
}
void TraceRay(inout vec3 rayPos, vec3 dir){
	vec3 retVal = vec3(0, 0, 0);
	for(int i = 0; i < maxSteps; i++){
		rayPos += dir;
		if(rayIsOutofScreen(rayPos.xy)) return;
		float sampleDepth = texture(inDepth, rayPos.xy).r;
		float depthDif = rayPos.z - sampleDepth;
		if(depthDif >= 0){ //we have a hit
			retVal.z = 1;
			dir *= 0.5;
			rayPos -= dir;
			for(int j = 0; j < numBinarySearchSteps; j++){
				float sampleDepth = texture(inDepth, rayPos.xy).r;
				float depthDif = rayPos.z - sampleDepth;
				if(depthDif >= 0){
					rayPos -= dir;
				}else{
					rayPos += dir;
				}
				dir *= 0.5;
			}
			return;				
		}
	}
}
vec4 RayMarch(vec3 dir, inout vec3 hitCoord, out float dDepth)
{
	dir *= step;
	float depth;
	int steps;
	for(int i = 0; i < maxSteps; i++)
	{
		hitCoord += dir;
		float actualDepth = texture(inDepth, hitCoord.xy).x;
		if(hitCoord.z < actualDepth)
		{
			vec4 result;
			result = vec4(BinarySearch(dir, hitCoord, dDepth), 1.0);
			return result;
		}
	}
	return vec4(hitCoord.xy, depth, 0.0);
}
void main()
{
	vec2 texSize  = textureSize(inColor, 0).xy;
	vec2 tex_coord = fs_in.TexCoord;

	float metallic = texture(inMaterial, tex_coord).r;
	float roughness = texture(inMaterial, tex_coord).g;

	vec3 viewNormal = normalize((EE_CAMERAS[EE_CAMERA_INDEX].view * vec4(texture(inNormal, tex_coord).rgb, 0.0)).xyz);

	float ndcDepth = texture(inDepth, tex_coord).x;
	vec3 viewPos = EE_DEPTH_TO_VIEW_POS(EE_CAMERA_INDEX, tex_coord, ndcDepth);

	vec3 texturePixelPosition;
	texturePixelPosition.xy = tex_coord;
	texturePixelPosition.z = ndcDepth;

	vec3 viewReflection = normalize(reflect(viewPos, normalize(viewNormal)));
	
	vec3 viewRayEndPosition = viewPos + viewReflection * 100.0;
	vec4 textureRayEndPosition = EE_CAMERAS[EE_CAMERA_INDEX].projection * vec4(viewRayEndPosition, 1.0);

	textureRayEndPosition /= textureRayEndPosition.w;
	textureRayEndPosition.xyz = (textureRayEndPosition.xyz + vec3(1.0)) * 0.5;
	vec3 textureRayDir = textureRayEndPosition.xyz - texturePixelPosition;
	
	ivec2 screenSpaceStartPosition = ivec2(texturePixelPosition.x * texSize.x, texturePixelPosition.y * texSize.y); 
	ivec2 screenSpaceEndPosition = ivec2(textureRayEndPosition.x * texSize.x, textureRayEndPosition.y * texSize.y); 
	ivec2 screenSpaceDistance = screenSpaceEndPosition - screenSpaceStartPosition;
	int screenSpaceMaxDistance = max(abs(screenSpaceDistance.x), abs(screenSpaceDistance.y)) / 2;
	textureRayDir /= max(screenSpaceMaxDistance, 0.001f);

	float dDepth;
	vec3 jittering = vec3(0, 0, 0);//mix(vec3(0.0), vec3(hash(worldPos)), roughness);

	
	
	vec2 dCoord = smoothstep(0.2, 0.6, abs(vec2(0.5, 0.5) - tex_coord.xy));
	float screenEdgefactor = clamp(1.0 - (dCoord.x + dCoord.y), 0.0, 1.0);
	float reflectionMultiplier = pow(metallic, reflectionSpecularFalloffExponent) * screenEdgefactor * -textureRayDir.z;
	// Get color

	TraceRay(texturePixelPosition, textureRayDir);
	//vec4 coords = RayMarch(textureRayDir * step, texturePixelPosition, dDepth);

	outOriginalColorVisibility = clamp(vec4(texture(inColor, texturePixelPosition.xy).rgb, 1.0), vec4(0, 0, 0, 0), vec4(1, 1, 1, 1));
	//outOriginalColorVisibility = clamp(vec4(outColor, 1.0), vec4(0, 0, 0, 0), vec4(1, 1, 1, 1));
	outOriginalColor = texture(inColor, tex_coord).rgb;
}



vec3 hash(vec3 a)
{
	a = fract(a * Scale);
	a += dot(a, a.yxz + K);
	return fract((a.xxy + a.yxx)*a.zyx);
}