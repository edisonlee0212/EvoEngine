#extension GL_ARB_shading_language_include : enable

#include "SSRConstants.glsl"
#include "Basic.glsl"

precision highp float;
layout (location = 0) out vec4 outSrcColor;
layout (location = 1) out vec4 outDstColorVisibility;

layout (location = 0) in VS_OUT {
	vec2 TexCoord;
} fs_in;


layout(set = 1, binding = 17) uniform sampler2D inDepth;
layout(set = 1, binding = 18) uniform sampler2D inNormal;
layout(set = 1, binding = 19) uniform sampler2D inMaterial;
layout(set = 1, binding = 20) uniform sampler2D inColor;


vec3 EE_FUNC_FRESNEL_SCHLICK_ROUGHNESS(float cosTheta, vec3 F0, float roughness)
{
	return F0 + (max(vec3(1.0 - roughness), F0) - F0) * pow(max(1.0 - cosTheta, 0.0), 5.0);
}




vec3 GetNormal(in vec2 texCoord){
	return texture(inNormal, texCoord).rgb;
}

vec3 GetViewNormal(in vec2 texCoord){
	return normalize((EE_CAMERAS[EE_CAMERA_INDEX].view * vec4(GetNormal(texCoord), 0.0f)).xyz);
}

bool GetPosition(in vec2 texCoord, out vec3 position){
	float ndcDepth = texture(inDepth, texCoord).x;
	if(ndcDepth == 1.0f) return false;
	position = EE_DEPTH_TO_WORLD_POS(EE_CAMERA_INDEX, texCoord, ndcDepth);
	return true;
}

bool GetViewPosition(in vec2 texCoord, out vec3 position){
	float ndcDepth = texture(inDepth, texCoord).x;
	if(ndcDepth == 1.0f) return false;
	position = EE_DEPTH_TO_VIEW_POS(EE_CAMERA_INDEX, texCoord, ndcDepth);
	return true;
}

vec3 GetPosition(in vec2 texCoord){
	float ndcDepth = texture(inDepth, texCoord).x;
	return EE_DEPTH_TO_WORLD_POS(EE_CAMERA_INDEX, texCoord, ndcDepth);
}

vec3 GetViewPosition(in vec2 texCoord){
	float ndcDepth = texture(inDepth, texCoord).x;
	return EE_DEPTH_TO_VIEW_POS(EE_CAMERA_INDEX, texCoord, ndcDepth);
}

vec3 GetScreenPosition(in vec2 texCoord){
	float ndcDepth = texture(inDepth, texCoord).x;
	return EE_DEPTH_TO_CLIP_POS(texCoord, ndcDepth);
}

bool GetScreenPosition(in vec2 texCoord, out vec3 position){
	float ndcDepth = texture(inDepth, texCoord).x;
	if(ndcDepth == 1.0f) return false;
	position = EE_DEPTH_TO_CLIP_POS(texCoord, ndcDepth);
	return true;
}

void main()
{
	vec2 texSize  = textureSize(inColor, 0).xy;
	vec2 texCoord = fs_in.TexCoord;
	outSrcColor = texture(inColor, texCoord);

	int steps = initial_steps;
	vec2 uv = vec2(0.0);
	vec3 positionFrom;
	
	if(!GetViewPosition(texCoord, positionFrom)){
		outDstColorVisibility = vec4(0, 0, 0, 0);
		return;
	}
	vec3 positionTo = positionFrom;

	vec3 unitPositionFrom = normalize(positionFrom.xyz);
	vec3 normal           = GetViewNormal(texCoord);
	vec3 pivot            = normalize(reflect(unitPositionFrom, normal));
	
	vec4 startView = vec4(positionFrom.xyz + (pivot *           0), 1);
	vec4 endView   = vec4(positionFrom.xyz + (pivot * maxDistance), 1);

	vec4 startFragTemp = EE_CAMERAS[EE_CAMERA_INDEX].projection * startView;
			startFragTemp.xyz /= startFragTemp.w;
			startFragTemp.xy   = startFragTemp.xy * 0.5f + 0.5f;

	vec4 endFragTemp = EE_CAMERAS[EE_CAMERA_INDEX].projection * endView;
			endFragTemp.xyz /= endFragTemp.w;
			endFragTemp.xy   = endFragTemp.xy * 0.5f + 0.5f;

	vec2 startFrag = startFragTemp.xy * texSize;
	vec2 endFrag = endFragTemp.xy * texSize;

	
	vec2 frag  = startFrag;
	uv = frag / texSize;
	outSrcColor = vec4(endFrag / texSize, 0, 1);
	float deltaX    = endFrag.x - startFrag.x;
	float deltaY    = endFrag.y - startFrag.y;
	float useX      = abs(deltaX) >= abs(deltaY) ? 1.0 : 0.0;
	float delta     = mix(abs(deltaY), abs(deltaX), useX) * clamp(resolution, 0.0, 1.0);
	vec2  increment = vec2(deltaX, deltaY) / max(delta, 0.001);

	float search0 = 0;
	float search1 = 0;

	int hit0 = 0;
	int hit1 = 0;

	float viewDistance = startView.y;
	float depth        = thickness;

	float i = 0;

	for (i = 0; i < int(delta); ++i) {
		frag      += increment;
		uv      = frag / texSize;
		bool valid = GetViewPosition(uv, positionTo);

		search1 =
			mix((frag.y - startFrag.y) / deltaY, 
				(frag.x - startFrag.x) / deltaX, useX);

		search1 = clamp(search1, 0.0, 1.0);

		viewDistance = (startView.y * endView.y) / mix(endView.y, startView.y, search1);
		depth        = viewDistance - positionTo.y;

		if (valid && depth > 0 && depth < thickness) {
			hit0 = 1;
			break;
		} else {
			search0 = search1;
		}
	}

	search1 = search0 + ((search1 - search0) / 2.0);

	steps *= hit0;

	for (i = 0; i < steps; ++i) {
		frag       = mix(startFrag.xy, endFrag.xy, search1);
		uv.xy      = frag / texSize;
		bool valid = GetViewPosition(uv, positionTo);

		viewDistance = (startView.y * endView.y) / mix(endView.y, startView.y, search1);
		depth        = viewDistance - positionTo.y;

		if (valid && depth > 0 && depth < thickness) {
			hit1 = 1;
			search1 = search0 + ((search1 - search0) / 2);
		} else {
			float temp = search1;
			search1 = search1 + ((search1 - search0) / 2);
			search0 = temp;
		}
	}

	float visibility =
		hit1
		* (1
		- max
			(dot(-unitPositionFrom, pivot), 0)
		)
		* (1
		- clamp
			(depth / thickness, 0, 1)
		)
		* (1
		- clamp
			(length(positionTo - positionFrom) / maxDistance, 0, 1)
		)
		* (uv.x < 0 || uv.x > 1 ? 0 : 1)
		* (uv.y < 0 || uv.y > 1 ? 0 : 1);

	visibility = clamp(visibility, 0, 1);

	if(visibility != 0){
		outDstColorVisibility = clamp(vec4(texture(inColor, uv).rgb, visibility), vec4(0, 0, 0, 0), vec4(1, 1, 1, 1));
	}else{
		outDstColorVisibility = vec4(0, 0, 0, 0);
	}
	
	
}