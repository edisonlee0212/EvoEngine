precision highp float;
layout (location = 0) out vec4 FragColor;
layout (location = 0) in vec3 inWorldPos;

layout(set = 0, binding = 0) uniform sampler2D equirectangularMap;

layout(push_constant) uniform EE_PUSH_CONSTANTS{
	mat4 PROJECTION_VIEW;
	float ENVIRONMENT_PDF_SCALE;
};

const float PI = 3.14159265359;
const vec2 invAtan = vec2(0.1591, 0.3183);
vec2 SampleSphericalMap(vec3 v)
{
	vec2 uv = vec2(atan(v.z, v.x), asin(v.y));
	uv *= invAtan;
	uv += 0.5;
	return uv;
}

void main()
{		
	vec2 uv = SampleSphericalMap(normalize(inWorldPos));
	vec3 color = texture(equirectangularMap, uv).rgb;
	float luminance = dot(max(color, vec3(0.0)), vec3(0.2126, 0.7152, 0.0722));
	float pdf = ENVIRONMENT_PDF_SCALE > 0.0 ? luminance * ENVIRONMENT_PDF_SCALE : 1.0 / (4.0 * PI);
	
	FragColor = vec4(color, pdf);
}
