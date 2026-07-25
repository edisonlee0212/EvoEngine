#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_ray_tracing : require

#include "PointCloudRayTracingPayload.glsl"

layout(location = 0) rayPayloadInEXT PointCloudRayTracingPayload hit_value;

void main() 
{
	hit_value.hit_info.color = vec4(0.0f, 0.0f, 0.0f, 1.0f);
	hit_value.handle = 0;
	hit_value.hit_info.position = vec3(0, 0, 0);
	hit_value.hit_info.normal = vec3(0, 0, 0);
	hit_value.hit_info.tangent = vec3(0, 0, 0);
	hit_value.hit_info.tex_coord = vec2(0, 0);

	hit_value.hit_info.vertex_info1 = 0;
    hit_value.hit_info.vertex_info2 = 0;
    hit_value.hit_info.vertex_info3 = 0;
    hit_value.hit_info.vertex_info4 = vec2(0, 0);
}
