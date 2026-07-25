#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_ray_tracing : require

#define EE_GLTF_USE_EXPLICIT_TEXTURE_LOD
#define EE_GLTF_TEXTURE_LOD 0.0
#include "RayTracingBasic.glsl"
#include "GltfRasterMaterial.glsl"
#include "PointCloudRayTracingPayload.glsl"
layout(location = 0) rayPayloadInEXT PointCloudRayTracingPayload hit_value;

hitAttributeEXT vec2 attribs;

void main() 
{
	const int instance_index = int(gl_InstanceCustomIndexEXT);
	Instance instance = EE_INSTANCES[instance_index];
	const uint material_index = uint(instance.material_index);
	const int triangle_offset = instance.triangle_offset + gl_PrimitiveID;

	// Vertex of the triangle
	const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
	const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
	const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];

	const vec3 barycentrics = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

	const vec3 position      = v0.position * barycentrics.x + v1.position * barycentrics.y + v2.position * barycentrics.z;
	const vec2 tex_coord      = v0.tex_coord * barycentrics.x + v1.tex_coord * barycentrics.y + v2.tex_coord * barycentrics.z;
	vec3 normal      = v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z;
	const vec3 tangent      = v0.tangent * barycentrics.x + v1.tangent * barycentrics.y + v2.tangent * barycentrics.z;
	const GltfRasterMaterial surface = EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, tex_coord, tex_coord);
	normal = EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coord, tex_coord, normal, tangent);
	
	const vec3 worldPosition = vec3(gl_ObjectToWorldEXT * vec4(position, 1.0));  // Transforming the position to world space
	const vec3 worldNormal = normalize(vec3(normal * gl_WorldToObjectEXT));  // Transforming the normal to world space
	const vec3 worldTangent = normalize(vec3(tangent * gl_WorldToObjectEXT));  // Transforming the normal to world space
	
	vec4 albedo = surface.base_color;

	hit_value.hit_count += 1;
	hit_value.hit_info.color = vec4(surface.emissive * albedo.xyz, 1.0f);

	hit_value.hit_info.position = worldPosition;
	hit_value.hit_info.normal = worldNormal;
	hit_value.hit_info.tangent = worldTangent;
	hit_value.hit_info.tex_coord = tex_coord;
	hit_value.handle = instance.renderer_handle;

	// Interpolate vertex info
	if (barycentrics.x > barycentrics.z && barycentrics.x > barycentrics.y) {
      hit_value.hit_info.vertex_info1 = v1.vertex_info1;
      hit_value.hit_info.vertex_info2 = v1.vertex_info2;
      hit_value.hit_info.vertex_info3 = v1.vertex_info3;
      hit_value.hit_info.vertex_info4 = v1.vertex_info4;
    } else if (barycentrics.y > barycentrics.z) {
      hit_value.hit_info.vertex_info1 = v2.vertex_info1;
      hit_value.hit_info.vertex_info2 = v2.vertex_info2;
      hit_value.hit_info.vertex_info3 = v2.vertex_info3;
      hit_value.hit_info.vertex_info4 = v2.vertex_info4;
    } else {
      hit_value.hit_info.vertex_info1 = v0.vertex_info1;
      hit_value.hit_info.vertex_info2 = v0.vertex_info2;
      hit_value.hit_info.vertex_info3 = v0.vertex_info3;
      hit_value.hit_info.vertex_info4 = v0.vertex_info4;
    }

}
