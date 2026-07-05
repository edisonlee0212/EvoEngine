#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_ray_tracing : require

#define EE_GLTF_USE_EXPLICIT_TEXTURE_LOD
#define EE_GLTF_TEXTURE_LOD 0.0
#include "RayTracingBasic.glsl"
#include "GltfRasterMaterial.glsl"
#include "PointCloudRayTracingPayload.glsl"
#include "Random.glsl"
layout(location = 0) rayPayloadInEXT PointCloudRayTracingPayload hit_value;

hitAttributeEXT vec2 attribs;

vec3 Reflect(in vec3 incident, in vec3 normal) {
  return incident - 2.0f * dot(incident, normal) * normal;
}

layout(push_constant) uniform EE_POINT_CLOUD_CONSTANTS {
  uint bounce;
  uint envIndex;
  uint skybox_tex_index;
  uint use_clear_color;
  vec4 clear_color;
};

vec3 EE_SKY_COLOR(vec3 direction) {
	return use_clear_color == 1 ?
		clear_color.xyz * clear_color.w
		: pow(texture(EE_CUBEMAPS[skybox_tex_index], normalize(direction)).rgb, vec3(1.0f / EE_ENVIRONMENT.gamma)) * clear_color.w;
}

mat3 GetTangentSpace(in vec3 normal) {
  // Choose a helper vector for the cross product
  vec3 helper = vec3(1.0f, 0.0f, 0.0f);
  if (abs(normal.x) > 0.99f)
    helper = vec3(0.0f, 0.0f, 1.0f);
  // Generate vectors
  const vec3 tangent = normalize(cross(normal, helper));
  const vec3 binormal = normalize(cross(normal, tangent));
  return mat3(tangent, binormal, normal);
}

vec3 RandomSampleHemisphere(inout uint seed, in vec3 normal, in float alpha) {
  // Uniformly sample hemisphere direction
  const float cosTheta = 1.0f - EE_RANDOM(seed) * (1.0f - alpha) * (1.0f - alpha);
  const float sinTheta = sqrt(max(0.0f, 1.0f - cosTheta * cosTheta));
  const float phi = 2.0f * 3.1415926f * EE_RANDOM(seed);
  const vec3 tangentSpaceDir = vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
  // Transform direction to world space
  return GetTangentSpace(normal) * tangentSpaceDir;
}

vec3 BRDF(in float metallic, inout uint seed, in vec3 inDirection, in vec3 inNormal) {
  const vec3 reflected = Reflect(inDirection, inNormal);
  return RandomSampleHemisphere(seed, reflected, metallic);
}

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
	
	float roughness = surface.roughness;
	float metallic = surface.metallic;
	float ao = surface.occlusion;
	vec4 albedo = surface.base_color;

	//Proceed...
	float f = 1.0f;
	if (metallic >= 0.0f){
		f = (metallic + 2) / (metallic + 1);
	}

	hit_value.hit_count += 1;

	vec3 combined_color = vec3(0.0f, 0.0f, 0.0f);
	if(hit_value.hit_count <= bounce){
		const vec3 sample_direction = BRDF(metallic, hit_value.seed, gl_WorldRayDirectionEXT, worldNormal);
		traceRayEXT(EE_TLAS, gl_RayFlagsOpaqueEXT, 0xff, 0, 0, 0, worldPosition, 1e-3f, sample_direction, 1e20f, 0);
		const vec3 received_color = hit_value.hit_info.color.xyz;
		combined_color = albedo.xyz * clamp(abs(dot(worldNormal, sample_direction)) * roughness + (1.f - roughness) * f, 0.0f, 1.0f) * received_color;
	}else{
		combined_color = EE_SKY_COLOR(worldNormal) * 1e-3f;
	}
	hit_value.hit_info.color = vec4(combined_color + surface.emissive * albedo.xyz, 1.0f);

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
