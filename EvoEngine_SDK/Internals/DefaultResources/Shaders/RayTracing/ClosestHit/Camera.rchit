#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_ray_tracing : require

#include "RayTracingBasic.glsl"
#include "CameraRayTracingPayload.glsl"
#include "Random.glsl"
layout(location = 0) rayPayloadInEXT CameraRayTracingPayload hit_value;

hitAttributeEXT vec2 attribs;

vec3 Reflect(in vec3 incident, in vec3 normal) {
  return incident - 2.0f * dot(incident, normal) * normal;
}

layout(push_constant) uniform EE_CAMERA_CONSTANTS {
  uint EE_CAMERA_INDEX;
  uint EE_FRAME_ID;
};


vec3 EE_SKY_COLOR(vec3 direction) {
	Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
	return camera.use_clear_color == 1 ?
		camera.clear_color.xyz * camera.clear_color.w
		: pow(texture(EE_CUBEMAPS[camera.skybox_tex_index], normalize(direction)).rgb, vec3(1.0f / EE_ENVIRONMENT.gamma)) * camera.clear_color.w;
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
	const MaterialProperties materialProperties = EE_MATERIAL_PROPERTIES[EE_INSTANCES[instance_index].material_index];

	const int triangle_offset = EE_INSTANCES[instance_index].triangle_offset + gl_PrimitiveID;

	// Vertex of the triangle
	const Vertex v0 = EE_VERTICES[EE_INDICES[triangle_offset * 3]];
	const Vertex v1 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 1]];
	const Vertex v2 = EE_VERTICES[EE_INDICES[triangle_offset * 3 + 2]];

	const vec3 barycentrics = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

	const vec3 position      = v0.position * barycentrics.x + v1.position * barycentrics.y + v2.position * barycentrics.z;
	const vec2 tex_coord      = v0.tex_coord * barycentrics.x + v1.tex_coord * barycentrics.y + v2.tex_coord * barycentrics.z;
	vec3 normal      = v0.normal * barycentrics.x + v1.normal * barycentrics.y + v2.normal * barycentrics.z;
	const vec3 tangent      = v0.tangent * barycentrics.x + v1.tangent * barycentrics.y + v2.tangent * barycentrics.z;
	normal = EE_SAMPLE_NORMAL(materialProperties.normal_map_index, tex_coord, normal, tangent);
	
	const vec3 worldPosition = vec3(gl_ObjectToWorldEXT * vec4(position, 1.0));  // Transforming the position to world space
	const vec3 worldNormal = normalize(vec3(normal * gl_WorldToObjectEXT));  // Transforming the normal to world space
	const vec3 worldTangent = normalize(vec3(tangent * gl_WorldToObjectEXT));  // Transforming the normal to world space
	
	float roughness = EE_SAMPLE_TEXTURE_2D(materialProperties.roughness_map_index, tex_coord, vec4(materialProperties.roughness, 0, 0, 0)).r;
	float metallic = EE_SAMPLE_TEXTURE_2D(materialProperties.metallic_map_index, tex_coord, vec4(materialProperties.metallic, 0, 0, 0)).r;
	float specular = clamp(materialProperties.specular, 0.0f, 1.0f);
	float transmission = clamp(materialProperties.transmission, 0.0f, 1.0f);
	float emission = materialProperties.emission;
	float ao = EE_SAMPLE_TEXTURE_2D(materialProperties.ao_texture_index, tex_coord, vec4(materialProperties.ambient_occulusion, 0, 0, 0)).r;
	vec4 albedo = EE_SAMPLE_TEXTURE_2D(materialProperties.albedo_map_index, tex_coord, materialProperties.albedo);

	// Aggregate procedural meshes (e.g. Scots-pine needles) bake their per-segment
	// colors into vertex.color and rely on the raster pipeline's vertex-tint path.
	// Mirror that behaviour here so ray-traced renders see the same per-vertex
	// palette and don't collapse to black after a few bounces against a white
	// material albedo. Gated on materialProperties.vertex_color_only to avoid
	// double-modulating PBR assets that don't author per-vertex color.
	if (materialProperties.vertex_color_only != 0) {
		const vec4 vertex_tint = v0.color * barycentrics.x +
		                         v1.color * barycentrics.y +
		                         v2.color * barycentrics.z;
		albedo.rgb *= clamp(vertex_tint.rgb, vec3(0.0), vec3(1.0));
	}

	hit_value.hit_count += 1;
	hit_value.position = worldPosition;
	hit_value.normal = worldNormal;
	const Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
	
	vec3 combined_color = vec3(0.0f, 0.0f, 0.0f);
	if(hit_value.hit_count <= camera.bounce){
		const vec3 sample_direction = BRDF(metallic, hit_value.seed, gl_WorldRayDirectionEXT, worldNormal);
		traceRayEXT(EE_TLAS, gl_RayFlagsOpaqueEXT, 0xff, 0, 0, 0, worldPosition, 1e-3f, sample_direction, 1e20f, 0);
		const vec3 received_color = hit_value.color;
		const vec3 reflected_dir = normalize(Reflect(gl_WorldRayDirectionEXT, worldNormal));
		const float diffuse_lobe = mix(max(dot(worldNormal, sample_direction), 0.0f),
		                               max(dot(-worldNormal, sample_direction), 0.0f),
		                               transmission);
		const float reflected_alignment = max(dot(reflected_dir, sample_direction), 0.0f);
		const float specular_power = mix(48.0f, 4.0f, roughness);
		const float specular_lobe = pow(reflected_alignment, specular_power);
		const float shading_term = clamp((1.0f - specular) * diffuse_lobe + specular * specular_lobe, 0.0f, 1.0f);
		combined_color = albedo.xyz * shading_term * received_color;
	}else{
		combined_color = EE_SKY_COLOR(worldNormal) * 1e-3f;
	}
	hit_value.color = combined_color * ao + emission * albedo.xyz;

	hit_value.initial_normal = worldNormal;
	hit_value.initial_position = worldPosition;
}