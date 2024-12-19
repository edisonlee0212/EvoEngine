#extension GL_ARB_shader_draw_parameters : enable
#extension GL_ARB_shading_language_include : enable

#include "DynamicStrandsRenderingConstants.glsl"

#include "PerFrame.glsl"
#define EE_PER_GROUP_SET 1
#include "Lighting.glsl"

layout (location = 0) in VS_OUT {
	vec3 FragPos;
	vec3 Normal;
	vec3 Tangent;
	vec2 TexCoord;
	vec4 Color;
} fs_in;

layout (location = 0) out vec4 out_color;

layout(location = 5) in flat uint currentInstanceIndex;

void main(){
	/*
	uint instanceIndex = currentInstanceIndex;

	MaterialProperties materialProperties = EE_MATERIAL_PROPERTIES[EE_INSTANCES[instanceIndex].material_index];
	vec2 tex_coord = fs_in.TexCoord;
	vec4 albedo = materialProperties.albedo;
	if (materialProperties.albedo_map_index != -1) 
		albedo = texture(EE_TEXTURE_2DS[materialProperties.albedo_map_index], tex_coord);
	if (albedo.a <= 0.5) discard;

	vec3 normal = fs_in.Normal;
	if (materialProperties.normal_map_index != -1){
		vec3 B = cross(fs_in.Normal, fs_in.Tangent);
		mat3 TBN = mat3(fs_in.Tangent, B, fs_in.Normal);
		normal = texture(EE_TEXTURE_2DS[materialProperties.normal_map_index], tex_coord).rgb;
		normal = normal * 2.0 - 1.0;
		normal = normalize(TBN * normal);
	}

	float roughness = materialProperties.roughness;
	float metallic = materialProperties.metallic;
	float emission = materialProperties.emission;
	float ao = materialProperties.ambient_occulusion;
	vec4 albedo = materialProperties.albedo;

	if (materialProperties.roughness_map_index != -1) roughness = texture(EE_TEXTURE_2DS[materialProperties.roughness_map_index], materialTexCoord).r;
	if (materialProperties.metallic_map_index != -1) metallic = texture(EE_TEXTURE_2DS[materialProperties.metallic_map_index], materialTexCoord).r;
	if (materialProperties.ao_texture_index != -1) ao = texture(EE_TEXTURE_2DS[materialProperties.ao_texture_index], materialTexCoord).r;
	if (materialProperties.albedo_map_index != -1) albedo = texture(EE_TEXTURE_2DS[materialProperties.albedo_map_index], materialTexCoord);

	vec3 cameraPosition = EE_CAMERA_POSITION(EE_CAMERA_INDEX);
	vec3 viewDir = normalize(cameraPosition - fragPos);
	bool receiveShadow = true;
	vec3 F0 = vec3(0.04); 
	F0 = mix(F0, albedo.xyz, metallic);
	vec3 result = EE_FUNC_CALCULATE_LIGHTS(receiveShadow, albedo.xyz, 1.0, depth, normal, viewDir, fragPos, metallic, roughness, F0);
	vec3 ambient = EE_FUNC_CALCULATE_ENVIRONMENTAL_LIGHT(albedo.xyz, normal, viewDir, metallic, roughness, F0);
	vec3 color = result + emission * normalize(albedo.xyz) + ambient * ao;
	color = pow(color, vec3(1.0 / EE_RENDER_INFO.gamma));
	*/
	out_color = vec4(fs_in.Color);
}