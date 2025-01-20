#extension GL_ARB_shader_draw_parameters : enable
#extension GL_ARB_shading_language_include : enable

#include "DynamicStrandsSmallSegmentsRenderingConstants.glsl"

#include "PerFrame.glsl"
#define EE_PER_GROUP_SET 2
#include "Lighting.glsl"

layout (location = 0) in VS_OUT {
	vec3 FragPos;
	vec3 Normal;
	vec3 Tangent;
	vec2 TexCoord;
	vec4 Color;
} fs_in;

layout (location = 0) out vec4 outNormal;
layout (location = 1) out vec4 outMaterial;

void main(){
	Instance instance = EE_INSTANCES[EE_INSTANCE_INDEX];
	MaterialProperties materialProperties = EE_MATERIAL_PROPERTIES[instance.material_index];
	vec2 tex_coord = fs_in.TexCoord;
	vec4 albedo = materialProperties.albedo;
	if (materialProperties.albedo_map_index != -1) 
		albedo = texture(EE_TEXTURE_2DS[materialProperties.albedo_map_index], tex_coord);
	if (albedo.a <= 0.5f) discard;

	vec3 normal = fs_in.Normal;
	if (materialProperties.normal_map_index != -1){
		vec3 B = cross(fs_in.Normal, fs_in.Tangent);
		mat3 TBN = mat3(fs_in.Tangent, B, fs_in.Normal);
		normal = texture(EE_TEXTURE_2DS[materialProperties.normal_map_index], tex_coord).rgb;
		normal = normal * 2.0f - 1.0f;
		normal = normalize(TBN * normal);
	}

	// also store the per-fragment normals into the gbuffer
	outNormal.rgb = normalize((gl_FrontFacing ? 1.0 : -1.0) * normal);
	outNormal.a = EE_INSTANCE_INDEX;

	if(fs_in.Color.a > 0.5f){
		outMaterial = vec4(fs_in.Color.xyz, instance.info_index + 2);
	}else{
		outMaterial = vec4(tex_coord.x, tex_coord.y, instance.material_index, instance.info_index);
	}
}