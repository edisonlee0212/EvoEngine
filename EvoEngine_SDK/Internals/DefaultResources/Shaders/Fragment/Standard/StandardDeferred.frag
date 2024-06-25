#extension GL_ARB_shader_draw_parameters : enable

layout (location = 0) in VS_OUT {
	vec3 FragPos;
	vec3 Normal;
	vec3 Tangent;
	vec2 TexCoord;
} fs_in;

layout (location = 0) out vec4 outNormal;
layout (location = 1) out vec4 outMaterial;

layout(location = 5) in flat uint currentInstanceIndex;

void main()
{
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

	// also store the per-fragment normals into the gbuffer
	outNormal.rgb = normalize((gl_FrontFacing ? 1.0 : -1.0) * normal);
	outNormal.a = instanceIndex + 1;
	
	outMaterial = vec4(tex_coord.x, tex_coord.y, EE_INSTANCES[instanceIndex].info_index, EE_INSTANCES[instanceIndex].material_index);
}