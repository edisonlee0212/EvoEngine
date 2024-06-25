
layout (location = 0) in VS_OUT {
	vec2 TexCoord;
} fs_in;

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
}