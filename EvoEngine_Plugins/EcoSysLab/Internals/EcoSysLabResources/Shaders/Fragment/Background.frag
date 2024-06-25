out vec4 FragColor;

in VS_OUT {
	vec3 FragPos;
	vec3 Normal;
	vec3 Tangent;
	vec2 TexCoords;
} fs_in;

void main()
{
	vec2 texCoords = fs_in.TexCoords;
	vec4 albedo = UE_PBR_ALBEDO;
	if(UE_ALBEDO_MAP_ENABLED) albedo = vec4(texture(UE_ALBEDO_MAP, texCoords).rgb, 1.0);
	else if(UE_DIFFUSE_MAP_ENABLED) albedo = texture(UE_DIFFUSE_MAP, texCoords).rgba;
	FragColor = albedo;
}
