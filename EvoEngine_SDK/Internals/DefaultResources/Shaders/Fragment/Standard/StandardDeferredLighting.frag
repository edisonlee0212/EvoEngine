precision highp float;

layout (location = 0) in VS_OUT {
	vec2 TexCoord;
} fs_in;

layout(set = EE_PER_PASS_SET, binding = 18) uniform sampler2D inDepth;
layout(set = EE_PER_PASS_SET, binding = 19) uniform sampler2D inNormal;
layout(set = EE_PER_PASS_SET, binding = 20) uniform sampler2D inMaterial;

layout (location = 0) out vec4 FragColor;

void main()
{
	float ndcDepth = 	texture(inDepth, fs_in.TexCoord).x;
	vec3 fragPos = EE_DEPTH_TO_WORLD_POS(fs_in.TexCoord, ndcDepth);

	if(ndcDepth == 1.0) {
		vec3 cameraPosition = EE_CAMERA_POSITION();
		Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
		vec3 envColor = EE_SKY_COLOR(fragPos - cameraPosition);
		FragColor = vec4(envColor, 1.0);
		return;
	}

	vec3 normal = 		texture(inNormal, fs_in.TexCoord).xyz;
	float depth = EE_LINEARIZE_DEPTH(ndcDepth);

	int material_index = int(round(texture(inMaterial, fs_in.TexCoord).w));
	vec2 materialTexCoord = texture(inMaterial, fs_in.TexCoord).xy;
	MaterialProperties materialProperties = EE_MATERIAL_PROPERTIES[material_index];

	float roughness = materialProperties.roughness;
	float metallic = materialProperties.metallic;
	float emission = materialProperties.emission;
	float ao = materialProperties.ambient_occulusion;
	vec4 albedo = materialProperties.albedo;

	if (materialProperties.roughness_map_index != -1) roughness = texture(EE_TEXTURE_2DS[materialProperties.roughness_map_index], materialTexCoord).r;
	if (materialProperties.metallic_map_index != -1) metallic = texture(EE_TEXTURE_2DS[materialProperties.metallic_map_index], materialTexCoord).r;
	if (materialProperties.ao_texture_index != -1) ao = texture(EE_TEXTURE_2DS[materialProperties.ao_texture_index], materialTexCoord).r;
	if (materialProperties.albedo_map_index != -1) albedo = texture(EE_TEXTURE_2DS[materialProperties.albedo_map_index], materialTexCoord);

	vec3 cameraPosition = EE_CAMERA_POSITION();
	vec3 viewDir = normalize(cameraPosition - fragPos);
	bool receiveShadow = true;
	vec3 F0 = vec3(0.04); 
	F0 = mix(F0, albedo.xyz, metallic);
	vec3 result = EE_FUNC_CALCULATE_LIGHTS(receiveShadow, albedo.xyz, 1.0, depth, normal, viewDir, fragPos, metallic, roughness, F0);
	vec3 ambient = EE_FUNC_CALCULATE_ENVIRONMENTAL_LIGHT(albedo.xyz, normal, viewDir, metallic, roughness, F0);
	vec3 color = result + emission * normalize(albedo.xyz) + ambient * ao;
	color = pow(color, vec3(1.0 / EE_RENDER_INFO.gamma));
	FragColor = vec4(color, 1.0);
}