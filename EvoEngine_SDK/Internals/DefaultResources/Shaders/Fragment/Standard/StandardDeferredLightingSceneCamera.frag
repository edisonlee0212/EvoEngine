#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"
#include "Lighting.glsl"

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
	vec3 fragPos = EE_DEPTH_TO_WORLD_POS(EE_CAMERA_INDEX, fs_in.TexCoord, ndcDepth);
	int infoIndex = int(round(texture(inMaterial, fs_in.TexCoord).z));
	bool instanceSelected = infoIndex == 1;

	if(ndcDepth == 1.0) {
		vec3 cameraPosition = EE_CAMERA_POSITION(EE_CAMERA_INDEX);
		Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
		vec3 envColor = EE_SKY_COLOR(fragPos - cameraPosition);
		if(!instanceSelected){
			vec2 texOffset = 1.0 / textureSize(inMaterial, 0); // gets size of single texel
			for(int i = -3; i <= 3; i++){
				for(int j = -3; j <= 3; j++){
					float temp2 = texture(inMaterial, fs_in.TexCoord + vec2(texOffset.x * i, texOffset.y * j)).z;
					int infoIndex = int(round(temp2));
					if(infoIndex == 1){
						FragColor = mix(vec4(1, 0.75, 0.0, 1.0), vec4(envColor, 1.0), 0.1);
						return;
					}
				}
			}
			FragColor = mix(vec4(0.5, 0.5, 0.5, 1.0), vec4(envColor, 1.0), float(EE_LIGHT_SPLIT_INDEX) / 256.0);
		}else{
			FragColor = vec4(envColor, 1.0);
		}
		return;
	}

	float depth = EE_LINEARIZE_DEPTH(EE_CAMERA_INDEX, ndcDepth);

	vec3 normal = 		texture(inNormal, fs_in.TexCoord).xyz;
	
	int instanceIndex = int(round(texture(inNormal, fs_in.TexCoord).a));

	int material_index = int(round(texture(inMaterial, fs_in.TexCoord).w));
	vec2 materialTexCoord = texture(inMaterial, fs_in.TexCoord).xy;
	MaterialProperties materialProperties = EE_MATERIAL_PROPERTIES[material_index];

	float roughness = 0.0;
	float metallic = 0.0;
	float emission = 1.0;
	float ao = 1.0;
	vec4 albedo = vec4(1.0);
	if(EE_RENDER_INFO.debug_visualization == 0) {
		roughness = materialProperties.roughness;
		metallic = materialProperties.metallic;
		emission = materialProperties.emission;
		ao = materialProperties.ambient_occulusion;
		albedo = materialProperties.albedo;

		if (materialProperties.roughness_map_index != -1) roughness = texture(EE_TEXTURE_2DS[materialProperties.roughness_map_index], materialTexCoord).r;
		if (materialProperties.metallic_map_index != -1) metallic = texture(EE_TEXTURE_2DS[materialProperties.metallic_map_index], materialTexCoord).r;
		if (materialProperties.ao_texture_index != -1) ao = texture(EE_TEXTURE_2DS[materialProperties.ao_texture_index], materialTexCoord).r;
		if (materialProperties.albedo_map_index != -1) albedo = texture(EE_TEXTURE_2DS[materialProperties.albedo_map_index], materialTexCoord);
	}else if(EE_RENDER_INFO.debug_visualization == 1){
		albedo = vec4(EE_UNIFORM_KERNEL[material_index % MAX_KERNEL_AMOUNT].xyz, 1.0);
	}else if(EE_RENDER_INFO.debug_visualization == 2){
		albedo = vec4(EE_UNIFORM_KERNEL[instanceIndex % MAX_KERNEL_AMOUNT].xyz, 1.0);
	}
	vec3 cameraPosition = EE_CAMERA_POSITION(EE_CAMERA_INDEX);
	vec3 viewDir = normalize(cameraPosition - fragPos);
	bool receiveShadow = true;
	vec3 F0 = vec3(0.04); 
	F0 = mix(F0, albedo.xyz, metallic);
	vec3 result = EE_FUNC_CALCULATE_LIGHTS(receiveShadow, albedo.xyz, 1.0, depth, normal, viewDir, fragPos, metallic, roughness, F0);
	vec3 ambient = EE_FUNC_CALCULATE_ENVIRONMENTAL_LIGHT(albedo.xyz, normal, viewDir, metallic, roughness, F0);
	vec3 color = result + emission * normalize(albedo.xyz) + ambient * ao;
	color = pow(color, vec3(1.0 / EE_RENDER_INFO.gamma));

	if(!instanceSelected && EE_INSTANCE_INDEX == 1){
		vec2 texOffset = 1.0 / textureSize(inNormal, 0); // gets size of single texel
		for(int i = -3; i <= 3; i++){
			for(int j = -3; j <= 3; j++){
				float temp2 = texture(inMaterial, fs_in.TexCoord + vec2(texOffset.x * i, texOffset.y * j)).z;
				int infoIndex = int(round(temp2));
				if(infoIndex == 1){
					FragColor = mix(vec4(1, 0.75, 0.0, 1.0), vec4(color, 1.0), 0.1);
					return;
				}
			}
		}
		FragColor = mix(vec4(0.5, 0.5, 0.5, 1.0), vec4(color, 1.0), float(EE_LIGHT_SPLIT_INDEX) / 256.0);
	}else{
		FragColor = vec4(color, 1.0);
	}
	
}