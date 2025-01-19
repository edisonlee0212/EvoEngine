#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"
#include "Lighting.glsl"

precision highp float;

layout (location = 0) in VS_OUT {
	vec2 TexCoord;
} fs_in;

layout(set = EE_PER_PASS_SET, binding = 17) uniform sampler2D inDepth;
layout(set = EE_PER_PASS_SET, binding = 18) uniform sampler2D inNormal;
layout(set = EE_PER_PASS_SET, binding = 19) uniform sampler2D inMaterial;

layout (location = 0) out vec4 FragColor;
void main()
{
	float ndcDepth = 	texture(inDepth, fs_in.TexCoord).x;
	vec3 fragPos = EE_DEPTH_TO_WORLD_POS(EE_CAMERA_INDEX, fs_in.TexCoord, ndcDepth);
	int info_index = int(round(texture(inMaterial, fs_in.TexCoord).w));
	bool instance_selected = info_index % 2 == 1;
	vec3 cameraPosition = EE_CAMERA_POSITION(EE_CAMERA_INDEX);
	vec3 skyColor = EE_SKY_COLOR(fragPos - cameraPosition);		
	if(ndcDepth == 1.0) {
		if(!instance_selected && EE_INSTANCE_INDEX == 1){
			vec2 texOffset = 1.0 / textureSize(inMaterial, 0); // gets size of single texel
			for(int i = -3; i <= 3; i++){
				for(int j = -3; j <= 3; j++){
					int info_index_local = int(round(texture(inMaterial, fs_in.TexCoord + vec2(texOffset.x * i, texOffset.y * j)).w));
					if(info_index_local % 2 == 1){
						FragColor = mix(vec4(1, 0.75, 0.0, 1.0), vec4(skyColor, 1.0), 0.1);
						return;
					}
				}
			}
			FragColor = mix(vec4(0.5, 0.5, 0.5, 1.0), vec4(skyColor, 1.0), float(EE_LIGHT_SPLIT_INDEX) / 256.0);
		}else{
			FragColor = vec4(skyColor, 1.0);
		}
		return;
	}

	float depth = EE_LINEARIZE_DEPTH(EE_CAMERA_INDEX, ndcDepth);

	vec3 normal = 		texture(inNormal, fs_in.TexCoord).xyz;
	
	int instance_index = int(round(texture(inNormal, fs_in.TexCoord).a));

	


	int material_index = int(round(float(texture(inMaterial, fs_in.TexCoord).z)));

	vec2 materialTexCoord = texture(inMaterial, fs_in.TexCoord).xy;
	MaterialProperties materialProperties = EE_MATERIAL_PROPERTIES[material_index];

	float roughness = 0.0;
	float metallic = 0.0;
	float emission = 0.0;
	float ao = 1.0;
	vec4 albedo = vec4(1.0);
	if(EE_RENDER_INFO.debug_visualization == 0) {
		if(info_index > 1){
			albedo = vec4(texture(inMaterial, fs_in.TexCoord).xyz, 1.f);
		}else{
			roughness = materialProperties.roughness;
			metallic = materialProperties.metallic;
			emission = materialProperties.emission;
			ao = materialProperties.ambient_occulusion;
			albedo = materialProperties.albedo;

			if (materialProperties.roughness_map_index != -1) roughness = texture(EE_TEXTURE_2DS[materialProperties.roughness_map_index], materialTexCoord).r;
			if (materialProperties.metallic_map_index != -1) metallic = texture(EE_TEXTURE_2DS[materialProperties.metallic_map_index], materialTexCoord).r;
			if (materialProperties.ao_texture_index != -1) ao = texture(EE_TEXTURE_2DS[materialProperties.ao_texture_index], materialTexCoord).r;
			if (materialProperties.albedo_map_index != -1) albedo = texture(EE_TEXTURE_2DS[materialProperties.albedo_map_index], materialTexCoord);
		}
	}else if(EE_RENDER_INFO.debug_visualization == 1){
		albedo = vec4(abs(EE_UNIFORM_KERNEL[material_index % MAX_KERNEL_AMOUNT].xyz), 1.0);
	}else if(EE_RENDER_INFO.debug_visualization == 2){
		albedo = vec4(abs(EE_UNIFORM_KERNEL[instance_index % MAX_KERNEL_AMOUNT].xyz), 1.0);
	}else if(EE_RENDER_INFO.debug_visualization == 3){
		albedo = vec4(abs(EE_UNIFORM_KERNEL[info_index % MAX_KERNEL_AMOUNT].xyz), 1.0);
	}

	vec3 viewDir = normalize(cameraPosition - fragPos);
	bool receiveShadow = true;
	vec3 F0 = vec3(0.04); 
	F0 = mix(F0, albedo.xyz, metallic);
	vec3 result = EE_FUNC_CALCULATE_LIGHTS(receiveShadow, albedo.xyz, 1.0, depth, normal, viewDir, fragPos, metallic, roughness, F0);
	vec3 ambient = EE_FUNC_CALCULATE_ENVIRONMENTAL_LIGHT(albedo.xyz, normal, viewDir, metallic, roughness, F0);
	vec3 color = result + emission * normalize(albedo.xyz) + ambient * ao;
	
	vec4 outputColor = vec4(0, 0, 0, 1);
	if(!instance_selected && EE_INSTANCE_INDEX == 1){
		vec2 texOffset = 1.0 / textureSize(inNormal, 0); // gets size of single texel
		bool colorSet = false;
		for(int i = -3; i <= 3; i++){
			for(int j = -3; j <= 3; j++){
				int info_index_local = int(round(texture(inMaterial, fs_in.TexCoord + vec2(texOffset.x * i, texOffset.y * j)).w));
				if(info_index_local % 2 == 1){
					outputColor = mix(vec4(1, 0.75, 0.0, 1.0), vec4(color, 1.0), 0.1);
					colorSet = true;
				}
			}
		}
		if(!colorSet) outputColor = mix(vec4(0.5, 0.5, 0.5, 1.0), vec4(color, 1.0), float(EE_LIGHT_SPLIT_INDEX) / 256.0);
	}else{
		outputColor = vec4(color, 1.0f);
	}

	float fade_ratio = EE_CAMERA_FADE_RATIO(EE_CAMERA_INDEX);
	if(depth > EE_CAMERA_FAR(EE_CAMERA_INDEX) * fade_ratio){
		outputColor.xyz = mix(outputColor.xyz, skyColor, (depth - EE_CAMERA_FAR(EE_CAMERA_INDEX) * fade_ratio) / (EE_CAMERA_FAR(EE_CAMERA_INDEX) * (1.f - fade_ratio)));
	}

	FragColor = outputColor;
}