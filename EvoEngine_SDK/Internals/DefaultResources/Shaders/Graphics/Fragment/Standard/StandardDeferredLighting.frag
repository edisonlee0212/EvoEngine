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
	vec3 cameraPosition = EE_CAMERA_POSITION(EE_CAMERA_INDEX);
	vec3 skyColor = EE_SKY_COLOR(fragPos - cameraPosition);
	if(ndcDepth == 1.0f) {
		FragColor = vec4(skyColor, 1.0f);
		return;
	}

	vec4 normalSample = texture(inNormal, fs_in.TexCoord);
	vec3 normal = normalSample.xyz;
	int instance_index = int(round(normalSample.a));
	float depth = EE_LINEARIZE_DEPTH(EE_CAMERA_INDEX, ndcDepth);

	vec4 matSample = texture(inMaterial, fs_in.TexCoord);
	int info_index = int(round(matSample.w));
	int material_index = int(round(matSample.z));
	int max_instance_index = max(EE_RENDER_INFO.instance_size - 1, 0);
	int max_material_index = max(EE_RENDER_INFO.material_size - 1, 0);
	instance_index = clamp(instance_index, 0, max_instance_index);
	material_index = clamp(material_index, 0, max_material_index);

	vec4 albedo;
	float roughness, metallic, specular, emission, ao;
	bool receiveShadow = true;

	if (info_index > 1) {
		// Tinted path stores albedo in rgb and keeps instance index in normal.a.
		// Recover material properties from the instance so shaded materials do
		// not collapse to hardcoded roughness/metallic values.
		Instance instance = EE_INSTANCES[instance_index];
		int instance_material_index = clamp(instance.material_index, 0, max_material_index);
		MaterialProperties materialProperties = EE_MATERIAL_PROPERTIES[instance_material_index];
		albedo = vec4(matSample.rgb, 1.0);
		roughness = materialProperties.roughness;
		metallic = materialProperties.metallic;
		specular = materialProperties.specular;
		emission = materialProperties.emission;
		ao = materialProperties.ambient_occulusion;
		receiveShadow = materialProperties.receive_shadow;
	} else {
		vec2 tex_coord = matSample.xy;
		MaterialProperties materialProperties = EE_MATERIAL_PROPERTIES[material_index];
		roughness = EE_SAMPLE_TEXTURE_2D(materialProperties.roughness_map_index, tex_coord, vec4(materialProperties.roughness, 0, 0, 0)).r;
		metallic = EE_SAMPLE_TEXTURE_2D(materialProperties.metallic_map_index, tex_coord, vec4(materialProperties.metallic, 0, 0, 0)).r;
		specular = materialProperties.specular;
		emission = materialProperties.emission;
		ao = EE_SAMPLE_TEXTURE_2D(materialProperties.ao_texture_index, tex_coord, vec4(materialProperties.ambient_occulusion, 0, 0, 0)).r;
		albedo = EE_SAMPLE_TEXTURE_2D(materialProperties.albedo_map_index, tex_coord, materialProperties.albedo);
		receiveShadow = materialProperties.receive_shadow;
	}

	vec3 viewDir = normalize(cameraPosition - fragPos);
	vec3 F0 = vec3(0.04f); 
	F0 = mix(F0, albedo.xyz, metallic);
	vec3 result = EE_FUNC_CALCULATE_LIGHTS(receiveShadow, albedo.xyz, specular, depth, normal, viewDir, fragPos, metallic, roughness, F0);
	vec3 ambient = EE_FUNC_CALCULATE_ENVIRONMENTAL_LIGHT(albedo.xyz, normal, viewDir, metallic, roughness, F0);
	vec3 safe_albedo_dir = dot(albedo.xyz, albedo.xyz) > 1e-8 ? normalize(albedo.xyz) : vec3(0.0f);
	vec3 outputColor = result + emission * safe_albedo_dir + ambient * ao;

	float fade_ratio = EE_CAMERA_FADE_RATIO(EE_CAMERA_INDEX);
	if(depth > EE_CAMERA_FAR(EE_CAMERA_INDEX) * fade_ratio){
		outputColor.xyz = mix(outputColor.xyz, skyColor, (depth - EE_CAMERA_FAR(EE_CAMERA_INDEX) * fade_ratio) / (EE_CAMERA_FAR(EE_CAMERA_INDEX) * (1.f - fade_ratio)));
	}
	FragColor = vec4(outputColor, 1.0f);
}