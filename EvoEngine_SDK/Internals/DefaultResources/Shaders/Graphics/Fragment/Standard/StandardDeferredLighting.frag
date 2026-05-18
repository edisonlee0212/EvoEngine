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
	int encoded_info_index = int(round(matSample.w));
	int packed_material_index = encoded_info_index > 1 ? (encoded_info_index >> 2) : -1;
	int info_index = encoded_info_index > 1 ? (encoded_info_index & 3) : encoded_info_index;
	int material_index = int(round(matSample.z));
	int max_instance_index = max(EE_RENDER_INFO.instance_size - 1, 0);
	int max_material_index = max(EE_RENDER_INFO.material_size - 1, 0);
	instance_index = clamp(instance_index, 0, max_instance_index);
	material_index = clamp(material_index, 0, max_material_index);
	if (packed_material_index >= 0) {
		packed_material_index = clamp(packed_material_index, 0, max_material_index);
	}
	int instance_material_index = material_index;
	bool instance_vertex_color_only = false;
	if (EE_RENDER_INFO.instance_size > 0) {
		instance_material_index = clamp(EE_INSTANCES[instance_index].material_index, 0, max_material_index);
		instance_vertex_color_only = EE_MATERIAL_PROPERTIES[instance_material_index].vertex_color_only != 0;
	}

	vec4 albedo;
	float roughness, metallic, specular, emission, ao;
	bool receiveShadow = true;
	bool vertex_color_only = false;

	if (info_index > 1) {
		// Tinted path stores albedo in rgb and packs material index in alpha.
		int resolved_material_index = packed_material_index >= 0
			? packed_material_index
			: material_index;
		MaterialProperties materialProperties = EE_MATERIAL_PROPERTIES[resolved_material_index];
		albedo = vec4(matSample.rgb, 1.0);
		roughness = materialProperties.roughness;
		metallic = materialProperties.metallic;
		specular = materialProperties.specular;
		emission = materialProperties.emission;
		ao = materialProperties.ambient_occulusion;
		receiveShadow = materialProperties.receive_shadow;
		vertex_color_only = materialProperties.vertex_color_only != 0;
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
		vertex_color_only = materialProperties.vertex_color_only != 0;
	}

	// Recover from decode drift by trusting the instance material contract.
	if (!vertex_color_only && instance_vertex_color_only) {
		MaterialProperties fallbackProperties = EE_MATERIAL_PROPERTIES[instance_material_index];
		roughness = fallbackProperties.roughness;
		metallic = fallbackProperties.metallic;
		specular = fallbackProperties.specular;
		emission = fallbackProperties.emission;
		ao = fallbackProperties.ambient_occulusion;
		receiveShadow = fallbackProperties.receive_shadow;
		vertex_color_only = true;
	}

	// Vertex-color-only instanced stems should always use g-buffer rgb tint.
	// If info decode drifts on alpha, matSample.rgb still carries the tint.
	if (vertex_color_only) {
		int tint_material_index = instance_material_index;
		if (info_index > 1 && packed_material_index >= 0) {
			tint_material_index = packed_material_index;
		} else if (info_index <= 1) {
			tint_material_index = material_index;
		}
		vec3 gbuffer_tint = clamp(matSample.rgb, vec3(0.0f), vec3(1.0f));
		vec3 material_tint = clamp(EE_MATERIAL_PROPERTIES[tint_material_index].albedo.rgb, vec3(0.0f), vec3(1.0f));
		float gbuffer_chroma = max(max(gbuffer_tint.r, gbuffer_tint.g), gbuffer_tint.b) -
			min(min(gbuffer_tint.r, gbuffer_tint.g), gbuffer_tint.b);
		float material_chroma = max(max(material_tint.r, material_tint.g), material_tint.b) -
			min(min(material_tint.r, material_tint.g), material_tint.b);
		bool gbuffer_tint_valid = info_index > 1;
		bool has_gbuffer_tint = gbuffer_tint_valid && dot(gbuffer_tint, gbuffer_tint) > 1e-6f;
		bool has_material_tint = dot(material_tint, material_tint) > 1e-6f;
		bool gbuffer_chromatic = gbuffer_chroma > 0.05f;
		bool material_chromatic = material_chroma > 0.05f;
		if (has_gbuffer_tint && gbuffer_chromatic) {
			albedo.xyz = gbuffer_tint;
		} else if (has_material_tint && material_chromatic) {
			albedo.xyz = material_tint;
		} else if (has_gbuffer_tint) {
			albedo.xyz = gbuffer_tint;
		} else if (has_material_tint) {
			// Non-tinted payload encodes texcoord/material metadata in rgb.
			// Never reinterpret that metadata as color for vertex_color_only.
			albedo.xyz = material_tint;
		}
	}

	vec3 viewDir = normalize(cameraPosition - fragPos);
	float normal_len2 = dot(normal, normal);
	bool normal_has_nan = any(notEqual(normal, normal));
	if (normal_has_nan || normal_len2 <= 1e-8f || normal_len2 > 1e8f) {
		normal = vec3(0.0f, 1.0f, 0.0f);
	} else {
		normal *= inversesqrt(normal_len2);
	}
	if (vertex_color_only && dot(normal, viewDir) < 0.0f) {
		normal = -normal;
	}
	vec3 F0 = vec3(0.04f); 
	F0 = mix(F0, albedo.xyz, metallic);
	vec3 result = EE_FUNC_CALCULATE_LIGHTS(receiveShadow, albedo.xyz, specular, depth, normal, viewDir, fragPos, metallic, roughness, F0);
	vec3 ambient = EE_FUNC_CALCULATE_ENVIRONMENTAL_LIGHT(albedo.xyz, normal, viewDir, metallic, roughness, F0);
	vec3 safe_albedo_dir = dot(albedo.xyz, albedo.xyz) > 1e-8 ? normalize(albedo.xyz) : vec3(0.0f);
	vec3 outputColor = result + emission * safe_albedo_dir + ambient * ao;
	if (vertex_color_only) {
		float lit = dot(max(outputColor, vec3(0.0f)), vec3(0.2126f, 0.7152f, 0.0722f));
		lit = clamp(lit, 0.22f, 1.35f);
		outputColor = albedo.xyz * lit;
		bool color_has_nan = any(notEqual(outputColor, outputColor));
		float color_len2 = dot(outputColor, outputColor);
		float albedo_len2 = dot(albedo.xyz, albedo.xyz);
		if (color_has_nan || color_len2 < 1e-6f) {
			outputColor = albedo_len2 > 1e-8f ? albedo.xyz * 0.35f : vec3(0.35f);
		}
	}

	float fade_ratio = EE_CAMERA_FADE_RATIO(EE_CAMERA_INDEX);
	if(depth > EE_CAMERA_FAR(EE_CAMERA_INDEX) * fade_ratio){
		outputColor.xyz = mix(outputColor.xyz, skyColor, (depth - EE_CAMERA_FAR(EE_CAMERA_INDEX) * fade_ratio) / (EE_CAMERA_FAR(EE_CAMERA_INDEX) * (1.f - fade_ratio)));
	}
	FragColor = vec4(outputColor, 1.0f);
}