#extension GL_ARB_shader_draw_parameters : enable
#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"
#include "GltfRasterMaterial.glsl"
#include "Lighting.glsl"

layout (location = 0) in VS_OUT {
	vec3 FragPos;
	vec3 Normal;
	vec3 Tangent;
	flat float TangentHandedness;
	vec4 TexCoord01;
	vec4 TexCoord23;
	vec4 Color;
} fs_in;

layout(location = 7) in flat uint currentInstanceIndex;
layout(location = 8) in flat float transformHandedness;

layout (location = 0) out vec4 FragColor;

void main()
{
	const int indirectLightingDebugView = EE_INDIRECT_LIGHTING_DEBUG_VIEW();
	int instance_index = int(currentInstanceIndex);
	Instance instance = EE_INSTANCES[instance_index];
	uint material_index = uint(instance.material_index);
	float facing_sign = (gl_FrontFacing ? 1.0 : -1.0) * transformHandedness;
	if (EE_GLTF_MATERIALS[material_index].double_sided == 0 && facing_sign < 0.0) discard;
	GltfRasterMaterial surface = EE_EVALUATE_GLTF_RASTER_SURFACE(
		material_index, fs_in.TexCoord01.xy, fs_in.TexCoord01.zw, fs_in.TexCoord23.xy, fs_in.TexCoord23.zw,
		fs_in.Color);
	if (EE_GLTF_RASTER_SHOULD_DISCARD(surface)) discard;
	if (EE_GLTF_MATERIALS[material_index].unlit != 0) {
		FragColor = vec4(indirectLightingDebugView == 0 ? surface.base_color.rgb : vec3(0.0f),
		                 indirectLightingDebugView == 0 ? EE_GLTF_RASTER_OPACITY(surface) : 1.0f);
		return;
	}

	vec3 normal = EE_EVALUATE_GLTF_RASTER_NORMAL(
		material_index, fs_in.TexCoord01.xy, fs_in.TexCoord01.zw, fs_in.TexCoord23.xy, fs_in.TexCoord23.zw,
		fs_in.Normal, fs_in.Tangent, fs_in.TangentHandedness);
	normal = normalize(facing_sign * normal);

	vec3 cameraPosition = EE_CAMERA_POSITION(EE_CAMERA_INDEX);
	vec3 viewDir = normalize(cameraPosition - fs_in.FragPos);
	float depth = EE_LINEARIZE_DEPTH(EE_CAMERA_INDEX, gl_FragCoord.z);
	vec4 albedo = surface.base_color;
	float roughness = surface.roughness;
	float metallic = surface.metallic;
	vec3 F0 = surface.specular_f0;

	vec3 direct = EE_FUNC_CALCULATE_LIGHTS(true, albedo.rgb, 1.0, depth, normal, viewDir, fs_in.FragPos, metallic, roughness, F0, surface.specular_f90);
	vec3 ambient = EE_FUNC_CALCULATE_DDGI_ENVIRONMENTAL_LIGHT(
		albedo.rgb, normal, viewDir, fs_in.FragPos, metallic, roughness, F0, surface.specular_f90, surface.occlusion,
		1.0f);
	if (indirectLightingDebugView != 0) {
		FragColor = vec4(ambient, 1.0f);
		return;
	}
	vec3 outputColor = direct + EE_GLTF_RASTER_COATED_EMISSION(
	                                material_index, surface, fs_in.TexCoord01.xy, fs_in.TexCoord01.zw,
	                                fs_in.TexCoord23.xy, fs_in.TexCoord23.zw, fs_in.Normal,
	                                fs_in.Tangent, fs_in.TangentHandedness, facing_sign, viewDir) +
	                   ambient;

	FragColor = vec4(outputColor, EE_GLTF_RASTER_OPACITY(surface));
}
