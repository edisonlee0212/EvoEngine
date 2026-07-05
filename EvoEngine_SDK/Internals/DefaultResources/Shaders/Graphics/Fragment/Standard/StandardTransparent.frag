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
	vec2 TexCoord;
} fs_in;

layout(location = 5) in flat uint currentInstanceIndex;

layout (location = 0) out vec4 FragColor;

void main()
{
	int instance_index = int(currentInstanceIndex);
	Instance instance = EE_INSTANCES[instance_index];
	vec2 tex_coord = fs_in.TexCoord;
	uint material_index = uint(instance.material_index);
	GltfRasterMaterial surface = EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, tex_coord, tex_coord);
	if (EE_GLTF_RASTER_SHOULD_DISCARD(surface)) discard;

	vec3 normal = EE_EVALUATE_GLTF_RASTER_NORMAL(
		material_index, tex_coord, tex_coord, fs_in.Normal, fs_in.Tangent, fs_in.TangentHandedness);
	normal = normalize((gl_FrontFacing ? 1.0 : -1.0) * normal);

	vec3 cameraPosition = EE_CAMERA_POSITION(EE_CAMERA_INDEX);
	vec3 viewDir = normalize(cameraPosition - fs_in.FragPos);
	float depth = EE_LINEARIZE_DEPTH(EE_CAMERA_INDEX, gl_FragCoord.z);
	vec4 albedo = surface.base_color;
	float roughness = surface.roughness;
	float metallic = surface.metallic;
	vec3 F0 = mix(vec3(0.04f), albedo.rgb, metallic);

	vec3 direct = EE_FUNC_CALCULATE_LIGHTS(true, albedo.rgb, 1.0, depth, normal, viewDir, fs_in.FragPos, metallic, roughness, F0);
	vec3 ambient = EE_FUNC_CALCULATE_ENVIRONMENTAL_LIGHT(albedo.rgb, normal, viewDir, metallic, roughness, F0) +
	               EE_FUNC_CALCULATE_DDGI_DIFFUSE(albedo.rgb, normal, viewDir, fs_in.FragPos);
	vec3 outputColor = direct + surface.emissive + ambient * surface.occlusion;

	FragColor = vec4(outputColor, clamp(albedo.a, 0.0, 1.0));
}
