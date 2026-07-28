#extension GL_ARB_shader_draw_parameters : enable
#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.slangh"
#include "Basic.slangh"
#include "GltfRasterMaterial.slangh"

layout (location = 0) in VS_OUT {
	vec3 FragPos;
	vec3 Normal;
	vec3 Tangent;
	flat float TangentHandedness;
	vec4 TexCoord01;
	vec4 TexCoord23;
	vec4 Color;
} fs_in;

layout (location = 0) out vec4 outGBufferBaseColorAO;
layout (location = 1) out vec4 outGBufferNormalRoughness;
layout (location = 2) out vec4 outGBufferPbrFlags;
layout (location = 3) out vec4 outGBufferEmissive;
layout (location = 4) out vec4 outGBufferUtility;

layout(location = 7) in flat uint currentInstanceIndex;
layout(location = 8) in flat float transformHandedness;

void main()
{
	int instance_index = int(currentInstanceIndex);
	Instance instance = EE_INSTANCES[instance_index];
	uint material_index = uint(instance.material_index);
	float facing_sign = (gl_FrontFacing ? 1.0 : -1.0) * transformHandedness;
	if (EE_GLTF_MATERIALS[material_index].double_sided == 0 && facing_sign < 0.0) discard;
	GltfRasterMaterial surface = EE_EVALUATE_GLTF_RASTER_SURFACE(
		material_index, fs_in.TexCoord01.xy, fs_in.TexCoord01.zw, fs_in.TexCoord23.xy, fs_in.TexCoord23.zw,
		fs_in.Color);
	if (EE_GLTF_RASTER_SHOULD_DISCARD(surface)) discard;
	vec3 normal = EE_EVALUATE_GLTF_RASTER_NORMAL(
		material_index, fs_in.TexCoord01.xy, fs_in.TexCoord01.zw, fs_in.TexCoord23.xy, fs_in.TexCoord23.zw,
		fs_in.Normal, fs_in.Tangent, fs_in.TangentHandedness);
	vec3 world_normal = normalize(facing_sign * normal);
	vec3 view_direction = normalize(EE_CAMERA_POSITION(EE_CAMERA_INDEX) - fs_in.FragPos);
	vec3 coated_emissive = EE_GLTF_RASTER_COATED_EMISSION(
		material_index, surface, fs_in.TexCoord01.xy, fs_in.TexCoord01.zw, fs_in.TexCoord23.xy,
		fs_in.TexCoord23.zw, fs_in.Normal, fs_in.Tangent,
		fs_in.TangentHandedness, facing_sign, view_direction);
	outGBufferBaseColorAO = vec4(max(surface.base_color.rgb, vec3(0.0)), max(surface.occlusion, 0.0));
	outGBufferNormalRoughness = vec4(world_normal, surface.roughness);
	outGBufferPbrFlags = vec4(surface.metallic, surface.specular_f0);
	float encoded_specular_f90 = EE_GLTF_MATERIALS[material_index].unlit != 0 ? -1.0 : surface.specular_f90;
	outGBufferEmissive = vec4(coated_emissive, encoded_specular_f90);
	outGBufferUtility = vec4(float(instance_index), float(instance.info_index), float(instance.material_index), 0.0);
}
