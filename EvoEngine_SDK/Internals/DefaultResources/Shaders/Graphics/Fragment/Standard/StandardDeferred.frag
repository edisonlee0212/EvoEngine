#extension GL_ARB_shader_draw_parameters : enable
#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"
#include "GltfRasterMaterial.glsl"

layout (location = 0) in VS_OUT {
	vec3 FragPos;
	vec3 Normal;
	vec3 Tangent;
	flat float TangentHandedness;
	vec2 TexCoord;
} fs_in;

layout (location = 0) out vec4 outNormal;
layout (location = 1) out vec4 outMaterial;
layout (location = 2) out vec4 outGBufferBaseColorAO;
layout (location = 3) out vec4 outGBufferNormalRoughness;
layout (location = 4) out vec4 outGBufferPbrFlags;
layout (location = 5) out vec4 outGBufferEmissive;
layout (location = 6) out vec4 outGBufferUtility;

layout(location = 5) in flat uint currentInstanceIndex;

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
	vec3 world_normal = normalize((gl_FrontFacing ? 1.0 : -1.0) * normal);
	// also store the per-fragment normals into the gbuffer
	outNormal.rgb = world_normal;
	outNormal.a = instance_index;
	outMaterial = vec4(tex_coord.x, tex_coord.y, instance.material_index, instance.info_index);
	outGBufferBaseColorAO = vec4(max(surface.base_color.rgb, vec3(0.0)), max(surface.occlusion, 0.0));
	outGBufferNormalRoughness = vec4(world_normal, surface.roughness);
	outGBufferPbrFlags = vec4(surface.metallic, 0.0, 0.0, 0.0);
	outGBufferEmissive = vec4(surface.emissive, 0.0);
	outGBufferUtility = vec4(float(instance_index), float(instance.info_index), float(instance.material_index), 0.0);
}
