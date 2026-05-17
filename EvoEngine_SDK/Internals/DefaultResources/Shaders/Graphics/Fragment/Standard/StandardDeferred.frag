#extension GL_ARB_shader_draw_parameters : enable
#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"

layout (location = 0) in VS_OUT {
	vec3 FragPos;
	vec3 Normal;
	vec3 Tangent;
	vec2 TexCoord;
	vec4 Color;
} fs_in;

layout (location = 0) out vec4 outNormal;
layout (location = 1) out vec4 outMaterial;

layout(location = 5) in flat uint currentInstanceIndex;

void main()
{
	int instance_index = int(currentInstanceIndex);
	Instance instance = EE_INSTANCES[instance_index];
	MaterialProperties materialProperties = EE_MATERIAL_PROPERTIES[instance.material_index];
	vec2 tex_coord = fs_in.TexCoord;
	bool vertex_color_only = materialProperties.vertex_color_only != 0;
	vec4 albedo = vertex_color_only
		? vec4(materialProperties.albedo.rgb, 1.0)
		: EE_SAMPLE_TEXTURE_2D(materialProperties.albedo_map_index, tex_coord, materialProperties.albedo);
	if (vertex_color_only) {
		vec3 vertex_tint = clamp(fs_in.Color.rgb, vec3(0.0), vec3(1.0));
		if (dot(vertex_tint, vertex_tint) < 1e-6f) {
			vertex_tint = vec3(1.0);
		}
		albedo.rgb *= vertex_tint;
	} else if (albedo.a <= 0.5f) {
		discard;
	}
	vec3 normal = vertex_color_only
		? normalize(fs_in.Normal)
		: EE_SAMPLE_NORMAL(materialProperties.normal_map_index, tex_coord, fs_in.Normal, fs_in.Tangent);
	// also store the per-fragment normals into the gbuffer
	outNormal.rgb = normalize((gl_FrontFacing ? 1.0 : -1.0) * normal);
	outNormal.a = instance_index;
	if (vertex_color_only) {
		int packed_info = instance.material_index * 4 + instance.info_index + 2;
		outMaterial = vec4(albedo.rgb, float(packed_info));
	} else {
		outMaterial = vec4(tex_coord.x, tex_coord.y, instance.material_index, instance.info_index);
	}
}