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
layout(location = 6) in vec4 instanceColor;

void main()
{
	int instance_index = int(currentInstanceIndex);
	Instance instance = EE_INSTANCES[instance_index];
	MaterialProperties materialProperties = EE_MATERIAL_PROPERTIES[instance.material_index];
	vec2 tex_coord = fs_in.TexCoord;
	vec4 albedo = EE_SAMPLE_TEXTURE_2D(materialProperties.albedo_map_index, tex_coord, materialProperties.albedo);
	if (albedo.a <= 0.5f) discard;
	vec3 normal = EE_SAMPLE_NORMAL(materialProperties.normal_map_index, tex_coord, fs_in.Normal, fs_in.Tangent);
	// also store the per-fragment normals into the gbuffer
	outNormal.rgb = normalize((gl_FrontFacing ? 1.0 : -1.0) * normal);
	outNormal.a = instance_index;

	// Optional per-vertex tint path for aggregate procedural meshes.
	// Alpha < 0.5 selects vertex-color tinting to avoid changing default
	// instanced behavior for assets that only use instance color.
	bool use_vertex_tint = instanceColor.a < 0.5;
	bool has_instance_tint = instanceColor.r < 0.999 || instanceColor.g < 0.999 || instanceColor.b < 0.999;
	if (use_vertex_tint) {
		vec3 tinted = albedo.rgb * clamp(fs_in.Color.rgb, vec3(0.0), vec3(1.0));
		outMaterial = vec4(tinted, instance.info_index + 2);
	} else if (has_instance_tint) {
		vec3 tinted = albedo.rgb * instanceColor.rgb;
		outMaterial = vec4(tinted, instance.info_index + 2);
	} else {
		outMaterial = vec4(tex_coord.x, tex_coord.y, instance.material_index, instance.info_index);
	}
}
