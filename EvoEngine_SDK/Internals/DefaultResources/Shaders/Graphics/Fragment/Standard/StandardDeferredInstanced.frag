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

uint ee_debug_hash_u32(uint x) {
	x ^= x >> 16;
	x *= 0x7feb352du;
	x ^= x >> 15;
	x *= 0x846ca68bu;
	x ^= x >> 16;
	return x;
}

vec3 ee_debug_hash3(uint seed) {
	uint hx = ee_debug_hash_u32(seed * 747796405u + 2891336453u);
	uint hy = ee_debug_hash_u32(seed * 277803737u + 122949829u);
	uint hz = ee_debug_hash_u32(seed * 104395301u + 374761393u);
	return vec3(float(hx), float(hy), float(hz)) / 4294967295.0;
}

void main()
{
	int instance_index = int(currentInstanceIndex);
	Instance instance = EE_INSTANCES[instance_index];
	MaterialProperties materialProperties = EE_MATERIAL_PROPERTIES[instance.material_index];

	// Probe modes for iterative instanced-needle GPU debugging.
	// 10: instance/entity identity hash
	// 11: material/info identity hash
	// 12: vertex color only
	// 13: material sample only
	// 14: particle instance color only (set 1)
	// 15: constant flat color
	int debug_mode = EE_RENDER_INFO.debug_visualization;
	if (debug_mode >= 10 && debug_mode <= 15) {
		vec3 probe_color = vec3(1.0, 0.0, 1.0);
		outNormal.rgb = normalize((gl_FrontFacing ? 1.0 : -1.0) * fs_in.Normal);
		outNormal.a = instance_index;
		if (debug_mode == 10) {
			uint seed = currentInstanceIndex * 4099u + instance.entity_index * 131u;
			probe_color = ee_debug_hash3(seed);
		} else if (debug_mode == 11) {
			uint seed = uint(instance.material_index) * 8191u + uint(instance.info_index + 17);
			probe_color = ee_debug_hash3(seed);
		} else if (debug_mode == 12) {
			probe_color = clamp(fs_in.Color.rgb, vec3(0.0), vec3(1.0));
		} else if (debug_mode == 13) {
			vec4 albedo_debug = EE_SAMPLE_TEXTURE_2D(
				materialProperties.albedo_map_index, fs_in.TexCoord, materialProperties.albedo);
			probe_color = clamp(albedo_debug.rgb, vec3(0.0), vec3(1.0));
		} else if (debug_mode == 14) {
			probe_color = clamp(instanceColor.rgb, vec3(0.0), vec3(1.0));
		} else if (debug_mode == 15) {
			probe_color = vec3(0.15, 0.95, 0.35);
		}
		int packed_info = instance.material_index * 4 + instance.info_index + 2;
		outMaterial = vec4(probe_color, float(packed_info));
		return;
	}

	vec2 tex_coord = fs_in.TexCoord;
	vec4 safe_instance_color = clamp(instanceColor, vec4(0.0), vec4(1.0));
	bool vertex_color_only = materialProperties.vertex_color_only != 0;
	// For vertex_color_only materials (e.g. ScotsPine internodes), default to
	// instance tint unless alpha explicitly selects vertex-color tint override.
	bool use_vertex_tint = vertex_color_only && safe_instance_color.a < 0.5;
	bool has_instance_tint =
		safe_instance_color.r < 0.999 || safe_instance_color.g < 0.999 || safe_instance_color.b < 0.999;
	bool use_instance_tint = vertex_color_only ? !use_vertex_tint : has_instance_tint;
	bool uses_tint_path = use_vertex_tint || use_instance_tint;
	vec4 albedo = vertex_color_only
		? vec4(materialProperties.albedo.rgb, 1.0)
		: (materialProperties.albedo_map_index < 0
			? vec4(materialProperties.albedo.rgb, 1.0)
			: EE_SAMPLE_TEXTURE_2D(materialProperties.albedo_map_index, tex_coord, materialProperties.albedo));
	if (!vertex_color_only && !uses_tint_path && albedo.a <= 0.5f) discard;
	vec3 normal = vertex_color_only
		? normalize(fs_in.Normal)
		: EE_SAMPLE_NORMAL(materialProperties.normal_map_index, tex_coord, fs_in.Normal, fs_in.Tangent);
	// also store the per-fragment normals into the gbuffer.
	// Keep normal.a as instance index (legacy contract used by non-tinted paths).
	// Tinted paths encode material index in outMaterial.a with info bits.
	outNormal.rgb = normalize((gl_FrontFacing ? 1.0 : -1.0) * normal);
	outNormal.a = float(instance_index);

	int packed_info = instance.material_index * 4 + instance.info_index + 2;

	// Optional per-vertex tint path for aggregate procedural meshes.
	// Alpha < 0.5 selects vertex-color tinting to avoid changing default
	// instanced behavior for assets that only use instance color.
	if (use_vertex_tint) {
		vec3 tinted = albedo.rgb * clamp(fs_in.Color.rgb, vec3(0.0), vec3(1.0));
		outMaterial = vec4(tinted, float(packed_info));
	} else if (use_instance_tint) {
		vec3 tinted = albedo.rgb * safe_instance_color.rgb;
		outMaterial = vec4(tinted, float(packed_info));
	} else {
		outMaterial = vec4(tex_coord.x, tex_coord.y, instance.material_index, instance.info_index);
	}
}
