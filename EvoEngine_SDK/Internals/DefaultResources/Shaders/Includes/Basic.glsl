#extension GL_EXT_nonuniform_qualifier : enable
#extension GL_EXT_shader_explicit_arithmetic_types_int8 : require
#extension GL_ARB_shading_language_include : enable

#define EE_PER_PASS_SET 1
#define EE_PER_GROUP_SET 2
#define EE_PER_COMMAND_SET 3

#include "PerFrame.glsl"

#define EE_MESHLETS_BLOCK_SET 1
#define EE_VERTICES_BLOCK_BINDING 0
#define EE_MESHLETS_BLOCK_BINDING 1
#include "Meshlet.glsl"

struct InstancedData {
	mat4 instance_matrix;
	vec4 color;
};

layout(set = EE_PER_PASS_SET, binding = 0) readonly buffer EE_ANIM_BONES_BLOCK
{
	mat4 EE_ANIM_BONES[];
};

layout(set = EE_PER_PASS_SET, binding = 0) readonly buffer EE_INSTANCED_DATA_BLOCK
{
	InstancedData EE_INSTANCED_DATA[];
};


#include "Strands.glsl"
