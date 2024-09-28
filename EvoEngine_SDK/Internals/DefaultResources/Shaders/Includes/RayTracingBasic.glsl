
#extension GL_EXT_ray_tracing : enable

#extension GL_EXT_nonuniform_qualifier : enable
#extension GL_EXT_shader_explicit_arithmetic_types_int8 : require
#extension GL_ARB_shading_language_include : enable

#include "PerFrame.glsl"

#include "Vertex.glsl"
layout(set = 1, binding = 0) readonly buffer EE_VERTICES_BLOCK {
  Vertex EE_VERTICES[];
};

layout(std430, set = 1, binding = 1) readonly buffer EE_INDICES_BLOCK {
  uint EE_INDICES[];
};

layout(set = 1, binding = 2) uniform accelerationStructureEXT EE_TLAS;

