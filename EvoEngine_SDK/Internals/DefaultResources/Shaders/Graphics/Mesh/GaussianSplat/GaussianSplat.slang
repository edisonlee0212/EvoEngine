#extension GL_EXT_mesh_shader : enable
#extension GL_ARB_shading_language_include : enable

#define EE_GAUSSIAN_SPLAT_ENABLE_DRAW_COMMAND 1
#include "GaussianSplat.slangh"

const uint EE_GAUSSIAN_SPLAT_MESH_VERTICES_PER_TASK = EE_GAUSSIAN_SPLAT_MESH_SPLATS_PER_TASK * 4u;
const uint EE_GAUSSIAN_SPLAT_MESH_PRIMITIVES_PER_TASK = EE_GAUSSIAN_SPLAT_MESH_SPLATS_PER_TASK * 2u;

struct EEGaussianSplatMeshOutput {
  vec4 position : SV_Position;
  [[vk::location(0)]]
  vec3 Color;
  [[vk::location(1)]]
  vec2 LocalCoord;
  [[vk::location(2)]]
  nointerpolation float Opacity;
  [[vk::location(3)]]
  nointerpolation float CutoffRadiusSquared;
};

[outputtopology("triangle")]
[numthreads(EE_GAUSSIAN_SPLAT_MESH_VERTICES_PER_TASK, 1, 1)]
[shader("mesh")]
void main(OutputVertices<EEGaussianSplatMeshOutput, 128> vertices, OutputIndices<uint3, 64> triangles) {
  const uint draw_count = EE_GAUSSIAN_SPLAT_DRAW_COUNT();
  const uint task_splat_base = gl_WorkGroupID.x * EE_GAUSSIAN_SPLAT_MESH_SPLATS_PER_TASK;
  const uint active_splats =
      task_splat_base >= draw_count ? 0u : min(EE_GAUSSIAN_SPLAT_MESH_SPLATS_PER_TASK, draw_count - task_splat_base);
  SetMeshOutputCounts(active_splats * 4u, active_splats * 2u);

  const uint vertex_index = gl_LocalInvocationID.x;
  if (vertex_index < active_splats * 4u) {
    const uint splat_offset = vertex_index / 4u;
    const uint corner_index = vertex_index - splat_offset * 4u;
    vec4 clip_position;
    vec3 color;
    vec2 local_coord;
    float opacity;
    float cutoff_radius_squared;
    EE_GAUSSIAN_SPLAT_BUILD_VERTEX(task_splat_base + splat_offset, EE_GAUSSIAN_SPLAT_QUAD_CORNERS[corner_index],
                                   clip_position, color, local_coord, opacity, cutoff_radius_squared);
    vertices[vertex_index].position = clip_position;
    vertices[vertex_index].Color = color;
    vertices[vertex_index].LocalCoord = local_coord;
    vertices[vertex_index].Opacity = opacity;
    vertices[vertex_index].CutoffRadiusSquared = cutoff_radius_squared;
  }

  for (uint primitive_index = vertex_index; primitive_index < active_splats * 2u;
       primitive_index += gl_WorkGroupSize.x) {
    const uint splat_offset = primitive_index / 2u;
    const uint local_primitive = primitive_index - splat_offset * 2u;
    const uint vertex_base = splat_offset * 4u;
    triangles[primitive_index] =
        local_primitive == 0u ? uint3(vertex_base, vertex_base + 1u, vertex_base + 2u)
                              : uint3(vertex_base + 2u, vertex_base + 1u, vertex_base + 3u);
  }
}
