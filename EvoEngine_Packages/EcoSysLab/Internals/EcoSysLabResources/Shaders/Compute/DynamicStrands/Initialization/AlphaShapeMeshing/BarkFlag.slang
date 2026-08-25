import EcoSysLab.AlphaShape;
import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet0;
import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint ITERATIONS_PER_TASK =
    (COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE;
static const uint2 TET_EDGES[] = {
    uint2(0, 1), uint2(0, 2), uint2(0, 3), uint2(1, 2), uint2(1, 3), uint2(2, 3),
};

struct PushConstants {
  uint tetrahedrons_size;
  float alpha;
  float bifurcation_alpha;
  float max_dist_squared;
};

[[vk::push_constant]] ConstantBuffer<PushConstants> constants;

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint base_id = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  [[unroll]] for (uint i = 0; i < ITERATIONS_PER_TASK; ++i) {
    const uint tetrahedron_index = base_id + lane_id + i * WORKGROUP_SIZE;
    if (tetrahedron_index >= constants.tetrahedrons_size)
      break;

    DelaunayTetrahedron tetrahedron = delaunay_tetrahedrons[tetrahedron_index];
    for (int j = 0; j < 4; ++j) {
      UniformParticle particle = uniform_particles[tetrahedron.indices[j]];
      if ((particle.position_t.w == 0.0f && segments[particle.segment_handle].prev_handle == -1) ||
          (particle.position_t.w == 1.0f && segments[particle.segment_handle].next_handle == -1))
        uniform_particles[tetrahedron.indices[j]].is_on_surface = 2;
    }

    if (tetrahedron.inside == 1) {
      [[unroll]] for (uint j = 0; j < 4; ++j)
        uniform_particles[tetrahedron.indices[j]].is_bark = 0;

      [[unroll]] for (int triangle_index = 0; triangle_index < 4; ++triangle_index) {
        if (tetrahedron.neighbor_tet_ids[triangle_index] != -1) {
          DelaunayTetrahedron neighbor = delaunay_tetrahedrons[tetrahedron.neighbor_tet_ids[triangle_index]];
          tetrahedron.is_bark[triangle_index] = 1 - neighbor.inside;
        } else {
          tetrahedron.is_bark[triangle_index] = 1;
        }

        bool triangle_on_end = true;
        [[unroll]] for (int j = 0; j < 4; ++j) {
          if (triangle_index != j)
            triangle_on_end = triangle_on_end && uniform_particles[tetrahedron.indices[j]].is_on_surface == 2;
        }

        if (tetrahedron.is_bark[triangle_index] == 1) {
          const float3 normal = ComputeTetTriangleNormal(tetrahedron, triangle_index);
          [[unroll]] for (int j = 0; j < 4; ++j) {
            if (j != triangle_index) {
              InterlockedAdd(uniform_particles[tetrahedron.indices[j]].normal_deg.x, normal.x);
              InterlockedAdd(uniform_particles[tetrahedron.indices[j]].normal_deg.y, normal.y);
              InterlockedAdd(uniform_particles[tetrahedron.indices[j]].normal_deg.z, normal.z);
            }
          }
          tetrahedron.is_bark[triangle_index] = triangle_on_end ? 0 : 1;
        }
      }

      float3 vertices[4];
      [[unroll]] for (int vertex = 0; vertex < 4; ++vertex)
        vertices[vertex] = uniform_particles[tetrahedron.indices[vertex]].initial_position.xyz;
      [[unroll]] for (int edge = 0; edge < 6; ++edge)
        tetrahedron.sidelengths[edge] =
            sqrt(DistSquared(vertices[TET_EDGES[edge].x], vertices[TET_EDGES[edge].y]));
    } else {
      [[unroll]] for (int triangle_index = 0; triangle_index < 4; ++triangle_index)
        tetrahedron.is_bark[triangle_index] = 0;
    }
    delaunay_tetrahedrons[tetrahedron_index] = tetrahedron;
  }
}
