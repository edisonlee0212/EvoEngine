import EcoSysLab.Math;
import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet0;
import EcoSysLab.AlphaShape;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint delaunay_tetrahedron_size;
  float alpha;
  float bifurcation_alpha;
  float max_dist_squared;
  int render_complex;
  float degen_triangle_threshold;
  float break_threshold;
  int persistent_damage;
};
bool VertexConnected(in DelaunayTetrahedron delaunay_tetrahedron, uint vertex_index) {
  int connected = 0;
  int disconnected = 0;
  // int unknown = 0;

  [[unroll]]
  for (int i = 0; i < 3; i++) {
    int segment_pair_index = delaunay_tetrahedron.segment_pair_index[tet_edge_indices[vertex_index][i]];
    if (segment_pair_index == -1) {
      // unknown++;
      continue;
    }

    SegmentPair segment_pair = segment_pairs[segment_pair_index];
    if (segment_pair.connectivity_integrity > 0.0f || segment_pair.bend_twist_bundle_integrity > 0.0f) {
      connected++;
    } else {
      disconnected++;
    }
  }

  // Note: this connection can be skipped for more aggressive mesh breaking
  if (connected > 0) {
    return true;
  }

  return (disconnected == 0);
}

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  float min_distance_squared;

  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint delaunay_tetrahedron_index = baseID + local_index;

    if (delaunay_tetrahedron_index >= delaunay_tetrahedron_size)
      break;

    DelaunayTetrahedron delaunay_tetrahedron = delaunay_tetrahedrons[delaunay_tetrahedron_index];

    // reset flags; TODO: move to separate shader for more efficiency
    [[unroll]]
    for (uint j = 0; j < 4; ++j) {
      uniform_particles[delaunay_tetrahedron.indices[j]].is_single_strand_particle = 1;
      if (uniform_particles[delaunay_tetrahedron.indices[j]].is_on_surface == 0) {
        uniform_particles[delaunay_tetrahedron.indices[j]].normal_deg = float4(0.0, 0.0, 0.0, 0.0);
      }
    }

    if (persistent_damage == 0 && delaunay_tetrahedron.inside_at_init == 1) {
      delaunay_tetrahedron.inside = 1;
    }

    // If a tetrahedron is ever outside the interior, it can never become part of the inside again
    if (delaunay_tetrahedron.inside == 1) {
      delaunay_tetrahedron.inside = int(RemainInSameGroup(delaunay_tetrahedron));
    }

    // Now check against stretch factor
    if (delaunay_tetrahedron.inside == 1) {
      // get vertices
      float3 v[4];
      [[unroll]]
      for (int k = 0; k < 4; ++k) {
        v[k] = uniform_particles[delaunay_tetrahedron.indices[k]].position_t.xyz;
      }

      // Compute sidelengths
      [[unroll]]
      for (int edge_index = 0; edge_index < 6; ++edge_index) {
        float sidelength_squared = DistSquared(v[tet_edges[edge_index].x], v[tet_edges[edge_index].y]);

        if (sqrt(sidelength_squared) > break_threshold + delaunay_tetrahedron.sidelengths[edge_index]) {
          delaunay_tetrahedron.inside = 0;
          break;
        }
      }
    }

    // Check against Segment pairs
    if (delaunay_tetrahedron.inside == 1) {
      [[unroll]]
      for (int k = 0; k < 4; ++k) {
        if (VertexConnected(delaunay_tetrahedron, k) == false) {
          delaunay_tetrahedron.inside = 0;
          break;
        }
      }
    }

    delaunay_tetrahedrons[delaunay_tetrahedron_index] = delaunay_tetrahedron;
  }
}
