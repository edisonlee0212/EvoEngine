import EcoSysLab.Math;
import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet0;
import EcoSysLab.AlphaShapeTopology;
import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;

public static const uint2 tet_edges[] = {
    uint2(0, 1), uint2(0, 2), uint2(0, 3), uint2(1, 2), uint2(1, 3), uint2(2, 3),
};

public static const uint3 tet_edge_indices[] = {
    uint3(0, 1, 2),
    uint3(0, 3, 4),
    uint3(1, 3, 5),
    uint3(2, 4, 5),
};

public void SortFourElements(inout uint a[4]) {
  uint min1, min2, max1, max2;

  if (a[0] < a[1]) {
    min1 = a[0];
    max1 = a[1];
  } else {
    min1 = a[1];
    max1 = a[0];
  }

  if (a[2] < a[3]) {
    min2 = a[2];
    max2 = a[3];
  } else {
    min2 = a[3];
    max2 = a[2];
  }

  if (min1 < min2) {
    a[0] = min1;
    a[1] = min2;
  } else {
    a[0] = min2;
    a[1] = min1;
  }

  if (max1 > max2) {
    a[3] = max1;
    a[2] = max2;
  } else {
    a[3] = max2;
    a[2] = max1;
  }

  if (a[1] > a[2]) {
    uint tmp = a[1];
    a[1] = a[2];
    a[2] = tmp;
  }
}

// Note: seems to be unstable with a physics simulation
/*float CircumsphereRadius(uint indices[4]) {
  // We sort the indices because a different order can lead to a different result due to numerical instability
  // This is important because the order varies depending on which neighbor calls this function, but it must be
  // consistent for a valid alpha-shape
  SortFourElements(indices);

  // first compute circumcenter
  // compare http://rodolphe-vaillant.fr/entry/127/find-a-tetrahedron-circumcenter
  float3 v[4];

  for (uint i = 0; i < 4; i++) {
    v[i] = uniform_particles[indices[i]].position_t.xyz;
  }

  float3x3 A;

  for (uint i = 1; i < 4; i++) {
    A[i - 1] = v[i] - v[0];
  }

  // TODO: could be problematic in degenerate or nearly degenerate cases
  float3 circumcenter = v[0] + 1 * (2.0f / determinant(A)) *
                                 (dot(A[2], A[2]) * cross(A[0], A[1]) + dot(A[1], A[1]) * cross(A[2], A[0]) +
                                  dot(A[0], A[0]) * cross(A[1], A[2]));

  // test a few different ones for debugging
  float3 center_to_vertex = v[0] - circumcenter;
  float distance_squared = dot(center_to_vertex, center_to_vertex);

  float3 center_to_vertex_1 = v[1] - circumcenter;
  float distance_squared_1 = dot(center_to_vertex_1, center_to_vertex_1);

  float3 center_to_vertex_2 = v[2] - circumcenter;
  float distance_squared_2 = dot(center_to_vertex_2, center_to_vertex_2);

  float max_distance_squared = max(distance_squared, max(distance_squared_1, distance_squared_2));
  float min_distance_squared = min(distance_squared, min(distance_squared_1, distance_squared_2));
  return min_distance_squared;
}

public float LongestSide(uint indices[4]) {
  float3 v[4];

  for (uint i = 0; i < 4; i++) {
    v[i] = uniform_particles[indices[i]].position_t.xyz;
  }
  float  d = 0.0;

  // just compare all sidelengths
  [[unroll]]
  for (uint i = 0; i < 4; i++) {
    [[unroll]]
    for (uint j = i + 1; j < 4; j++) {
      float3 vij = v[j] - v[i];
      float tmp = dot(vij, vij);
      if (tmp > d) {
        d = tmp;
      }
    }
  }
  return d;
}*/

public bool InSameGroup(in UniformParticle p0, in UniformParticle p1) {
  return segments[p0.segment_handle].group_index == segments[p1.segment_handle].group_index;
}

public bool RemainInSameGroup(in DelaunayTetrahedron tet) {
  UniformParticle p[4];
  [[unroll]]
  for (uint i = 0; i < 4; i++) {
    p[i] = uniform_particles[tet.indices[i]];
  }

  bool in_same_group = true;
  [[unroll]] for (uint i = 0; i < 4; i++) {
    [[unroll]] for (uint j = i + 1; j < 4; j++) {
      in_same_group = in_same_group && InSameGroup(p[i], p[j]);
    }
  }
  return in_same_group;
}

public bool AreNeighbors(uint index0, uint index1, float alpha, float bifurcation_alpha, float max_dist_squared)
{
  UniformParticle p0 = uniform_particles[index0];
  UniformParticle p1 = uniform_particles[index1];

  // generally filter for stuff too far apart
  if (DistSquared(p0.initial_position.xyz, p1.initial_position.xyz) > max_dist_squared) {
    return false;
  }

  int node_handle0 = p0.node_index;
  int node_handle1 = p1.node_index;

  // horizontal neighbors
  if (p0.segment_index == p1.segment_index &&
      node_handle0 == node_handle1) {

    // same plane, now take alpha into account
    float3 vij = p0.initial_position.xyz - p1.initial_position.xyz;
    float dist_squared = dot(vij, vij);

    // distinguish bifurcation point
    if (p0.next_node_index == p1.next_node_index) {
      return dist_squared < alpha;
    } else {
      return dist_squared < bifurcation_alpha;
    }
  }

  // vertical neighbors -> always true
  // node handle does not need to checked because the strand is the same
  if (p0.segment_index - 1 == p1.segment_index) { // p0 is higher
    if (p0.prev_particle_handle == index1) {
      return true;
    }
  } else if (p0.segment_index == p1.segment_index - 1) {
    if (index0 == p1.prev_particle_handle) {
      return true;
    }
  }

  // diagonal neighbors
  if (p0.segment_index - 1 == p1.segment_index) {  // p0 is higher
    int prev_node_handle0 = nodes[node_handle0].prev_handle;
    if (node_handle0 == node_handle1 || prev_node_handle0 == node_handle1) {

      // use pythagorean theorem to determine adapted alpha:
      //
      // p0 *
      //    |\
      //    | \ sqrt(adapted_alpha)
      //    |  \
      //    *---* p1
      //  sqrt(alpha)
      //
      float vertical_dist_squared = DistSquared(uniform_particles[p0.prev_particle_handle].initial_position.xyz, p0.initial_position.xyz);
      float dist_squared = DistSquared(p0.initial_position.xyz, p1.initial_position.xyz);

      // distinguish bifurcation point
      if ((node_handle0 == node_handle1 && p0.next_node_index == p1.next_node_index) ||
          (prev_node_handle0 == node_handle1 && node_handle0 == p1.next_node_index)) {
        return dist_squared < alpha + vertical_dist_squared; // = adapted_alpha
      } else {
        //return dist_squared < bifurcation_alpha + vertical_dist_squared; // = adapted_alpha
        return dist_squared < alpha + vertical_dist_squared; // TODO: adapted bifurcation_alpha gave odd results
      }
    }
  } else if (p1.segment_index - 1 == p0.segment_index) {  // p1 is higher
    int prev_node_handle1 = nodes[node_handle1].prev_handle;
    if (node_handle1 == node_handle0 || prev_node_handle1 == node_handle0) {

      // use pythagorean theorem to determine adapted alpha, same as above
      float vertical_dist_squared = DistSquared(uniform_particles[p1.prev_particle_handle].initial_position.xyz, p1.initial_position.xyz);
      float dist_squared = DistSquared(p1.initial_position.xyz, p0.initial_position.xyz);

      // distinguish bifurcation point
      if ((node_handle1 == node_handle0 && p1.next_node_index == p0.next_node_index) ||
          (prev_node_handle1 == node_handle0 && node_handle1 == p0.next_node_index)) {
        return dist_squared < alpha + vertical_dist_squared; // = adapted_alpha
      } else {
        //return dist_squared < bifurcation_alpha + vertical_dist_squared; // = adapted_alpha
        return dist_squared < alpha + vertical_dist_squared;
      }
    }
  }


  return false;

}

public float SkeletonStructure(in DelaunayTetrahedron tet, float alpha, float bifurcation_alpha,
                               float max_dist_squared) {

  float d = 0.0;

  [[unroll]] for (uint i = 0; i < 4; i++) {
    [[unroll]] for (uint j = i + 1; j < 4; j++) {

      if (!AreNeighbors(tet.indices[i], tet.indices[j], alpha, bifurcation_alpha, max_dist_squared)) {
        return 2 * alpha; // something certainly larger than alpha
      }
      // TODO: compute longest edge nontheless, maybe filtered by distance, though
    }
  }

  return d;
}

public bool InsideAlpha(in DelaunayTetrahedron tet, out float d, float alpha, float bifurcation_alpha,
                        float max_dist_squared) {

  //d = LongestSide(indices);
  d = SkeletonStructure(tet, alpha, bifurcation_alpha, max_dist_squared);

  return d <= alpha;
}

public float3 ComputeTriangleNormal(in float3 v0, in float3 v1, in float3 v2) {
  // Compute the two edges of the triangle
  float3 edge1 = v1 - v0;
  float3 edge2 = v2 - v0;

  // Compute the cross product of the two edges to get the normal
  float3 normal = cross(edge1, edge2);

  // Normalize the result to ensure the normal has unit length
  return normalize(normal);
}

public float3 ComputeTetTriangleNormal(in DelaunayTetrahedron tet, uint triangle_index) {

  uint3 triangle = triangles[triangle_index];
  return ComputeTriangleNormal(uniform_particles[tet.indices[lookup[triangle[0]]]].position_t.xyz,
                               uniform_particles[tet.indices[lookup[triangle[1]]]].position_t.xyz,
                               uniform_particles[tet.indices[lookup[triangle[2]]]].position_t.xyz);
}

public bool IsDegenerateTriangle(in float3 v0, in float3 v1, in float3 v2, float epsilon) {
  // Compute the edges of the triangle
  float3 edge1 = v1 - v0;
  float3 edge2 = v2 - v0;

  // Compute the cross product of the edges
  float3 crossProduct = cross(edge1, edge2);

  // Check the squared length of the cross product
  float areaSquared = dot(crossProduct, crossProduct);

  // If the squared area is less than epsilon, the triangle is degenerate
  return areaSquared < epsilon;
}
