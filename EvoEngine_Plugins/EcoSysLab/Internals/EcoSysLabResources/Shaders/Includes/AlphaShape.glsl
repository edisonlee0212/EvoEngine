#extension GL_EXT_control_flow_attributes : require

void SortFourElements(inout uint a[4]) {
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
float CircumsphereRadius(uint indices[4]) {
  // We sort the indices because a different order can lead to a different result due to numerical instability
  // This is important because the order varies depending on which neighbor calls this function, but it must be
  // consistent for a valid alpha-shape
  SortFourElements(indices);

  // first compute circumcenter
  // compare http://rodolphe-vaillant.fr/entry/127/find-a-tetrahedron-circumcenter
  vec3 v[4];

  for (uint i = 0; i < 4; i++) {
    v[i] = uniform_particles[indices[i]].position_t.xyz;
  }

  mat3 A;

  for (uint i = 1; i < 4; i++) {
    A[i - 1] = v[i] - v[0];
  }

  // TODO: could be problematic in degenerate or nearly degenerate cases
  vec3 circumcenter = v[0] + 1 * (2.0f / determinant(A)) *
                                 (dot(A[2], A[2]) * cross(A[0], A[1]) + dot(A[1], A[1]) * cross(A[2], A[0]) +
                                  dot(A[0], A[0]) * cross(A[1], A[2]));

  // test a few different ones for debugging
  vec3 center_to_vertex = v[0] - circumcenter;
  float distance_squared = dot(center_to_vertex, center_to_vertex);

  vec3 center_to_vertex_1 = v[1] - circumcenter;
  float distance_squared_1 = dot(center_to_vertex_1, center_to_vertex_1);

  vec3 center_to_vertex_2 = v[2] - circumcenter;
  float distance_squared_2 = dot(center_to_vertex_2, center_to_vertex_2);

  float max_distance_squared = max(distance_squared, max(distance_squared_1, distance_squared_2));
  float min_distance_squared = min(distance_squared, min(distance_squared_1, distance_squared_2));
  return min_distance_squared;
}

float LongestSide(uint indices[4]) {
  vec3 v[4];

  for (uint i = 0; i < 4; i++) {
    v[i] = uniform_particles[indices[i]].position_t.xyz;
  }
  float  d = 0.0;

  // just compare all sidelengths
  [[unroll]]
  for (uint i = 0; i < 4; i++) {
    [[unroll]]
    for (uint j = i + 1; j < 4; j++) {
      vec3 vij = v[j] - v[i];
      float tmp = dot(vij, vij);
      if (tmp > d) {
        d = tmp;
      }
    }
  }
  return d;
}

bool AreNeighbors(uint index0, uint index1)
{
  int node_handle0 = uniform_particles[index0].node_index;
  int node_handle1 = uniform_particles[index1].node_index;

  // horizontal neighbors
  if (uniform_particles[index0].segment_index == uniform_particles[index1].segment_index &&
      (node_handle0 == node_handle1 ||
       nodes[node_handle0].prev_handle == node_handle1 ||
       node_handle0 == nodes[node_handle1].prev_handle)) {

    // same plane, now take alpha into account
    vec3 vij = uniform_particles[index0].position_t.xyz - uniform_particles[index1].position_t.xyz;
    float dist_squared = dot(vij, vij);
    return dist_squared < alpha;
  }

  // vertical and diagonal neighbors

  // index0 is higher
  if (uniform_particles[index0].segment_index - 1 == uniform_particles[index1].segment_index) {
    int prev_node_handle0 = nodes[node_handle0].prev_handle;
    if(node_handle0 == node_handle1 || prev_node_handle0 == node_handle1) {
      vec3 vij = uniform_particles[index0].position_t.xyz - uniform_particles[index1].position_t.xyz;
      float dist_squared = dot(vij, vij);
      return dist_squared < sqrt(2.0f) * alpha; // account for diagonal (TODO: refine by actual size)
    }

  // index1 is higher
  } else if (uniform_particles[index0].segment_index == uniform_particles[index1].segment_index - 1) {
    int prev_node_handle1 = nodes[node_handle1].prev_handle;
    if (node_handle0 == node_handle1 || node_handle0 == prev_node_handle1) {
      vec3 vij = uniform_particles[index0].position_t.xyz - uniform_particles[index1].position_t.xyz;
      float dist_squared = dot(vij, vij);
      return dist_squared < sqrt(2.0f) * alpha;  // account for diagonal (TODO: refine by actual size)
    }
  }

  return false;

}

float SkeletonStructure(uint indices[4]) {
  vec3 v[4];

  for (uint i = 0; i < 4; i++) {
    v[i] = uniform_particles[indices[i]].position_t.xyz;
  }
  float d = 0.0;

  [[unroll]] for (uint i = 0; i < 4; i++) {
    [[unroll]] for (uint j = i + 1; j < 4; j++) {
      
      if (!AreNeighbors(indices[i], indices[j])) {
        return 2 * alpha; // something certainly larger than alpha
      }
      // TODO: compute longest edge nontheless, maybe filtered by distance, though
    }
  }

  return d;
}

bool InsideAlpha(DelaunayTetrahedron tet, int neighbor_index, out float d) {
  // check if neighbor is invalid
  if (neighbor_index != -1) {
    d = 100000.0f;  // marker for this condition
    // if all indices are at the same distance from root, always return true
    // TODO: we will see how consistent this is
    bool all_same_dist = true;
    [[unroll]] for (uint i = 0; i < 4; i++) {
      if (i == neighbor_index) {
        continue;
      }
      [[unroll]] for (uint j = i + 1; j < 4; j++) {
        if (j == neighbor_index) {
          continue;
        }

        if (uniform_particles[tet.indices[i]].segment_index != uniform_particles[tet.indices[j]].segment_index) {
          all_same_dist = false;
          break;
        }
      }
    }

    if (all_same_dist) {
      return true;
    }

    if (tet.neighbors[neighbor_index] == -1)
    {
      return false;

    }
  }
  //return true; // debug: should give us the convex hull
  // prepare indices
  uint indices[4];

  [[unroll]]
  for (uint i = 0; i < 4; i++) {
    if (i != neighbor_index) {
      indices[i] = tet.indices[i];
    } else {
      indices[i] = tet.neighbors[i];
    }
  }

  //d = LongestSide(indices);
  d = SkeletonStructure(indices);

  return d <= alpha;
}