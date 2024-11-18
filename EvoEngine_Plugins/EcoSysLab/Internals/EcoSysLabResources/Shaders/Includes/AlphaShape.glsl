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

bool InsideAlpha(DelaunayTetrahedron tet, int neighbor_index, out float max_dist_squared) {
  // check if neighbor is invalid
  if (neighbor_index != -1 && tet.neighbors[neighbor_index] == -1) {
    max_dist_squared = 100000.0f;  // marker for this condition
    return false;
  }
  //return true; // debug: should give us the convex hull
  // prepare indices
  uint indices[4];
  for (uint i = 0; i < 4; i++) {
    if (i != neighbor_index) {
      indices[i] = tet.indices[i];
    } else {
      indices[i] = tet.neighbors[i];
    }
  }

  // We sort the indices because a different order can lead to a different result due to numerical instability
  // This is important because the order varies depending on which neighbor calls this function, but it must be
  // consistent for a valid alpha-shape

  /*
  SortFourElements(indices);

  // first compute circumcenter
  // compare http://rodolphe-vaillant.fr/entry/127/find-a-tetrahedron-circumcenter
  vec3 v[4];

  for (uint i = 0; i < 4; i++) {
    v[i] = particles[indices[i]].x_node_handle.xyz;
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
  min_distance_squared = min(distance_squared, min(distance_squared_1, distance_squared_2));*/

  vec3 v[4];

  for (uint i = 0; i < 4; i++) {
    v[i] = particles[indices[i]].x_node_handle.xyz;
  }
  max_dist_squared = 0.0;

  // just compare all sidelengths
  for (uint i = 0; i < 4; i++) {
    for (uint j = i + 1; j < 4; j++)
    {
      vec3 vij = v[j] - v[i];
      float tmp = dot(vij, vij);
      if (tmp > max_dist_squared) {
        max_dist_squared = tmp;
      }
    }
  }

  return max_dist_squared <= alpha;
}