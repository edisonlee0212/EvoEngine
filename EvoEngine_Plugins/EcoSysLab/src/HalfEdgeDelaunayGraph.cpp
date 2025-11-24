#include "HalfEdgeDelaunayGraph.hpp"
#include <iostream>

using namespace kinDS;

void HalfEdgeDelaunayGraph::build(const std::vector<size_t>& index_buffer) {
  EVOENGINE_LOG("Building half-edge mesh from triangle index buffer of size " << index_buffer.size());
  assert(index_buffer.size() % 3 == 0 && "Input must be a triangle index buffer.");
  const int num_tris = index_buffer.size() / 3;

  // note that the following reserves are off by one compared to Euler's formula because there is an implicit vertex at
  // infinity that is not counted in the vertex count
  half_edges.clear();
  half_edges.reserve(
      6 * (vertex_count - 1));  // Reserve space for half-edges, at most 6 * (V - 1) half-edges in a triangulation

  triangles.clear();
  triangles.reserve(2 * (vertex_count - 1));  // Reserve space for faces, at most 2 * (V - 1) faces in a triangulation
  triangles.resize(num_tris);

  // Store outgoing edge along boundary from vertex, -1 for non-boundary vertices
  std::vector<int> boundary_edge_map(vertex_count, -1);

  struct TmpHalfEdge {
    size_t v;
    size_t f;
    size_t j;  // index of the half-edge in the triangle
  };

  // Sort up- and down-edges into an adjacency list by the higher vertex index
  std::vector<std::vector<TmpHalfEdge>> adjacency_list_up_in(vertex_count);
  std::vector<std::vector<TmpHalfEdge>> adjacency_list_down_out(vertex_count);

  // iterate through the triangles
  for (size_t i = 0; i < num_tris; ++i) {
    size_t v[3];
    v[0] = index_buffer[i * 3];
    v[1] = index_buffer[i * 3 + 1];
    v[2] = index_buffer[i * 3 + 2];

    // iterate through the triangle's edges
    for (size_t j = 0; j < 3; ++j) {
      size_t a = v[j];
      size_t b = v[(j + 1) % 3];

      if (a > b) {
        adjacency_list_down_out[a].push_back({b, i, j});
      } else {
        adjacency_list_up_in[b].push_back({a, i, j});
      }
    }
  }

  // now sort the opposite way to bring them into order
  std::vector<std::vector<TmpHalfEdge>> adjacency_list_up_out(vertex_count);
  std::vector<std::vector<TmpHalfEdge>> adjacency_list_down_in(vertex_count);

  for (size_t u = 0; u < vertex_count; u++) {
    for (TmpHalfEdge& the : adjacency_list_up_in[u]) {
      adjacency_list_up_out[the.v].push_back({u, the.f, the.j});
    }

    for (TmpHalfEdge& the : adjacency_list_down_out[u]) {
      adjacency_list_down_in[the.v].push_back({u, the.f, the.j});
    }
  }

  // finally, iterate over both adjacency lists to build the half-edges and faces
  for (size_t u = 0; u < vertex_count; u++) {
    // go through both adjacency lists in a merge-like way
    auto up_it = adjacency_list_up_out[u].begin();
    auto down_it = adjacency_list_down_in[u].begin();

    while (up_it != adjacency_list_up_out[u].end() || down_it != adjacency_list_down_in[u].end()) {
      int v_up = (up_it != adjacency_list_up_out[u].end()) ? up_it->v : std::numeric_limits<int>::max();
      int v_down = (down_it != adjacency_list_down_in[u].end()) ? down_it->v : std::numeric_limits<int>::max();

      // always create both half-edges, but if one of them is not present, it will be a boundary half-edge
      HalfEdge he_up;
      he_up.origin = u;
      he_up.face = (v_up <= v_down) ? up_it->f : -1;  // -1 means boundary
      he_up.next = -1;                                // will be set later

      if (he_up.face != -1) {
        triangles[he_up.face].half_edges[up_it->j] = half_edges.size();  // store the half-edge index in the face
      } else {
        boundary_edge_map[u] = half_edges.size();  // store the boundary edge for this vertex
      }
      half_edges.push_back(he_up);

      size_t v = std::min(v_up, v_down);

      HalfEdge he_down;
      he_down.origin = v;
      he_down.face = (v_down <= v_up) ? down_it->f : -1;  // -1 means boundary
      he_down.next = -1;                                  // will be set later

      if (he_down.face != -1) {
        triangles[he_down.face].half_edges[down_it->j] = half_edges.size();  // store the half-edge index in the face
      } else {
        boundary_edge_map[v] = half_edges.size();  // store the boundary edge for this vertex
      }
      half_edges.push_back(he_down);

      // increment the iterators
      if (v_up <= v_down) {
        up_it++;
      }
      if (v_down <= v_up) {
        down_it++;
      }
    }
  }

  // now link the half-edges together
  // triangle edges first
  for (auto& face : triangles) {
    for (size_t j = 0; j < 3; ++j) {
      size_t he_index = face.half_edges[j];
      if (he_index != -1) {
        HalfEdge& he = half_edges[he_index];
        he.next = face.half_edges[(j + 1) % 3];  // link to the next half-edge in the face
      } else {
        EVOENGINE_LOG("Face " << (&face - &triangles[0]) << " has a missing half-edge at index " << j << ".");
      }
    }
  }

  // Store incoming edge from infinity to vertex, -1 for non-boundary vertices
  std::vector<int> incoming_edge_map(vertex_count, -1);

  // at the boundary, create additional faces and half-edges that connect to a vertex at infinity
  for (size_t u = 0; u < vertex_count; ++u) {
    int boundary_edge_index = boundary_edge_map[u];
    if (boundary_edge_index != -1) {
      HalfEdge& he = half_edges[boundary_edge_index];
      size_t v = destination(boundary_edge_index);  // situated between he and next_he
      size_t next_he_id = boundary_edge_map[v];     // next boundary half-edge
      HalfEdge& next_he = half_edges[next_he_id];
      // create new pair of half-edges that connect to a vertex at infinity
      HalfEdge he_infinity;
      he_infinity.origin = v;       // origin is the current vertex
      he_infinity.face = -1;        // -1 means boundary, assign proper id later
      he.next = half_edges.size();  // link the current half-edge to the new one

      HalfEdge he_infinity_twin;
      he_infinity_twin.origin = -1;  // vertex at infinity
      he_infinity_twin.face = -1;    // -1 means boundary, assign proper id later
      he_infinity_twin.next = next_he_id;

      half_edges.push_back(he_infinity);
      // store the incoming edge from infinity so we can later find it
      incoming_edge_map[v] = half_edges.size();
      half_edges.push_back(he_infinity_twin);
    }
  }

  // need another run to create faces and link the half-edges that meet at infinity
  for (size_t u = 0; u < vertex_count; ++u) {
    int incoming_edge_index = incoming_edge_map[u];
    if (incoming_edge_index == -1) {
      continue;  // not a boundary vertex
    }
    HalfEdge& incoming = half_edges[incoming_edge_index];
    HalfEdge& boundary = half_edges[incoming.next];
    assert(incoming.next == boundary_edge_map[u]);
    HalfEdge& outgoing = half_edges[boundary.next];

    outgoing.next = incoming_edge_index;  // link the outgoing half-edge to the incoming one

    // create new face for the boundary edge
    Triangle new_face;
    new_face.half_edges[0] = incoming_edge_index;  // first half-edge is the incoming one
    new_face.half_edges[1] = incoming.next;        // second half-edge is the boundary half-edge
    new_face.half_edges[2] = boundary.next;        // third half-edge is the outgoing one

    incoming.face = triangles.size();  // assign the new face index to the incoming half-edge
    boundary.face = triangles.size();  // assign the new face index to the boundary half-edge
    outgoing.face = triangles.size();  // assign the new face index to the outgoing half-edge

    triangles.push_back(new_face);  // add the new face to the list
  }

  // iterate over all edges to set vertex to half-edge mapping
  for (size_t he_id = 0; he_id < half_edges.size(); ++he_id) {
    const HalfEdge& he = half_edges[he_id];
    if (he.origin != -1) {
      vertex_to_half_edge[he.origin] = he_id;
    }
  }

  EVOENGINE_LOG("Half-edge mesh built with " << half_edges.size() << " half-edges and " << triangles.size()
                                             << " faces.");
}

void kinDS::HalfEdgeDelaunayGraph::flipEdge(size_t he_id) {
  if (he_id >= half_edges.size()) {
    EVOENGINE_ERROR("Invalid half-edge ID for flipping: " << he_id);
    return;
  }
  HalfEdge& he = half_edges[he_id];
  HalfEdge& twin = half_edges[he_id ^ 1];

  size_t u = he.origin;    // origin vertex of the half-edge
  size_t v = twin.origin;  // origin vertex of the twin half-edge

  // Check if we can flip the edge
  if (he.face == -1 || twin.face == -1) {
    EVOENGINE_ERROR("Cannot flip edge " << he_id << " because one of the triangles is a boundary triangle.");
    return;
  }

  // check if edge is referenced in vertex_to_half_edge and update
  if (vertex_to_half_edge[u] == he_id) {
    // just set to next half-edge on the vertex
    vertex_to_half_edge[u] = neighborEdgeId(he_id);
  }

  if (vertex_to_half_edge[v] == HalfEdgeDelaunayGraph::twin(he_id)) {
    vertex_to_half_edge[v] = neighborEdgeId(HalfEdgeDelaunayGraph::twin(he_id));  // update to point to the half-edge
  }

  int he_next_id = he.next;
  int twin_next_id = twin.next;

  size_t he_last_id = half_edges[he_next_id].next;      // The next half-edge in the triangle
  size_t twin_last_id = half_edges[twin_next_id].next;  // The next half-edge in the twin triangle

  size_t he_opposite_id = triangleOppositeVertex(he_id);
  size_t twin_opposite_id = triangleOppositeVertex(he_id ^ 1);

  // Update the half-edges to flip the edge
  he.origin = twin_opposite_id;
  twin.origin = he_opposite_id;

  // Update the next pointers
  // bridge the two triangles where the edge used to be
  half_edges[he_last_id].next = twin_next_id;
  half_edges[twin_last_id].next = he_next_id;

  // intercept at the new end points of the flipped edge
  half_edges[he_next_id].next = he_id ^ 1;
  half_edges[twin_next_id].next = he_id;

  // finally, update the twin pointers
  half_edges[he_id].next = he_last_id;        // the half-edge now points to the twin
  half_edges[he_id ^ 1].next = twin_last_id;  // the twin of he now points to the next half-edge

  // Update the faces
  std::swap(half_edges[he_next_id].face, half_edges[twin_next_id].face);

  // Update the half-edge indices in the faces
  triangles[he.face].half_edges[0] = he_id;         // Update the first half-edge of the face
  triangles[he.face].half_edges[1] = he_last_id;    // Update the second half-edge of the face
  triangles[he.face].half_edges[2] = twin_next_id;  // Update the third half-edge of the face

  triangles[twin.face].half_edges[0] = he_id ^ 1;     // Update the first half-edge of the twin face
  triangles[twin.face].half_edges[1] = twin_last_id;  // Update the second half-edge of the twin face
  triangles[twin.face].half_edges[2] = he_next_id;    // Update the third half-edge of the twin face

  EVOENGINE_LOG("Flipped edge " << he_id << " between vertices " << u << " and " << v << ".");

  // printDebug();
}

void HalfEdgeDelaunayGraph::printDebug() const {
  std::cout << "Half-Edges:\n";
  for (size_t i = 0; i < half_edges.size(); ++i) {
    const HalfEdge& he = half_edges[i];
    std::cout << "  [" << i << "] origin = " << he.origin << ", next = " << he.next << ", face = " << he.face
              << ", twin = " << (i ^ 1) << "\n";
  }

  std::cout << "\nFaces:\n";
  for (size_t i = 0; i < triangles.size(); ++i) {
    const Triangle& f = triangles[i];
    std::cout << "  [" << i << "] half_edge = " << f.half_edges[0] << "\n";

    // Walk the face's boundary
    int start = f.half_edges[0];
    if (start < 0)
      continue;
    int he = start;
    std::cout << "    Vertices: ";
    do {
      std::cout << half_edges[he].origin << " ";
      he = half_edges[he].next;
    } while (he != start && he != -1);
    std::cout << "\n";

    he = start;
    std::cout << "    Half-Edges: ";
    do {
      std::cout << he << " ";
      he = half_edges[he].next;
    } while (he != start && he != -1);
    std::cout << "\n";
  }

  std::cout << "Vertex incident half-edges:\n";
  for (size_t u = 0; u < vertex_count; ++u) {
    std::cout << " Outgoing edges from vertex " << u << ":\n";

    for (IncidentEdgeIterator it = incidentEdgesBegin(u); it != incidentEdgesEnd(u); ++it) {
      size_t he_id = *it;
      const HalfEdge& he = half_edges[he_id];
      std::cout << "  Half-edge " << he_id << " to vertex " << destination(he_id) << " in face " << he.face << "\n";
    }
  }
}

void HalfEdgeDelaunayGraph::init(const std::vector<CubicHermiteSpline<2>>& splines) {
  vertex_count = splines.size();
  vertex_to_half_edge.assign(vertex_count, -1);
  std::vector<float> coords;
  coords.reserve(splines.size() * 2);  // Reserve space for x and y coordinates
  for (const auto& spline : splines) {
    Point<2> point = spline.evaluate(0.0);
    coords.push_back(point[0]);
    coords.push_back(point[1]);
  }

  Delaunator::Delaunator2D delaunator(coords);

  build(delaunator.triangles);
}

Point<2> HalfEdgeDelaunayGraph::circumcenter(const Point<2>& a, const Point<2>& b, const Point<2>& c) {
  // Calculate the circumcenter of the triangle formed by points a, b, c
  double D = 2 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]));
  if (D == 0) {
    // Degenerate case, return a point at infinity
    EVOENGINE_ERROR("Circumcenter calculation failed due to zero denominator. Points may be collinear.");
    return Point<2>{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  }
  double Ux = ((a[0] * a[0] + a[1] * a[1]) * (b[1] - c[1]) + (b[0] * b[0] + b[1] * b[1]) * (c[1] - a[1]) +
               (c[0] * c[0] + c[1] * c[1]) * (a[1] - b[1])) /
              D;
  double Uy = ((a[0] * a[0] + a[1] * a[1]) * (c[0] - b[0]) + (b[0] * b[0] + b[1] * b[1]) * (a[0] - c[0]) +
               (c[0] * c[0] + c[1] * c[1]) * (b[0] - a[0])) /
              D;
  return {Ux, Uy};
}

std::vector<std::pair<Point<2>, bool>> HalfEdgeDelaunayGraph::computeCircumcenters(
    const std::vector<Point<2>>& vertices) const {
  // give either the position of the circumcenter or a direction vector if the triangle is infinite, the boolean
  // indicates if the circumcenter is infinite
  std::vector<std::pair<Point<2>, bool>> circumcenters(triangles.size());

  for (size_t triangle_id = 0; triangle_id < triangles.size(); triangle_id++) {
    const Triangle& triangle = triangles[triangle_id];
    const HalfEdge& he0 = half_edges[triangle.half_edges[0]];
    const HalfEdge& he1 = half_edges[triangle.half_edges[1]];
    const HalfEdge& he2 = half_edges[triangle.half_edges[2]];

    // Filter for infinity vertices
    if (he0.origin == -1) {
      const Point<2>& v1 = vertices[he1.origin];
      const Point<2>& v2 = vertices[he2.origin];
      const Point<2> dir = v2 - v1;
      circumcenters[triangle_id] = {Point<2>{dir[1], -dir[0]}, true};
      continue;
    }

    if (he1.origin == -1) {
      const Point<2>& v0 = vertices[he0.origin];
      const Point<2>& v2 = vertices[he2.origin];
      const Point<2> dir = v0 - v2;
      circumcenters[triangle_id] = {Point<2>{dir[1], -dir[0]}, true};
      continue;
    }

    if (he2.origin == -1) {
      const Point<2>& v0 = vertices[he0.origin];
      const Point<2>& v1 = vertices[he1.origin];
      const Point<2> dir = v1 - v0;
      circumcenters[triangle_id] = {Point<2>{dir[1], -dir[0]}, true};
      continue;
    }

    // Get the vertices of the triangle
    const Point<2>& v0 = vertices[he0.origin];
    const Point<2>& v1 = vertices[he1.origin];
    const Point<2>& v2 = vertices[he2.origin];
    // Compute the circumcenter of the triangle
    circumcenters[triangle_id] = {circumcenter(v0, v1, v2), false};
  }

  return circumcenters;
}

// utils
bool HalfEdgeDelaunayGraph::isOutsideBoundary(size_t he_id) const {
  // walk the triangle and check if any vertex is -1
  for (size_t i = 0; i < 3; i++) {
    if (half_edges[he_id].origin == -1) {
      return true;
    }
    he_id = half_edges[he_id].next;
  }

  return false;
}

bool HalfEdgeDelaunayGraph::isOnBoundary(size_t he_id) const {
  // XOR this as one half-edge will be inside and the other one outside
  return isOutsideBoundary(he_id) != isOutsideBoundary(he_id ^ 1);
}

bool kinDS::HalfEdgeDelaunayGraph::isOnBoundaryOutside(size_t he_id) const {
  return isOutsideBoundary(he_id) && isOnBoundary(he_id);
}

bool kinDS::HalfEdgeDelaunayGraph::isInfinite(size_t he_id) const {
  return (half_edges[he_id].origin == -1) || (destination(he_id) == -1);
}

int HalfEdgeDelaunayGraph::destination(size_t he_id) const {
  return half_edges[he_id ^ 1].origin;
}

int HalfEdgeDelaunayGraph::triangleOppositeVertex(size_t he_id) const {
  // Returns the vertex opposite to the half-edge in its triangle
  size_t next_he_id = half_edges[he_id].next;
  next_he_id = half_edges[next_he_id].next;
  return half_edges[next_he_id].origin;
}

std::array<int, 3> HalfEdgeDelaunayGraph::adjacentTriangleVertices(size_t he_id) const {
  // Returns the vertices of the triangle that the half-edge belongs to
  std::array<int, 3> vertices;
  for (size_t i = 0; i < 3; i++) {
    vertices[i] = half_edges[he_id].origin;
    he_id = half_edges[he_id].next;
  }
  return vertices;
}

size_t kinDS::HalfEdgeDelaunayGraph::neighborEdgeId(size_t he_id) const {
  return half_edges[twin(he_id)].next;
}

size_t kinDS::HalfEdgeDelaunayGraph::nextOnBoundaryId(size_t he_id) const {
  size_t next_he_id = half_edges[he_id].next;
  next_he_id = twin(next_he_id);
  next_he_id = half_edges[next_he_id].next;
  return next_he_id;
}

size_t HalfEdgeDelaunayGraph::twin(size_t he_id) {
  return he_id ^ 1;
}

// getters
const std::vector<HalfEdgeDelaunayGraph::HalfEdge>& HalfEdgeDelaunayGraph::getHalfEdges() const {
  return half_edges;
}

const std::vector<HalfEdgeDelaunayGraph::Triangle>& HalfEdgeDelaunayGraph::getFaces() const {
  return triangles;
}

size_t HalfEdgeDelaunayGraph::getVertexCount() const {
  return vertex_count;
}