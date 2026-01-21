#include "SegmentBuilder.hpp"
#include "PolygonIntersection.hpp"
#include "assimp/code/AssetLib/3MF/3MFXmlTags.h"

using namespace kinDS;

static bool raySegmentIntersection(const VoronoiPoint<2>& C, const VoronoiPoint<2>& D, const VoronoiPoint<2>& A,
                                   const VoronoiPoint<2>& B, double& t_out) {
  VoronoiPoint<2> E = B - A;
  VoronoiPoint<2> AC = A - C;

  double det = D % E;
  if (std::abs(det) < 1e-12)
    return false;

  double t = (AC % E) / det;
  double u = (AC % D) / det;

  if (t >= 0.0 && u >= 0.0 && u <= 1.0) {
    t_out = t;
    return true;
  }
  return false;
}

static std::vector<double> rayCast(const std::vector<BoundaryPoint>& polygon, const VoronoiPoint<2>& origin,
                                   const VoronoiPoint<2>& dir) {
  double lenCP = dir.len();

  if (lenCP < 1e-12)
    return {};

  double t_max = 0.0;

  std::vector<double> hits;
  const size_t n = polygon.size();
  for (size_t i = 0; i < n; ++i) {
    const VoronoiPoint<2>& A = polygon[i].p;
    const VoronoiPoint<2>& B = polygon[(i + 1) % n].p;

    double t;
    if (raySegmentIntersection(origin, dir, A, B, t)) {
      hits.emplace_back(t);
    }
  }

  return hits;
}

static std::vector<double> rayCast(const std::vector<VoronoiPoint<2>>& polygon, const VoronoiPoint<2>& origin,
                                   const VoronoiPoint<2>& dir) {
  double lenCP = dir.len();

  if (lenCP < 1e-12)
    return {};

  double t_max = 0.0;

  std::vector<double> hits;
  const size_t n = polygon.size();
  for (size_t i = 0; i < n; ++i) {
    const VoronoiPoint<2>& A = polygon[i];
    const VoronoiPoint<2>& B = polygon[(i + 1) % n];

    double t;
    if (raySegmentIntersection(origin, dir, A, B, t)) {
      hits.emplace_back(t);
    }
  }

  return hits;
}

static double relativeDistanceFromCenter(const std::vector<BoundaryPoint>& polygon, const VoronoiPoint<2>& center,
                                         const VoronoiPoint<2>& point) {
  VoronoiPoint<2> dir = point - center;
  auto hits = rayCast(polygon, center, dir);

  if (hits.empty())
    return std::numeric_limits<double>::quiet_NaN();

  double t_max = -std::numeric_limits<double>::infinity();

  for (double t : hits) {
    t_max = std::max(t_max, t);
  }

  if (t_max < 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  // |B - C| = t_max * |D|
  return 1.0 / t_max;
}

static bool isInside(const std::vector<BoundaryPoint>& polygon, const VoronoiPoint<2>& center,
                     const VoronoiPoint<2>& point) {
  double rel_dist = relativeDistanceFromCenter(polygon, center, point);
  if (std::isnan(rel_dist)) {
    throw std::runtime_error("Center lies outside of polygon");
  }
  return rel_dist <= 1.0;
}

[[nodiscard]] VoronoiPoint<3> kinDS::SegmentBuilder::computeVoronoiVertex(size_t half_edge_id, double t,
                                                                          size_t segment_mesh_pair_index) const {
  const auto& graph = kin_del.getGraph();
  const auto& half_edges = graph.getHalfEdges();
  const auto& he = half_edges[half_edge_id];
  const auto& twin_he = half_edges[half_edge_id ^ 1];

  // Compute the positions of the Voronoi vertices at t = 0.0
  // First get the two adjacent triangles
  std::array<int, 3> triVertices = graph.adjacentTriangleVertices(half_edge_id);

  // now compute the circumcenters if the triangles are not infinite
  VoronoiPoint<2> circumcenter;

  bool infinite = false;

  std::vector<VoronoiPoint<2>> points;
  std::vector<size_t> vertex_indices;

  size_t infinite_vertex_index = -1;

  for (size_t i = 0; i < 3; ++i) {
    if (triVertices[i] != -1) {
      points.push_back(kin_del.getPointAt(t, triVertices[i]));
      vertex_indices.push_back(triVertices[i]);
    } else {
      infinite_vertex_index = i;
    }
  }

  if (points.size() == 3) {
    circumcenter = graph.circumcenter(points[0], points[1], points[2]);
    // circumcenter = (points[0] + points[1] + points[2]) / 3.0;
  } else {
    infinite = true;

    // get the triangle on the opposite side of the non-infinite edge
    size_t finite_he_id = half_edge_id;

    while (half_edges[finite_he_id].origin != -1) {
      finite_he_id = half_edges[finite_he_id].next;
    }
    finite_he_id = half_edges[finite_he_id].next;
    size_t inner_twin = graph.twin(finite_he_id);
    size_t opposite_vertex = graph.triangleOppositeVertex(inner_twin);
    VoronoiPoint<2> opposite_point = kin_del.getPointAt(t, opposite_vertex);

    VoronoiPoint<2> neighboring_circumcenter = graph.circumcenter(points[0], points[1], opposite_point);
    // VoronoiPoint<2> neighboring_circumcenter = (points[0] + points[1] + opposite_point) / 3.0;

    // make sure edge points in the correct direction
    if (triVertices[1] == -1) {
      std::swap(points[0], points[1]);
    }

    // For now just take the midpoint of the edge
    // circumcenter = (points[0] + points[1]) * 0.5;

    // move circumcenter far out in the direction perpendicular to the edge
    VoronoiVector<2> edge_dir = (points[1] - points[0]).normalized();
    VoronoiVector<2> perp_dir = VoronoiVector<2>{-edge_dir[1], edge_dir[0]};

    // compute intersection with the boundary

    std::vector<double> hits;

    if (kin_del.getDummyBoundary().empty()) {
      size_t component_id = kin_del.component_data.component_map[vertex_indices[0]];
      auto& boundary = kin_del.component_data.component_boundaries[component_id][0];
      hits = rayCast(boundary, neighboring_circumcenter, perp_dir);
    } else {
      hits = rayCast(kin_del.getDummyBoundary(), neighboring_circumcenter, perp_dir);
    }

    double t_min = std::numeric_limits<double>::infinity();

    for (auto t : hits) {
      // negative hits are not permitted
      if (t >= 0) {
        t_min = std::min(t, t_min);
      }
    }

    if (t_min == std::numeric_limits<double>::infinity()) {
      EVOENGINE_ERROR("Found no suitable hit!");
    }

    circumcenter = neighboring_circumcenter - perp_dir * t_min;
  }

  // place circumcenters into the mesh
  return VoronoiPoint<3>{circumcenter[0], circumcenter[1], t};
}

bool clampVoronoiVertices(VoronoiPoint<3>& left_vertex, VoronoiPoint<3>& right_vertex,
                          const std::vector<BoundaryPoint>& boundary_points, const VoronoiPoint<2>& centroid) {
  // don't do this for now
  return true;

  bool left_inside = isInside(boundary_points, centroid, VoronoiPoint<2>{left_vertex[0], left_vertex[1]});
  bool right_inside = isInside(boundary_points, centroid, VoronoiPoint<2>{right_vertex[0], right_vertex[1]});

  if (left_inside && !right_inside) {
    EVOENGINE_LOG("Clamping right vertex: (" << right_vertex[0] << ", " << right_vertex[1] << ", " << right_vertex[2]
                                             << ")");
    // clamp right to the boundary
    VoronoiPoint<2> origin{left_vertex[0], left_vertex[1]};
    VoronoiVector<2> dir{right_vertex[0] - left_vertex[0], right_vertex[1] - left_vertex[1]};
    auto hits = rayCast(boundary_points, origin, dir);

    double s_min = std::numeric_limits<double>::infinity();

    for (double s : hits) {
      if (s >= 0) {
        s_min = std::min(s_min, s);
      }
    }

    auto right_vertex_2d = origin + s_min * dir;
    right_vertex = VoronoiPoint<3>{right_vertex_2d[0], right_vertex_2d[1], right_vertex[2]};
    EVOENGINE_LOG("Clamped right vertex to: (" << right_vertex[0] << ", " << right_vertex[1] << ", " << right_vertex[2]
                                               << ")");

  } else if (!left_inside && right_inside) {
    EVOENGINE_LOG("Clamping left vertex: (" << left_vertex[0] << ", " << left_vertex[1] << ", " << left_vertex[2]
                                            << ")");
    // clamp left to the boundary
    VoronoiPoint<2> origin{right_vertex[0], right_vertex[1]};
    VoronoiVector<2> dir{left_vertex[0] - right_vertex[0], left_vertex[1] - right_vertex[1]};
    auto hits = rayCast(boundary_points, origin, dir);

    double s_min = std::numeric_limits<double>::infinity();

    for (double s : hits) {
      if (s >= 0) {
        s_min = std::min(s_min, s);
      }
    }

    auto left_vertex_2d = origin + s_min * dir;
    left_vertex = VoronoiPoint<3>{left_vertex_2d[0], left_vertex_2d[1], left_vertex[2]};
    EVOENGINE_LOG("Clamped left vertex to: (" << left_vertex[0] << ", " << left_vertex[1] << ", " << left_vertex[2]
                                              << ")");
  } else if (!left_inside && !right_inside) {
    // I'm not sure yet if this will work, especially for connections, but for now we will just discard it
    return false;
  }
  return true;
}

void kinDS::SegmentBuilder::finishMesh(size_t he_id, double t, const std::vector<BoundaryPoint>& boundary_points) {
  size_t segment_mesh_pair_index = half_edge_index_to_segment_mesh_pair_index[he_id];
  // Get corresponding mesh
  VoronoiMesh& mesh = meshes[segment_mesh_pair_index];
  // Insert Voronoi vertex

  VoronoiPoint<3> left_vertex = computeVoronoiVertex(he_id & ~1, t, segment_mesh_pair_index);
  VoronoiPoint<3> right_vertex = computeVoronoiVertex((he_id & ~1) + 1, t, segment_mesh_pair_index);
  auto& he = kin_del.getGraph().getHalfEdges()[he_id & ~1];
  VoronoiPoint<2> centroid = polygonCentroid(boundary_points);

  clampVoronoiVertices(left_vertex, right_vertex, boundary_points, centroid);

  size_t v = he.origin;
  if (v == -1) {
    v = kin_del.getGraph().destination(he_id & ~1);
  }

  // TODO: Compute UVs here
  size_t new_left_vertex_index = addMeshletVertex(mesh, boundary_points, centroid, left_vertex, v, t);
  size_t new_right_vertex_index = addMeshletVertex(mesh, boundary_points, centroid, right_vertex, v, t);
  // build triangles
  const auto& last_vertices = segment_mesh_pair_last_left_and_right_vertex[segment_mesh_pair_index];
  // create two triangles
  // split quad differently depending on which side is closer
  if (last_vertices.first == last_vertices.second) {
    addMeshletTriangle(mesh, new_left_vertex_index, last_vertices.second, new_right_vertex_index);
  } else if (mesh.getVertices()[last_vertices.first][2] < mesh.getVertices()[last_vertices.second][2]) {
    addMeshletTriangle(mesh, last_vertices.first, last_vertices.second, new_left_vertex_index);
    addMeshletTriangle(mesh, new_left_vertex_index, last_vertices.second, new_right_vertex_index);
  } else {
    addMeshletTriangle(mesh, last_vertices.first, last_vertices.second, new_right_vertex_index);
    addMeshletTriangle(mesh, last_vertices.first, new_right_vertex_index, new_left_vertex_index);
  }

  // update last vertex indices
  segment_mesh_pair_last_left_and_right_vertex[segment_mesh_pair_index] =
      std::make_pair(new_left_vertex_index, new_right_vertex_index);
}

SegmentBuilder::SegmentBuilder(KineticDelaunay& kin_del, std::vector<std::pair<size_t, double>> subdivisions,
                               bool create_transformed_mesh)
    : kin_del(kin_del),
      subdivisions(std::move(subdivisions)),
      create_transformed_mesh(create_transformed_mesh),
      boundary_mesh({"bark", "interior"}) {
  // Assert that the subdivisions are sorted by time
  assert(std::is_sorted(this->subdivisions.begin(), this->subdivisions.end(), [](const auto& a, const auto& b) {
    return a.second < b.second;
  }));
}

SegmentBuilder::SegmentBuilder(KineticDelaunay& kin_del, bool create_transformed_mesh)
    : kin_del(kin_del), create_transformed_mesh(create_transformed_mesh), boundary_mesh({"bark", "interior"}) {
}

void SegmentBuilder::startNewMesh(size_t half_edge_id, double t) {
  size_t even_id = half_edge_id & ~1;
  size_t odd_id = even_id + 1;

  const auto& graph = kin_del.getGraph();
  const auto& he = graph.getHalfEdges()[even_id];
  const auto& twin_he = graph.getHalfEdges()[odd_id];

  MeshStructure::SegmentMeshPair segment_mesh_pair;
  segment_mesh_pair.segment_index0 = he.origin == -1 ? -1 : strand_to_segment_indices[he.origin].back();
  segment_mesh_pair.segment_index1 = twin_he.origin == -1 ? -1 : strand_to_segment_indices[twin_he.origin].back();

  half_edge_index_to_segment_mesh_pair_index[even_id] = segment_mesh_pairs.size();
  half_edge_index_to_segment_mesh_pair_index[odd_id] = segment_mesh_pairs.size();

  segment_mesh_pairs.push_back(segment_mesh_pair);

  // For now also create a mesh, but this might be changed later
  VoronoiMesh mesh;

  VoronoiPoint<3> left_vertex = computeVoronoiVertex(even_id, t, half_edge_index_to_segment_mesh_pair_index[even_id]);
  VoronoiPoint<3> right_vertex = computeVoronoiVertex(odd_id, t, half_edge_index_to_segment_mesh_pair_index[even_id]);

  size_t vertex = std::max(he.origin, twin_he.origin);
  size_t component_id = kin_del.component_data.component_map[vertex];

  std::vector<bool> he_visited(graph.getHalfEdges().size(), false);
  updateBoundary(t, he_visited, component_id);

  auto& boundary_polygon = kin_del.component_data.component_boundaries[component_id][0];
  auto& centroid = kin_del.component_data.component_centroids[component_id];

  bool inside = clampVoronoiVertices(left_vertex, right_vertex, boundary_polygon, centroid);

  /*if (!inside) {
    EVOENGINE_LOG("Both points outside, couldn't clamp:\n(" << left_vertex[0] << ", " << left_vertex[1] << ", "
                                                            << left_vertex[2] << ")\n(" << right_vertex[0] << ", "
                                                            << right_vertex[1] << ", " << right_vertex[2] << ")");
    meshes.push_back(mesh);  // push empty mesh
    segment_mesh_pair_last_left_and_right_vertex.emplace_back(-1, -1);
    return;
  }*/
  size_t v = he.origin;
  if (v == -1) {
    v = graph.destination(even_id);
  }

  size_t left_index = addMeshletVertex(mesh, boundary_polygon, centroid, left_vertex, v, t);
  size_t right_index = addMeshletVertex(mesh, boundary_polygon, centroid, right_vertex, v, t);
  meshes.push_back(mesh);

  // add last vertex indices
  segment_mesh_pair_last_left_and_right_vertex.emplace_back(left_index, right_index);

  assert(segment_mesh_pairs.size() == segment_mesh_pair_last_left_and_right_vertex.size());
}

void kinDS::SegmentBuilder::completeBoundaryMeshSection(size_t he_id, size_t new_left, size_t new_right) {
  const auto& last_left_and_right = boundary_mesh_last_left_and_right_vertex[he_id];
  if (last_left_and_right.first != -1) {
    // distinguish the case that we have previously flipped an infinite edge that became a boundary edge
    if (half_edge_to_boundary_vertex_index[he_id] == -1) {
      // no edge flip
      addBoundaryTriangle(last_left_and_right.first, new_right, new_left);
      if (last_left_and_right.second != -1) {
        addBoundaryTriangle(last_left_and_right.second, new_right, last_left_and_right.first);
      }
    } else {
      assert(last_left_and_right.second != -1);
      // he_id was previously flipped, it's corresponding vertex is no longer part of the boundary
      addBoundaryTriangle(last_left_and_right.second, new_right, half_edge_to_boundary_vertex_index[he_id]);
      addBoundaryTriangle(new_left, last_left_and_right.first, half_edge_to_boundary_vertex_index[he_id]);
      addBoundaryTriangle(new_left, half_edge_to_boundary_vertex_index[he_id], new_right);

      // reset the half-edge to boundary vertex index
      half_edge_to_boundary_vertex_index[he_id] = -1;
    }
  } else {
    assert(last_left_and_right.second == -1);
  }
}

size_t kinDS::SegmentBuilder::addBoundaryTriangle(size_t u, size_t v, size_t w) {
  // check bounds
  if (u >= boundary_mesh.getVertexCount() || v >= boundary_mesh.getVertexCount() ||
      w >= boundary_mesh.getVertexCount()) {
    EVOENGINE_ERROR("Vertex index out of boundary mesh range.");
    return -1;
  }

  if (u >= boundary_mesh_raw_uvs.size() || v >= boundary_mesh_raw_uvs.size() || w >= boundary_mesh_raw_uvs.size()) {
    EVOENGINE_ERROR("Vertex index out of raw uv range.");
    return -1;
  }

  // get raw UVs
  VoronoiPoint<3> uv_u = {boundary_mesh_raw_uvs[u][0], boundary_mesh_raw_uvs[u][1], 0.0};
  VoronoiPoint<3> uv_v = {boundary_mesh_raw_uvs[v][0], boundary_mesh_raw_uvs[v][1], 0.0};
  VoronoiPoint<3> uv_w = {boundary_mesh_raw_uvs[w][0], boundary_mesh_raw_uvs[w][1], 0.0};

  // output UVs
  /*EVOENGINE_LOG("Adding boundary triangle with raw UVs: u(" + std::to_string(uv_u[0]) + ", " + std::to_string(uv_u[1])
     +
                "), v(" + std::to_string(uv_v[0]) + ", " + std::to_string(uv_v[1]) + "), w(" + std::to_string(uv_w[0]) +
                ", " + std::to_string(uv_w[1]) + ")");*/

  // adjust UVs to avoid seams, first coordinate is the angle normalized to [-0.5, 0.5]
  // As a heuristic, we take the first angle and adjust the others such that they have less than 0.5 difference
  double base_angle = uv_u[0];
  double& angle_v = uv_v[0];
  double diff_v = angle_v - base_angle;
  double adjustment = std::round(diff_v);
  angle_v -= adjustment;

  double& angle_w = uv_w[0];
  double diff_w = angle_w - base_angle;
  adjustment = std::round(diff_w);
  angle_w -= adjustment;

  uv_u[0] *= uv_circum_factor;
  uv_v[0] *= uv_circum_factor;
  uv_w[0] *= uv_circum_factor;
  uv_u[1] *= uv_height_factor;
  uv_v[1] *= uv_height_factor;
  uv_w[1] *= uv_height_factor;

  // add adjusted UVs
  size_t uv_index_u = boundary_mesh.addUV(uv_u);
  size_t uv_index_v = boundary_mesh.addUV(uv_v);
  size_t uv_index_w = boundary_mesh.addUV(uv_w);

  /*EVOENGINE_LOG("UVs after adjustment: u(" + std::to_string(uv_u[0]) + ", " + std::to_string(uv_u[1]) + "), v(" +
                std::to_string(uv_v[0]) + ", " + std::to_string(uv_v[1]) + "), w(" + std::to_string(uv_w[0]) + ", " +
                std::to_string(uv_w[1]) + ")");*/
  return boundary_mesh.addTriangle(u, v, w, uv_index_u, uv_index_v, uv_index_w, 0);
}

size_t kinDS::SegmentBuilder::addBoundaryVertex(VoronoiPoint<3> vertex, VoronoiPoint<2> centroid, size_t strand_id,
                                                double t) {
  double angle = std::atan2(centroid[1] - vertex[1], centroid[0] - vertex[0]);

  VoronoiPoint<2> raw_uv{angle / (2.0 * glm::pi<double>()), vertex[2]};
  if (create_transformed_mesh) {
    vertex = kin_del.getBranchTrajectories().transformToObjectSpace(vertex, strand_id, t);
  }

  size_t index = boundary_mesh.addVertex(vertex);
  boundary_mesh_raw_uvs.resize(index + 1, VoronoiPoint<2>{});
  boundary_mesh_raw_uvs[index] = raw_uv;

  return index;
}

size_t kinDS::SegmentBuilder::addMeshletTriangle(VoronoiMesh& mesh, size_t u, size_t v, size_t w) {
  return mesh.addTriangle(u, v, w, u, v, w);  // For meshlets, the UVs are assigned per vertex so the indices match
}

size_t kinDS::SegmentBuilder::addMeshletVertex(VoronoiMesh& mesh, const std::vector<BoundaryPoint>& boundary_polygon,
                                               const VoronoiPoint<2>& centroid, VoronoiPoint<3> vertex,
                                               size_t strand_id, double t) {
  if (create_transformed_mesh) {
    vertex = kin_del.getBranchTrajectories().transformToObjectSpace(vertex, strand_id, t);
  }
  size_t index = mesh.addVertex(vertex);
  double rel_dist = relativeDistanceFromCenter(boundary_polygon, centroid, VoronoiPoint<2>{vertex[0], vertex[1]});

  /*if (rel_dist > 1.0 + std::numeric_limits<double>::epsilon()) {
    EVOENGINE_WARNING("Adding vertex that is too far outside, relative distance: " << rel_dist);
  }*/

  // TODO: this can be simplified to not use trigonometric functions
  double angle = std::atan2(centroid[1] - vertex[1], centroid[0] - vertex[0]);
  double u = 0.5 + texture_diameter * rel_dist * 0.5 * std::cos(angle);
  double v = 0.5 + texture_diameter * rel_dist * 0.5 * std::sin(angle);
  mesh.addUV(u, v, vertex[2] * uv_height_factor);
  return index;
}

void kinDS::SegmentBuilder::addVoronoiTriangulationToBoundaryMesh(double t, bool invert_orientation, double offset) {
  auto& graph = kin_del.getGraph();

  size_t index_offset = boundary_mesh.getVertices().size();
  std::vector<double> relative_center_distances;
  // add all vertices
  for (size_t i = 0; i < graph.getVertexCount(); i++) {
    VoronoiPoint<2> vertex = kin_del.getPointAt(t, i);

    auto component_index = kin_del.component_data.component_map[i];
    auto& boundary_polygon = kin_del.component_data.component_boundaries[component_index][0];
    auto& centroid = kin_del.component_data.component_centroids[component_index];

    addBoundaryVertex(VoronoiPoint<3>{vertex[0], vertex[1], t + offset}, centroid, i, t);
    // EVOENGINE_LOG("New raw uv: " << raw_uv[0] << ", " << raw_uv[1] << " for vertex: " << vertex_index);

    double rel_dist = relativeDistanceFromCenter(boundary_polygon, centroid, vertex);
    relative_center_distances.push_back(rel_dist);
  }
  // add all triangles
  // for (const auto& triangle : graph.getFaces()) {
  for (size_t face_index = 0; face_index < graph.getFaces().size(); face_index++) {
    const auto& triangle = graph.getFaces()[face_index];
    const auto& he_ids = triangle.half_edges;
    auto vertices = graph.adjacentTriangleVertices(triangle.half_edges[0]);

    // check if on component boundary and update last left and right vertices accordingly
    // store last left and right
    for (size_t i = 0; i < 3; i++) {
      if (kin_del.isOnComponentBoundaryOutside(he_ids[i])) {
        completeBoundaryMeshSection(he_ids[i], index_offset + vertices[i], index_offset + vertices[(i + 1) % 3]);
        // EVOENGINE_LOG("Assigning boundary last vertices for he_id " << he_ids[i]);
        boundary_mesh_last_left_and_right_vertex[he_ids[i]] =
            std::make_pair(index_offset + vertices[i], index_offset + vertices[(i + 1) % 3]);
      }
    }
    // skip faces that are outside
    if (!kin_del.getFaceInside(face_index)) {
      continue;
    }

    // check for infinite vertices
    if (vertices[0] == -1 || vertices[1] == -1 || vertices[2] == -1) {
      continue;  // skip triangles with infinite vertices
    }

    if (invert_orientation) {
      std::swap(vertices[1], vertices[2]);
    }

    // next, get the angles from the raw UVs and convert back to cartesian coordinates centered at (0.5, 0.5)
    size_t uv_indices[3];

    for (size_t i = 0; i < 3; i++) {
      double rel_dist = relative_center_distances[vertices[i]];
      double angle = boundary_mesh_raw_uvs[index_offset + vertices[i]][0] * 2.0 * glm::pi<double>();
      double u = 0.5 + texture_diameter * rel_dist * 0.5 * std::cos(angle);
      double v = 0.5 + texture_diameter * rel_dist * 0.5 * std::sin(angle);
      uv_indices[i] = boundary_mesh.addUV(u, v, 0.0);
    }

    // as an exception, we directly add the triangle here to have access to the UV indices
    boundary_mesh.addTriangle(index_offset + vertices[0], index_offset + vertices[1], index_offset + vertices[2],
                              uv_indices[0], uv_indices[1], uv_indices[2], 1);
  }
}

std::vector<BoundaryPoint> kinDS::SegmentBuilder::traceConvexHull(double t) const {
  const auto& graph = kin_del.getGraph();
  std::vector<BoundaryPoint> convex_hull_points;
  for (HalfEdgeDelaunayGraph::ConvexHullEdgeIterator it = graph.boundaryEdgesBegin(), end = graph.boundaryEdgesEnd();
       it != end; ++it) {
    size_t he_id = *it;
    size_t strand_index = graph.getHalfEdges()[he_id].origin;

    VoronoiPoint<2> convex_hull_point = kin_del.getPointAt(t, strand_index);
    convex_hull_points.push_back({strand_index, he_id, convex_hull_point});
  }

  return convex_hull_points;
}

void kinDS::SegmentBuilder::advanceBoundaryMesh(double t, const std::vector<BoundaryPoint>& boundary_points,
                                                const VoronoiPoint<2>& centroid) {
  std::vector<size_t> new_vertex_indices;

  for (size_t i = 0; i < boundary_points.size(); i++) {
    VoronoiPoint<2> boundary_point = boundary_points[i].p;

    new_vertex_indices.push_back(addBoundaryVertex(VoronoiPoint<3>{boundary_point[0], boundary_point[1], t}, centroid,
                                                   boundary_points[i].vertex_id, t));
  }

  for (size_t i = 0; i < boundary_points.size(); i++) {
    size_t he_id = boundary_points[i].he_id;
    size_t left_vertex_index = new_vertex_indices[i];
    size_t right_vertex_index = new_vertex_indices[(i + 1) % boundary_points.size()];
    auto& left_and_right = boundary_mesh_last_left_and_right_vertex[he_id];
    completeBoundaryMeshSection(he_id, left_vertex_index, right_vertex_index);

    // EVOENGINE_LOG("Assigning boundary last vertices for he_id " << he_id);
    left_and_right.first = left_vertex_index;
    left_and_right.second = right_vertex_index;
  }
}

void kinDS::SegmentBuilder::updateBoundary(double t, std::vector<bool>& visited, size_t component_index) {
  if (kin_del.component_data.component_last_updated[component_index] != t) {
    kin_del.component_data.component_boundaries[component_index] =
        kin_del.extractComponentBoundaries(kin_del.component_data.components[component_index], t, visited);
    kin_del.component_data.component_centroids[component_index] =
        polygonCentroid(kin_del.component_data.component_boundaries[component_index][0]);
    kin_del.component_data.component_last_updated[component_index] = t;
  }
}

void kinDS::SegmentBuilder::updateBoundaries(double t) {
  std::vector<bool> visited(kin_del.getGraph().getHalfEdges().size(), false);

  for (size_t component_index = 0; component_index < kin_del.component_data.components.size(); component_index++) {
    updateBoundary(t, visited, component_index);
  }
}

void kinDS::SegmentBuilder::advanceBoundaryMeshes(double t) {
  updateBoundaries(t);

  for (size_t component_index = 0; component_index < kin_del.component_data.components.size(); component_index++) {
    auto& boundaries = kin_del.component_data.component_boundaries[component_index];
    auto& centroid = kin_del.component_data.component_centroids[component_index];

    for (auto& boundary_points : boundaries) {
      advanceBoundaryMesh(t, boundary_points, centroid);
    }
  }
}

std::vector<VoronoiPoint<3>> SegmentBuilder::computeClampedVoronoiVertices(
    size_t strand_id, double t, const std::vector<BoundaryPoint>& boundary_polygon, const VoronoiPoint<2>& centroid) {
  auto& graph = kin_del.getGraph();
  std::vector<VoronoiPoint<3>> voronoi_vertices;

  // we just create a triangle fan because the Voronoi cell is convex
  // iterate over all segment indices of the strand
  for (HalfEdgeDelaunayGraph::IncidentEdgeIterator it = graph.incidentEdgesBegin(strand_id),
                                                   end = graph.incidentEdgesEnd(strand_id);
       it != end; ++it) {
    VoronoiPoint<3> voronoi_vertex = computeVoronoiVertex(*it, t, half_edge_index_to_segment_mesh_pair_index[*it]);
    voronoi_vertices.emplace_back(voronoi_vertex);
    // addMeshletVertex(mesh, boundary_polygon, centroid, voronoi_vertex);
  }

  std::vector<VoronoiPoint<3>> clamped_voronoi_vertices;
  // go through the vertices and clamp them
  for (size_t i = 0; i < voronoi_vertices.size(); i++) {
    auto left = voronoi_vertices[i];
    auto right = voronoi_vertices[(i + 1) % voronoi_vertices.size()];

    bool inside = clampVoronoiVertices(left, right, boundary_polygon, centroid);

    if (inside) {
      clamped_voronoi_vertices.push_back(left);
      clamped_voronoi_vertices.push_back(right);
    }
  }

  voronoi_vertices.clear();
  // Put back into original vector and remove duplicates
  voronoi_vertices.emplace_back(clamped_voronoi_vertices.front());
  for (size_t i = 1; i < clamped_voronoi_vertices.size() - 1; i++) {
    if (clamped_voronoi_vertices[i] != voronoi_vertices.back()) {
      voronoi_vertices.push_back(clamped_voronoi_vertices[i]);
    }
  }
  if (clamped_voronoi_vertices.front() != clamped_voronoi_vertices.back()) {
    voronoi_vertices.push_back(clamped_voronoi_vertices.back());
  }

  return voronoi_vertices;
}

size_t kinDS::SegmentBuilder::createClosingMesh(size_t strand_id, double t,
                                                const std::vector<BoundaryPoint>& boundary_polygon,
                                                const VoronoiPoint<2>& centroid) {
  auto& graph = kin_del.getGraph();

  MeshStructure::SegmentMeshPair segment_mesh_pair;
  segment_mesh_pairs.push_back(segment_mesh_pair);

  VoronoiMesh mesh;

  auto voronoi_vertices = computeClampedVoronoiVertices(strand_id, t, boundary_polygon, centroid);

  for (auto& voronoi_vertex : voronoi_vertices) {
    addMeshletVertex(mesh, boundary_polygon, centroid, voronoi_vertex, strand_id, t);
  }

  // create triangles
  size_t apex_index = 0;

  for (size_t voronoi_index = 2; voronoi_index < mesh.getVertices().size(); ++voronoi_index) {
    addMeshletTriangle(mesh, apex_index, voronoi_index - 1, voronoi_index);
  }

  size_t index = meshes.size();
  meshes.push_back(mesh);
  segment_mesh_pair_last_left_and_right_vertex.push_back(
      std::make_pair(-1, -1));  // not needed, so we just set it to -1,-1

  return index;
}

void kinDS::SegmentBuilder::accumulateSegmentProperties() {
  // Iterate through all pairs and accumulate properties
  for (size_t pair_id = 0; pair_id < segment_mesh_pairs.size(); ++pair_id) {
    auto& pair = segment_mesh_pairs[pair_id];
    if (pair.segment_index0 != -1) {
      // make sure there is space left
      if (segment_properties[pair.segment_index0].neighbor_count >= MeshStructure::SegmentProperties::MAX_NEIGHBORS) {
        EVOENGINE_ERROR("Exceeded maximum number of neighbors for segment: "
                        << segment_properties[pair.segment_index0].neighbor_count
                        << " >= " << MeshStructure::SegmentProperties::MAX_NEIGHBORS);
        // throw std::runtime_error("Exceeded maximum number of neighbors for segment.");
      } else {
        segment_properties[pair.segment_index0]
            .mesh_pair_indices[segment_properties[pair.segment_index0].neighbor_count] =
            pair_id;  // add mesh pair index
        segment_properties[pair.segment_index0]
            .neighbor_indices[segment_properties[pair.segment_index0].neighbor_count] =
            pair.segment_index1;  // add neighbor
        segment_properties[pair.segment_index0].neighbor_count++;
      }
    }

    if (pair.segment_index1 != -1) {
      // make sure there is space left
      if (segment_properties[pair.segment_index1].neighbor_count >= MeshStructure::SegmentProperties::MAX_NEIGHBORS) {
        EVOENGINE_ERROR("Exceeded maximum number of neighbors for segment: "
                        << segment_properties[pair.segment_index1].neighbor_count
                        << " >= " << MeshStructure::SegmentProperties::MAX_NEIGHBORS);
        // throw std::runtime_error("Exceeded maximum number of neighbors for segment.");
      } else {
        segment_properties[pair.segment_index1]
            .mesh_pair_indices[segment_properties[pair.segment_index1].neighbor_count] =
            pair_id;  // add mesh pair index
        segment_properties[pair.segment_index1]
            .neighbor_indices[segment_properties[pair.segment_index1].neighbor_count] =
            pair.segment_index0;  // add neighbor
        segment_properties[pair.segment_index1].neighbor_count++;
      }
    }
  }
}

void kinDS::SegmentBuilder::splitComponent(size_t component_id, const std::vector<std::vector<size_t>>& new_components,
                                           double t) {
  if (new_components.empty()) {
    return;
  }

  // Update component data
  std::vector<size_t> component_ids(new_components.size(), -1);

  component_ids[0] = component_id;
  size_t new_component_id = kin_del.component_data.components.size();
  for (size_t i = 1; i < new_components.size(); i++) {
    component_ids[i] = new_component_id;
    new_component_id++;
  }

  size_t new_size = kin_del.component_data.components.size() + new_components.size() - 1;
  kin_del.component_data.components.resize(new_size);
  kin_del.component_data.component_boundaries.resize(new_size);
  kin_del.component_data.component_centroids.resize(new_size);

  std::vector<bool> he_visited(kin_del.getGraph().getHalfEdges().size(), false);

  for (size_t i = 0; i < new_components.size(); i++) {
    size_t cid = component_ids[i];

    for (size_t v : new_components[i]) {
      kin_del.component_data.component_map[v] = cid;
    }

    kin_del.component_data.components[cid] = new_components[i];
    kin_del.component_data.component_boundaries[cid] =
        kin_del.extractComponentBoundaries(new_components[i], t, he_visited);
    kin_del.component_data.component_centroids[cid] =
        polygonCentroid(kin_del.component_data.component_boundaries[cid][0]);
    kin_del.component_data.component_last_updated[cid] = t;
  }
}

void SegmentBuilder::init() {
  auto& graph = kin_del.getGraph();

  size_t strand_count = graph.getVertexCount();
  strand_to_segment_indices.resize(strand_count);
  half_edge_index_to_segment_mesh_pair_index.resize(graph.getHalfEdges().size(), -1);
  corner_to_cutoff_mesh_indices.resize(graph.getHalfEdges().size(), -1);

  // Initialize the strand geometries at t = 0.0
  double t = 0.0;  // TODO: might be customized later

  // We need a ruled surface for each half-edge in the graph except those having the infinite vertex as
  // origin
  size_t half_edge_count = graph.getHalfEdges().size();

  // initialize segment mesh properties for each strand
  for (size_t strand_id = 0; strand_id < strand_count; ++strand_id) {
    size_t new_segment_id = segment_properties.size();
    MeshStructure::SegmentProperties properties;
    segment_properties.push_back(properties);
    strand_to_segment_indices[strand_id].push_back(new_segment_id);

    size_t component_index = kin_del.component_data.component_map[strand_id];
    const auto& component = kin_del.component_data.components[component_index];
    const auto& boundary_polygon = kin_del.component_data.component_boundaries[component_index][0];
    const auto& centroid = kin_del.component_data.component_centroids[component_index];
    // create a closing mesh
    size_t closing_mesh_index = createClosingMesh(strand_id, t, boundary_polygon, centroid);
    MeshStructure::SegmentMeshPair& segment_mesh_pair = segment_mesh_pairs[new_segment_id];
    segment_mesh_pair.segment_index0 = -1;
    segment_mesh_pair.segment_index1 = strand_to_segment_indices[strand_id].back();
  }

  // now go through all half-edges and create a segment mesh pair
  for (size_t i = 0; i < half_edge_count; i += 2) {
    startNewMesh(i, t);
  }

  // initialize boundary mesh
  boundary_mesh_last_left_and_right_vertex.resize(half_edge_count, std::make_pair(-1, -1));
  half_edge_to_boundary_vertex_index.resize(half_edge_count, -1);
  addVoronoiTriangulationToBoundaryMesh(t, false, -0.01);
}

void SegmentBuilder::betweenSections(size_t index) {
  // Check if we need to insert a subdivision before handling this event
  while (subdivision_index < subdivisions.size() && subdivisions[subdivision_index].second <= index) {
    insertSubdivision(subdivisions[subdivision_index].first, subdivisions[subdivision_index].second);
    subdivision_index++;
  }

  auto& graph = kin_del.getGraph();

  advanceBoundaryMeshes(index);

  size_t half_edge_count = graph.getHalfEdges().size();
  for (size_t i = 0; i < half_edge_count; i += 2) {
    // use the origin of the half edge to obtain the correct component
    auto vertex = graph.getHalfEdges()[i].origin;

    // fall back for infinite vertices
    if (vertex == -1) {
      vertex = graph.destination(i);
    }
    size_t component_index = kin_del.component_data.component_map[vertex];
    auto& boundary_points = kin_del.component_data.component_boundaries[component_index][0];

    finishMesh(i, index, boundary_points);
  }
}

void SegmentBuilder::beforeEvent(KineticDelaunay::Event& e) {
  auto& graph = kin_del.getGraph();
  // Check if we need to insert a subdivision before handling this event
  while (subdivision_index < subdivisions.size() && subdivisions[subdivision_index].second <= e.time) {
    insertSubdivision(subdivisions[subdivision_index].first, subdivisions[subdivision_index].second);
    subdivision_index++;
  }

  auto vertex = graph.getHalfEdges()[e.half_edge_id].origin;
  if (vertex == -1) {
    vertex = graph.destination(e.half_edge_id);
  }

  size_t component_id = kin_del.component_data.component_map[vertex];
  auto& boundary_polygon = kin_del.component_data.component_boundaries[component_id][0];
  auto centroid = polygonCentroid(boundary_polygon);

  // Finish the segment mesh pair of the edge being flipped
  VoronoiPoint<3> event_point{e.position[0], e.position[1], e.time};
  size_t segment_mesh_pair_index = half_edge_index_to_segment_mesh_pair_index[e.half_edge_id];
  VoronoiMesh& mesh = meshes[segment_mesh_pair_index];
  size_t event_vertex_index = addMeshletVertex(mesh, boundary_polygon, centroid, event_point, vertex, e.time);
  const auto& last_vertices = segment_mesh_pair_last_left_and_right_vertex[segment_mesh_pair_index];
  // create one triangle to the event point
  addMeshletTriangle(mesh, last_vertices.first, last_vertices.second, event_vertex_index);

  // For the boundary mesh, handle the case that a boundary edge is flipped. This means the opposite vertex becomes a
  // boundary vertex
  // Only applies if the edge is on the component boundary as well
  if (kin_del.isOnComponentBoundary(e.half_edge_id)) {
    /* The mesh will look like this here:
     *
     *  o-o-o  <-- boundary mesh after the flip consisting of two edges
     *  |\|/|
     *  | o |  <-- event point
     *  |/ \|
     *  o---o  <-- boundary mesh before the flip consisting of one edge
     *
     * In the following, we add the new boundary vertex at the event point and create the lower triangle.
     * The mesh can later be completed as usual because we update the last left and right vertex indices accordingly.
     */

    // TODO: make sure this is equivalent to component boundary
    size_t outer_he_id =
        kin_del.isOnComponentBoundaryOutside(e.half_edge_id) ? e.half_edge_id : graph.twin(e.half_edge_id);
    size_t inner_he_id = outer_he_id ^ 1;

    size_t opposite_vertex = graph.triangleOppositeVertex(inner_he_id);
    const auto& boundary_last_vertices = boundary_mesh_last_left_and_right_vertex[outer_he_id];

    VoronoiPoint<2> new_boundary_vertex = kin_del.getPointAt(e.time, opposite_vertex);

    size_t new_boundary_vertex_index = boundary_mesh.getVertices().size();
    // TODO: raw UVs
    addBoundaryVertex(VoronoiPoint<3>{new_boundary_vertex[0], new_boundary_vertex[1], e.time}, centroid,
                      opposite_vertex, e.time);

    // create one triangle to the event point
    addBoundaryTriangle(boundary_last_vertices.first, boundary_last_vertices.second, new_boundary_vertex_index);

    // update last left and right indices of the other two half-edges of the triangle
    size_t he1_id = graph.getHalfEdges()[inner_he_id].next;
    size_t he2_id = graph.getHalfEdges()[he1_id].next;

    // EVOENGINE_LOG("Assigning boundary last vertices for he_id " << he1_id);
    boundary_mesh_last_left_and_right_vertex[he1_id] =
        std::make_pair(boundary_last_vertices.first, new_boundary_vertex_index);
    // EVOENGINE_LOG("Assigning boundary last vertices for he_id " << he2_id);
    boundary_mesh_last_left_and_right_vertex[he2_id] =
        std::make_pair(new_boundary_vertex_index, boundary_last_vertices.second);

    // reset last left and right vertices of the half-edge because it is not on the boundary anymore
    // EVOENGINE_LOG("Resetting boundary last vertices for he_id " << outer_he_id);
    boundary_mesh_last_left_and_right_vertex[outer_he_id] = std::make_pair(-1, -1);
  }
}

void SegmentBuilder::afterEvent(KineticDelaunay::Event& e) {
  updateBoundaries(e.time);
  auto& graph = kin_del.getGraph();
  const auto& he = graph.getHalfEdges()[e.half_edge_id];
  const auto& twin_he = graph.getHalfEdges()[e.half_edge_id ^ 1];
  // Create a new segment mesh pair for the two new edges created by the flip
  MeshStructure::SegmentMeshPair segment_mesh_pair;
  segment_mesh_pair.segment_index0 = he.origin == -1 ? -1 : strand_to_segment_indices[he.origin].back();
  segment_mesh_pair.segment_index1 = twin_he.origin == -1 ? -1 : strand_to_segment_indices[twin_he.origin].back();

  half_edge_index_to_segment_mesh_pair_index[e.half_edge_id] = segment_mesh_pairs.size();
  half_edge_index_to_segment_mesh_pair_index[e.half_edge_id ^ 1] = segment_mesh_pairs.size();

  segment_mesh_pairs.push_back(segment_mesh_pair);

  // TODO: we should be able to reuse these from beforeEvent()
  auto vertex = graph.getHalfEdges()[e.half_edge_id].origin;
  if (vertex == -1) {
    vertex = graph.destination(e.half_edge_id);
  }

  size_t component_id = kin_del.component_data.component_map[vertex];
  auto& boundary_polygon = kin_del.component_data.component_boundaries[component_id][0];
  auto centroid = polygonCentroid(boundary_polygon);

  // For now also create a mesh, but this might be changed later
  VoronoiMesh mesh;
  size_t index = addMeshletVertex(mesh, boundary_polygon, centroid,
                                  VoronoiPoint<3>{e.position[0], e.position[1], e.time}, vertex, e.time);

  // add last vertex indices
  segment_mesh_pair_last_left_and_right_vertex.push_back(std::make_pair(index, index));

  meshes.push_back(mesh);

  // first get the other half-edges of the quadrilateral
  size_t he0_id = graph.getHalfEdges()[e.half_edge_id].next;      // Next half-edge in the quadrilateral
  size_t he1_id = graph.getHalfEdges()[he0_id].next;              // Next half-edge in the quadrilateral
  size_t he2_id = graph.getHalfEdges()[e.half_edge_id ^ 1].next;  // Next half-edge in the quadrilateral
  size_t he3_id = graph.getHalfEdges()[he2_id].next;              // Next half-edge in the quadrilateral

  // for each of them, insert the event vertex with one triangle on one side
  for (size_t he_id : {he0_id, he1_id, he2_id, he3_id}) {
    size_t segment_mesh_pair_index = half_edge_index_to_segment_mesh_pair_index[he_id];
    VoronoiMesh& mesh = meshes[segment_mesh_pair_index];
    size_t index = addMeshletVertex(mesh, boundary_polygon, centroid,
                                    VoronoiPoint<3>{e.position[0], e.position[1], e.time}, vertex, e.time);
    const auto& last_vertices = segment_mesh_pair_last_left_and_right_vertex[segment_mesh_pair_index];
    // create one triangle to the event point
    addMeshletTriangle(mesh, last_vertices.first, last_vertices.second, index);
    // update last vertex indices
    const MeshStructure::SegmentMeshPair& segment_mesh_pair = segment_mesh_pairs[segment_mesh_pair_index];
    // Determine whether we have to update the left or right vertex here
    int origin = graph.getHalfEdges()[he_id].origin;

    if (origin != -1) {
      if (segment_mesh_pair.segment_index1 == strand_to_segment_indices[origin].back()) {
        segment_mesh_pair_last_left_and_right_vertex[segment_mesh_pair_index] =
            std::make_pair(last_vertices.first, index);
      } else {
        segment_mesh_pair_last_left_and_right_vertex[segment_mesh_pair_index] =
            std::make_pair(index, last_vertices.second);
      }
    } else {
      assert(graph.destination(he_id) != -1);
      if (segment_mesh_pair.segment_index0 == strand_to_segment_indices[graph.destination(he_id)].back()) {
        segment_mesh_pair_last_left_and_right_vertex[segment_mesh_pair_index] =
            std::make_pair(last_vertices.first, index);
      } else {
        segment_mesh_pair_last_left_and_right_vertex[segment_mesh_pair_index] =
            std::make_pair(index, last_vertices.second);
      }
    }
  }
  // For the boundary mesh, handle the case that a formerly infinite edge is flipped to a boundary. This means the
  // opposite vertex is no longer a boundary vertex
  // Only applies if the edge is on the component boundary as well
  if (kin_del.isOnComponentBoundary(e.half_edge_id)) {
    /* The mesh will look like this here:
     *
     *  o---o  <-- boundary mesh after the flip consisting of one edge
     *  |\ /|
     *  | o |  <-- event point
     *  |/|\|
     *  o-o-o  <-- boundary mesh before the flip consisting of two edges
     *
     * In the following, we insert the vertex at the event point and create the two lower triangles.
     * To create the two side triangles and the upper one, we buffer the new vertex index and complete the mesh later.
     */

    size_t outer_he_id =
        kin_del.isOnComponentBoundaryOutside(e.half_edge_id) ? e.half_edge_id : graph.twin(e.half_edge_id);
    size_t inner_he_id = outer_he_id ^ 1;

    size_t opposite_vertex = graph.triangleOppositeVertex(inner_he_id);
    const auto& boundary_last_vertices = boundary_mesh_last_left_and_right_vertex[outer_he_id];

    VoronoiPoint<2> old_boundary_vertex = kin_del.getPointAt(e.time, opposite_vertex);

    size_t old_boundary_vertex_index = boundary_mesh.getVertices().size();
    // TODO: UV should correspond to a relative distance of 1.0
    addBoundaryVertex(VoronoiPoint<3>{old_boundary_vertex[0], old_boundary_vertex[1], e.time}, centroid,
                      opposite_vertex, e.time);

    size_t he1_id = graph.getHalfEdges()[inner_he_id].next;
    size_t he2_id = graph.getHalfEdges()[he1_id].next;

    // create two triangles to the event point
    size_t index =
        addBoundaryTriangle(boundary_mesh_last_left_and_right_vertex[he1_id].first,
                            boundary_mesh_last_left_and_right_vertex[he1_id].second, old_boundary_vertex_index);

    // print info if there was an error
    if (index == size_t(-1)) {
      EVOENGINE_LOG("\he1_id: " << he1_id << "\ntwin: " << (he1_id ^ 1)
                                << "\nlast_vertices: " << boundary_mesh_last_left_and_right_vertex[he1_id].first << ", "
                                << boundary_mesh_last_left_and_right_vertex[he1_id].second << "\ntwin last vertices: "
                                << boundary_mesh_last_left_and_right_vertex[he1_id ^ 1].first << ", "
                                << boundary_mesh_last_left_and_right_vertex[he1_id ^ 1].second
                                << "\nopposite: " << opposite_vertex);
    }

    index = addBoundaryTriangle(boundary_mesh_last_left_and_right_vertex[he2_id].first,
                                boundary_mesh_last_left_and_right_vertex[he2_id].second, old_boundary_vertex_index);

    if (index == size_t(-1)) {
      EVOENGINE_LOG("\he2_id: " << he2_id << "\ntwin: " << (he2_id ^ 1)
                                << "\nlast_vertices: " << boundary_mesh_last_left_and_right_vertex[he2_id].first << ", "
                                << boundary_mesh_last_left_and_right_vertex[he2_id].second << "\ntwin last vertices: "
                                << boundary_mesh_last_left_and_right_vertex[he2_id ^ 1].first << ", "
                                << boundary_mesh_last_left_and_right_vertex[he2_id ^ 1].second
                                << "\nopposite: " << opposite_vertex);
    }

    // Furthermore, we need to buffer this new vertex for the next event at the new boundary half-edge to complete the
    // mesh
    half_edge_to_boundary_vertex_index[outer_he_id] = old_boundary_vertex_index;

    // EVOENGINE_LOG("Assigning boundary last vertices for he_id " << outer_he_id);
    boundary_mesh_last_left_and_right_vertex[outer_he_id] =
        std::make_pair(boundary_mesh_last_left_and_right_vertex[he1_id].first,
                       boundary_mesh_last_left_and_right_vertex[he2_id].second);

    // reset last left and right vertices of the half-edges because it is not on the boundary anymore
    // EVOENGINE_LOG("Resetting boundary last vertices for he_id " << he1_id);
    boundary_mesh_last_left_and_right_vertex[he1_id] = std::make_pair(-1, -1);
    // EVOENGINE_LOG("Resetting boundary last vertices for he_id " << he2_id);
    boundary_mesh_last_left_and_right_vertex[he2_id] = std::make_pair(-1, -1);
  }

  // update the boundaries (TODO: maybe do this beforehand for the UV mapping?)
  std::vector<bool> visited(kin_del.getGraph().getHalfEdges().size(), false);
  kin_del.component_data.component_boundaries[component_id] =
      kin_del.extractComponentBoundaries(kin_del.component_data.components[component_id], e.time, visited);
  kin_del.component_data.component_centroids[component_id] =
      polygonCentroid(kin_del.component_data.component_boundaries[component_id][0]);
  kin_del.component_data.component_last_updated[component_id] = e.time;
}

void kinDS::SegmentBuilder::beforeBoundaryEvent(KineticDelaunay::Event& e) {
  // Build the boundary mesh at the event time
  size_t face_id = kin_del.getGraph().getHalfEdges()[e.half_edge_id].face;
  bool is_inside = kin_del.getFaceInside(face_id);
  // EVOENGINE_LOG("Face inside: " << is_inside << ", face id: " << face_id);
  // For each half-edge of the face, check if it is on the boundary
  auto& graph = kin_del.getGraph();
  const auto& half_edges = graph.getFaces()[face_id].half_edges;

  std::array<bool, 3> is_boundary_edge;
  size_t boundary_edge_count = 0;
  for (size_t i = 0; i < 3; ++i) {
    size_t he_id = half_edges[i];
    is_boundary_edge[i] = kin_del.isOnComponentBoundary(he_id);
    if (is_boundary_edge[i]) {
      boundary_edge_count++;
    }
  }

  // EVOENGINE_LOG("Boundary case " << boundary_edge_count);
  switch (boundary_edge_count) {
    case 0: {
      // start a new mesh for this face
      // get all triangle vertices
      size_t vertices[3];
      for (size_t i = 0; i < 3; ++i) {
        vertices[i] = graph.getHalfEdges()[half_edges[i]].origin;

        if (vertices[i] == size_t(-1)) {
          EVOENGINE_ERROR("Boundary triangle was turned at " << e.time << ", that is impossible and will be ignored!");
          break;
        }
      }
      VoronoiPoint<2> p0 = kin_del.getPointAt(e.time, vertices[0]);
      VoronoiPoint<2> p1 = kin_del.getPointAt(e.time, vertices[1]);
      VoronoiPoint<2> p2 = kin_del.getPointAt(e.time, vertices[2]);
      VoronoiPoint<2> new_point = (p0 + p1 + p2) / 3.0;

      size_t new_vertex_index = boundary_mesh.getVertices().size();
      addBoundaryVertex(VoronoiPoint<3>{new_point[0], new_point[1], e.time}, VoronoiPoint<2>{0.0, 0.0}, vertices[0],
                        e.time);

      // set the last left and right vertices for all half-edges
      if (is_inside) {
        for (size_t i = 0; i < 3; ++i) {
          // EVOENGINE_LOG("Assigning boundary last vertices for he_id " << half_edges[i]);
          boundary_mesh_last_left_and_right_vertex[half_edges[i]] = std::make_pair(new_vertex_index, new_vertex_index);
        }
      } else {
        for (size_t i = 0; i < 3; ++i) {
          size_t outer_he_id = half_edges[i] ^ 1;
          // EVOENGINE_LOG("Assigning boundary last vertices for he_id " << outer_he_id);
          boundary_mesh_last_left_and_right_vertex[outer_he_id] = std::make_pair(new_vertex_index, new_vertex_index);
        }
      }

      break;
    }
    case 1: {
      // we go from one boundary edge to two, this is similar to a convex boundary edge flip
      /* The mesh will look like this here:
       *
       *  o-o-o  <-- component boundary mesh after the triangle was added
       *  |\|/|
       *  | o |  <-- event point
       *  |/ \|
       *  o---o  <-- component boundary mesh while the triangle is still outside
       *
       */

      // find the boundary edge
      size_t boundary_he_index = 0;
      for (size_t i = 0; i < 3; ++i) {
        if (is_boundary_edge[i]) {
          boundary_he_index = i;
          break;
        }
      }

      size_t inner_he_id = half_edges[boundary_he_index];
      size_t outer_he_id = inner_he_id ^ 1;

      // get the opposite vertex
      size_t opposite_vertex = graph.triangleOppositeVertex(inner_he_id);

      // place the new vertex at the center of the triangle
      VoronoiPoint<2> opposite_point = kin_del.getPointAt(e.time, opposite_vertex);
      size_t u = graph.getHalfEdges()[inner_he_id].origin;
      VoronoiPoint<2> p_u = kin_del.getPointAt(e.time, u);
      size_t v = graph.getHalfEdges()[outer_he_id].origin;
      VoronoiPoint<2> p_v = kin_del.getPointAt(e.time, v);

      VoronoiPoint<2> new_boundary_vertex = (opposite_point + p_u + p_v) / 3.0;

      // TODO: correct the boundary using the new vertex
      size_t component_id = kin_del.component_data.component_map[v];
      auto& boundary_polygon = kin_del.component_data.component_boundaries[component_id][0];
      auto centroid = polygonCentroid(boundary_polygon);

      size_t boundary_he_id = outer_he_id;

      if (!is_inside) {
        boundary_he_id = inner_he_id;
      }

      const auto& boundary_last_vertices = boundary_mesh_last_left_and_right_vertex[boundary_he_id];

      size_t new_boundary_vertex_index = addBoundaryVertex(
          VoronoiPoint<3>{new_boundary_vertex[0], new_boundary_vertex[1], e.time}, centroid, opposite_vertex, e.time);

      // create one triangle to the event point
      size_t index =
          addBoundaryTriangle(boundary_last_vertices.first, boundary_last_vertices.second, new_boundary_vertex_index);

      // print info if there was an error
      if (index == size_t(-1)) {
        EVOENGINE_LOG("\ninner_he_id: " << inner_he_id << "\nouter_he_id: " << outer_he_id
                                        << "\nlast_vertices: " << boundary_last_vertices.first << ", "
                                        << boundary_last_vertices.second << "\ninner last vertices: "
                                        << boundary_mesh_last_left_and_right_vertex[inner_he_id].first << ", "
                                        << boundary_mesh_last_left_and_right_vertex[inner_he_id].second << "\nu: " << u
                                        << ", v: " << v << ", opposite: " << opposite_vertex);
      }

      // update last left and right indices of the other two half-edges of the triangle
      size_t he1_id = graph.getHalfEdges()[inner_he_id].next;
      size_t he2_id = graph.getHalfEdges()[he1_id].next;

      if (!is_inside) {
        he1_id = he1_id ^ 1;
        he2_id = he2_id ^ 1;
      }

      // EVOENGINE_LOG("Assigning boundary last vertices for he_id " << he1_id);
      boundary_mesh_last_left_and_right_vertex[he1_id] =
          std::make_pair(boundary_last_vertices.first, new_boundary_vertex_index);

      // EVOENGINE_LOG("Assigning boundary last vertices for he_id " << he2_id);
      boundary_mesh_last_left_and_right_vertex[he2_id] =
          std::make_pair(new_boundary_vertex_index, boundary_last_vertices.second);

      // reset last left and right vertices of the half-edge because it is not on the boundary anymore
      // EVOENGINE_LOG("Resetting boundary last vertices for he_id " << boundary_he_id);
      boundary_mesh_last_left_and_right_vertex[boundary_he_id] = std::make_pair(-1, -1);

      break;
    }
    case 2: {
      // we go from two boundary edges to one, this is similar to a convex boundary edge flip
      /* The mesh will look like this here:
       *
       *  o---o  <-- component boundary mesh after the triangle was removed
       *  |\ /|
       *  | o |  <-- event point
       *  |/|\|
       *  o-o-o  <-- component boundary mesh while the triangle is still inside
       *
       */

      // find the non-boundary edge
      size_t non_boundary_he_index = 0;
      for (size_t i = 0; i < 3; ++i) {
        if (!is_boundary_edge[i]) {
          non_boundary_he_index = i;
          break;
        }
      }

      size_t inner_he_id = half_edges[non_boundary_he_index];
      size_t outer_he_id = inner_he_id ^ 1;

      size_t opposite_vertex = graph.triangleOppositeVertex(inner_he_id);
      const auto& boundary_last_vertices = boundary_mesh_last_left_and_right_vertex[outer_he_id];

      // place the new vertex at the center of the triangle
      VoronoiPoint<2> opposite_point = kin_del.getPointAt(e.time, opposite_vertex);
      size_t u = graph.getHalfEdges()[inner_he_id].origin;
      VoronoiPoint<2> p_u = kin_del.getPointAt(e.time, u);
      size_t v = graph.getHalfEdges()[outer_he_id].origin;
      VoronoiPoint<2> p_v = kin_del.getPointAt(e.time, v);

      VoronoiPoint<2> old_boundary_vertex = (opposite_point + p_u + p_v) / 3.0;

      // TODO: correct the boundary using the new vertex
      std::vector<bool> visited(kin_del.getGraph().getVertexCount(), false);
      size_t component_id = kin_del.component_data.component_map[v];
      auto& boundary_polygon = kin_del.component_data.component_boundaries[component_id][0];
      auto centroid = polygonCentroid(boundary_polygon);

      size_t old_boundary_vertex_index = boundary_mesh.getVertices().size();
      // TODO: raw UVs
      addBoundaryVertex(VoronoiPoint<3>{old_boundary_vertex[0], old_boundary_vertex[1], e.time}, centroid,
                        opposite_vertex, e.time);

      size_t he1_id = graph.getHalfEdges()[inner_he_id].next;
      size_t he2_id = graph.getHalfEdges()[he1_id].next;

      // use the twins because the boundary is inverted
      if (is_inside) {
        he1_id = he1_id ^ 1;
        he2_id = he2_id ^ 1;
      }
      // create two triangles to the event point
      size_t index =
          addBoundaryTriangle(boundary_mesh_last_left_and_right_vertex[he1_id].first,
                              boundary_mesh_last_left_and_right_vertex[he1_id].second, old_boundary_vertex_index);

      if (index == size_t(-1)) {
        EVOENGINE_LOG("\he1_id: " << he1_id << "\ntwin: " << (he1_id ^ 1) << "\nlast_vertices: "
                                  << boundary_mesh_last_left_and_right_vertex[he1_id].first << ", "
                                  << boundary_mesh_last_left_and_right_vertex[he1_id].second << "\ntwin last vertices: "
                                  << boundary_mesh_last_left_and_right_vertex[he1_id ^ 1].first << ", "
                                  << boundary_mesh_last_left_and_right_vertex[he1_id ^ 1].second
                                  << "\nopposite: " << opposite_vertex);
      }

      index = addBoundaryTriangle(boundary_mesh_last_left_and_right_vertex[he2_id].first,
                                  boundary_mesh_last_left_and_right_vertex[he2_id].second, old_boundary_vertex_index);

      if (index == size_t(-1)) {
        EVOENGINE_LOG("\he2_id: " << he2_id << "\ntwin: " << (he2_id ^ 1) << "\nlast_vertices: "
                                  << boundary_mesh_last_left_and_right_vertex[he2_id].first << ", "
                                  << boundary_mesh_last_left_and_right_vertex[he2_id].second << "\ntwin last vertices: "
                                  << boundary_mesh_last_left_and_right_vertex[he2_id ^ 1].first << ", "
                                  << boundary_mesh_last_left_and_right_vertex[he2_id ^ 1].second
                                  << "\nopposite: " << opposite_vertex);
      }

      // Furthermore, we need to buffer this new vertex for the next event at the new boundary half-edge to complete the
      // mesh
      half_edge_to_boundary_vertex_index[outer_he_id] = old_boundary_vertex_index;

      if (!is_inside) {
        // EVOENGINE_LOG("Assigning boundary last vertices for he_id " << outer_he_id);
        boundary_mesh_last_left_and_right_vertex[outer_he_id] =
            std::make_pair(boundary_mesh_last_left_and_right_vertex[he1_id].first,
                           boundary_mesh_last_left_and_right_vertex[he2_id].second);
      } else {
        // EVOENGINE_LOG("Assigning boundary last vertices for he_id " << inner_he_id);
        boundary_mesh_last_left_and_right_vertex[inner_he_id] =
            std::make_pair(boundary_mesh_last_left_and_right_vertex[he2_id].second,
                           boundary_mesh_last_left_and_right_vertex[he1_id].first);
      }

      // reset last left and right vertices of the half-edges because it is not on the boundary anymore
      // EVOENGINE_LOG("Resetting boundary last vertices for he_id " << he1_id);
      boundary_mesh_last_left_and_right_vertex[he1_id] = std::make_pair(-1, -1);
      // EVOENGINE_LOG("Resetting boundary last vertices for he_id " << he2_id);
      boundary_mesh_last_left_and_right_vertex[he2_id] = std::make_pair(-1, -1);

      break;
    }
    case 3: {
      // finish the mesh for this face
      // get all triangle vertices
      size_t vertices[3];
      for (size_t i = 0; i < 3; ++i) {
        vertices[i] = graph.getHalfEdges()[half_edges[i]].origin;
      }
      VoronoiPoint<2> p0 = kin_del.getPointAt(e.time, vertices[0]);
      VoronoiPoint<2> p1 = kin_del.getPointAt(e.time, vertices[1]);
      VoronoiPoint<2> p2 = kin_del.getPointAt(e.time, vertices[2]);
      VoronoiPoint<2> new_point = (p0 + p1 + p2) / 3.0;

      size_t new_vertex_index = addBoundaryVertex(VoronoiPoint<3>{new_point[0], new_point[1], e.time},
                                                  VoronoiPoint<2>{0.0, 0.0}, vertices[0], e.time);

      // TODO: we might have to reverse the order if inside and outside are reversed
      // connect to all last left and right vertices
      for (size_t i = 0; i < 3; ++i) {
        const auto& last_vertices = boundary_mesh_last_left_and_right_vertex[half_edges[i]];
        addBoundaryTriangle(last_vertices.first, last_vertices.second, new_vertex_index);
      }

      break;
    }
  }
}

void kinDS::SegmentBuilder::afterBoundaryEvent(KineticDelaunay::Event& e) {
  // update the component data structure
  auto& graph = kin_del.getGraph();
  auto vertices = graph.adjacentTriangleVertices(e.half_edge_id);
  size_t component_id = kin_del.component_data.component_map[vertices[0]];

  auto split = kin_del.checkForSplit(vertices);
  splitComponent(component_id, split, e.time);

  if (split.empty()) {
    // no split occurred, just update the boundary
    std::vector<bool> visited(kin_del.getGraph().getHalfEdges().size(), false);
    kin_del.component_data.component_boundaries[component_id] =
        kin_del.extractComponentBoundaries(kin_del.component_data.components[component_id], e.time, visited);
    kin_del.component_data.component_centroids[component_id] =
        polygonCentroid(kin_del.component_data.component_boundaries[component_id][0]);
    kin_del.component_data.component_last_updated[component_id] = e.time;
  }
}

void kinDS::SegmentBuilder::insertSubdivision(size_t strand_id, double t) {
  // EVOENGINE_LOG("Inserting subdivision for strand " << strand_id << " at t = " << t);
  //  Traverse all half-edges around this strand and insert a new vertex into the corresponding segment meshes
  auto& graph = kin_del.getGraph();

  // compute boundary polygon for component at time t
  std::vector<bool> he_visited(graph.getHalfEdges().size(), false);
  size_t component_id = kin_del.component_data.component_map[strand_id];
  updateBoundary(t, he_visited, component_id);

  auto& boundary_polygon = kin_del.component_data.component_boundaries[component_id][0];
  auto centroid = polygonCentroid(boundary_polygon);

  // finish old meshes
  for (HalfEdgeDelaunayGraph::IncidentEdgeIterator it = graph.incidentEdgesBegin(strand_id),
                                                   end = graph.incidentEdgesEnd(strand_id);
       it != end; ++it) {
    finishMesh(*it, t, boundary_polygon);
  }

  size_t new_segment_id = segment_properties.size();

  // create a closing mesh
  size_t closing_mesh_index = createClosingMesh(strand_id, t, boundary_polygon, centroid);
  MeshStructure::SegmentMeshPair& segment_mesh_pair = segment_mesh_pairs[closing_mesh_index];
  segment_mesh_pair.segment_index0 = strand_to_segment_indices[strand_id].back();
  segment_mesh_pair.segment_index1 = new_segment_id;

  // Create a new segment mesh property for the new segment
  MeshStructure::SegmentProperties properties;
  segment_properties.push_back(properties);
  strand_to_segment_indices[strand_id].push_back(new_segment_id);

  // Start new meshes
  for (HalfEdgeDelaunayGraph::IncidentEdgeIterator it = graph.incidentEdgesBegin(strand_id),
                                                   end = graph.incidentEdgesEnd(strand_id);
       it != end; ++it) {
    startNewMesh(*it, t);

    // insert vertices into adjacent meshes
    auto& he = graph.getHalfEdges()[*it];

    size_t adjacent_he_id = he.next;
    auto& adjacent_he = graph.getHalfEdges()[adjacent_he_id];
    size_t adjacent_segment_mesh_pair_index = half_edge_index_to_segment_mesh_pair_index[adjacent_he_id];
    auto& adjacent_segment_mesh_pair = segment_mesh_pairs[adjacent_segment_mesh_pair_index];
    VoronoiMesh& adjacent_mesh = meshes[adjacent_segment_mesh_pair_index];

    // TODO: also apply clamping
    VoronoiPoint<3> vertex = computeVoronoiVertex(adjacent_he_id, t, adjacent_segment_mesh_pair_index);
    size_t new_vertex_index = addMeshletVertex(adjacent_mesh, boundary_polygon, centroid, vertex, strand_id, t);
    auto& last_vertices = segment_mesh_pair_last_left_and_right_vertex[adjacent_segment_mesh_pair_index];
    addMeshletTriangle(adjacent_mesh, last_vertices.first, last_vertices.second, new_vertex_index);

    if (adjacent_he_id % 2 == 0) {
      last_vertices.first = new_vertex_index;
    } else {
      last_vertices.second = new_vertex_index;
    }
  }
}

void SegmentBuilder::finalize(double t) {
  // Check if we need to insert a subdivision before handling this event
  while (subdivision_index < subdivisions.size() && subdivisions[subdivision_index].second <= t) {
    insertSubdivision(subdivisions[subdivision_index].first, subdivisions[subdivision_index].second);
    subdivision_index++;
  }

  updateBoundaries(t);

  // Finalize the segments by finishing all meshes
  auto& graph = kin_del.getGraph();
  size_t half_edge_count = graph.getHalfEdges().size();

  for (size_t i = 0; i < half_edge_count; i += 2) {
    auto vertex = graph.getHalfEdges()[i].origin;

    // fall back for infinite vertices
    if (vertex == -1) {
      vertex = graph.destination(i);
    }

    size_t component_index = kin_del.component_data.component_map[vertex];
    auto& boundary_points = kin_del.component_data.component_boundaries[component_index][0];

    finishMesh(i, t, boundary_points);
  }

  // finalize closing meshes
  for (size_t strand_id = 0; strand_id < graph.getVertexCount(); ++strand_id) {
    // create a closing mesh
    size_t component_index = kin_del.component_data.component_map[strand_id];
    auto& boundary_points = kin_del.component_data.component_boundaries[component_index][0];
    auto& centroid = kin_del.component_data.component_centroids[component_index];
    size_t closing_mesh_index = createClosingMesh(strand_id, t, boundary_points, centroid);
    MeshStructure::SegmentMeshPair& segment_mesh_pair = segment_mesh_pairs[closing_mesh_index];
    segment_mesh_pair.segment_index0 = strand_to_segment_indices[strand_id].back();
    segment_mesh_pair.segment_index1 = -1;
  }

  accumulateSegmentProperties();

  addVoronoiTriangulationToBoundaryMesh(t, true, 0.01);

  // compute normals
  for (auto& meshlet : meshes) {
    meshlet.computeNormals(NormalMode::PerTriangleCorner);
  }

  boundary_mesh.mergeDuplicateVertices();
  boundary_mesh.removeDegenerateTriangles();
  boundary_mesh.removeIsolatedVertices();
  boundary_mesh.computeNormals(NormalMode::PerTriangleCorner);

  finalized = true;  // Set the finalized flag to true
}

std::vector<VoronoiMesh> kinDS::SegmentBuilder::extractMeshes() const {
  return meshes;
}

std::pair<std::vector<VoronoiMesh>, std::vector<std::vector<int>>> kinDS::SegmentBuilder::extractSegmentMeshlets()
    const {
  std::vector<VoronoiMesh> meshlets;
  std::vector<std::vector<int>> neighbor_segments;  // accessed as [segment_id][triangle_index]
  for (size_t segment_id = 0; segment_id < segment_properties.size(); ++segment_id) {
    VoronoiMesh segment_mesh;
    std::vector<int> neighbor_segments_for_meshlet;
    const auto& properties = segment_properties[segment_id];
    for (size_t neighbor_index = 0; neighbor_index < properties.neighbor_count; ++neighbor_index) {
      size_t mesh_pair_index = properties.mesh_pair_indices[neighbor_index];
      const auto& mesh_pair = segment_mesh_pairs[mesh_pair_index];
      VoronoiMesh mesh = meshes[mesh_pair_index];
      if (segment_mesh_pairs[mesh_pair_index].segment_index0 != segment_id) {
        mesh.flipOrientation();
      }
      // Append the mesh to the segment mesh
      neighbor_segments_for_meshlet.insert(neighbor_segments_for_meshlet.end(), mesh.getTriangleCount(),
                                           properties.neighbor_indices[neighbor_index]);
      segment_mesh += mesh;
    }
    neighbor_segments.push_back(neighbor_segments_for_meshlet);
    segment_mesh.mergeDuplicateVertices(1e-4);
    meshlets.push_back(segment_mesh);
  }

  return std::make_pair(meshlets, neighbor_segments);
}

const VoronoiMesh& kinDS::SegmentBuilder::getBoundaryMesh() const {
  return boundary_mesh;
}

const std::vector<std::vector<size_t>>& kinDS::SegmentBuilder::getStrandToSegmentIndices() const {
  return strand_to_segment_indices;
}