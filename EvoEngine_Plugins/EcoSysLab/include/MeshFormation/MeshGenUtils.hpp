#pragma once

#include "Delaunator2D.hpp"

#include "Delaunator2D.hpp"

namespace eco_sys_lab_plugin {

/// @brief An undirected graph represented as an unordered adjacency list
/// @tparam VertexProperty any property that should be attached to the vertices, e.g. glm::vec2 for a geometric graph in
/// 2D space
template <typename VertexProperty>
class AdjacencyList {
 public:
  AdjacencyList() {
  }

  ~AdjacencyList() {
  }
  struct Edge {
    size_t m_source;
    size_t m_target;
  };

  std::vector<VertexProperty> m_vertices;
  std::vector<Edge> m_edges;

  // for now let's only implement undirected
  std::vector<std::vector<size_t> > m_outEdges;
  // std::vector<std::vector<size_t> > m_inEdges;

  VertexProperty& operator[](size_t i) {
    return m_vertices[i];
  }

  const VertexProperty& operator[](size_t i) const {
    return m_vertices[i];
  }

  static size_t source(Edge& e) {
    return e.m_source;
  }

  static size_t target(Edge& e) {
    return e.m_target;
  }

  size_t addVertex(const VertexProperty& property = VertexProperty()) {
    size_t index = m_vertices.size();

    m_vertices.push_back(property);
    m_outEdges.push_back({});

    return index;
  }

  void addEdge(size_t u, size_t v) {
    m_edges.push_back(Edge{u, v});

    m_outEdges[u].push_back(v);
    m_outEdges[v].push_back(u);
  }

  void clearEdges() {
    m_edges.clear();
    m_outEdges = std::vector<std::vector<size_t> >(m_vertices.size());
  }

  const std::vector<size_t>& adjacentVertices(size_t v) const {
    return m_outEdges[v];
  }
};

typedef AdjacencyList<glm::vec2> Graph;

inline void outputGraph(Graph& g, std::string filename, const std::vector<StrandHandle>& pipesInPrevious) {
  std::ofstream file(filename + ".plt");

  // header
  file << "set term wxt font \", 9\" enhanced\n";
  file << "set title\n";
  file << "set xlabel  # when no options, clear the xlabel\n";
  file << "set ylabel\n";
  file << "unset key\n";
  file << "set size ratio -1\n";
  file << "unset xtics\n";
  file << "unset ytics\n";
  file << "unset border\n\n";

  file << "set style arrow 1 nohead lc rgb \"black\"\n\n";

  file << "# edges\n\n";

  // list edges
  for (Graph::Edge& e : g.m_edges) {
    file << "set arrow from " << g[Graph::source(e)].x << "," << g[Graph::source(e)].y << " to "
         << g[Graph::target(e)].x << "," << g[Graph::target(e)].y << " as 1\n";
  }

  file << "# end of edges\n\n";

  file << "plot \"" << filename << ".v\" with points pt 7 ps 0.8 lt rgb \"blue\"\n";

  // now output vertices
  std::ofstream fileV(filename + ".v");

  for (size_t v = 0; v < g.m_vertices.size(); v++) {
    fileV << g[v].x << " " << g[v].y << "\n";
    file << "set label \"" << v << ":" << pipesInPrevious[v] << "\" at " << g[v].x << "," << g[v].y << "\n";
  }

  fileV.flush();
  fileV.close();

  file.flush();
  file.close();
}

/// @brief A depth first search starting at v
/// @param g Graph
/// @param v start vertex
/// @return component_members vector containing all vertices in the same connected component as v
inline std::vector<size_t> Dfs(const Graph& g, size_t v) {
  std::vector<bool> visited(g.m_vertices.size(), false);
  std::vector<size_t> component_members;
  std::vector<size_t> stack;
  stack.push_back(v);

  while (!stack.empty()) {
    size_t u = stack.back();
    stack.pop_back();

    if (!visited[u]) {
      component_members.push_back(u);
      visited[u] = true;
    }

    for (size_t av : g.adjacentVertices(u)) {
      if (!visited[av]) {
        stack.push_back(av);
      }
    }
  }

  return component_members;
}

/// @brief Compute a Delaunay triangulation of all vertices in vertex_subset and store the resulting edges in g
/// @param g graph, should contain no edges
/// @param removal_length threshold above which triangles should be removed if one of their sidelenghts exceeds it
/// @param vertex_subset vertices to be considered. This allows to only consider a subset of the graph vertices
inline void Delaunay(Graph& g, float removal_length, std::vector<size_t>& candidates) {
  std::vector<float> positions;

  for (size_t index : candidates) {
    positions.push_back(g[index].x);
    positions.push_back(g[index].y);
  }

  const Delaunator::Delaunator2D d(positions);
  for (std::size_t i = 0; i < d.triangles.size(); i += 3) {
    const auto& v0 = candidates[d.triangles[i]];
    const auto& v1 = candidates[d.triangles[i + 1]];
    const auto& v2 = candidates[d.triangles[i + 2]];

    if (glm::distance(g[v0], g[v1]) > removal_length || glm::distance(g[v1], g[v2]) > removal_length ||
        glm::distance(g[v0], g[v2]) > removal_length)
      continue;

    g.addEdge(v0, v1);
    g.addEdge(v1, v2);
    g.addEdge(v2, v0);
  }
}

/// @brief Get the next boundary vertex of the graph g in counter-clockwise (?) order
/// @param g graph to be worked on
/// @param cur current boundary vertex
/// @param prev previous boundary vertex
/// @param prev_angle angle describing the direction of a vector from the previous to the current boundary vertex
/// @return next vertex on boundary
inline size_t GetNextOnBoundary(Graph& g, size_t cur, size_t prev, float& prev_angle) {
  // first rotate prevAngel by 180 degrees because we are looking from the other side of the edge now

  if (prev_angle < 0)  // TODO: should this be <=
  {
    prev_angle += glm::pi<float>();
  } else {
    prev_angle -= glm::pi<float>();
  }

  size_t next = -1;
  float next_angle = std::numeric_limits<float>::infinity();

  // The next angle must either be the smallest that is larger than the current one or if such an angle does not exist
  // the smallest overall
  for (size_t av : g.adjacentVertices(cur)) {
    if (av == prev) {
      continue;
    }

    glm::vec2 dir = g[av] - g[cur];
    float angle = atan2f(dir.y, dir.x);

    // remap such that all angles are larger than the previous
    float test_angle = angle;
    if (test_angle < prev_angle) {
      test_angle += 2 * glm::pi<float>();
    }

    if (test_angle < next_angle) {
      next_angle = test_angle;
      next = av;
    }
  }

  if (next == -1) {
    EVOENGINE_WARNING("reached a leaf");

    return prev;
  }

  // confine to range
  if (next_angle > glm::pi<float>()) {
    next_angle -= 2 * glm::pi<float>();
  }

  // TODO: could we have a situation where we reach a leaf?
  prev_angle = next_angle;
  return next;
}

/// @brief Trace the boundary (i.e. outermost cycle) of a planar graph with connectivity of at least 2 in
/// counter-clockwise direction
/// @param g a planar graph
/// @param start_vertex a vertex on the boundary
/// @return vector containing all vertices on the boundary
inline std::vector<size_t> TraceBoundary(Graph& g, size_t start_vertex) {
  // traverse the boundary using the angles to its neighbors
  // going counterclockwise from here, the next point must be the one with the smallest angle
  size_t next = -1;
  float min_angle = glm::pi<float>();

  for (size_t av : g.adjacentVertices(start_vertex)) {
    glm::vec2 dir = g[av] - g[start_vertex];
    float angle = atan2f(dir.y, dir.x);
    if (angle < min_angle) {
      min_angle = angle;
      next = av;
    }
  }

  if (next == -1) {
    EVOENGINE_ERROR("leftmost index has no neighbors");
  }

  // from now on we will have to find the closest neighboring edge in counter-clockwise order for each step until we
  // reach the first index again
  size_t prev = start_vertex;
  size_t cur = next;
  float prev_angle = min_angle;

  std::vector<size_t> boundary{start_vertex};
  std::vector<bool> visited(g.m_vertices.size(), false);
  visited[start_vertex] = true;

  while (!visited[cur]) {
    boundary.push_back(cur);

    next = GetNextOnBoundary(g, cur, prev, prev_angle);
    prev = cur;
    cur = next;
  }

  // TODO: should find a solution for graphs that have a connectivity of 1.
  if (cur != start_vertex) {
    EVOENGINE_ERROR("Visited the same vertex twice when attempting to trace the boundary!");
  }

  return boundary;
}

/// @brief Find the leftmost vertex in a graoh
/// @param g graph
/// @param vertex_subset limit search to a subset of the vertices
/// @return leftmost vertex in graph g
inline size_t FindLeftmostIndex(Graph& g, std::vector<size_t>& vertex_subset) {
  size_t leftmost_index = 0;
  float leftmost_coord = std::numeric_limits<float>::infinity();

  for (size_t index : vertex_subset) {
    if (g[index].x < leftmost_coord) {
      leftmost_coord = g[index].x;
      leftmost_index = index;
    }
  }

  return leftmost_index;
}

inline auto GetSegPos(const StrandModel& strand_model, const StrandSegmentHandle seg_handle, float t = 1.0f)
    -> glm::vec3 {
  auto ret_val = strand_model.InterpolateStrandSegmentPosition(seg_handle, t);
  for (size_t i = 0; i < 3; i++) {
    if (std::isinf(ret_val[i])) {
      EVOENGINE_ERROR("Interpolated segment position is infinity");
    }

    if (std::isnan(ret_val[i])) {
      EVOENGINE_ERROR("Interpolated segment position is not a number");
    }
  }
  return ret_val;
}

inline glm::vec3 GetPipePos(const StrandModel& strand_model, const StrandHandle& pipe_handle, float t) {
  const auto& pipe = strand_model.strand_model_skeleton.data.strand_group.PeekStrand(pipe_handle);
  const auto seg_handle = pipe.PeekStrandSegmentHandles()[t];
  return GetSegPos(strand_model, seg_handle, fmod(t, 1.0));
}

inline void ForEachSegment(const StrandModelStrandGroup& pipes, const std::vector<StrandSegmentHandle>& seg_group,
                           std::function<void(const StrandSegment&)> func) {
  for (auto& seg_handle : seg_group) {
    auto& seg = pipes.PeekStrandSegment(seg_handle);
    func(seg);
  }
}

inline std::vector<StrandSegmentHandle> GetSegGroup(const StrandModelStrandGroup& pipes, size_t index) {
  std::vector<StrandSegmentHandle> seg_group;

  for (auto& pipe : pipes.PeekStrands()) {
    if (pipe.PeekStrandSegmentHandles().size() > index) {
      seg_group.push_back(pipe.PeekStrandSegmentHandles()[index]);
    } else {
      seg_group.push_back(-1);
    }
  }

  return seg_group;
}

inline auto RoundInDir(float val, const int dir) -> int {
  if (dir > 0) {
    return static_cast<int>(std::ceilf(val));
  }
  return static_cast<int>(std::floorf(val));
}

inline glm::ivec3 RoundInDir(glm::vec3 val, glm::ivec3 dir) {
  return glm::ivec3(RoundInDir(val[0], dir[0]), RoundInDir(val[1], dir[1]), RoundInDir(val[2], dir[2]));
}

inline void VerifyMesh(std::vector<Vertex>& vertices, std::vector<unsigned>& indices) {
  for (size_t index : indices) {
    if (index >= vertices.size()) {
      EVOENGINE_ERROR(std::string("index ") + std::to_string(index) + " is out of range");
    }
  }
}
}  // namespace eco_sys_lab_plugin