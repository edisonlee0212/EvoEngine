#pragma once

namespace eco_sys_lab {
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

  size_t addVertex() {
    size_t index = m_vertices.size();

    m_vertices.push_back(VertexProperty());
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

void outputGraph(Graph& g, std::string filename, const std::vector<StrandHandle>& pipesInPrevious) {
  std::ofstream file(filename + ".plt");
  // std::cout << "outputting graph with filename " << filename << std::endl;

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
  // Graph::edge_iterator e, eend;
  for (Graph::Edge& e : g.m_edges) {
    file << "set arrow from " << g[Graph::source(e)].x << "," << g[Graph::source(e)].y << " to "
         << g[Graph::target(e)].x << "," << g[Graph::target(e)].y << " as 1\n";
  }

  file << "# end of edges\n\n";

  file << "plot \"" << filename << ".v\" with points pt 7 ps 0.8 lt rgb \"blue\"\n";

  // now output vertices
  std::ofstream fileV(filename + ".v");

  // Graph::vertex_iterator v, vend;
  for (size_t v = 0; v < g.m_vertices.size(); v++) {
    fileV << g[v].x << " " << g[v].y << "\n";
    file << "set label \"" << v << ":" << pipesInPrevious[v] << "\" at " << g[v].x << "," << g[v].y << "\n";
  }

  fileV.flush();
  fileV.close();

  file.flush();
  file.close();
}

class Grid2D {
  float m_rasterSize = 0.0f;
  glm::ivec2 m_minIndex;
  std::vector<std::vector<std::vector<size_t> > > m_gridCells;

  void handleCells(Graph& g, float maxDistanceSqr, const std::vector<size_t>& cell0,
                   const std::vector<size_t>& cell1) const {
    for (size_t u : cell0) {
      for (size_t v : cell1) {
        if (u == v) {
          continue;
        }

        // std::cout << "comparing vertex " << u << " to " << v << " with distance " << std::sqrt(glm::distance2(g[u],
        // g[v])) << std::endl;
        if (glm::distance2(g[u], g[v]) < maxDistanceSqr) {
          // std::cout << "adding edge " << u << " to " << v << std::endl;
          g.addEdge(u, v);
        }
      }
    }
  }

 public:
  Grid2D(float rasterSize, glm::vec2 min, glm::vec2 max) : m_rasterSize(rasterSize) {
    // std::cout << "setting up grid with raster size: " << rasterSize << ", min: " << min << ", max: " << max <<
    // std::endl;
    m_minIndex = glm::floor(min / rasterSize);
    glm::ivec2 maxIndex = glm::floor(max / rasterSize);

    m_gridCells = std::vector<std::vector<std::vector<size_t> > >(
        maxIndex[0] - m_minIndex[0] + 1, std::vector<std::vector<size_t> >(maxIndex[1] - m_minIndex[1] + 1));

    // std::cout << "maxIndex: " << maxIndex << ", minIndex: " << m_minIndex << std::endl;
  }
  ~Grid2D() {
  }

  void insert(const Graph& g, size_t vertexIndex) {
    const glm::vec2& pos = g[vertexIndex];
    glm::ivec2 index = glm::ivec2(glm::floor(pos / m_rasterSize)) - m_minIndex;

    // std::cout << "inserting vertex " << vertexIndex << " into grid cell " << index << std::endl;

    m_gridCells[index[0]][index[1]].push_back(vertexIndex);
  }

  void connectNeighbors(Graph& g, float maxDistance) const {
    float maxDistanceSqr = maxDistance * maxDistance;
    // std::cout << "max distance: " << maxDistance << std::endl;

    // simply check all points against each other in neighboring cells (including diagonal)
    for (size_t i = 0; i < m_gridCells.size() - 1; i++) {
      for (size_t j = 0; j < m_gridCells[i].size() - 1; j++) {
        handleCells(g, maxDistanceSqr, m_gridCells[i][j], m_gridCells[i][j]);
        handleCells(g, maxDistanceSqr, m_gridCells[i][j], m_gridCells[i][j + 1]);
        handleCells(g, maxDistanceSqr, m_gridCells[i][j], m_gridCells[i + 1][j]);
        handleCells(g, maxDistanceSqr, m_gridCells[i][j], m_gridCells[i + 1][j + 1]);
        handleCells(g, maxDistanceSqr, m_gridCells[i + 1][j], m_gridCells[i][j + 1]);
      }
    }

    // handle last row and column
    for (size_t i = 0; i < m_gridCells.size() - 1; i++) {
      handleCells(g, maxDistanceSqr, m_gridCells[i].back(), m_gridCells[i].back());
      handleCells(g, maxDistanceSqr, m_gridCells[i].back(), m_gridCells[i + 1].back());
    }

    for (size_t j = 0; j < m_gridCells.back().size() - 1; j++) {
      handleCells(g, maxDistanceSqr, m_gridCells.back()[j], m_gridCells.back()[j]);
      handleCells(g, maxDistanceSqr, m_gridCells.back()[j], m_gridCells.back()[j + 1]);
    }

    handleCells(g, maxDistanceSqr, m_gridCells.back().back(), m_gridCells.back().back());
  }
};
}  // namespace eco_sys_lab