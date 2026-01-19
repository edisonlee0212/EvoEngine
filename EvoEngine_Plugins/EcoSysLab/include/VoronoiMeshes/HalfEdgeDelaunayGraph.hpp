#pragma once
#include <array>
#include <vector>
#include "CubicHermiteSpline.hpp"
#include "Delaunator2D.hpp"

namespace kinDS {
// A graph that can represent the delaunay triangulation such that edges are explicitly stored and can be flipped in its
// quadrilateral.
class HalfEdgeDelaunayGraph {
 public:
  struct HalfEdge {
    int origin = -1;  // index into vertices
    int next = -1;    // index into half_edges, always represents the next half-edge in the face
    int face = -1;    // index into faces
    // twin = index ^ 1
  };

  struct Triangle {
    std::array<size_t, 3> half_edges;
  };

 private:
  size_t vertex_count = 0;  // Number of vertices in the triangulation
  std::vector<Triangle> triangles;
  std::vector<HalfEdge> half_edges;         // List of half-edges in the triangulation
  std::vector<size_t> vertex_to_half_edge;  // Maps each vertex to one of its outgoing half-edges for easy access

  /**
   * @brief Build the data structure from an index buffer of triangles.
   *
   * Constructs the half-edge data structure from a list of triangle indices. Construction is done in O(n) time.
   * Twin edges are stored implicitly by storing them next to each other. The twin index can be computed as `index ^ 1`.
   *
   * @param index_buffer index buffer of triangles, size must be multiple of 3 to be valid and each index must be in
   * range [0, vertex_count).
   */
  void build(const std::vector<size_t>& index_buffer);

 public:
  HalfEdgeDelaunayGraph() = default;

  void init(const std::vector<std::vector<VoronoiPoint<2>>>& splines);
  // Flips an edge between two triangles by rotating it counter-clockwise in its quadrilateral
  void flipEdge(size_t he_id);
  // Other methods to manipulate and query the triangulation can be added here.
  void printDebug() const;

  static VoronoiPoint<2> circumcenter(const VoronoiPoint<2>& a, const VoronoiPoint<2>& b, const VoronoiPoint<2>& c);

  std::vector<std::pair<VoronoiPoint<2>, bool>> computeCircumcenters(
      const std::vector<VoronoiPoint<2>>& vertices) const;

  // utils

  /**
   * Determines whether a half-edge is outside the boundary. This includes all edges from/to infinity as well as the
   * outer half-edges along the boundary.
   */
  bool isOutsideConvexBoundary(size_t he_id) const;

  /**
   * Determines whether a half-edge is on the boundary. This includes both, the inner and the outer half-edges along the
   * boundary.
   */
  bool isOnConvexBoundary(size_t he_id) const;

  /**
   * Determines whether a half-edge is on the outer side along the boundary.
   */
  bool isOnConvexBoundaryOutside(size_t he_id) const;

  /**
   * Determines whether a half-edge is connected to the vertex at infinity.
   */
  bool isInfinite(size_t he_id) const;
  int destination(size_t he_id) const;
  int triangleOppositeVertex(size_t he_id) const;
  std::array<int, 3> adjacentTriangleVertices(size_t he_id) const;
  size_t neighborEdgeId(size_t he_id) const;
  size_t nextOnConvexBoundaryId(size_t he_id) const;

  std::vector<size_t> neighbors(size_t v);
  std::vector<size_t> inducedNeighbors(size_t v, const std::vector<bool>& face_inside) const;

  static size_t twin(size_t he_id);

  // getters
  const std::vector<HalfEdge>& getHalfEdges() const;
  const std::vector<Triangle>& getFaces() const;
  size_t getVertexCount() const;

  // ---------------- Iterator definition ----------------
  class IncidentEdgeIterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = size_t;  // returning halfedge indices
    using difference_type = std::ptrdiff_t;
    using pointer = const size_t*;
    using reference = const size_t&;

    IncidentEdgeIterator(const HalfEdgeDelaunayGraph* g, size_t v, bool end = false)
        : g_(g),
          v_(v),
          start_he_(g ? g->vertex_to_half_edge[v] : npos),
          curr_he_(end ? npos : start_he_),
          first_(true) {
    }

    value_type operator*() const {
      return curr_he_;
    }

    IncidentEdgeIterator& operator++() {
      if (curr_he_ == npos)
        return *this;  // already at end

      curr_he_ = g_->neighborEdgeId(curr_he_);

      // if we are back at start, mark as end
      if (curr_he_ == start_he_ && !first_) {
        curr_he_ = npos;
      }
      first_ = false;
      return *this;
    }

    IncidentEdgeIterator operator++(int) {
      IncidentEdgeIterator tmp = *this;
      ++(*this);
      return tmp;
    }

    bool operator==(const IncidentEdgeIterator& other) const {
      return curr_he_ == other.curr_he_ && v_ == other.v_ && g_ == other.g_;
    }

    bool operator!=(const IncidentEdgeIterator& other) const {
      return !(*this == other);
    }

   private:
    const HalfEdgeDelaunayGraph* g_;
    size_t v_;
    size_t start_he_;
    size_t curr_he_;
    bool first_;
    static constexpr size_t npos = static_cast<size_t>(-1);
  };

  // helper functions to get iterator ranges
  IncidentEdgeIterator incidentEdgesBegin(size_t v) const {
    return IncidentEdgeIterator(this, v, false);
  }

  IncidentEdgeIterator incidentEdgesEnd(size_t v) const {
    return IncidentEdgeIterator(this, v, true);
  }

  // ---------------- Iterator definition ----------------
  class ConvexHullEdgeIterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = size_t;  // returning halfedge indices
    using difference_type = std::ptrdiff_t;
    using pointer = const size_t*;
    using reference = const size_t&;

    ConvexHullEdgeIterator(const HalfEdgeDelaunayGraph* g, size_t he_id, bool end = false)
        : g_(g), start_he_(he_id), curr_he_(end ? npos : he_id), first_(true) {
      if (!end) {
        assert(g_->triangleOppositeVertex(curr_he_) == -1 && "Iterator started on non-boundary half-edge!");
      }
    }

    value_type operator*() const {
      return curr_he_;
    }

    ConvexHullEdgeIterator& operator++() {
      if (curr_he_ == npos)
        return *this;  // already at end

      curr_he_ = g_->nextOnConvexBoundaryId(curr_he_);

      assert(g_->triangleOppositeVertex(curr_he_) == -1 && "Iterator moved to non-boundary half-edge!");

      // if we are back at start, mark as end
      if (curr_he_ == start_he_ && !first_) {
        curr_he_ = npos;
      }
      first_ = false;
      return *this;
    }

    ConvexHullEdgeIterator operator++(int) {
      ConvexHullEdgeIterator tmp = *this;
      ++(*this);
      return tmp;
    }

    bool operator==(const ConvexHullEdgeIterator& other) const {
      return curr_he_ == other.curr_he_ && g_ == other.g_;
    }

    bool operator!=(const ConvexHullEdgeIterator& other) const {
      return !(*this == other);
    }

   private:
    const HalfEdgeDelaunayGraph* g_;
    size_t start_he_;
    size_t curr_he_;
    bool first_;
    static constexpr size_t npos = static_cast<size_t>(-1);
  };

  // helper functions to get iterator ranges
  ConvexHullEdgeIterator boundaryEdgesBegin() const {
    size_t start_boundary_edge_index = static_cast<size_t>(-1);

    // TODO: this could be more efficient if we store and maintain a boundary edge
    for (size_t i = 0; i < getHalfEdges().size(); i++) {
      if (isOnConvexBoundaryOutside(i)) {
        start_boundary_edge_index = i;
        break;
      }
    }

    if (start_boundary_edge_index != static_cast<size_t>(-1)) {
      return ConvexHullEdgeIterator(this, start_boundary_edge_index, false);
    } else {
      return boundaryEdgesEnd();
    }
  }

  ConvexHullEdgeIterator boundaryEdgesEnd() const {
    return ConvexHullEdgeIterator(this, 0, true);
  }
};
}  // namespace kinDS