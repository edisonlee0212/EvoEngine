#pragma once
#include <format>
#include <queue>
#include "BranchTrajectories.hpp"
#include "CubicHermiteSpline.hpp"
#include "HalfEdgeDelaunayGraph.hpp"
#include "Polynomial.hpp"
#include "ProgressBar.hpp"

namespace kinDS {
struct BoundaryPoint {
  size_t vertex_id;
  size_t he_id;
  VoronoiPoint<2> p;
};

static VoronoiPoint<2> polygonCentroid(const std::vector<BoundaryPoint>& polygon) {
  double A = 0.0;
  VoronoiPoint<2> C{0.0, 0.0};

  const size_t n = polygon.size();
  for (size_t i = 0; i < n; ++i) {
    const VoronoiPoint<2>& p = polygon[i].p;
    const VoronoiPoint<2>& q = polygon[(i + 1) % n].p;

    double cross = p % q;
    A += cross;
    C += (p + q) * cross;
  }

  A *= 0.5;

  if (std::abs(A) < 1e-12)
    return C;  // degenerate polygon

  return C / (6.0 * A);
}

static std::vector<size_t> buildComponentMap(const std::vector<std::vector<size_t>>& components, size_t vertex_count) {
  std::vector<size_t> component_map(vertex_count);

  for (size_t i = 0; i < components.size(); i++) {
    for (const auto v : components[i]) {
      component_map[v] = i;
    }
  }

  return component_map;
}

/**
 * \brief Class for computing the Delaunay triangulation of a set of cubic Hermite splines.
 *
 * This follows Guibas, L.J., Mitchell, J.S.B., Roos, T. (1992).
 * Voronoi diagrams of moving points in the plane. In:
 * Schmidt, G., Berghammer, R. (eds) Graph-Theoretic Concepts in Computer Science. WG 1991.
 * Lecture Notes in Computer Science, vol 570. Springer, Berlin, Heidelberg.
 * https://doi.org/10.1007/3-540-55121-2_11
 */
class KineticDelaunay {
 public:
  class Event {
   public:
    double time;           // Time of the event
    size_t half_edge_id;   // Half-edge index associated with the event
    double creation_time;  // Time when the event was created, used do check validity after a quadrilateral is updated
    VoronoiPoint<2> position;  // Position of the event

    enum Type { SWAP, BOUNDARY, RIGHT_ANGLED } type;

    Event(double t, size_t he_id, double creation_time, VoronoiPoint<2> position, Type type)
        : time(t), half_edge_id(he_id), creation_time(creation_time), position(position), type(type) {
    }

    bool operator<(const Event& other) const {
      return time > other.time;  // For priority queue, we want the earliest event first
    }
  };

  // EventHandler class, inherit from this class to handle events in the KineticDelaunay algorithm
  class EventHandler {
   public:
    virtual ~EventHandler() = default;
    /**
     * \brief Handle a SWAP event before it is processed, i.e. before any edges are swapped
     *
     * @param e The event to handle.
     */
    virtual void beforeEvent(Event& e) {
    }

    /**
     * \brief Handle a SWAP event after it is processed, i.e. after edges are swapped
     *
     * @param e The event to handle.
     */
    virtual void afterEvent(Event& e) {
    }

    /**
     * \brief Handle a BOUNDARY event before it is processed
     *
     * @param e The event to handle
     */
    virtual void beforeBoundaryEvent(Event& e) {
    }

    /**
     * \brief Handle a BOUNDARY event after it is processed
     *
     * @param e The event to handle
     */
    virtual void afterBoundaryEvent(Event& e) {
    }

    virtual void betweenSections(size_t index) {
    }

    /**
     * \brief initialize event handler.
     */
    virtual void init() {
    }

    /**
     * \brief Finalize after all events have been handled
     */
    virtual void finalize(double t) {
    }
  };

  struct ComponentData {
    std::vector<std::vector<size_t>> components;
    std::vector<size_t> component_map;
    // [component_index][boundary_no][point_no] - the first boundary is the outer one, any additional ones are holes in
    // the polygon
    std::vector<std::vector<std::vector<BoundaryPoint>>> component_boundaries;
    std::vector<VoronoiPoint<2>> component_centroids;
    std::vector<double> component_last_updated;
  };

  ComponentData component_data;

 private:
  typedef std::priority_queue<Event> EventQueue;

  // std::vector<CubicHermiteSpline<2>> splines;
  BranchTrajectories branch_trajs;
  HalfEdgeDelaunayGraph graph;
  EventQueue events;
  size_t sections_advanced = 0;               // Counter for the number of sections advanced
  double cutoff;                              // Cutoff radius for boundary events
  std::vector<bool> face_inside;              // Tracks whether faces are inside or outside the boundary
  std::vector<std::vector<size_t>> branches;  // track which vertices/splines belong to which branch
  std::vector<VoronoiPoint<2>> dummy_boundary;
  bool add_dummy_boundary;
  const std::vector<std::vector<size_t>> branch_indices;  // Create a branch index lookup using [strand_id][h]
  std::vector<std::vector<std::vector<size_t>>>
      strands_by_branch_id;  // Maintain the branches as [h][branch_id][strand_no]
  size_t prev_component_count = 1;

  /* Compare to Leonidas Guibas and Jorge Stolfi. 1985. Primitives for the manipulation of general subdivisions and the
   * computation of Voronoi. ACM Trans. Graph. 4, 2 (April 1985), 74–123. https://doi.org/10.1145/282918.282923
   */
  static Polynomial inCircle(const Polynomial& ax, const Polynomial& ay, const Polynomial& bx, const Polynomial& by,
                             const Polynomial& cx, const Polynomial& cy, const Polynomial& px, const Polynomial& py) {
    const Polynomial dx = ax - px;
    const Polynomial dy = ay - py;
    const Polynomial ex = bx - px;
    const Polynomial ey = by - py;
    const Polynomial fx = cx - px;
    const Polynomial fy = cy - py;

    const Polynomial ap = dx * dx + dy * dy;
    const Polynomial bp = ex * ex + ey * ey;
    const Polynomial cp = fx * fx + fy * fy;

    return (dx * (ey * cp - bp * fy) - dy * (ex * cp - bp * fx) + ap * (ex * fy - ey * fx));
  }

  static Polynomial ccw(const Polynomial& ax, const Polynomial& ay, const Polynomial& bx, const Polynomial& by,
                        const Polynomial& cx, const Polynomial& cy) {
    return (ax * by) + (bx * cy) + (cx * ay) - (ay * bx) - (by * cx) - (cy * ax);
  }

  // Polynomial that evaluates to zero iff the distance from A to the circumcenter equals the value r
  static Polynomial circumradiusEquals(const Polynomial& ax, const Polynomial& ay, const Polynomial& bx,
                                       const Polynomial& by, const Polynomial& cx, const Polynomial& cy, double r) {
    // We first do the same computations as for the circumcenter
    Polynomial D = (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by)) * 2.0;

    // only compute the numerators
    Polynomial Nx =
        ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by));
    Polynomial Ny =
        ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax));

    // By taking the distance formula between the circumcenter and the point A and setting it equal to r, we get the
    // following after rearranging:
    Polynomial circumradius_eq = (Nx - ax * D) * (Nx - ax * D) + (Ny - ay * D) * (Ny - ay * D) - (D * D * (r * r));
    return circumradius_eq;
  }

  // Polynomial that evaluates to zero iff there is a right angle at c
  static Polynomial rightAngled(const Polynomial& ax, const Polynomial& ay, const Polynomial& bx, const Polynomial& by,
                                const Polynomial& cx, const Polynomial& cy) {
    return (ax - cx) * (bx - cx) + (ay - cy) * (by - cy);
  }

  double circumradius(const VoronoiPoint<2>& p0, const VoronoiPoint<2>& p1, const VoronoiPoint<2>& p2) {
    const double x0 = p0[0], y0 = p0[1];
    const double x1 = p1[0], y1 = p1[1];
    const double x2 = p2[0], y2 = p2[1];

    // Side lengths
    const double a = std::hypot(x1 - x2, y1 - y2);
    const double b = std::hypot(x0 - x2, y0 - y2);
    const double c = std::hypot(x0 - x1, y0 - y1);

    // Twice the triangle area (cross product magnitude)
    const double area2 = std::abs((x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0));

    if (area2 == 0.0) {
      throw std::runtime_error("Degenerate triangle: circumradius undefined");
    }

    // R = (a * b * c) / (4 * A), and area2 = 2 * A
    return (a * b * c) / (2.0 * area2);
  }

  void computeRightAngleEvents(double t, size_t he_id) {
    // generally don't do this for now:
    return;

    if (cutoff == std::numeric_limits<double>::infinity()) {
      // no boundary events wanted
      return;
    }

    const size_t section = static_cast<size_t>(t);
    const float fraction = t - section;

    size_t face_id = graph.getHalfEdges()[he_id].face;
    size_t u = graph.getHalfEdges()[he_id].origin;
    size_t v = graph.destination(he_id);
    size_t w = graph.triangleOppositeVertex(he_id);

    if (u == -1 || v == -1 || w == -1) {
      // one of the vertices is at infinity, no event possible
      return;
    }

    std::vector<Trajectory<2>> trajs;

    trajs.push_back(branch_trajs.getPiecePolynomial(u, section));
    trajs.push_back(branch_trajs.getPiecePolynomial(v, section));
    trajs.push_back(branch_trajs.getPiecePolynomial(w, section));

    Polynomial event_trigger =
        rightAngled(trajs[0][0], trajs[0][1], trajs[1][0], trajs[1][1], trajs[2][0], trajs[2][1]);

    event_trigger.trim();
    auto zeros = event_trigger.realRoots();

    // print roots:
    for (const auto& root : zeros) {
      if (isnan(root)) {
        continue;  // Skip NaN roots
      }
      if (root > fraction && root <= 1) {  // Check if the root is within the valid range
        double event_time = root + section;
        // std::cout << "Root found at t = " << event_time << std::endl;

        VoronoiPoint<2> center{};

        for (const auto& traj : trajs) {
          center[0] += traj[0](root);
          center[1] += traj[1](root);
        }
        center[0] /= trajs.size();
        center[1] /= trajs.size();
        /*EVOENGINE_LOG("Right Angle Event at time " << event_time << " for half-edge ID " << he_id
                                                   << " at center position " << center.toString().c_str());*/

        events.emplace(Event(event_time, he_id, t, center,
                             Event::RIGHT_ANGLED));  // Store the event with the time and half-edge index
      }
    }
  }

  void computeBoundaryEvents(double t, size_t he_id) {
    if (cutoff == std::numeric_limits<double>::infinity()) {
      // no boundary events wanted
      return;
    }

    const size_t section = static_cast<size_t>(t);
    const float fraction = t - section;

    size_t face_id = graph.getHalfEdges()[he_id].face;
    size_t u = graph.getHalfEdges()[he_id].origin;
    size_t v = graph.destination(he_id);
    size_t w = graph.triangleOppositeVertex(he_id);

    if (u == -1 || v == -1 || w == -1) {
      // one of the vertices is at infinity, no event possible
      return;
    }

    std::vector<Trajectory<2>> trajs;

    trajs.push_back(branch_trajs.getPiecePolynomial(u, section));
    trajs.push_back(branch_trajs.getPiecePolynomial(v, section));
    trajs.push_back(branch_trajs.getPiecePolynomial(w, section));

    Polynomial event_trigger =
        circumradiusEquals(trajs[0][0], trajs[0][1], trajs[1][0], trajs[1][1], trajs[2][0], trajs[2][1], cutoff);

    event_trigger.trim();
    auto zeros = event_trigger.realRoots();

    // print roots:
    for (const auto& root : zeros) {
      if (isnan(root)) {
        continue;  // Skip NaN roots
      }
      if (root > fraction && root <= 1) {  // Check if the root is within the valid range
        double event_time = root + section;
        // std::cout << "Root found at t = " << event_time << std::endl;

        VoronoiPoint<2> center{};

        for (const auto& traj : trajs) {
          center[0] += traj[0](root);
          center[1] += traj[1](root);
        }
        center[0] /= trajs.size();
        center[1] /= trajs.size();
        /*EVOENGINE_LOG("Boundary Event at time " << event_time << " for half-edge ID " << he_id << " at center position
           "
                                                << center.toString().c_str());*/

        events.emplace(
            Event(event_time, he_id, t, center, Event::BOUNDARY));  // Store the event with the time and half-edge index
      }
    }
  }

  void computeSwapEvents(double t, size_t quad_id) {
    const size_t section = static_cast<size_t>(t);
    const float fraction = t - section;

    size_t he_id = quad_id * 2;
    Polynomial event_trigger;

    std::vector<Trajectory<2>> trajs;

    if (graph.isOnConvexBoundary(he_id) || graph.isOutsideConvexBoundary(he_id)) {
      // boundary edges must be treated separately using ccw

      // need to get the inner half-edge so we have access to the triangle
      // in case both are outside, this swap does not matter, so we just let it happen
      if (graph.isOutsideConvexBoundary(he_id)) {
        he_id = he_id ^ 1;  // use the twin half-edge if the current one is on the boundary
      }

      // Depending on the half-edge, the infinite vertex could be in different places, so we just collect all and filter
      // it out
      int indices[4];
      indices[0] = graph.getHalfEdges()[he_id].origin;       // First vertex
      indices[1] = graph.triangleOppositeVertex(he_id ^ 1);  // Second vertex
      indices[2] = graph.getHalfEdges()[he_id ^ 1].origin;   // Third vertex
      indices[3] = graph.triangleOppositeVertex(he_id);      // Fourth vertex

      std::vector<int> filtered_indices;

      std::copy_if(indices, indices + 4, std::back_inserter(filtered_indices), [this](int index) {
        return index != -1;
      });

      int& a = filtered_indices[0];  // First vertex
      int& b = filtered_indices[1];  // Second vertex
      int& c = filtered_indices[2];  // Third vertex

      // print the triangle vertices:
      // std::cout << "Triangle vertices: " << a << ", " << b << ", " << c << std::endl;

      trajs.push_back(branch_trajs.getPiecePolynomial(a, section));
      trajs.push_back(branch_trajs.getPiecePolynomial(b, section));
      trajs.push_back(branch_trajs.getPiecePolynomial(c, section));

      event_trigger = ccw(trajs[0][0], trajs[0][1], trajs[1][0], trajs[1][1], trajs[2][0], trajs[2][1]);
    } else {
      int a = graph.getHalfEdges()[he_id].origin;       // First vertex
      int b = graph.triangleOppositeVertex(he_id ^ 1);  // Second vertex
      int c = graph.getHalfEdges()[he_id ^ 1].origin;   // Third vertex
      int d = graph.triangleOppositeVertex(he_id);      // Fourth vertex

      // print the quadrilateral vertices:
      // std::cout << "Quadrilateral vertices: " << a << ", " << b << ", " << c << ", " << d << std::endl;

      trajs.push_back(branch_trajs.getPiecePolynomial(a, section));
      trajs.push_back(branch_trajs.getPiecePolynomial(b, section));
      trajs.push_back(branch_trajs.getPiecePolynomial(c, section));
      trajs.push_back(branch_trajs.getPiecePolynomial(d, section));

      event_trigger = inCircle(trajs[0][0], trajs[0][1], trajs[1][0], trajs[1][1], trajs[2][0], trajs[2][1],
                               trajs[3][0], trajs[3][1]);
    }
    event_trigger.trim();
    auto zeros = event_trigger.realRoots();

    // print roots:
    for (const auto& root : zeros) {
      if (isnan(root)) {
        continue;  // Skip NaN roots
      }

      if (root > fraction && root <= 1) {
        // Check if the root is within the valid range
        double event_time = root + section;
        // std::cout << "Root found at t = " << event_time << std::endl;

        VoronoiPoint<2> center{};

        for (const auto& traj : trajs) {
          center[0] += traj[0](root);
          center[1] += traj[1](root);
        }
        center[0] /= trajs.size();
        center[1] /= trajs.size();

        /*EVOENGINE_LOG("Swap Event at time " << event_time << " for half-edge ID " << he_id << " at center position "
                                            << center.toString().c_str());*/

        events.emplace(
            Event(event_time, he_id, t, center, Event::SWAP));  // Store the event with the time and half-edge index
      }
    }
  }

  void precomputeStep(double t) {
    // TODO: Make sure it works where no change of sign occurs in the polynomial, i.e., roots that do not lead to a
    // change in the triangulation.
    size_t quad_count = graph.getHalfEdges().size() / 2;
    for (size_t i = 0; i < quad_count; i++) {
      computeSwapEvents(t, i);
    }

    size_t he_count = graph.getHalfEdges().size();
    for (size_t i = 0; i < face_inside.size(); i++) {
      size_t he_id = graph.getFaces()[i].half_edges[0];
      computeBoundaryEvents(t, he_id);
    }

    for (size_t he_id = 0; he_id < he_count; he_id++) {
      computeRightAngleEvents(t, he_id);
    }
  }

  void handleSwapEvent(EventHandler& event_handler, Event& event, std::vector<double>& quadrilateral_last_updated) {
    // Check if the event is still valid
    if (event.creation_time < quadrilateral_last_updated[event.half_edge_id / 2]) {
      // This event is outdated, skip it
      return;
    }

    // Process the event at the given time
    size_t face_id = graph.getHalfEdges()[event.half_edge_id].face;
    size_t twin_face_id = graph.getHalfEdges()[event.half_edge_id ^ 1].face;
    /*EVOENGINE_LOG("Processing swap event at time " << event.time << " for half-edge ID " << event.half_edge_id
                                                   << ". Faces inside " << face_inside[face_id] << " | "
                                                   << face_inside[twin_face_id]);*/

    // Call the event handler if provided
    event_handler.beforeEvent(event);

    // Faces swapped to the inside start out with an infinite circumradius, therefore their state depends on the cutoff
    if (graph.getHalfEdges()[event.half_edge_id].origin == -1) {
      /*EVOENGINE_LOG("Swapping face of half-edge " << event.half_edge_id << " to the inside at t = " << event.time);
      face_inside[twin_face_id] = (cutoff == std::numeric_limits<double>::infinity());*/
    }

    if (graph.getHalfEdges()[event.half_edge_id ^ 1].origin == -1) {
      /*EVOENGINE_LOG("Swapping face of twin half-edge " << (event.half_edge_id ^ 1)
                                                       << " to the inside at t = " << event.time);*/
      face_inside[face_id] = (cutoff == std::numeric_limits<double>::infinity());
    }

    /*EVOENGINE_LOG("Pre-flip: " << event.time << " for half-edge ID " << event.half_edge_id << ". Faces inside "
                               << face_inside[face_id] << " | " << face_inside[twin_face_id]);*/

    graph.flipEdge(event.half_edge_id);

    /*EVOENGINE_LOG("Post-flip:  " << event.time << " for half-edge ID " << event.half_edge_id << ". Faces inside "
                                 << face_inside[face_id] << " | " << face_inside[twin_face_id]);*/

    // one of the triangles might have been swapped outside
    auto tri_verts1 = graph.adjacentTriangleVertices(event.half_edge_id);

    for (auto& v : tri_verts1) {
      if (v == -1) {
        size_t face_id = graph.getHalfEdges()[event.half_edge_id].face;
        /*EVOENGINE_LOG("Swapped face " << face_id << " of half-edge " << event.half_edge_id
                                      << " to the outside at t = " << event.time);*/
        setFaceInside(face_id, false);
      }
    }

    auto tri_verts2 = graph.adjacentTriangleVertices(event.half_edge_id ^ 1);
    for (auto& v : tri_verts2) {
      if (v == -1) {
        size_t face_id = graph.getHalfEdges()[event.half_edge_id ^ 1].face;
        /*EVOENGINE_LOG("Swapped face " << face_id << " of half-edge " << (event.half_edge_id ^ 1)
                                      << " to the outside at t = " << event.time);*/
        setFaceInside(face_id, false);
      }
    }

    /*EVOENGINE_LOG("Processed swap event at time " << event.time << " for half-edge ID " << event.half_edge_id
                                                  << ". Faces inside " << face_inside[face_id] << " | "
                                                  << face_inside[twin_face_id]);*/

    // After flipping the edge, we need to recompute the events for all surrounding half-edges
    size_t next1 = graph.getHalfEdges()[event.half_edge_id].next;
    size_t next2 = graph.getHalfEdges()[next1].next;

    size_t twin_next1 = graph.getHalfEdges()[event.half_edge_id ^ 1].next;
    size_t twin_next2 = graph.getHalfEdges()[twin_next1].next;

    computeSwapEvents(event.time, next1 / 2);
    quadrilateral_last_updated[next1 / 2] = event.time;  // Update the last updated time for the quadrilateral

    computeSwapEvents(event.time, next2 / 2);
    quadrilateral_last_updated[next2 / 2] = event.time;  // Update the last updated time for the quadrilateral

    computeSwapEvents(event.time, twin_next1 / 2);
    quadrilateral_last_updated[twin_next1 / 2] = event.time;  // Update the last updated time for the quadrilateral

    computeSwapEvents(event.time, twin_next2 / 2);
    quadrilateral_last_updated[twin_next2 / 2] = event.time;  // Update the last updated time for the quadrilateral

    event_handler.afterEvent(event);  // Call the event handler after processing the event
  }

  void handleBoundaryEvent(EventHandler& event_handler, Event& event) {
    assert(event.type == Event::BOUNDARY);
    // Process the event at the given time
    // EVOENGINE_LOG("Processing boundary event at time " << event.time << " for half-edge ID " << event.half_edge_id);
    // Call the event handler if provided
    event_handler.beforeBoundaryEvent(event);
    size_t face_id = graph.getHalfEdges()[event.half_edge_id].face;
    setFaceInside(face_id, !face_inside[face_id]);

    event_handler.afterBoundaryEvent(event);
  }

  void handleEvents(EventHandler& event_handler) {
    std::vector<double> quadrilateral_last_updated(graph.getHalfEdges().size() / 2, 0.0);
    while (!events.empty()) {
      Event event = events.top();
      events.pop();

      switch (event.type) {
        case Event::SWAP:
          handleSwapEvent(event_handler, event, quadrilateral_last_updated);
          break;
        case Event::BOUNDARY:
          handleBoundaryEvent(event_handler, event);
          break;
          // TODO: right angle events
      }
    }
  }

  size_t getBranchIndex(size_t strand_id, size_t t) const {
    return branch_indices[strand_id][t];
  }

  const std::vector<std::vector<size_t>>& getBranches(size_t t) const {
    return strands_by_branch_id[t];
  }

  const std::vector<size_t>& getBranchStrands(size_t t, size_t branch_id) {
    return strands_by_branch_id[t][branch_id];
  }

 public:
  KineticDelaunay(const BranchTrajectories& branch_trajs, double cutoff, bool add_dummy_splines,
                  const std::vector<std::vector<size_t>>& branch_indices,
                  const std::vector<std::vector<std::vector<size_t>>>& strands_by_branch_id)
      : branch_trajs(branch_trajs),
        cutoff(cutoff),
        add_dummy_boundary(add_dummy_splines),
        branch_indices(branch_indices),
        strands_by_branch_id(strands_by_branch_id) {
    if (add_dummy_splines) {
      // first compute a bounding box:
      VoronoiPoint<2> p_min{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
      VoronoiPoint<2> p_max{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};

      for (const auto& points : branch_trajs.getPoints()) {
        for (const auto& p : points) {
          for (int dim = 0; dim < 2; dim++) {
            if (p[dim] < p_min[dim]) {
              p_min[dim] = p[dim];
            }
            if (p[dim] > p_max[dim]) {
              p_max[dim] = p[dim];
            }
          }
        }
      }

      // We will need dummy points such that no voronoi vertices can slip outside
      double range = std::max(p_max[0] - p_min[0], p_max[1] - p_min[1]);
      double dist_from_bb = std::max(range, 2 * cutoff);

      dummy_boundary = {
          {p_min[0] - 0.75 * dist_from_bb, p_max[1] + 0.75 * dist_from_bb},  // corner_tl
          {p_min[0], p_max[1] + dist_from_bb},                               // top_left
          {p_max[0], p_max[1] + dist_from_bb},                               // top_right
          {p_max[0] + 0.75 * dist_from_bb, p_max[1] + 0.75 * dist_from_bb},  // corner_tr
          {p_max[0] + dist_from_bb, p_max[1]},                               // right_top
          {p_max[0] + dist_from_bb, p_min[1]},                               // right_bottom
          {p_max[0] + 0.75 * dist_from_bb, p_min[1] - 0.75 * dist_from_bb},  // corner_br
          {p_max[0], p_min[1] - dist_from_bb},                               // bottom_right
          {p_min[0], p_min[1] - dist_from_bb},                               // bottom_left
          {p_min[0] - 0.75 * dist_from_bb, p_min[1] - 0.75 * dist_from_bb},  // corner_bl
          {p_min[0] - dist_from_bb, p_min[1]},                               // left_bottom
          {p_min[0] - dist_from_bb, p_max[1]}                                // left_top
      };

      size_t length = branch_trajs.getHeight() + 1;

      for (const auto& p : dummy_boundary) {
        std::vector<VoronoiPoint<2>> new_spline;
        for (size_t i = 0; i < length; i++) {
          new_spline.push_back(p);
        }
        this->branch_trajs.addTrajectory(new_spline);
      }
    }
  }

  bool isDummyBoundary(size_t v) {
    if (add_dummy_boundary) {
      return v >= branch_trajs.getPoints().size() - 12;
    }
    return false;
  }

  VoronoiPoint<2> getPointAt(size_t v, double t) const {
    // get point transformed such that all points in the same component match
    size_t component_id = component_data.component_map[v];
    size_t representative_vertex = component_data.components[component_id].front();
    size_t reference_branch = branch_indices[std::ceil(t)][representative_vertex];

    return branch_trajs.evaluateTransformed(v, t, reference_branch);
  }

  VoronoiPoint<2> getPointAt(double t, size_t v) const {
    return getPointAt(v, t);
  }

  VoronoiPoint<3> getPointInObjectSpace(size_t v, double t) const {
    return branch_trajs.getPointInObjectSpace(v, t);
  }

  const BranchTrajectories& getBranchTrajectories() const {
    return branch_trajs;
  }

  void computeComponentData(double t) {
    auto& graph = getGraph();
    component_data.components = extractConnectedComponents();
    EVOENGINE_LOG("Extracted " << component_data.components.size() << " components.");
    component_data.component_map = buildComponentMap(component_data.components, graph.getVertexCount());
    component_data.component_boundaries.resize(component_data.components.size());

    std::vector<bool> he_visited(graph.getHalfEdges().size(), false);

    for (size_t component_index = 0; component_index < component_data.components.size(); component_index++) {
      component_data.component_boundaries[component_index] =
          extractComponentBoundaries(component_data.components[component_index], t, he_visited);
    }

    component_data.component_centroids.resize(component_data.components.size());
    for (size_t component_index = 0; component_index < component_data.components.size(); component_index++) {
      if (!component_data.component_boundaries[component_index].empty()) {
        component_data.component_centroids[component_index] =
            polygonCentroid(component_data.component_boundaries[component_index][0]);
      } else {
        // compute centroid from points in the component
        VoronoiPoint<2> centroid{0.0, 0.0};
        for (auto& v : component_data.components[component_index]) {
          VoronoiPoint<2> p = getPointAt(t, v);
          centroid += p;
        }
        component_data.component_centroids[component_index] =
            centroid / double(component_data.components[component_index].size());
      }
    }

    component_data.component_last_updated.resize(component_data.components.size(), t);
  }

  const HalfEdgeDelaunayGraph& init() {
    graph.init(branch_trajs.getPoints());
    sections_advanced = 0;  // Reset the section counter

    face_inside.resize(graph.getFaces().size(), false);

    for (size_t face_index = 0; face_index < graph.getFaces().size(); face_index++) {
      const HalfEdgeDelaunayGraph::Triangle& tri = graph.getFaces()[face_index];

      // compute circumradius at t = 0 and check if within cutoff
      auto vertices = graph.adjacentTriangleVertices(tri.half_edges[0]);
      std::vector<VoronoiPoint<2>> points;
      bool outer_face = false;
      for (const auto& v : vertices) {
        if (v == -1) {
          outer_face = true;
          break;
        }
        // assume that only one plane as frame of reference exists
        points.push_back(branch_trajs.evaluate(v, 0.0));
      }

      if (outer_face) {
        continue;
      }

      double r = circumradius(points[0], points[1], points[2]);
      // EVOENGINE_LOG("Circumradius: " << r);
      if (r < cutoff) {
        setFaceInside(face_index, true);
      }
    }
    computeComponentData(0.0);

    return graph;
  }

  const HalfEdgeDelaunayGraph& advanceOneSection(EventHandler& event_handler) {
    size_t section_count = branch_trajs.getHeight();
    assert(sections_advanced < section_count);  // Ensure we do not exceed the number of sections
    // EVOENGINE_LOG("Advancing to section " << (sections_advanced + 1) << " of " << section_count);

    // update delaunay graph according to the components
    // For now we assume they can never be merged again
    if (component_data.components.size() > prev_component_count) {
      graph.update(branch_trajs.getPoints(), sections_advanced, component_data.components);
    }

    precomputeStep(static_cast<double>(sections_advanced));
    handleEvents(event_handler);
    sections_advanced++;

    return graph;
  }

  const HalfEdgeDelaunayGraph& getGraph() const {
    return graph;
  }

  size_t getSectionCount() const {
    return branch_trajs.getHeight();
  }

  // Computes the Delaunay triangulation of the given splines
  void compute(EventHandler& event_handler) {
    size_t section_count = getSectionCount();  // Assuming all splines have the same number of points

    evo_engine::ProgressBar progress_bar(0, branch_trajs.getHeight(), "Computing Kinetic Voronoi Sections",
                                         evo_engine::ProgressBar::Display::Absolute);
    for (size_t i = 0; i < section_count; ++i) {
      progress_bar.Update(i);

      assert(i == sections_advanced);  // Ensure we are advancing one section at a time
      if (i != 0)
        event_handler.betweenSections(i);  // Call the event handler for the section
      advanceOneSection(event_handler);
    }
    progress_bar.Finish();
  }

  std::vector<size_t> extractConnectedComponent(size_t u, std::vector<bool>& visited) const {
    std::vector<size_t> component;

    // Perform an iterative DFS with edges induced by inside faces

    std::vector<size_t> stack;
    stack.push_back(u);

    while (!stack.empty()) {
      size_t v = stack.back();
      stack.pop_back();

      if (visited[v])
        continue;

      visited[v] = true;
      component.push_back(v);

      const auto nbrs = graph.inducedNeighbors(v, face_inside);

      // Push neighbors in reverse order, the same order as recursive DFS
      for (auto it = nbrs.rbegin(); it != nbrs.rend(); ++it) {
        size_t w = *it;
        if (!visited[w])
          stack.push_back(w);
      }
    }

    return component;
  }

  const std::vector<VoronoiPoint<2>>& getDummyBoundary() const {
    return dummy_boundary;
  }

  std::vector<std::vector<size_t>> checkForSplit(const std::array<int, 3>& tri_vertices) const {
    std::vector<std::vector<size_t>> components;
    std::vector<bool> visited(graph.getVertexCount(), false);

    size_t u = tri_vertices[0];

    std::vector<size_t> component;

    std::vector<size_t> queue;
    queue.push_back(u);
    visited[u] = true;

    size_t head = 0;

    while (head < queue.size()) {
      size_t v = queue[head++];
      component.push_back(v);

      const auto nbrs = graph.inducedNeighbors(v, face_inside);

      for (size_t w : nbrs) {
        if (!visited[w]) {
          visited[w] = true;

          // quit early if we found all triangle vertices
          if (visited[tri_vertices[1]] && visited[tri_vertices[2]]) {
            return {};  // return empty to indicate no split
          }

          queue.push_back(w);
        }
      }
    }

    components.push_back(component);

    if (!visited[tri_vertices[1]]) {
      auto component2 = extractConnectedComponent(tri_vertices[1], visited);
      components.push_back(component2);
    }

    if (!visited[tri_vertices[2]]) {
      auto component3 = extractConnectedComponent(tri_vertices[2], visited);
      components.push_back(component3);
    }

    return components;
  }

  std::vector<std::vector<size_t>> extractConnectedComponents() const {
    std::vector<std::vector<size_t>> components;
    std::vector<bool> visited(graph.getVertexCount(), false);
    for (size_t u = 0; u < graph.getVertexCount(); u++) {
      if (visited[u]) {
        continue;
      }

      auto component = extractConnectedComponent(u, visited);
      components.push_back(component);
    }

    return components;
  }

  std::vector<BoundaryPoint> traverseBoundary(size_t start_he_id, double t) const {
    // Walk the boundary to extract the boundary half-edges
    std::vector<BoundaryPoint> boundary_points;
    size_t he_id = start_he_id;
    do {
      size_t origin = graph.getHalfEdges()[he_id].origin;
      if (origin == -1) {
        EVOENGINE_ERROR("Followed infinite edge.");
      }
      VoronoiPoint<2> pos = getPointAt(origin, t);
      boundary_points.emplace_back(BoundaryPoint{origin, he_id, pos});
      he_id = nextOnComponentBoundaryId(he_id);
    } while (he_id != start_he_id);

    return boundary_points;
  }

  std::vector<std::vector<BoundaryPoint>> extractComponentBoundaries(const std::vector<size_t>& component, double t,
                                                                     std::vector<bool>& he_visited) const {
    // EVOENGINE_LOG("Extracting component boundaries at t = " << t);
    if (component.size() < 3) {
      return {{}};
    }

    std::vector<std::vector<BoundaryPoint>> boundaries;
    double min_x = std::numeric_limits<double>::infinity();
    // TODO: this is not perfectly safe if points of the outer and an inner boundary coincide at the minimum
    size_t min_x_id = 0;
    for (size_t i = 0; i < component.size(); i++) {
      const size_t& v = component[i];

      for (auto it = graph.incidentEdgesBegin(v); it != graph.incidentEdgesEnd(v); it++) {
        auto he_id = *it;

        if (he_visited[he_id] || !isOnComponentBoundaryOutside(he_id)) {
          continue;
        }

        if (graph.destination(he_id) == -1) {
          EVOENGINE_ERROR("Destination of half-edge is invalid");
        }
        if (graph.getHalfEdges()[he_id].origin == -1) {
          EVOENGINE_ERROR("Origin of half-edge is invalid");
        }

        auto boundary_points = traverseBoundary(he_id, t);

        for (auto& bp : boundary_points) {
          he_visited[bp.he_id] = true;
          if (bp.p[0] < min_x) {
            min_x = bp.p[0];
            min_x_id = boundaries.size();
          }
        }

        boundaries.emplace_back(boundary_points);
      }
    }

    // swap the boundary with the minimum x to the front
    if (min_x_id != 0) {
      std::swap(boundaries[0], boundaries[min_x_id]);
    }

    return boundaries;
  }

  std::vector<BoundaryPoint> extractComponentBoundary(const std::vector<size_t>& component, double t) const {
    // Find an extreme point to start the boundary walk as it must be on the boundary
    // Note that merely being on the outside of the boundary is not sufficent as there can also be holes inside the
    // component

    size_t start_vertex_id = -1;
    double min_x = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < component.size(); i++) {
      const size_t& v = component[i];

      // Get position and check if it's the minimum x
      VoronoiPoint<2> pos = getPointAt(v, t);  // Evaluate at t=0 for starting point
      if (pos[0] < min_x) {
        min_x = pos[0];
        start_vertex_id = v;
      }
    }

    // From the starting vertex, find a half-edge that is on the boundary
    size_t start_he_id = -1;
    for (auto it = graph.incidentEdgesBegin(start_vertex_id); it != graph.incidentEdgesEnd(start_vertex_id); it++) {
      if (isOnComponentBoundaryOutside(*it)) {
        start_he_id = *it;
        break;
      }
    }

    return traverseBoundary(start_he_id, t);
  }

  bool getFaceInside(size_t face_index) const {
    return face_inside[face_index];
  }

  void setFaceInside(size_t face_index, bool value) {
    if (value) {
      auto tri_vertices = graph.adjacentTriangleVertices(graph.getFaces()[face_index].half_edges[0]);

      for (int& v : tri_vertices) {
        if (v == -1) {
          // cannot set face with infinite vertex to inside
          throw std::runtime_error("Cannot set face " + std::to_string(face_index) + " to inside!");
        }
      }
    }
    face_inside[face_index] = value;
  }

  bool isOnComponentBoundary(size_t he_id) const {
    size_t face_id = graph.getHalfEdges()[he_id].face;
    size_t twin_face_id = graph.getHalfEdges()[he_id ^ 1].face;
    return (face_inside[face_id] != face_inside[twin_face_id]);
  }

  bool isOnComponentBoundaryOutside(size_t he_id) const {
    size_t face_id = graph.getHalfEdges()[he_id].face;
    size_t twin_face_id = graph.getHalfEdges()[he_id ^ 1].face;
    return (!face_inside[face_id] && face_inside[twin_face_id]);
  }

  size_t nextOnComponentBoundaryId(size_t he_id) const {
    size_t next_he_id = graph.getHalfEdges()[he_id].next;

    while (!isOnComponentBoundaryOutside(next_he_id)) {
      next_he_id = graph.twin(next_he_id);
      next_he_id = graph.getHalfEdges()[next_he_id].next;
    }

    return next_he_id;
  }
};
}  // namespace kinDS