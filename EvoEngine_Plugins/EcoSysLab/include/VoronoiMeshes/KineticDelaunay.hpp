#pragma once
#include <format>
#include <queue>
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

 private:
  typedef std::priority_queue<Event> EventQueue;

  std::vector<CubicHermiteSpline<2>> splines;
  HalfEdgeDelaunayGraph graph;
  EventQueue events;
  size_t sections_advanced = 0;               // Counter for the number of sections advanced
  double cutoff;                              // Cutoff radius for boundary events
  std::vector<bool> face_inside;              // Tracks whether faces are inside or outside the boundary
  std::vector<size_t> branch_ids;             // track branch ID for each vertex/spline
  std::vector<std::vector<size_t>> branches;  // track which vertices/splines belong to which branch

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

    trajs.push_back(splines[u].getPiecePolynomial(section));
    trajs.push_back(splines[v].getPiecePolynomial(section));
    trajs.push_back(splines[w].getPiecePolynomial(section));

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
        EVOENGINE_LOG("Right Angle Event at time " << event_time << " for half-edge ID " << he_id
                                                   << " at center position " << center.toString().c_str());

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

    trajs.push_back(splines[u].getPiecePolynomial(section));
    trajs.push_back(splines[v].getPiecePolynomial(section));
    trajs.push_back(splines[w].getPiecePolynomial(section));

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

      trajs.push_back(splines[a].getPiecePolynomial(section));
      trajs.push_back(splines[b].getPiecePolynomial(section));
      trajs.push_back(splines[c].getPiecePolynomial(section));

      event_trigger = ccw(trajs[0][0], trajs[0][1], trajs[1][0], trajs[1][1], trajs[2][0], trajs[2][1]);
    } else {
      int a = graph.getHalfEdges()[he_id].origin;       // First vertex
      int b = graph.triangleOppositeVertex(he_id ^ 1);  // Second vertex
      int c = graph.getHalfEdges()[he_id ^ 1].origin;   // Third vertex
      int d = graph.triangleOppositeVertex(he_id);      // Fourth vertex

      // print the quadrilateral vertices:
      // std::cout << "Quadrilateral vertices: " << a << ", " << b << ", " << c << ", " << d << std::endl;

      trajs.push_back(splines[a].getPiecePolynomial(section));
      trajs.push_back(splines[b].getPiecePolynomial(section));
      trajs.push_back(splines[c].getPiecePolynomial(section));
      trajs.push_back(splines[d].getPiecePolynomial(section));

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
      }
    }
  }

 public:
  KineticDelaunay(const std::vector<CubicHermiteSpline<2>>& splines, double cutoff) : splines(splines), cutoff(cutoff) {
  }

  std::vector<VoronoiPoint<2>> getPointsAt(double t) const {
    std::vector<VoronoiPoint<2>> points;
    points.reserve(splines.size());
    for (const auto& spline : splines) {
      points.push_back(spline.evaluate(t));  // Get the first point of each spline
    }
    return points;
  }

  const HalfEdgeDelaunayGraph& init() {
    graph.init(splines);
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
        points.push_back(splines[v].evaluate(0.0));
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

    return graph;
  }

  const HalfEdgeDelaunayGraph& advanceOneSection(EventHandler& event_handler) {
    size_t section_count = splines[0].pointCount() - 1;
    assert(sections_advanced < section_count);  // Ensure we do not exceed the number of sections
    // EVOENGINE_LOG("Advancing to section " << (sections_advanced + 1) << " of " << section_count);
    precomputeStep(static_cast<double>(sections_advanced));
    handleEvents(event_handler);
    sections_advanced++;

    return graph;
  }

  const HalfEdgeDelaunayGraph& getGraph() const {
    return graph;
  }

  size_t getSectionCount() const {
    return splines[0].pointCount() - 1;  // Assuming all splines have the same number of points
  }

  // Computes the Delaunay triangulation of the given splines
  void compute(EventHandler& event_handler) {
    size_t section_count = getSectionCount();  // Assuming all splines have the same number of points

    evo_engine::ProgressBar progress_bar(0, splines.size(), "Computing Kinetic Voronoi Sections",
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
      VoronoiPoint<2> pos = splines[origin].evaluate(t);
      boundary_points.emplace_back(BoundaryPoint{origin, he_id, pos});
      he_id = nextOnComponentBoundaryId(he_id);
    } while (he_id != start_he_id);

    return boundary_points;
  }

  std::vector<std::vector<BoundaryPoint>> extractComponentBoundaries(const std::vector<size_t>& component, double t,
                                                                     std::vector<bool>& he_visited) const {
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
      VoronoiPoint<2> pos = splines[v].evaluate(t);  // Evaluate at t=0 for starting point
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

  size_t split(size_t branch_id, const std::vector<size_t>& new_subbranch) {
    size_t new_branch_id = branches.size();

    for (size_t strand_id : new_subbranch) {
      branch_ids[strand_id] = new_branch_id;
    }

    std::vector<size_t> old_branch;

    for (size_t strand_id : branches[branch_id]) {
      if (branch_ids[strand_id] == branch_id) {
        old_branch.push_back(strand_id);
      }
    }

    branches[branch_id] = old_branch;
    branches.push_back(new_subbranch);

    // TODO: separate the triangles

    return new_branch_id;
  }
};
}  // namespace kinDS