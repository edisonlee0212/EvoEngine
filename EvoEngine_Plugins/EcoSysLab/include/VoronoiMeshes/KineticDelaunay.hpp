#pragma once
#include <queue>
#include "CubicHermiteSpline.hpp"
#include "HalfEdgeDelaunayGraph.hpp"
#include "Polynomial.hpp"
#include "ProgressBar.hpp"

namespace kinDS {

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
    Point<2> position;     // Position of the event

    Event(double t, size_t he_id, double creation_time, Point<2> position)
        : time(t), half_edge_id(he_id), creation_time(creation_time), position(position) {
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
     * \brief Handle an event before it is processed.
     *
     * @param e The event to handle.
     */
    virtual void beforeEvent(Event& e) {
    }
    /**
     * \brief Handle an event after it is processed.
     *
     * @param e The event to handle.
     */
    virtual void afterEvent(Event& e) {
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
  size_t sections_advanced = 0;  // Counter for the number of sections advanced

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

  void computeEvents(double t, size_t quad_id) {
    const size_t section = static_cast<size_t>(t);
    const float fraction = t - section;

    size_t he_id = quad_id * 2;
    Polynomial event_trigger;

    std::vector<Trajectory<2>> trajs;

    if (graph.isOnBoundary(he_id) || graph.isOutsideBoundary(he_id)) {
      // boundary edges must be treated separately using ccw

      // need to get the inner half-edge so we have access to the triangle
      // in case both are outside, this swap does not matter, so we just let it happen
      if (graph.isOutsideBoundary(he_id)) {
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

      if (root > fraction && root <= 1) {  // Check if the root is within the valid range
        double event_time = root + section;
        // std::cout << "Root found at t = " << event_time << std::endl;

        Point<2> center{};

        for (const auto& traj : trajs) {
          center[0] += traj[0](root);
          center[1] += traj[1](root);
        }
        center[0] /= trajs.size();
        center[1] /= trajs.size();

        EVOENGINE_LOG("Event at time " << event_time << " for half-edge ID " << he_id << " at center position "
                                       << center.toString().c_str());

        events.emplace(Event(event_time, he_id, t, center));  // Store the event with the time and half-edge index
      }
    }
  }

  void precomputeStep(double t) {
    // TODO: Make sure it works where no change of sign occurs in the polynomial, i.e., roots that do not lead to a
    // change in the triangulation.
    size_t quad_count = graph.getHalfEdges().size() / 2;
    for (size_t i = 0; i < quad_count; i++) {
      computeEvents(t, i);
    }
  }

  void handleEvents(EventHandler& event_handler) {
    std::vector<double> quadrilateral_last_updated(graph.getHalfEdges().size() / 2, 0.0);
    while (!events.empty()) {
      Event event = events.top();
      events.pop();

      // Check if the event is still valid
      if (event.creation_time < quadrilateral_last_updated[event.half_edge_id / 2]) {
        // This event is outdated, skip it
        continue;
      }

      // Process the event at the given time
      EVOENGINE_LOG("Processing event at time " << event.time << " for half-edge ID " << event.half_edge_id);

      // Call the event handler if provided
      event_handler.beforeEvent(event);

      graph.flipEdge(event.half_edge_id);

      // After flipping the edge, we need to recompute the events for all surrounding half-edges
      size_t next1 = graph.getHalfEdges()[event.half_edge_id].next;
      size_t next2 = graph.getHalfEdges()[next1].next;

      size_t twin_next1 = graph.getHalfEdges()[event.half_edge_id ^ 1].next;
      size_t twin_next2 = graph.getHalfEdges()[twin_next1].next;

      computeEvents(event.time, next1 / 2);
      quadrilateral_last_updated[next1 / 2] = event.time;  // Update the last updated time for the quadrilateral

      computeEvents(event.time, next2 / 2);
      quadrilateral_last_updated[next2 / 2] = event.time;  // Update the last updated time for the quadrilateral

      computeEvents(event.time, twin_next1 / 2);
      quadrilateral_last_updated[twin_next1 / 2] = event.time;  // Update the last updated time for the quadrilateral

      computeEvents(event.time, twin_next2 / 2);
      quadrilateral_last_updated[twin_next2 / 2] = event.time;  // Update the last updated time for the quadrilateral

      event_handler.afterEvent(event);  // Call the event handler after processing the event
    }
  }

 public:
  KineticDelaunay(const std::vector<CubicHermiteSpline<2>>& splines) : splines(splines) {
  }

  std::vector<Point<2>> getPointsAt(double t) const {
    std::vector<Point<2>> points;
    points.reserve(splines.size());
    for (const auto& spline : splines) {
      points.push_back(spline.evaluate(t));  // Get the first point of each spline
    }
    return points;
  }

  const HalfEdgeDelaunayGraph& init() {
    graph.init(splines);
    sections_advanced = 0;  // Reset the section counter
    // graph.printDebug();

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
};
}  // namespace kinDS
