#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>
#include "KineticDelaunay.hpp"

namespace kinDS {
using Pt = glm::dvec2;

struct ClippedSegment {
  Pt p0, p1;
  int edge0;  // -1 for original endpoint
  int edge1;
};

static double cross(const Pt& a, const Pt& b, const Pt& c) {
  return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

static Pt lerp(const Pt& a, const Pt& b, double t) {
  Pt p;
  p[0] = a[0] + t * (b[0] - a[0]);
  p[1] = a[1] + t * (b[1] - a[1]);
  return p;
}

// Returns t on AB if intersects, else nullopt
static std::optional<double> SegmentIntersectParam(const Pt& a, const Pt& b, const Pt& c, const Pt& d) {
  double ax = a[0], ay = a[1];
  double bx = b[0], by = b[1];
  double cx = c[0], cy = c[1];
  double dx = d[0], dy = d[1];

  double r_px = bx - ax;
  double r_py = by - ay;
  double s_px = dx - cx;
  double s_py = dy - cy;

  double denom = r_px * s_py - r_py * s_px;
  if (std::abs(denom) < 1e-14)
    return std::nullopt;  // parallel/collinear

  double t = ((cx - ax) * s_py - (cy - ay) * s_px) / denom;
  double u = ((cx - ax) * r_py - (cy - ay) * r_px) / denom;

  if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0)
    return t;

  return std::nullopt;
}

// Winding number point-in-polygon (CCW, simple polygon)
static bool PointInPolygon(const Pt& p, const std::vector<Pt>& poly) {
  int wn = 0;
  size_t n = poly.size();
  for (size_t i = 0; i < n; ++i) {
    const Pt& a = poly[i];
    const Pt& b = poly[(i + 1) % n];

    if (a[1] <= p[1]) {
      if (b[1] > p[1] && cross(a, b, p) > 0)
        ++wn;
    } else {
      if (b[1] <= p[1] && cross(a, b, p) < 0)
        --wn;
    }
  }
  return wn != 0;
}

std::vector<ClippedSegment> ClipSegmentAgainstPolygon_All(const Pt& a, const Pt& b, const std::vector<Pt>& poly) {
  struct Event {
    double t;
    Pt p;
    int edge;  // polygon edge index, or -1
  };

  std::vector<Event> events;

  // 1. Intersection events
  size_t n = poly.size();
  for (size_t i = 0; i < n; ++i) {
    const Pt& c = poly[i];
    const Pt& d = poly[(i + 1) % n];

    auto t_opt = SegmentIntersectParam(a, b, c, d);
    if (t_opt.has_value()) {
      double t = *t_opt;
      Pt p = lerp(a, b, t);
      events.push_back({t, p, (int)i});
    }
  }

  // 2. Endpoint events if inside
  bool a_in = PointInPolygon(a, poly);
  bool b_in = PointInPolygon(b, poly);

  if (a_in)
    events.push_back({0.0, a, -1});
  if (b_in)
    events.push_back({1.0, b, -1});

  if (events.empty())
    return {};

  // 3. Sort and unique
  std::sort(events.begin(), events.end(), [](const Event& x, const Event& y) {
    return x.t < y.t;
  });

  std::vector<Event> unique_events;
  for (const auto& e : events) {
    if (unique_events.empty() ||
        std::hypot(e.p[0] - unique_events.back().p[0], e.p[1] - unique_events.back().p[1]) > 1e-12)
      unique_events.push_back(e);
  }

  // 4. Scan consecutive intervals
  std::vector<ClippedSegment> result;

  for (size_t i = 0; i + 1 < unique_events.size(); ++i) {
    double t0 = unique_events[i].t;
    double t1 = unique_events[i + 1].t;

    double tm = 0.5 * (t0 + t1);
    Pt mid = lerp(a, b, tm);

    if (PointInPolygon(mid, poly)) {
      // interior segment
      ClippedSegment seg;
      seg.p0 = unique_events[i].p;
      seg.p1 = unique_events[i + 1].p;
      seg.edge0 = unique_events[i].edge;
      seg.edge1 = unique_events[i + 1].edge;
      result.push_back(seg);
    }
  }

  return result;
}
};  // namespace kinDS