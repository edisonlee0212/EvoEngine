#include "AlphaHullFractalExperiment.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include <glm/geometric.hpp>
#include <glm/vec2.hpp>
#include "Delaunator2D.hpp"
#include "kinDS/kinDS/KineticDelaunayHelpers.hpp"

namespace eco_sys_lab_package {
namespace {

constexpr double kEps = 1e-12;

struct EdgeKey {
  size_t a = 0;
  size_t b = 0;
  bool operator==(const EdgeKey& o) const {
    return a == o.a && b == o.b;
  }
};

struct EdgeKeyHash {
  size_t operator()(const EdgeKey& e) const {
    return (e.a * 1315423911u) ^ (e.b + 0x9e3779b9u + (e.a << 6) + (e.a >> 2));
  }
};

EdgeKey MakeEdge(size_t i, size_t j) {
  return i < j ? EdgeKey{i, j} : EdgeKey{j, i};
}

double PolygonSignedArea(const std::vector<glm::dvec2>& poly) {
  if (poly.size() < 3) {
    return 0.0;
  }
  double a = 0.0;
  for (size_t i = 0; i < poly.size(); ++i) {
    const auto& p0 = poly[i];
    const auto& p1 = poly[(i + 1) % poly.size()];
    a += p0.x * p1.y - p1.x * p0.y;
  }
  return 0.5 * a;
}

double PolylinePerimeter(const std::vector<glm::dvec2>& poly, const bool closed) {
  if (poly.size() < 2) {
    return 0.0;
  }
  double p = 0.0;
  const size_t n = poly.size();
  const size_t edge_count = closed ? n : n - 1;
  for (size_t i = 0; i < edge_count; ++i) {
    p += glm::distance(poly[i], poly[(i + 1) % n]);
  }
  return p;
}

void BoundingBox(const std::vector<glm::dvec2>& pts, glm::dvec2& out_min, glm::dvec2& out_max) {
  out_min = glm::dvec2(std::numeric_limits<double>::infinity());
  out_max = glm::dvec2(-std::numeric_limits<double>::infinity());
  for (const auto& p : pts) {
    out_min = glm::min(out_min, p);
    out_max = glm::max(out_max, p);
  }
}

bool LinearRegressionLogLog(const std::vector<double>& xs, const std::vector<double>& ys, double& out_slope,
                            double& out_r2) {
  if (xs.size() != ys.size() || xs.size() < 2) {
    return false;
  }
  const size_t n = xs.size();
  double mean_x = 0.0;
  double mean_y = 0.0;
  for (size_t i = 0; i < n; ++i) {
    mean_x += xs[i];
    mean_y += ys[i];
  }
  mean_x /= static_cast<double>(n);
  mean_y /= static_cast<double>(n);
  double sxx = 0.0;
  double sxy = 0.0;
  double syy = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const double dx = xs[i] - mean_x;
    const double dy = ys[i] - mean_y;
    sxx += dx * dx;
    sxy += dx * dy;
    syy += dy * dy;
  }
  if (sxx < kEps) {
    return false;
  }
  out_slope = sxy / sxx;
  out_r2 = (syy < kEps) ? 1.0 : (sxy * sxy) / (sxx * syy);
  return true;
}

void DeduplicatePoints(std::vector<glm::dvec2>& points, const double tol = 1e-10) {
  std::vector<glm::dvec2> unique;
  unique.reserve(points.size());
  for (const auto& p : points) {
    bool found = false;
    for (const auto& q : unique) {
      if (glm::distance(p, q) <= tol) {
        found = true;
        break;
      }
    }
    if (!found) {
      unique.push_back(p);
    }
  }
  points.swap(unique);
}

std::vector<glm::dvec2> ExtractDelaunayConvexHull(const Delaunator::Delaunator2D& d) {
  std::vector<glm::dvec2> hull;
  if (d.coords.empty() || d.hull_next.empty()) {
    return hull;
  }
  size_t e = d.hull_start;
  const size_t start = e;
  do {
    hull.emplace_back(static_cast<double>(d.coords[2 * e]), static_cast<double>(d.coords[2 * e + 1]));
    e = d.hull_next[e];
  } while (e != start && hull.size() <= d.hull_next.size());
  return hull;
}

/// Directed boundary edges of the alpha complex: undirected edges with exactly one kept triangle.
std::unordered_map<size_t, std::vector<size_t>> BuildBoundaryAdjacency(const std::vector<std::size_t>& triangles,
                                                                       const std::vector<char>& keep_triangle) {
  std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_count;
  std::unordered_map<EdgeKey, std::pair<size_t, size_t>, EdgeKeyHash> edge_dir;

  const size_t tri_count = triangles.size() / 3;
  for (size_t t = 0; t < tri_count; ++t) {
    if (!keep_triangle[t]) {
      continue;
    }
    const size_t verts[3] = {triangles[3 * t], triangles[3 * t + 1], triangles[3 * t + 2]};
    for (int e = 0; e < 3; ++e) {
      const size_t a = verts[e];
      const size_t b = verts[(e + 1) % 3];
      const EdgeKey key = MakeEdge(a, b);
      edge_count[key] += 1;
      edge_dir[key] = {a, b};  // CCW: interior on left
    }
  }

  std::unordered_map<size_t, std::vector<size_t>> adj;
  for (const auto& [key, count] : edge_count) {
    if (count != 1) {
      continue;
    }
    const auto it = edge_dir.find(key);
    if (it == edge_dir.end()) {
      continue;
    }
    adj[it->second.first].push_back(it->second.second);
  }
  return adj;
}

std::vector<std::vector<glm::dvec2>> TraceBoundaryPolygons(const std::unordered_map<size_t, std::vector<size_t>>& adj,
                                                           const std::vector<float>& coords) {
  std::unordered_set<uint64_t> used_dir;
  auto pack = [](size_t a, size_t b) -> uint64_t {
    return (static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b);
  };

  std::vector<std::pair<size_t, size_t>> all_edges;
  for (const auto& [a, outs] : adj) {
    for (size_t b : outs) {
      all_edges.emplace_back(a, b);
    }
  }

  std::vector<std::vector<glm::dvec2>> polygons;
  for (const auto& [start_a, start_b] : all_edges) {
    if (used_dir.count(pack(start_a, start_b))) {
      continue;
    }
    std::vector<glm::dvec2> poly;
    size_t a = start_a;
    size_t b = start_b;
    size_t guard = 0;
    const size_t max_steps = all_edges.size() + 2;
    while (guard++ < max_steps) {
      const uint64_t key = pack(a, b);
      if (used_dir.count(key)) {
        break;
      }
      used_dir.insert(key);
      poly.emplace_back(static_cast<double>(coords[2 * a]), static_cast<double>(coords[2 * a + 1]));
      const auto it = adj.find(b);
      if (it == adj.end() || it->second.empty()) {
        break;
      }
      // Prefer continuing unused outgoing; if several, pick the one that turns most left (CCW).
      size_t best = it->second.front();
      bool found_unused = false;
      const glm::dvec2 pb(coords[2 * b], coords[2 * b + 1]);
      const glm::dvec2 pa(coords[2 * a], coords[2 * a + 1]);
      const glm::dvec2 in_dir = pb - pa;
      double best_cross = -std::numeric_limits<double>::infinity();
      for (size_t c : it->second) {
        if (used_dir.count(pack(b, c))) {
          continue;
        }
        const glm::dvec2 pc(coords[2 * c], coords[2 * c + 1]);
        const glm::dvec2 out_dir = pc - pb;
        const double cross = in_dir.x * out_dir.y - in_dir.y * out_dir.x;
        if (!found_unused || cross > best_cross) {
          found_unused = true;
          best_cross = cross;
          best = c;
        }
      }
      if (!found_unused) {
        break;
      }
      a = b;
      b = best;
      if (a == start_a && b == start_b) {
        break;
      }
    }
    if (poly.size() >= 3) {
      polygons.push_back(std::move(poly));
    }
  }
  return polygons;
}

std::vector<glm::dvec2> SelectPrimaryOuterPolygon(std::vector<std::vector<glm::dvec2>>& polygons) {
  if (polygons.empty()) {
    return {};
  }
  // Prefer largest |area| with positive orientation (outer CCW); fall back to largest perimeter.
  size_t best = 0;
  double best_score = -1.0;
  for (size_t i = 0; i < polygons.size(); ++i) {
    const double area = PolygonSignedArea(polygons[i]);
    const double score = std::abs(area);
    if (score > best_score) {
      best_score = score;
      best = i;
    }
  }
  auto poly = std::move(polygons[best]);
  if (PolygonSignedArea(poly) < 0.0) {
    std::reverse(poly.begin(), poly.end());
  }
  return poly;
}

void WriteSvg(const std::filesystem::path& path, const std::vector<glm::dvec2>& points,
              const std::vector<glm::dvec2>& hull, const std::vector<glm::dvec2>& alpha_hull,
              const std::string& title) {
  if (points.empty()) {
    return;
  }
  glm::dvec2 bmin, bmax;
  BoundingBox(points, bmin, bmax);
  const double pad = 0.05 * std::max(1e-9, std::max(bmax.x - bmin.x, bmax.y - bmin.y));
  bmin -= glm::dvec2(pad);
  bmax += glm::dvec2(pad);
  const double w = std::max(1e-9, bmax.x - bmin.x);
  const double h = std::max(1e-9, bmax.y - bmin.y);
  const double svg_w = 800.0;
  const double svg_h = 800.0 * h / w;
  auto mapx = [&](double x) {
    return (x - bmin.x) / w * svg_w;
  };
  auto mapy = [&](double y) {
    return svg_h - (y - bmin.y) / h * svg_h;
  };

  std::ofstream out(path);
  if (!out) {
    return;
  }
  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << svg_w << "\" height=\"" << svg_h << "\">\n";
  out << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";
  out << "<text x=\"10\" y=\"20\" font-size=\"14\">" << title << "</text>\n";
  for (const auto& p : points) {
    out << "<circle cx=\"" << mapx(p.x) << "\" cy=\"" << mapy(p.y) << "\" r=\"1.5\" fill=\"#444\"/>\n";
  }
  auto write_poly = [&](const std::vector<glm::dvec2>& poly, const char* stroke, double width) {
    if (poly.size() < 2) {
      return;
    }
    out << "<polyline fill=\"none\" stroke=\"" << stroke << "\" stroke-width=\"" << width << "\" points=\"";
    for (const auto& p : poly) {
      out << mapx(p.x) << "," << mapy(p.y) << " ";
    }
    out << mapx(poly.front().x) << "," << mapy(poly.front().y) << "\"/>\n";
  };
  write_poly(hull, "#2a6", 2.0);
  write_poly(alpha_hull, "#c33", 2.0);
  out << "</svg>\n";
}

}  // namespace

bool EstimatePolylineFractalDimension(const std::vector<glm::dvec2>& polyline, const bool closed, double& out_dimension,
                                      double& out_r2, std::string* out_note) {
  out_dimension = 0.0;
  out_r2 = 0.0;
  if (polyline.size() < 3) {
    if (out_note) {
      *out_note = "too few vertices";
    }
    return false;
  }

  glm::dvec2 bmin, bmax;
  BoundingBox(polyline, bmin, bmax);
  const double extent = std::max(bmax.x - bmin.x, bmax.y - bmin.y);
  if (extent < kEps) {
    if (out_note) {
      *out_note = "degenerate extent";
    }
    return false;
  }

  // Box sizes from ~extent/4 down to a few times the mean edge length.
  double mean_edge = 0.0;
  const size_t n = polyline.size();
  const size_t edge_count = closed ? n : n - 1;
  for (size_t i = 0; i < edge_count; ++i) {
    mean_edge += glm::distance(polyline[i], polyline[(i + 1) % n]);
  }
  mean_edge /= static_cast<double>(std::max<size_t>(1, edge_count));
  const double eps_max = extent * 0.25;
  const double eps_min = std::max(mean_edge * 2.0, extent * 1e-3);
  if (eps_min >= eps_max) {
    if (out_note) {
      *out_note = "scale range too small";
    }
    return false;
  }

  std::vector<double> log_inv_eps;
  std::vector<double> log_nbox;
  constexpr int kSteps = 12;
  for (int s = 0; s < kSteps; ++s) {
    const double t = static_cast<double>(s) / static_cast<double>(kSteps - 1);
    const double eps = eps_max * std::pow(eps_min / eps_max, t);
    if (eps < kEps) {
      continue;
    }
    const int nx = std::max(1, static_cast<int>(std::ceil((bmax.x - bmin.x) / eps)) + 1);
    const int ny = std::max(1, static_cast<int>(std::ceil((bmax.y - bmin.y) / eps)) + 1);
    std::unordered_set<uint64_t> occupied;
    occupied.reserve(static_cast<size_t>(nx) * 4);
    for (size_t i = 0; i < edge_count; ++i) {
      const glm::dvec2 a = polyline[i];
      const glm::dvec2 b = polyline[(i + 1) % n];
      // Sample along the edge and mark boxes (robust for long edges).
      const double len = glm::distance(a, b);
      const int samples = std::max(2, static_cast<int>(std::ceil(len / (eps * 0.5))) + 1);
      for (int k = 0; k <= samples; ++k) {
        const double u = static_cast<double>(k) / static_cast<double>(samples);
        const glm::dvec2 p = a + (b - a) * u;
        const int ix = static_cast<int>(std::floor((p.x - bmin.x) / eps));
        const int iy = static_cast<int>(std::floor((p.y - bmin.y) / eps));
        if (ix < 0 || iy < 0 || ix >= nx || iy >= ny) {
          continue;
        }
        occupied.insert((static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) | static_cast<uint32_t>(iy));
      }
    }
    if (occupied.size() < 2) {
      continue;
    }
    log_inv_eps.push_back(std::log(1.0 / eps));
    log_nbox.push_back(std::log(static_cast<double>(occupied.size())));
  }

  double slope = 0.0;
  if (!LinearRegressionLogLog(log_inv_eps, log_nbox, slope, out_r2) || log_inv_eps.size() < 3) {
    if (out_note) {
      *out_note = "box-count fit failed";
    }
    return false;
  }
  out_dimension = slope;
  if (out_note) {
    *out_note = "box-counting";
  }
  return true;
}

bool AnalyzeAlphaHullFractalDimensions(const std::vector<glm::dvec2>& input_points, const std::vector<double>& alphas,
                                       const size_t height, const size_t branch_id, AlphaHullFractalReport& out_report,
                                       const std::filesystem::path& export_dir) {
  out_report = {};
  out_report.height = height;
  out_report.branch_id = branch_id;
  out_report.points = input_points;
  DeduplicatePoints(out_report.points);
  out_report.point_count = out_report.points.size();
  if (out_report.points.size() < 3) {
    out_report.convex_hull.note = "need at least 3 unique points";
    return false;
  }

  std::vector<float> coords;
  coords.reserve(out_report.points.size() * 2);
  for (const auto& p : out_report.points) {
    coords.push_back(static_cast<float>(p.x));
    coords.push_back(static_cast<float>(p.y));
  }

  Delaunator::Delaunator2D delaunay(coords);
  out_report.delaunay_triangle_count = delaunay.triangles.size() / 3;
  out_report.convex_hull_polygon = ExtractDelaunayConvexHull(delaunay);
  out_report.convex_hull.vertex_count = out_report.convex_hull_polygon.size();
  out_report.convex_hull.circumference = PolylinePerimeter(out_report.convex_hull_polygon, true);
  out_report.convex_hull.corner_count = out_report.convex_hull_polygon.size();
  out_report.convex_hull.area = std::abs(PolygonSignedArea(out_report.convex_hull_polygon));
  out_report.convex_hull.ok =
      EstimatePolylineFractalDimension(out_report.convex_hull_polygon, true, out_report.convex_hull.fractal_dimension,
                                       out_report.convex_hull.fractal_fit_r2, &out_report.convex_hull.note);

  const double hull_circ = std::max(out_report.convex_hull.circumference, kEps);
  const double hull_corners = static_cast<double>(std::max<size_t>(1, out_report.convex_hull.corner_count));

  // Precompute squared circumradii for each triangle.
  const size_t tri_count = out_report.delaunay_triangle_count;
  std::vector<double> tri_r2(tri_count, std::numeric_limits<double>::infinity());
  for (size_t t = 0; t < tri_count; ++t) {
    const size_t i0 = delaunay.triangles[3 * t];
    const size_t i1 = delaunay.triangles[3 * t + 1];
    const size_t i2 = delaunay.triangles[3 * t + 2];
    const glm::dvec2 p0(coords[2 * i0], coords[2 * i0 + 1]);
    const glm::dvec2 p1(coords[2 * i1], coords[2 * i1 + 1]);
    const glm::dvec2 p2(coords[2 * i2], coords[2 * i2 + 1]);
    try {
      const double r = kinDS::circumradius(p0, p1, p2);
      tri_r2[t] = r * r;
    } catch (...) {
      tri_r2[t] = std::numeric_limits<double>::infinity();
    }
  }

  if (!export_dir.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(export_dir, ec);
  }

  out_report.alpha_rows.reserve(alphas.size());
  for (const double alpha : alphas) {
    AlphaHullFractalRow row;
    row.alpha = alpha;
    std::vector<char> keep(tri_count, 0);
    for (size_t t = 0; t < tri_count; ++t) {
      if (tri_r2[t] < alpha) {
        keep[t] = 1;
        ++row.kept_triangle_count;
      }
    }
    if (row.kept_triangle_count == 0) {
      row.note = "empty alpha complex";
      out_report.alpha_rows.push_back(std::move(row));
      continue;
    }

    const auto adj = BuildBoundaryAdjacency(delaunay.triangles, keep);
    size_t bedge = 0;
    for (const auto& [a, outs] : adj) {
      bedge += outs.size();
    }
    row.boundary_edge_count = bedge;
    auto polygons = TraceBoundaryPolygons(adj, coords);
    row.component_count = polygons.size();
    auto primary = SelectPrimaryOuterPolygon(polygons);
    row.polygon_vertex_count = primary.size();
    row.circumference = PolylinePerimeter(primary, true);
    row.circumference_to_hull_ratio = row.circumference / hull_circ;
    row.corner_count = primary.size();
    row.corner_count_to_hull_ratio = static_cast<double>(row.corner_count) / hull_corners;
    row.area = std::abs(PolygonSignedArea(primary));
    row.ok = EstimatePolylineFractalDimension(primary, true, row.fractal_dimension, row.fractal_fit_r2, &row.note);

    if (!export_dir.empty() && !primary.empty()) {
      std::ostringstream name;
      name << "alpha_hull_h" << height << "_b" << branch_id << "_alpha_" << alpha << ".svg";
      WriteSvg(export_dir / name.str(), out_report.points, out_report.convex_hull_polygon, primary,
               "alpha=" + std::to_string(alpha));
    }
    out_report.alpha_rows.push_back(std::move(row));
  }

  if (!export_dir.empty()) {
    WriteSvg(export_dir / ("convex_hull_h" + std::to_string(height) + "_b" + std::to_string(branch_id) + ".svg"),
             out_report.points, out_report.convex_hull_polygon, {}, "convex hull");
  }
  return true;
}

bool WriteAlphaHullFractalCsv(const AlphaHullFractalReport& report, const std::filesystem::path& csv_path) {
  std::error_code ec;
  std::filesystem::create_directories(csv_path.parent_path(), ec);
  std::ofstream out(csv_path, std::ios::out | std::ios::trunc);
  if (!out) {
    return false;
  }
  out << "kind,alpha,height,branch_id,point_count,delaunay_triangles,kept_triangles,boundary_edges,"
         "polygon_vertices,components,circumference,circumference_to_hull_ratio,corner_count,"
         "corner_count_to_hull_ratio,area,fractal_dimension,fractal_fit_r2,ok,note\n";

  const auto write_row = [&](const char* kind, const double alpha, const size_t kept, const size_t bedges,
                             const size_t verts, const size_t comps, const double circ, const double circ_ratio,
                             const size_t corners, const double corner_ratio, const double area, const double fd,
                             const double r2, const bool ok, const std::string& note) {
    out << kind << ',' << alpha << ',' << report.height << ',' << report.branch_id << ',' << report.point_count << ','
        << report.delaunay_triangle_count << ',' << kept << ',' << bedges << ',' << verts << ',' << comps << ',' << circ
        << ',' << circ_ratio << ',' << corners << ',' << corner_ratio << ',' << area << ',' << fd << ',' << r2 << ','
        << (ok ? 1 : 0) << ',' << note << '\n';
  };

  write_row("convex_hull", std::numeric_limits<double>::infinity(), report.delaunay_triangle_count,
            report.convex_hull.vertex_count, report.convex_hull.vertex_count, 1, report.convex_hull.circumference, 1.0,
            report.convex_hull.corner_count, 1.0, report.convex_hull.area, report.convex_hull.fractal_dimension,
            report.convex_hull.fractal_fit_r2, report.convex_hull.ok, report.convex_hull.note);
  for (const auto& row : report.alpha_rows) {
    write_row("alpha_hull", row.alpha, row.kept_triangle_count, row.boundary_edge_count, row.polygon_vertex_count,
              row.component_count, row.circumference, row.circumference_to_hull_ratio, row.corner_count,
              row.corner_count_to_hull_ratio, row.area, row.fractal_dimension, row.fractal_fit_r2, row.ok, row.note);
  }
  return true;
}

}  // namespace eco_sys_lab_package
