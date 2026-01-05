#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <glm/vec2.hpp>

namespace eco_sys_lab_package {

struct AlphaHullFractalRow {
  double alpha = 0.0;
  size_t kept_triangle_count = 0;
  size_t boundary_edge_count = 0;
  size_t polygon_vertex_count = 0;
  size_t component_count = 0;
  /// Closed boundary length of the primary alpha-hull polygon.
  double circumference = 0.0;
  /// @ref circumference / convex-hull circumference (1 when identical).
  double circumference_to_hull_ratio = 0.0;
  /// Vertices on the closed boundary polygon (all boundary vertices).
  size_t corner_count = 0;
  /// @ref corner_count / convex-hull corner count.
  double corner_count_to_hull_ratio = 0.0;
  double area = 0.0;
  double fractal_dimension = 0.0;
  double fractal_fit_r2 = 0.0;
  bool ok = false;
  std::string note;
};

struct ConvexHullFractalRow {
  size_t vertex_count = 0;
  double circumference = 0.0;
  size_t corner_count = 0;
  double area = 0.0;
  double fractal_dimension = 0.0;
  double fractal_fit_r2 = 0.0;
  bool ok = false;
  std::string note;
};

struct AlphaHullFractalReport {
  size_t height = 0;
  size_t branch_id = 0;
  size_t point_count = 0;
  size_t delaunay_triangle_count = 0;
  ConvexHullFractalRow convex_hull;
  std::vector<AlphaHullFractalRow> alpha_rows;
  std::vector<glm::dvec2> points;
  std::vector<glm::dvec2> convex_hull_polygon;
};

/// Box-counting fractal dimension of a closed or open polyline (boundary).
[[nodiscard]] bool EstimatePolylineFractalDimension(const std::vector<glm::dvec2>& polyline, bool closed,
                                                    double& out_dimension, double& out_r2,
                                                    std::string* out_note = nullptr);

/// Delaunay of @p points, alpha complexes for each alpha (keep triangles with R^2 < alpha),
/// extract outer alpha-hull polygons, measure fractal dimension; also measure Delaunay convex hull.
[[nodiscard]] bool AnalyzeAlphaHullFractalDimensions(const std::vector<glm::dvec2>& points,
                                                     const std::vector<double>& alphas, size_t height, size_t branch_id,
                                                     AlphaHullFractalReport& out_report,
                                                     const std::filesystem::path& export_dir = {});

/// Write CSV summary for one report.
[[nodiscard]] bool WriteAlphaHullFractalCsv(const AlphaHullFractalReport& report,
                                            const std::filesystem::path& csv_path);

}  // namespace eco_sys_lab_package
