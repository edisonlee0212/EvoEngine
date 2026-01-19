#pragma once
#include "Polynomial.hpp"
#include "VoronoiPoint.hpp"

namespace kinDS {
template <size_t dim>
using Trajectory = std::array<Polynomial, dim>;

template <size_t dim>
class CubicHermiteSpline {
 private:
  std::vector<VoronoiPoint<dim>> points;  // Control points
  static const bool use_linear = true;

  static Trajectory<dim> createPiece(const VoronoiPoint<dim>& P0, const VoronoiPoint<dim>& M0,
                                     const VoronoiPoint<dim>& P1, const VoronoiPoint<dim>& M1) {
    Trajectory<dim> result{};

    if (!use_linear) {
      // Create the cubic Hermite spline polynomials for each dimension
      for (int i = 0; i < dim; ++i) {
        result[i] = POLYNOMIAL((2 * (x ^ 3) - 3 * (x ^ 2) + 1) * P0[i] + ((x ^ 3) - 2 * (x ^ 2) + x) * M0[i] +
                               ((-2 * (x ^ 3)) + 3 * (x ^ 2)) * P1[i] + ((x ^ 3) - (x ^ 2)) * M1[i]);
      }
    } else {
      // Create linear polynomials for each dimension
      for (int i = 0; i < dim; ++i) {
        result[i] = POLYNOMIAL(P0[i] + (P1[i] - P0[i]) * x);
      }
    }
    return result;
  }

 public:
  CubicHermiteSpline() = default;

  CubicHermiteSpline(const std::vector<VoronoiPoint<dim>>& controlPoints) : points(controlPoints) {
  }

  void addControlPoint(const VoronoiPoint<dim>& point) {
    points.push_back(point);
  }

  void addControlPoints(const std::vector<VoronoiPoint<dim>>& newPoints) {
    points.insert(points.end(), newPoints.begin(), newPoints.end());
  }

  Trajectory<dim> getPiecePolynomial(size_t index, size_t reference_branch_id) const {
  }

  Trajectory<dim> getPiecePolynomial(size_t index) const {
    if (index >= points.size() - 1) {
      throw std::out_of_range("Index out of range for piece polynomial.");
    }
    const auto& P0 = points[index];
    const auto& P1 = points[index + 1];

    // Calculate tangents (M0, M1) as needed
    VoronoiPoint<dim> M0, M1;
    for (size_t i = 0; i < dim; ++i) {
      M0[i] = (index > 0) ? (P1[i] - points[index - 1][i]) / 2.0 : (P1[i] - P0[i]) / 2.0;
      M1[i] = (index < points.size() - 2) ? (points[index + 2][i] - P0[i]) / 2.0 : (P1[i] - P0[i]) / 2.0;
    }

    return createPiece(P0, M0, P1, M1);
  }

  VoronoiPoint<dim> evaluate(double t) const {
    if (points.empty()) {
      throw std::runtime_error("No control points defined.");
    }

    // catch t out of bounds
    if (t < 0 || t > points.size() - 1) {
      throw std::out_of_range("Parameter t is out of bounds for the spline.");
    }

    size_t segment = static_cast<size_t>(t);
    if (segment == points.size() - 1) {
      segment = points.size() - 2;  // Clamp to the last segment
    }

    double localT = t - segment;
    auto piece = getPiecePolynomial(segment);
    VoronoiPoint<dim> result{};
    for (size_t i = 0; i < dim; ++i) {
      result[i] = piece[i](localT);
    }
    return result;
  }

  const std::vector<VoronoiPoint<dim>> getPoints() const {
    return points;
  }

  size_t pointCount() const {
    return points.size();
  }
};
}  // namespace kinDS