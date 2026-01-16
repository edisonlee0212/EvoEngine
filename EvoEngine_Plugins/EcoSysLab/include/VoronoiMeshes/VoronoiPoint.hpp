#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace kinDS {
template <size_t dim>
class VoronoiPoint : public std::array<double, dim> {
 public:
  // These should be inherited from std::array

  // String representation for debugging
  std::string toString() const {
    std::string result = "(";
    for (size_t i = 0; i < dim; ++i) {
      result += std::to_string((*this)[i]);
      if (i < dim - 1)
        result += ", ";
    }
    result += ")";
    return result;
  }

  double operator*(const VoronoiPoint<dim>& other) const {
    double result = 0.0;
    for (size_t i = 0; i < dim; ++i) {
      result += (*this)[i] * other[i];
    }
    return result;
  }

  VoronoiPoint& operator+=(const VoronoiPoint& rhs) {
    *this = *this + rhs;
    return *this;
  }

  VoronoiPoint& operator-=(const VoronoiPoint& rhs) {
    *this = *this - rhs;
    return *this;
  }

  VoronoiPoint operator-() const {
    VoronoiPoint result;
    for (size_t i = 0; i < dim; ++i) {
      result[i] = -(*this)[i];
    }
    return result;
  }

  VoronoiPoint& operator*=(double scalar) {
    *this = *this * scalar;
    return *this;
  }

  double len_sqr() const {
    return (*this) * (*this);
  }

  double len() const {
    return std::sqrt(len_sqr());
  }

  double dist_sqr(const VoronoiPoint<dim>& other) const {
    return ((*this) - other).len_sqr();
  }

  double dist(const VoronoiPoint<dim>& other) const {
    return std::sqrt(dist_sqr(other));
  }

  VoronoiPoint<dim> normalized() const {
    double length = len();
    if (length == 0) {
      throw std::runtime_error("Cannot normalize a zero-length vector");
    }
    VoronoiPoint<dim> result{};
    for (size_t i = 0; i < dim; ++i) {
      result[i] = (*this)[i] / length;
    }
    return result;
  }
};

// operators for Point
template <size_t dim>
VoronoiPoint<dim> operator+(const VoronoiPoint<dim>& a, const VoronoiPoint<dim>& b) {
  VoronoiPoint<dim> result{};
  for (size_t i = 0; i < dim; ++i) {
    result[i] = a[i] + b[i];
  }
  return result;
}

template <size_t dim>
VoronoiPoint<dim> operator-(const VoronoiPoint<dim>& a, const VoronoiPoint<dim>& b) {
  VoronoiPoint<dim> result{};
  for (size_t i = 0; i < dim; ++i) {
    result[i] = a[i] - b[i];
  }
  return result;
}

template <size_t dim>
VoronoiPoint<dim> operator*(const VoronoiPoint<dim>& a, double scalar) {
  VoronoiPoint<dim> result{};
  for (size_t i = 0; i < dim; ++i) {
    result[i] = a[i] * scalar;
  }
  return result;
}

template <size_t dim>
VoronoiPoint<dim> operator/(const VoronoiPoint<dim>& a, double scalar) {
  VoronoiPoint<dim> result{};
  for (size_t i = 0; i < dim; ++i) {
    result[i] = a[i] / scalar;
  }
  return result;
}

// allow multiplication with a scalar before a point
template <size_t dim>
VoronoiPoint<dim> operator*(double scalar, const VoronoiPoint<dim>& a) {
  VoronoiPoint<dim> result{};
  for (size_t i = 0; i < dim; ++i) {
    result[i] = a[i] * scalar;
  }
  return result;
}

// cross product
VoronoiPoint<3> operator%(const VoronoiPoint<3>& a, const VoronoiPoint<3>& b);
double operator%(const VoronoiPoint<2>& a, const VoronoiPoint<2>& b);

// also provide Vector as alias
template <size_t dim>
using VoronoiVector = VoronoiPoint<dim>;
}  // namespace kinDS