#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace kinDS {
template <size_t dim>
class Point : public std::array<double, dim> {
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

  double operator*(const Point<dim>& other) const {
    double result = 0.0;
    for (size_t i = 0; i < dim; ++i) {
      result += (*this)[i] * other[i];
    }
    return result;
  }

  double len_sqr() const {
    return (*this) * (*this);
  }

  double len() const {
    return std::sqrt(len_sqr());
  }

  double dist_sqr(const Point<dim>& other) const {
    return ((*this) - other).len_sqr();
  }

  double dist(const Point<dim>& other) const {
    return std::sqrt(dist_sqr(other));
  }

  Point<dim> normalized() const {
    double length = len();
    if (length == 0) {
      throw std::runtime_error("Cannot normalize a zero-length vector");
    }
    Point<dim> result{};
    for (size_t i = 0; i < dim; ++i) {
      result[i] = (*this)[i] / length;
    }
    return result;
  }
};

// operators for Point
template <size_t dim>
Point<dim> operator+(const Point<dim>& a, const Point<dim>& b) {
  Point<dim> result{};
  for (size_t i = 0; i < dim; ++i) {
    result[i] = a[i] + b[i];
  }
  return result;
}

template <size_t dim>
Point<dim> operator-(const Point<dim>& a, const Point<dim>& b) {
  Point<dim> result{};
  for (size_t i = 0; i < dim; ++i) {
    result[i] = a[i] - b[i];
  }
  return result;
}

template <size_t dim>
Point<dim> operator*(const Point<dim>& a, double scalar) {
  Point<dim> result{};
  for (size_t i = 0; i < dim; ++i) {
    result[i] = a[i] * scalar;
  }
  return result;
}

// allow multiplication with a scalar before a point

template <size_t dim>
Point<dim> operator*(double scalar, const Point<dim>& a) {
  Point<dim> result{};
  for (size_t i = 0; i < dim; ++i) {
    result[i] = a[i] * scalar;
  }
  return result;
}

// cross product for 3D points
Point<3> operator%(const Point<3>& a, const Point<3>& b);

// also provide Vector as alias
template <size_t dim>
using Vector = Point<dim>;
}  // namespace kinDS
