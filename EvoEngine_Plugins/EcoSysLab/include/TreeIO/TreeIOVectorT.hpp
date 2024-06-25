/**
 * @author Tomas Polasek
 * @date 2.20.2020
 * @version 1.0
 * @brief Vector in 3D space.
 */

#ifndef TREEIO_VECTORT_H
#define TREEIO_VECTORT_H

#include <cstring>
#include <ostream>

namespace treeutil {

/// @brief Vector in 2D space.
template <typename T>
struct Vector2DT {
  /// Number of elements in this vector.
  static constexpr auto ELEMENT_COUNT{2u};
  /// Type of values within this vector.
  using ValueT = T;
  /// Shortcut for type of this vector
  using VectorT = Vector2DT<ValueT>;

  /// @brief Initialize null vector.
  constexpr Vector2DT();
  /// @brief Initialize all elements to one value.
  explicit constexpr Vector2DT(const T &v);
  /// @brief Initialize vector with given values.
  constexpr Vector2DT(const T &x, const T &y);
  /// @brief Initialize vector from array.
  explicit Vector2DT(const float (&arr)[ELEMENT_COUNT]);
  /// @brief Initialize all elements to one value.
  template <typename TT>
  explicit Vector2DT(const TT &v);
  /// @brief Initialize vector with given values.
  template <typename TT>
  Vector2DT(const TT &x, const TT &y);
#ifdef HAS_GLM
  /// @brief Convert glm::vec2 to vector.
  Vector2DT(const glm::vec2 &vec);
#endif  // HAS_GLM

  /// @brief Create the zero vector.
  static constexpr VectorT zero();
  /// @brief Create the one vector.
  static constexpr VectorT one();
  /// @brief Create maximum valued vector.
  static constexpr VectorT maxVector();
  /// @brief Create minimum valued vector.
  static constexpr VectorT minVector();

  // Copy and move operators:
  Vector2DT(const Vector2DT &other);
  Vector2DT &operator=(const Vector2DT &other);
  Vector2DT(Vector2DT &&other);
  Vector2DT &operator=(Vector2DT &&other);

  /// @brief Swap operator.
  void swap(Vector2DT &other);

#ifdef HAS_GLM
  /// @brief Convert this vector to glm vector with same values.
  glm::vec2 toGlm() const;
#endif  // HAS_GLM

  /// @brief Get length of this vector = sqrt(val_1^2 + ... + val_n^2).
  T length() const;
  /// @brief Get squared length of this vector = val_1^2 + ... + val_n^2.
  T squaredLength() const;

  /// @brief Calculate distance between given points.
  static T distance(const VectorT &first, const VectorT &second);
  /// @brief Calculate squared distance between given points.
  static T squaredDistance(const VectorT &first, const VectorT &second);

  /// @brief Calculate distance from this to the other point.
  T distanceTo(const VectorT &other) const;
  /// @brief Calculate squared distance from this to the other point.
  T squaredDistanceTo(const VectorT &other) const;

  /// @brief Get normalized version of this vector.
  VectorT normalized() const;

  /// @brief Normalize this vector and return its original length.
  T normalize();

  /// @brief Access element of the vector.
  T &operator[](std::size_t idx);
  /// @brief Access element of the vector.
  const T &operator[](std::size_t idx) const;

  // @brief Unary plus, returns the original vector.
  VectorT operator+() const;
  // @brief Unary minus, returns the negation of the original vector.
  VectorT operator-() const;

  /// @brief Add the point vectors.
  VectorT &operator+=(const VectorT &other);
  /// @brief Add the point vectors.
  VectorT operator+(const VectorT &other) const;

  /// @brief Substract the point vectors.
  VectorT &operator-=(const VectorT &other);
  /// @brief Substract the point vectors.
  VectorT operator-(const VectorT &other) const;

  /// @brief Divide whole point vector by given number.
  VectorT &operator/=(const ValueT &denominator);
  /// @brief Divide whole point vector by given number.
  VectorT operator/(const ValueT &denominator) const;

  /// @brief Multiply whole point vector by given number.
  VectorT &operator*=(const ValueT &multiplier);
  /// @brief Multiply whole point vector by given number.
  VectorT operator*(const ValueT &multiplier) const;

  /// @brief Perform element-wise division by the other vector.
  VectorT &operator/=(const VectorT &other);
  /// @brief Perform element-wise division by the other vector.
  VectorT operator/(const VectorT &other) const;

  /// @brief Perform element-wise multiplication by the other vector.
  VectorT &operator*=(const VectorT &other);
  /// @brief Perform element-wise multiplication by the other vector.
  VectorT operator*(const VectorT &other) const;

  /// @brief Get element-wise min for given vectors.
  static VectorT elementMin(const VectorT &first, const VectorT &second);
  /// @brief Get element-wise max for given vectors.
  static VectorT elementMax(const VectorT &first, const VectorT &second);

  /// @brief Set element-wise min for this and the other vector.
  VectorT &elementMin(const VectorT &other);
  /// @brief Set element-wise max for this and the other vector.
  VectorT &elementMax(const VectorT &other);

  /// @brief Get minimum element from this vector.
  const ValueT &min() const;
  /// @brief Get maximum element from this vector.
  const ValueT &max() const;

  /// @brief Calculate dot product of first . second.
  static ValueT dotProduct(const VectorT &first, const VectorT &second);
  /// @brief Calculate cross product of first x second.
  static T crossProduct(const VectorT &first, const VectorT &second);

  /// @brief Calculate dot product between this and the other vector.
  ValueT dot(const VectorT &other) const;
  /// @brief Calculate cross product between this and the other vector.
  T crossProduct(const VectorT &other) const;

  /// @brief Perform sgn(vector), resulting in vector in {-1.0f, 0.0f, 1.0f}.
  static VectorT sgn(const VectorT &vector);
  /// @brief Perform sgn(vector), resulting in vector in {-1.0f, 0.0f, 1.0f}.
  VectorT sgn() const;

  /// @brief Cast vector to given type.
  template <typename TT>
  explicit operator TT() const;

  /// @brief Cast vector to given vector type.
  template <typename TT>
  explicit operator Vector2DT<TT>() const;

  union {  // Internal data:
    /// Contiguous array of vector values.
    ValueT values[ELEMENT_COUNT];
    struct {
      /// X-axis coordinate.
      ValueT x;
      /// Y-axis coordinate.
      ValueT y;
    };
  };
};  // struct Vector2DT

/// @brief Divide whole point vector by given number.
template <typename T>
Vector2DT<T> operator/(const T &numerator, const Vector2DT<T> &denominator);

/// @brief Multiply whole point vector by given number.
template <typename T>
Vector2DT<T> operator*(const T &multiplier, const Vector2DT<T> &vector);

/// @brief Vector in 3D space.
template <typename T>
struct Vector3DT {
  /// Number of elements in this vector.
  static constexpr auto ELEMENT_COUNT{3u};
  /// Type of values within this vector.
  using ValueT = T;
  /// Shortcut for type of this vector
  using VectorT = Vector3DT<ValueT>;

  /// @brief Create a random 3D vector.
  static Vector3DT uniformRandom();

  /// @brief Initialize null vector.
  constexpr Vector3DT();
  /// @brief Initialize all elements to one value.
  explicit constexpr Vector3DT(const T &v);
  /// @brief Initialize vector with given values.
  constexpr Vector3DT(const T &x, const T &y, const T &z);
  /// @brief Initialize vector from array.
  explicit Vector3DT(const float (&arr)[ELEMENT_COUNT]);
  /// @brief Initialize all elements to one value.
  template <typename TT>
  explicit Vector3DT(const TT &v);
  /// @brief Initialize vector with given values.
  template <typename TT>
  Vector3DT(const TT &x, const TT &y, const TT &z);
#ifdef HAS_GLM
  /// @brief Convert glm::vec3 to vector.
  Vector3DT(const glm::vec3 &vec);

  /// @brief Convert glm::vec4 to vector.
  Vector3DT(const glm::vec4 &vec);

  /// @brief Convert glm::vec4 to vector, dividing by w.
  Vector3DT(const glm::vec4 &vec, bool);
#endif  // HAS_GLM

  /// @brief Create the zero vector.
  static constexpr VectorT zero();
  /// @brief Create the one vector.
  static constexpr VectorT one();
  /// @brief Create maximum valued vector.
  static constexpr VectorT maxVector();
  /// @brief Create minimum valued vector.
  static constexpr VectorT minVector();

  // Copy and move operators:
  Vector3DT(const VectorT &other);
  Vector3DT &operator=(const VectorT &other);
  Vector3DT(VectorT &&other);
  Vector3DT &operator=(VectorT &&other);

  /// @brief Swap operator.
  void swap(VectorT &other);

#ifdef HAS_GLM
  /// @brief Convert this vector to glm vector with same values.
  glm::vec3 toGlm() const;
#endif  // HAS_GLM

  /// @brief Get length of this vector = sqrt(val_1^2 + ... + val_n^2).
  T length() const;
  /// @brief Get squared length of this vector = val_1^2 + ... + val_n^2.
  T squaredLength() const;

  /// @brief Calculate distance between given points.
  static T distance(const VectorT &first, const VectorT &second);
  /// @brief Calculate squared distance between given points.
  static T squaredDistance(const VectorT &first, const VectorT &second);

  /// @brief Calculate distance from this to the other point.
  T distanceTo(const VectorT &other) const;
  /// @brief Calculate squared distance from this to the other point.
  T squaredDistanceTo(const VectorT &other) const;

  /// @brief Get normalized version of this vector.
  VectorT normalized() const;

  /// @brief Normalize this vector and return its original length.
  T normalize();

  /// @brief Access element of the vector.
  T &operator[](std::size_t idx);
  /// @brief Access element of the vector.
  const T &operator[](std::size_t idx) const;

  // @brief Unary plus, returns the original vector.
  VectorT operator+() const;
  // @brief Unary minus, returns the negation of the original vector.
  VectorT operator-() const;

  /// @brief Add the point vectors.
  VectorT &operator+=(const VectorT &other);
  /// @brief Add the point vectors.
  VectorT operator+(const VectorT &other) const;

  /// @brief Substract the point vectors.
  VectorT &operator-=(const VectorT &other);
  /// @brief Substract the point vectors.
  VectorT operator-(const VectorT &other) const;

  /// @brief Divide whole point vector by given number.
  VectorT &operator/=(const ValueT &denominator);
  /// @brief Divide whole point vector by given number.
  VectorT operator/(const ValueT &denominator) const;

  /// @brief Multiply whole point vector by given number.
  VectorT &operator*=(const ValueT &multiplier);
  /// @brief Multiply whole point vector by given number.
  VectorT operator*(const ValueT &multiplier) const;

  /// @brief Perform element-wise division by the other vector.
  VectorT &operator/=(const VectorT &other);
  /// @brief Perform element-wise division by the other vector.
  VectorT operator/(const VectorT &other) const;

  /// @brief Perform element-wise multiplication by the other vector.
  VectorT &operator*=(const VectorT &other);
  /// @brief Perform element-wise multiplication by the other vector.
  VectorT operator*(const VectorT &other) const;

  /// @brief Get element-wise min for given vectors.
  static VectorT elementMin(const VectorT &first, const VectorT &second);
  /// @brief Get element-wise max for given vectors.
  static VectorT elementMax(const VectorT &first, const VectorT &second);

  /// @brief Set element-wise min for this and the other vector.
  VectorT &elementMin(const VectorT &other);
  /// @brief Set element-wise max for this and the other vector.
  VectorT &elementMax(const VectorT &other);

  /// @brief Get minimum element from this vector.
  const ValueT &min() const;
  /// @brief Get maximum element from this vector.
  const ValueT &max() const;

  /// @brief Calculate dot product of first . second.
  static ValueT dotProduct(const VectorT &first, const VectorT &second);
  /// @brief Calculate cross product of first x second.
  static VectorT crossProduct(const VectorT &first, const VectorT &second);

  /// @brief Calculate dot product between this and the other vector.
  ValueT dot(const VectorT &other) const;
  /// @brief Calculate cross product between this and the other vector.
  VectorT crossProduct(const VectorT &other) const;

  /// @brief Perform sgn(vector), resulting in vector in {-1.0f, 0.0f, 1.0f}.
  static VectorT sgn(const VectorT &vector);
  /// @brief Perform sgn(vector), resulting in vector in {-1.0f, 0.0f, 1.0f}.
  VectorT sgn() const;

  /// @brief Cast vector to given type.
  template <typename TT>
  explicit operator TT() const;

  /// @brief Cast vector to given vector type.
  template <typename TT>
  explicit operator Vector2DT<TT>() const;

  /// @brief Cast vector to given vector type.
  template <typename TT>
  explicit operator Vector3DT<TT>() const;

  union {  // Internal data:
    /// Contiguous array of vector values.
    ValueT values[ELEMENT_COUNT];
    struct {
      /// X-axis coordinate.
      ValueT x;
      /// Y-axis coordinate.
      ValueT y;
      /// Z-axis coordinate.
      ValueT z;
    };
  };
};  // struct Vector3DT

/// @brief Divide whole point vector by given number.
template <typename T>
Vector3DT<T> operator/(const T &numerator, const Vector3DT<T> &denominator);

/// @brief Multiply whole point vector by given number.
template <typename T>
Vector3DT<T> operator*(const T &multiplier, const Vector3DT<T> &vector);

/// Shortcut for Vector3DT<float>.
using Vector3D = Vector3DT<float>;
/// Shortcut for Vector2DT<float>.
using Vector2D = Vector2DT<float>;

namespace impl {

/// @brief Generate random real number from 0.0 to 1.0 .
template <typename T = double>
T uniformZeroToOne();

/// @brief Calculate sgn(val) in {T(-1), T(0), T(1)}.
template <typename T>
T sgn(const T &val);

}  // namespace impl

}  // namespace treeutil

namespace treeio {

// Shortcut for vectors in ::treeio
using treeutil::Vector2D;
using treeutil::Vector3D;

}  // namespace treeio

namespace std {

// Min and max operations for 2D vector.
template <typename T>
::treeutil::Vector2DT<T> min(const ::treeutil::Vector2DT<T> &first, const ::treeutil::Vector2DT<T> &second);
template <typename T>
::treeutil::Vector2DT<T> max(const ::treeutil::Vector2DT<T> &first, const ::treeutil::Vector2DT<T> &second);

// Min and max operations for 3D vector.
template <typename T>
::treeutil::Vector3DT<T> min(const ::treeutil::Vector3DT<T> &first, const ::treeutil::Vector3DT<T> &second);
template <typename T>
::treeutil::Vector3DT<T> max(const ::treeutil::Vector3DT<T> &first, const ::treeutil::Vector3DT<T> &second);

}  // namespace std

/// @brief Print vector to given stream.
template <typename T>
inline std::ostream &operator<<(std::ostream &out, const treeutil::Vector3DT<T> &vec);

/// @brief Print vector to given stream.
template <typename T>
inline std::ostream &operator<<(std::ostream &out, const treeutil::Vector2DT<T> &vec);

// Template implementation

namespace treeutil {

template <typename T>
constexpr Vector2DT<T>::Vector2DT() : Vector2DT(T{}) {
}
template <typename T>
constexpr Vector2DT<T>::Vector2DT(const T &v) : Vector2DT(v, v) {
}
template <typename T>
constexpr Vector2DT<T>::Vector2DT(const T &vx, const T &vy) : x{vx}, y{vy} {
}
template <typename T>
Vector2DT<T>::Vector2DT(const float (&arr)[ELEMENT_COUNT]) : values{arr} {
}
template <typename T>
template <typename TT>
Vector2DT<T>::Vector2DT(const TT &v) : Vector2DT(static_cast<T>(v)) {
}
template <typename T>
template <typename TT>
Vector2DT<T>::Vector2DT(const TT &x, const TT &y) : Vector2DT(static_cast<T>(x), static_cast<T>(y)) {
}
#ifdef HAS_GLM
template <typename T>
Vector2DT<T>::Vector2DT(const glm::vec2 &vec) : Vector2DT(vec.x, vec.y) {
}
#endif  // HAS_GLM
template <typename T>
constexpr Vector2DT<T> Vector2DT<T>::zero() {
  return VectorT(T(0));
}
template <typename T>
constexpr Vector2DT<T> Vector2DT<T>::one() {
  return VectorT(T(1));
}
template <typename T>
constexpr Vector2DT<T> Vector2DT<T>::maxVector() {
  return VectorT(std::numeric_limits<T>::max());
}
template <typename T>
constexpr Vector2DT<T> Vector2DT<T>::minVector() {
  return VectorT(std::numeric_limits<T>::min());
}
template <typename T>
Vector2DT<T>::Vector2DT(const Vector2DT &other) {
  *this = other;
}
template <typename T>
Vector2DT<T> &Vector2DT<T>::operator=(const Vector2DT &other) {
  std::memcpy(values, other.values, sizeof(T) * ELEMENT_COUNT);
  return *this;
}
template <typename T>
Vector2DT<T>::Vector2DT(Vector2DT &&other) {
  *this = other;
}
template <typename T>
Vector2DT<T> &Vector2DT<T>::operator=(Vector2DT &&other) {
  std::memcpy(values, other.values, sizeof(T) * ELEMENT_COUNT);
  return *this;
}
template <typename T>
void Vector2DT<T>::swap(Vector2DT &other) {
  std::swap(values, other.values);
}

#ifdef HAS_GLM
template <typename T>
glm::vec2 Vector2DT<T>::toGlm() const {
  return glm::vec2{x, y};
}
#endif  // HAS_GLM

template <typename T>
T Vector2DT<T>::length() const {
  return static_cast<T>(std::sqrt(x * x + y * y));
}
template <typename T>
T Vector2DT<T>::squaredLength() const {
  return (x * x + y * y);
}

template <typename T>
T Vector2DT<T>::distance(const VectorT &first, const VectorT &second) {
  return (second - first).length();
}
template <typename T>
T Vector2DT<T>::squaredDistance(const VectorT &first, const VectorT &second) {
  return (second - first).squaredLength();
}

template <typename T>
T Vector2DT<T>::distanceTo(const VectorT &other) const {
  return distance(*this, other);
}
template <typename T>
T Vector2DT<T>::squaredDistanceTo(const VectorT &other) const {
  return squaredDistance(*this, other);
}

template <typename T>
Vector2DT<T> Vector2DT<T>::normalized() const {
  return *this / (length() + std::numeric_limits<T>::epsilon());
}

template <typename T>
T Vector2DT<T>::normalize() {
  const auto l{length()};
  *this /= l + std::numeric_limits<T>::epsilon();
  return l;
}

template <typename T>
T &Vector2DT<T>::operator[](std::size_t idx) {
  return values[idx];
}
template <typename T>
const T &Vector2DT<T>::operator[](std::size_t idx) const {
  return values[idx];
}

template <typename T>
Vector2DT<T> Vector2DT<T>::operator+() const {
  return *this;
}
template <typename T>
Vector2DT<T> Vector2DT<T>::operator-() const {
  return {-values[0], -values[1]};
}

template <typename T>
Vector2DT<T> &Vector2DT<T>::operator+=(const Vector2DT<T> &other) {
  values[0] += other[0];
  values[1] += other[1];
  return *this;
}
template <typename T>
Vector2DT<T> Vector2DT<T>::operator+(const Vector2DT<T> &other) const {
  auto p{*this};
  p += other;
  return p;
}

template <typename T>
Vector2DT<T> &Vector2DT<T>::operator-=(const Vector2DT<T> &other) {
  values[0] -= other[0];
  values[1] -= other[1];
  return *this;
}
template <typename T>
Vector2DT<T> Vector2DT<T>::operator-(const Vector2DT<T> &other) const {
  auto p{*this};
  p -= other;
  return p;
}

template <typename T>
Vector2DT<T> &Vector2DT<T>::operator/=(const T &denominator) {
  values[0] /= denominator;
  values[1] /= denominator;
  return *this;
}
template <typename T>
Vector2DT<T> Vector2DT<T>::operator/(const T &denominator) const {
  auto p{*this};
  p /= denominator;
  return p;
}

template <typename T>
Vector2DT<T> &Vector2DT<T>::operator*=(const T &multiplier) {
  values[0] *= multiplier;
  values[1] *= multiplier;
  return *this;
}
template <typename T>
Vector2DT<T> Vector2DT<T>::operator*(const T &multiplier) const {
  auto p{*this};
  p *= multiplier;
  return p;
}

template <typename T>
Vector2DT<T> &Vector2DT<T>::operator/=(const VectorT &other) {
  values[0] /= other[0];
  values[1] /= other[1];
  return *this;
}
template <typename T>
Vector2DT<T> Vector2DT<T>::operator/(const VectorT &other) const {
  auto p{*this};
  p /= other;
  return p;
}

template <typename T>
Vector2DT<T> &Vector2DT<T>::operator*=(const VectorT &other) {
  values[0] *= other[0];
  values[1] *= other[1];
  return *this;
}
template <typename T>
Vector2DT<T> Vector2DT<T>::operator*(const VectorT &other) const {
  auto p{*this};
  p *= other;
  return p;
}

template <typename T>
Vector2DT<T> Vector2DT<T>::elementMin(const VectorT &first, const VectorT &second) {
  return {std::min(first.x, second.x), std::min(first.y, second.y), std::min(first.z, second.z)};
}
template <typename T>
Vector2DT<T> Vector2DT<T>::elementMax(const VectorT &first, const VectorT &second) {
  return {std::max(first.x, second.x), std::max(first.y, second.y), std::max(first.z, second.z)};
}

template <typename T>
Vector2DT<T> &Vector2DT<T>::elementMin(const VectorT &other) {
  *this = VectorT::elementMin(*this, other);
  return *this;
}
template <typename T>
Vector2DT<T> &Vector2DT<T>::elementMax(const VectorT &other) {
  *this = VectorT::elementMax(*this, other);
  return *this;
}

template <typename T>
const T &Vector2DT<T>::min() const {
  return std::min(values[0], values[1]);
}
template <typename T>
const T &Vector2DT<T>::max() const {
  return std::max(values[0], values[1]);
}

template <typename T>
T Vector2DT<T>::dotProduct(const Vector2DT<T> &first, const Vector2DT<T> &second) {
  return first.x * second.x + first.y * second.y;
}
template <typename T>
T Vector2DT<T>::crossProduct(const Vector2DT<T> &first, const Vector2DT<T> &second) {
  return first.x * second.y - first.y * second.x;
}

template <typename T>
T Vector2DT<T>::dot(const Vector2DT<T> &other) const {
  return dotProduct(*this, other);
}
template <typename T>
T Vector2DT<T>::crossProduct(const Vector2DT<T> &other) const {
  return crossProduct(*this, other);
}

template <typename T>
Vector2DT<T> Vector2DT<T>::sgn(const VectorT &vector) {
  return {impl::sgn(vector[0]), impl::sgn(vector[1])};
}

template <typename T>
Vector2DT<T> Vector2DT<T>::sgn() const {
  return sgn(*this);
}

template <typename T>
Vector2DT<T> operator/(const T &numerator, const Vector2DT<T> &denominator) {
  return Vector2DT<T>(numerator) / denominator;
}

template <typename T>
Vector2DT<T> operator*(const T &multiplier, const Vector2DT<T> &vector) {
  return Vector2DT<T>(multiplier) * vector;
}

template <typename T>
template <typename TT>
Vector2DT<T>::operator TT() const {
  return static_cast<TT>(values[0]);
}

template <typename T>
template <typename TT>
Vector2DT<T>::operator Vector2DT<TT>() const {
  return Vector2DT<TT>{static_cast<TT>(values[0]), static_cast<TT>(values[1])};
}

template <typename T>
Vector3DT<T> Vector3DT<T>::uniformRandom() {
  return {impl::uniformZeroToOne<T>() * 2.0f - 1.0f, impl::uniformZeroToOne<T>() * 2.0f - 1.0f,
          impl::uniformZeroToOne<T>() * 2.0f - 1.0f};
}

template <typename T>
constexpr Vector3DT<T>::Vector3DT() : Vector3DT(T{}) {
}
template <typename T>
constexpr Vector3DT<T>::Vector3DT(const T &v) : Vector3DT(v, v, v) {
}
template <typename T>
constexpr Vector3DT<T>::Vector3DT(const T &vx, const T &vy, const T &vz) : x{vx}, y{vy}, z{vz} {
}
template <typename T>
Vector3DT<T>::Vector3DT(const float (&arr)[ELEMENT_COUNT]) : values{arr} {
}
template <typename T>
template <typename TT>
Vector3DT<T>::Vector3DT(const TT &v) : Vector3DT(static_cast<T>(v)) {
}
template <typename T>
template <typename TT>
Vector3DT<T>::Vector3DT(const TT &x, const TT &y, const TT &z)
    : Vector3DT(static_cast<T>(x), static_cast<T>(y), static_cast<T>(z)) {
}
#ifdef HAS_GLM
template <typename T>
Vector3DT<T>::Vector3DT(const glm::vec3 &vec) : Vector3DT(vec.x, vec.y, vec.z) {
}

template <typename T>
Vector3DT<T>::Vector3DT(const glm::vec4 &vec) : Vector3DT(vec.x, vec.y, vec.z) {
}

template <typename T>
Vector3DT<T>::Vector3DT(const glm::vec4 &vec, bool) : Vector3DT(vec.x / vec.w, vec.y / vec.w, vec.z / vec.w) {
}
#endif  // HAS_GLM
template <typename T>
constexpr Vector3DT<T> Vector3DT<T>::zero() {
  return VectorT(T(0));
}
template <typename T>
constexpr Vector3DT<T> Vector3DT<T>::one() {
  return VectorT(T(1));
}
template <typename T>
constexpr Vector3DT<T> Vector3DT<T>::maxVector() {
  return VectorT(std::numeric_limits<T>::max());
}
template <typename T>
constexpr Vector3DT<T> Vector3DT<T>::minVector() {
  return VectorT(std::numeric_limits<T>::min());
}
template <typename T>
Vector3DT<T>::Vector3DT(const Vector3DT<T> &other) {
  *this = other;
}
template <typename T>
Vector3DT<T> &Vector3DT<T>::operator=(const Vector3DT &other) {
  std::memcpy(values, other.values, sizeof(T) * ELEMENT_COUNT);
  return *this;
}
template <typename T>
Vector3DT<T>::Vector3DT(Vector3DT &&other) {
  *this = other;
}
template <typename T>
Vector3DT<T> &Vector3DT<T>::operator=(Vector3DT &&other) {
  std::memcpy(values, other.values, sizeof(T) * ELEMENT_COUNT);
  return *this;
}
template <typename T>
void Vector3DT<T>::swap(Vector3DT &other) {
  std::swap(values, other.values);
}

#ifdef HAS_GLM
template <typename T>
glm::vec3 Vector3DT<T>::toGlm() const {
  return glm::vec3{x, y, z};
}
#endif  // HAS_GLM

template <typename T>
T Vector3DT<T>::length() const {
  return static_cast<T>(std::sqrt(x * x + y * y + z * z));
}
template <typename T>
T Vector3DT<T>::squaredLength() const {
  return (x * x + y * y + z * z);
}

template <typename T>
T Vector3DT<T>::distance(const VectorT &first, const VectorT &second) {
  return (second - first).length();
}
template <typename T>
T Vector3DT<T>::squaredDistance(const VectorT &first, const VectorT &second) {
  return (second - first).squaredLength();
}

template <typename T>
T Vector3DT<T>::distanceTo(const VectorT &other) const {
  return distance(*this, other);
}
template <typename T>
T Vector3DT<T>::squaredDistanceTo(const VectorT &other) const {
  return squaredDistance(*this, other);
}

template <typename T>
Vector3DT<T> Vector3DT<T>::normalized() const {
  return *this / (length() + std::numeric_limits<T>::epsilon());
}

template <typename T>
T Vector3DT<T>::normalize() {
  const auto l{length()};
  *this /= l + std::numeric_limits<T>::epsilon();
  return l;
}

template <typename T>
T &Vector3DT<T>::operator[](std::size_t idx) {
  return values[idx];
}
template <typename T>
const T &Vector3DT<T>::operator[](std::size_t idx) const {
  return values[idx];
}

template <typename T>
Vector3DT<T> Vector3DT<T>::operator+() const {
  return *this;
}
template <typename T>
Vector3DT<T> Vector3DT<T>::operator-() const {
  return {-values[0], -values[1], -values[2]};
}

template <typename T>
Vector3DT<T> &Vector3DT<T>::operator+=(const Vector3DT<T> &other) {
  values[0] += other[0];
  values[1] += other[1];
  values[2] += other[2];
  return *this;
}
template <typename T>
Vector3DT<T> Vector3DT<T>::operator+(const Vector3DT<T> &other) const {
  auto p{*this};
  p += other;
  return p;
}

template <typename T>
Vector3DT<T> &Vector3DT<T>::operator-=(const Vector3DT<T> &other) {
  values[0] -= other[0];
  values[1] -= other[1];
  values[2] -= other[2];
  return *this;
}
template <typename T>
Vector3DT<T> Vector3DT<T>::operator-(const Vector3DT<T> &other) const {
  auto p{*this};
  p -= other;
  return p;
}

template <typename T>
Vector3DT<T> &Vector3DT<T>::operator/=(const T &denominator) {
  values[0] /= denominator;
  values[1] /= denominator;
  values[2] /= denominator;
  return *this;
}
template <typename T>
Vector3DT<T> Vector3DT<T>::operator/(const T &denominator) const {
  auto p{*this};
  p /= denominator;
  return p;
}

template <typename T>
Vector3DT<T> &Vector3DT<T>::operator*=(const T &multiplier) {
  values[0] *= multiplier;
  values[1] *= multiplier;
  values[2] *= multiplier;
  return *this;
}
template <typename T>
Vector3DT<T> Vector3DT<T>::operator*(const T &multiplier) const {
  auto p{*this};
  p *= multiplier;
  return p;
}

template <typename T>
Vector3DT<T> &Vector3DT<T>::operator/=(const VectorT &other) {
  values[0] /= other[0];
  values[1] /= other[1];
  values[2] /= other[2];
  return *this;
}
template <typename T>
Vector3DT<T> Vector3DT<T>::operator/(const VectorT &other) const {
  auto p{*this};
  p /= other;
  return p;
}

template <typename T>
Vector3DT<T> &Vector3DT<T>::operator*=(const VectorT &other) {
  values[0] *= other[0];
  values[1] *= other[1];
  values[2] *= other[2];
  return *this;
}
template <typename T>
Vector3DT<T> Vector3DT<T>::operator*(const VectorT &other) const {
  auto p{*this};
  p *= other;
  return p;
}

template <typename T>
Vector3DT<T> Vector3DT<T>::elementMin(const VectorT &first, const VectorT &second) {
  return {std::min(first.x, second.x), std::min(first.y, second.y), std::min(first.z, second.z)};
}
template <typename T>
Vector3DT<T> Vector3DT<T>::elementMax(const VectorT &first, const VectorT &second) {
  return {std::max(first.x, second.x), std::max(first.y, second.y), std::max(first.z, second.z)};
}

template <typename T>
Vector3DT<T> &Vector3DT<T>::elementMin(const VectorT &other) {
  *this = VectorT::elementMin(*this, other);
  return *this;
}
template <typename T>
Vector3DT<T> &Vector3DT<T>::elementMax(const VectorT &other) {
  *this = VectorT::elementMax(*this, other);
  return *this;
}

template <typename T>
const T &Vector3DT<T>::min() const {
  return std::min(values[0], std::min(values[1], values[2]));
}
template <typename T>
const T &Vector3DT<T>::max() const {
  return std::max(values[0], std::max(values[1], values[2]));
}

template <typename T>
T Vector3DT<T>::dotProduct(const Vector3DT<T> &first, const Vector3DT<T> &second) {
  return first.x * second.x + first.y * second.y + first.z * second.z;
}
template <typename T>
Vector3DT<T> Vector3DT<T>::crossProduct(const Vector3DT<T> &first, const Vector3DT<T> &second) {
  return {first.y * second.z - first.z * second.y, first.z * second.x - first.x * second.z,
          first.x * second.y - first.y * second.x};
}

template <typename T>
T Vector3DT<T>::dot(const Vector3DT<T> &other) const {
  return dotProduct(*this, other);
}
template <typename T>
Vector3DT<T> Vector3DT<T>::crossProduct(const Vector3DT<T> &other) const {
  return crossProduct(*this, other);
}

template <typename T>
Vector3DT<T> Vector3DT<T>::sgn(const VectorT &vector) {
  return {impl::sgn(vector[0]), impl::sgn(vector[1]), impl::sgn(vector[2])};
}

template <typename T>
Vector3DT<T> Vector3DT<T>::sgn() const {
  return sgn(*this);
}

template <typename T>
Vector3DT<T> operator/(const T &numerator, const Vector3DT<T> &denominator) {
  return Vector3DT<T>(numerator) / denominator;
}

template <typename T>
Vector3DT<T> operator*(const T &multiplier, const Vector3DT<T> &vector) {
  return Vector3DT<T>(multiplier) * vector;
}

template <typename T>
template <typename TT>
Vector3DT<T>::operator TT() const {
  return static_cast<TT>(values[0]);
}

template <typename T>
template <typename TT>
Vector3DT<T>::operator Vector2DT<TT>() const {
  return Vector2DT<TT>{static_cast<TT>(values[0]), static_cast<TT>(values[1])};
}

template <typename T>
template <typename TT>
Vector3DT<T>::operator Vector3DT<TT>() const {
  return Vector3DT<TT>{static_cast<TT>(values[0]), static_cast<TT>(values[1]), static_cast<TT>(values[2])};
}

namespace impl {

template <typename T>
T uniformZeroToOne() {
  static std::random_device rd;
  static std::mt19937 gen(rd());

  std::uniform_real_distribution<T> dis(0.0, 1.0);

  return dis(gen);
}

template <typename T>
T sgn(const T &val) {
  return (T(0) < val) - (val < T(0));
}

}  // namespace impl

}  // namespace treeutil

namespace std {

// Min and max operations for 2D vector.
template <typename T>
::treeutil::Vector2DT<T> min(const ::treeutil::Vector2DT<T> &first, const ::treeutil::Vector2DT<T> &second) {
  return ::treeutil::Vector2DT<T>::elementMin(first, second);
}
template <typename T>
::treeutil::Vector2DT<T> max(const ::treeutil::Vector2DT<T> &first, const ::treeutil::Vector2DT<T> &second) {
  return ::treeutil::Vector2DT<T>::elementMax(first, second);
}

// Min and max operations for 3D vector.
template <typename T>
::treeutil::Vector3DT<T> min(const ::treeutil::Vector3DT<T> &first, const ::treeutil::Vector3DT<T> &second) {
  return ::treeutil::Vector3DT<T>::elementMin(first, second);
}
template <typename T>
::treeutil::Vector3DT<T> max(const ::treeutil::Vector3DT<T> &first, const ::treeutil::Vector3DT<T> &second) {
  return ::treeutil::Vector3DT<T>::elementMax(first, second);
}

// Numeric limits for vectors.
template <typename T>
class numeric_limits<::treeutil::Vector2DT<T>> {
 public:
  static constexpr auto min() {
    return ::treeutil::Vector2DT<T>::minVector();
  }
  static constexpr auto max() {
    return ::treeutil::Vector2DT<T>::maxVector();
  }

 private:
 protected:
};  // class numeric_limits<Vector2DT<T>

template <typename T>
class numeric_limits<::treeutil::Vector3DT<T>> {
 public:
  static constexpr auto min() {
    return ::treeutil::Vector3DT<T>::minVector();
  }
  static constexpr auto max() {
    return ::treeutil::Vector3DT<T>::maxVector();
  }

 private:
 protected:
};  // class numeric_limits<Vector3DT<T>

}  // namespace std

template <typename T>
std::ostream &operator<<(std::ostream &out, const treeutil::Vector3DT<T> &vec) {
  out << vec.x << ", " << vec.y << ", " << vec.z;
  return out;
}

template <typename T>
std::ostream &operator<<(std::ostream &out, const treeutil::Vector2DT<T> &vec) {
  out << vec.x << ", " << vec.y;
  return out;
}

// Template implementation end

#endif  // TREEIO_VECTORT_H
