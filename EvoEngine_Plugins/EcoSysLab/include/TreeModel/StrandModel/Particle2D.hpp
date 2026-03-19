#pragma once
#include "ParticleGrid2D.hpp"
#include "Skeleton.hpp"
#include "StrandGroup.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @brief Tri-state status for 2D profile particles.
 *
 * Disabled particles are fully skipped (no physics, no collision).
 * Active particles undergo full physics simulation.
 * Frozen particles act as immovable collision obstacles — they block active particles
 * but do not move themselves. This prevents interior profile drift while keeping
 * the boundary mobile for new strand routing.
 */
enum class ParticleStatus : uint8_t {
  kDisabled = 0,  ///< Fully skipped (no physics, no collision obstacle).
  kActive = 1,    ///< Full physics simulation.
  kFrozen = 2     ///< Immovable collision obstacle — blocks active particles but does not move.
};

/**
 * @struct UpdateSettings
 * @brief Contains settings for updating particle properties.
 */
struct UpdateSettings {
  float dt;                   ///< Time step for the update.
  float damping = 0.0f;       ///< Damping factor to reduce velocity.
  float max_velocity = 1.0f;  ///< Maximum allowable velocity.
};

/**
 * @class Particle2D
 * @brief Represents a 2D particle for physics simulation.
 * @tparam T The data type associated with the particle.
 */
template <typename T>
class Particle2D {
  template <typename Pd>
  friend class StrandModelProfileSerializer;

  template <typename Pd>
  friend class StrandModelProfile;

  glm::vec3 color_ = glm::vec3(1.0f);           ///< Particle color.
  glm::vec2 position_ = glm::vec2(0.0f);        ///< Current position.
  glm::vec2 last_position_ = glm::vec2(0.0f);   ///< Previous position.
  glm::vec2 acceleration_ = glm::vec2(0.0f);    ///< Acceleration applied to the particle.
  glm::vec2 delta_position_ = glm::vec2(0.0f);  ///< Change in position.

  ParticleHandle handle_ = -1;  ///< Handle of the particle.

  bool boundary_ = false;              ///< Flag indicating if the particle is at a boundary.
  float distance_to_boundary_ = 0.0f;  ///< Distance to the boundary.

  bool initial_boundary_ = false;              ///< Initial boundary condition flag.
  float initial_distance_to_boundary_ = 0.0f;  ///< Initial distance to the boundary.

  glm::vec2 initial_position_ = glm::vec2(0.0f);  ///< Initial position of the particle.

 public:
  SkeletonNodeHandle corresponding_child_node_handle = -1;  ///< Corresponding skeleton node handle.
  StrandHandle strand_handle = -1;                          ///< Strand handle associated with this particle.
  StrandSegmentHandle strand_segment_handle = -1;           ///< Strand segment handle.
  bool main_child = false;                                  ///< Flag indicating if it's the main child particle.
  bool base = false;                                        ///< Flag indicating if it's the base particle.

  /**
   * @brief Sets the initial position of the particle.
   * @param initial_position The initial position to set.
   */
  void SetInitialPosition(const glm::vec2& initial_position);

  /**
   * @brief Gets the initial position of the particle.
   * @return The initial position.
   */
  [[nodiscard]] glm::vec2 GetInitialPosition() const;

  /**
   * @brief Gets the initial distance of the particle from the boundary.
   * @return The initial distance to the boundary.
   */
  [[nodiscard]] float GetInitialDistanceToBoundary() const;

  /**
   * @brief Gets the distance of the particle from the boundary.
   * @return The current distance to the boundary.
   */
  [[nodiscard]] float GetDistanceToBoundary() const;

  /**
   * @brief Particle physics status.
   *
   * kDisabled(0) = fully skipped; kActive(1) = full physics; kFrozen(2) = immovable obstacle.
   * Implicit conversion to bool is true for both Active and Frozen, preserving existing
   * boolean tests (e.g. `if (particle.enable)`) which treat both states as "present".
   */
  ParticleStatus status = ParticleStatus::kActive;

  /// @brief Legacy-compatible enable check. Returns true when the particle is Active or Frozen.
  [[nodiscard]] bool IsEnabled() const { return status != ParticleStatus::kDisabled; }

  /**
   * @brief Checks if the particle is a boundary particle.
   * @return True if the particle is at the boundary, false otherwise.
   */
  [[nodiscard]] bool IsBoundary() const;

  /**
   * @brief Checks if the particle was initially at a boundary.
   * @return True if it was initially at the boundary, false otherwise.
   */
  [[nodiscard]] bool IsInitialBoundary() const;

  T data;  ///< Additional data associated with the particle.

  /**
   * @brief Updates the particle properties based on physics simulation.
   * @param update_settings The settings for the update step.
   */
  void Update(const UpdateSettings& update_settings);

  /**
   * @brief Stops the motion of the particle.
   */
  void Stop();

  /**
   * @brief Gets the particle's handle.
   * @return The handle of the particle.
   */
  [[nodiscard]] ParticleHandle GetHandle() const;

  /**
   * @brief Gets the color of the particle.
   * @return The color of the particle.
   */
  [[nodiscard]] glm::vec3 GetColor() const;

  /**
   * @brief Sets the color of the particle.
   * @param color The new color to be assigned.
   */
  void SetColor(const glm::vec3& color);

  /**
   * @brief Gets the current position of the particle.
   * @return The current position.
   */
  [[nodiscard]] glm::vec2 GetPosition() const;

  /**
   * @brief Sets the position of the particle.
   * @param position The new position to set.
   */
  void SetPosition(const glm::vec2& position);

  /**
   * @brief Moves the particle to a new position.
   * @param position The new position to move to.
   */
  void Move(const glm::vec2& position);

  /**
   * @brief Gets the velocity of the particle.
   * @param dt The time step.
   * @return The velocity vector.
   */
  [[nodiscard]] glm::vec2 GetVelocity(float dt) const;

  /**
   * @brief Sets the velocity of the particle.
   * @param velocity The velocity to set.
   * @param dt The time step.
   */
  void SetVelocity(const glm::vec2& velocity, float dt);

  /**
   * @brief Gets the acceleration of the particle.
   * @return The acceleration vector.
   */
  [[nodiscard]] glm::vec2 GetAcceleration() const;

  /**
   * @brief Sets the acceleration of the particle.
   * @param acceleration The acceleration vector to set.
   */
  void SetAcceleration(const glm::vec2& acceleration);

  /**
   * @brief Gets the polar coordinates of the particle.
   * @return The polar coordinates (radius, angle).
   */
  [[nodiscard]] glm::vec2 GetPolarPosition() const;

  /**
   * @brief Gets the initial polar coordinates of the particle.
   * @return The initial polar coordinates.
   */
  [[nodiscard]] glm::vec2 GetInitialPolarPosition() const;

  /**
   * @brief Sets the position of the particle in polar coordinates.
   * @param position The polar coordinates (radius, angle).
   */
  void SetPolarPosition(const glm::vec2& position);
};

template <typename T>
void Particle2D<T>::SetInitialPosition(const glm::vec2& initial_position) {
  initial_position_ = initial_position;
}

template <typename T>
glm::vec2 Particle2D<T>::GetInitialPosition() const {
  return initial_position_;
}

template <typename T>
float Particle2D<T>::GetInitialDistanceToBoundary() const {
  return initial_distance_to_boundary_;
}

template <typename T>
float Particle2D<T>::GetDistanceToBoundary() const {
  return distance_to_boundary_;
}

template <typename T>
bool Particle2D<T>::IsBoundary() const {
  return boundary_;
}

template <typename T>
bool Particle2D<T>::IsInitialBoundary() const {
  return initial_boundary_;
}

template <typename T>
void Particle2D<T>::Update(const UpdateSettings& update_settings) {
  const auto last_v = position_ - last_position_ - update_settings.damping * (position_ - last_position_);
  last_position_ = position_;
  auto target_v = last_v + acceleration_ * update_settings.dt * update_settings.dt;
  const auto speed = glm::length(target_v);
  if (speed > glm::epsilon<float>()) {
    target_v = glm::min(update_settings.max_velocity * update_settings.dt, speed) * glm::normalize(target_v);
    position_ = position_ + target_v;
  }
  acceleration_ = {};
}

template <typename T>
void Particle2D<T>::Stop() {
  last_position_ = position_;
}

template <typename T>
ParticleHandle Particle2D<T>::GetHandle() const {
  return handle_;
}

template <typename T>
glm::vec3 Particle2D<T>::GetColor() const {
  return color_;
}

template <typename T>
void Particle2D<T>::SetColor(const glm::vec3& color) {
  color_ = color;
}

template <typename T>
glm::vec2 Particle2D<T>::GetPosition() const {
  return position_;
}

template <typename T>
void Particle2D<T>::SetPosition(const glm::vec2& position) {
  const auto velocity = position_ - last_position_;
  position_ = position;
  last_position_ = position_ - velocity;
}

template <typename T>
void Particle2D<T>::Move(const glm::vec2& position) {
  position_ = position;
}

template <typename T>
glm::vec2 Particle2D<T>::GetVelocity(const float dt) const {
  return (position_ - last_position_) / dt;
}

template <typename T>
void Particle2D<T>::SetVelocity(const glm::vec2& velocity, const float dt) {
  last_position_ = position_ - velocity * dt;
}

template <typename T>
glm::vec2 Particle2D<T>::GetAcceleration() const {
  return acceleration_;
}

template <typename T>
void Particle2D<T>::SetAcceleration(const glm::vec2& acceleration) {
  acceleration_ = acceleration;
}

template <typename T>
glm::vec2 Particle2D<T>::GetPolarPosition() const {
  const auto r = glm::length(position_);
  if (r <= glm::epsilon<float>()) {
    return {0, 0};
  }
  if (position_.y >= 0)
    return {r, glm::acos(position_.x / r)};
  return {r, -glm::acos(position_.x / r)};
}

template <typename T>
glm::vec2 Particle2D<T>::GetInitialPolarPosition() const {
  const auto r = glm::length(initial_position_);
  if (r <= glm::epsilon<float>()) {
    return {0, 0};
  }
  if (initial_position_.y >= 0)
    return {r, glm::acos(initial_position_.x / r)};
  return {r, -glm::acos(initial_position_.x / r)};
}

template <typename T>
void Particle2D<T>::SetPolarPosition(const glm::vec2& position) {
  SetPosition(glm::vec2(glm::cos(position.y) * position.x, glm::sin(position.y) * position.x));
}
}  // namespace eco_sys_lab_plugin