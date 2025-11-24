#pragma once

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @brief A class representing a 2D rigid body for physics simulations.
 * @tparam T The type of additional data associated with the rigid body.
 */
template <typename T>
class RigidBody2D {
  template <typename PD>
  friend class Physics2D;

  glm::vec4 color_ = glm::vec4(1.0f);          ///< The color of the rigid body.
  glm::vec2 position_ = glm::vec2(0.0f);       ///< The current position of the rigid body.
  glm::vec2 last_position_ = glm::vec2(0.0f);  ///< The last recorded position for velocity calculation.
  glm::vec2 acceleration_ = glm::vec2(0.0f);   ///< The acceleration applied to the rigid body.
  float thickness_ = 1.0f;                     ///< The radius or thickness of the rigid body.
  float damping_ = 0.0f;                       ///< The damping factor to simulate energy loss.

 public:
  T data;  ///< Additional data associated with the rigid body.

  /**
   * @brief Updates the rigid body's position based on its velocity and acceleration.
   * @param dt The time step to update the simulation.
   */
  void Update(float dt);

  /**
   * @brief Stops the rigid body's motion by resetting its velocity.
   */
  void Stop();

  /**
   * @brief Gets the color of the rigid body.
   * @return The color as a glm::vec4.
   */
  [[nodiscard]] glm::vec4 GetColor() const;

  /**
   * @brief Sets the color of the rigid body.
   * @param color The new color to be set.
   */
  void SetColor(const glm::vec4& color);

  /**
   * @brief Gets the position of the rigid body.
   * @return The position as a glm::vec2.
   */
  [[nodiscard]] glm::vec2 GetPosition() const;

  /**
   * @brief Sets the position of the rigid body.
   * @param position The new position to be set.
   */
  void SetPosition(const glm::vec2& position);

  /**
   * @brief Moves the rigid body to a new position without affecting velocity.
   * @param position The new position to move the rigid body.
   */
  void Move(const glm::vec2& position);

  /**
   * @brief Gets the velocity of the rigid body.
   * @return The velocity as a glm::vec2.
   */
  [[nodiscard]] glm::vec2 GetVelocity() const;

  /**
   * @brief Sets the velocity of the rigid body.
   * @param velocity The new velocity to be set.
   */
  void SetVelocity(const glm::vec2& velocity);

  /**
   * @brief Gets the acceleration of the rigid body.
   * @return The acceleration as a glm::vec2.
   */
  [[nodiscard]] glm::vec2 GetAcceleration() const;

  /**
   * @brief Sets the acceleration of the rigid body.
   * @param acceleration The new acceleration to be set.
   */
  void SetAcceleration(const glm::vec2& acceleration);

  /**
   * @brief Gets the damping factor of the rigid body.
   * @return The damping factor as a float.
   */
  [[nodiscard]] float GetDamping() const;

  /**
   * @brief Sets the damping factor of the rigid body.
   * @param damping The new damping factor, clamped between 0.0 and 1.0.
   */
  void SetDamping(float damping);

  /**
   * @brief Gets the radius or thickness of the rigid body.
   * @return The radius as a float.
   */
  [[nodiscard]] float GetRadius() const;

  /**
   * @brief Sets the radius or thickness of the rigid body.
   * @param radius The new radius to be set.
   */
  void SetRadius(float radius);
};

template <typename T>
void RigidBody2D<T>::Update(const float dt) {
  const auto velocity = position_ - last_position_ - damping_ * (position_ - last_position_);
  last_position_ = position_;
  position_ = position_ + velocity + acceleration_ * dt * dt;
  acceleration_ = {};
}

template <typename T>
void RigidBody2D<T>::Stop() {
  last_position_ = position_;
}

template <typename T>
glm::vec4 RigidBody2D<T>::GetColor() const {
  return color_;
}

template <typename T>
void RigidBody2D<T>::SetColor(const glm::vec4& color) {
  color_ = color;
}

template <typename T>
glm::vec2 RigidBody2D<T>::GetPosition() const {
  return position_;
}

template <typename T>
void RigidBody2D<T>::SetPosition(const glm::vec2& position) {
  const auto velocity = position_ - last_position_;
  position_ = position;
  last_position_ = position_ - velocity;
}

template <typename T>
void RigidBody2D<T>::Move(const glm::vec2& position) {
  position_ = position;
}

template <typename T>
glm::vec2 RigidBody2D<T>::GetVelocity() const {
  return position_ - last_position_;
}

template <typename T>
void RigidBody2D<T>::SetVelocity(const glm::vec2& velocity) {
  last_position_ = position_ - velocity;
}

template <typename T>
glm::vec2 RigidBody2D<T>::GetAcceleration() const {
  return acceleration_;
}

template <typename T>
void RigidBody2D<T>::SetAcceleration(const glm::vec2& acceleration) {
  acceleration_ = acceleration;
}

template <typename T>
float RigidBody2D<T>::GetDamping() const {
  return damping_;
}

template <typename T>
void RigidBody2D<T>::SetDamping(const float damping) {
  damping_ = glm::clamp(damping, 0.0f, 1.0f);
}

template <typename T>
float RigidBody2D<T>::GetRadius() const {
  return thickness_;
}

template <typename T>
void RigidBody2D<T>::SetRadius(const float radius) {
  thickness_ = radius;
}
}  // namespace eco_sys_lab_plugin