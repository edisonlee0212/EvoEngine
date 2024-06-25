#pragma once

using namespace evo_engine;
namespace eco_sys_lab {
template <typename T>
class RigidBody2D {
  template <typename PD>
  friend class Physics2D;
  glm::vec4 color_ = glm::vec4(1.0f);
  glm::vec2 position_ = glm::vec2(0.0f);
  glm::vec2 last_position_ = glm::vec2(0.0f);
  glm::vec2 acceleration_ = glm::vec2(0.0f);
  float thickness_ = 1.0f;
  float damping_ = 0.0f;

 public:
  T data;
  void Update(float dt);
  void Stop();

  [[nodiscard]] glm::vec4 GetColor() const;
  void SetColor(const glm::vec4& color);

  [[nodiscard]] glm::vec2 GetPosition() const;
  void SetPosition(const glm::vec2& position);

  void Move(const glm::vec2& position);

  [[nodiscard]] glm::vec2 GetVelocity() const;
  void SetVelocity(const glm::vec2& velocity);

  [[nodiscard]] glm::vec2 GetAcceleration() const;
  void SetAcceleration(const glm::vec2& acceleration);

  [[nodiscard]] float GetDamping() const;
  void SetDamping(float damping);

  [[nodiscard]] float GetRadius() const;
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
}  // namespace eco_sys_lab