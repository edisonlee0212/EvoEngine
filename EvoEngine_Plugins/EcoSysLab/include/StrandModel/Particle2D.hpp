#pragma once
#include "ParticleGrid2D.hpp"
#include "Skeleton.hpp"
#include "StrandGroup.hpp"
using namespace evo_engine;
namespace eco_sys_lab {

struct UpdateSettings {
  float dt;
  float damping = 0.0f;
  float max_velocity = 1.0f;
};

template <typename T>
class Particle2D {
  template <typename Pd>
  friend class StrandModelProfileSerializer;

  template <typename Pd>
  friend class StrandModelProfile;
  glm::vec3 color_ = glm::vec3(1.0f);
  glm::vec2 position_ = glm::vec2(0.0f);
  glm::vec2 last_position_ = glm::vec2(0.0f);
  glm::vec2 acceleration_ = glm::vec2(0.0f);
  glm::vec2 delta_position_ = glm::vec2(0.0f);

  ParticleHandle handle_ = -1;

  bool boundary_ = false;
  float distance_to_boundary_ = 0.0f;

  glm::vec2 initial_position_ = glm::vec2(0.0f);

 public:
  SkeletonNodeHandle corresponding_child_node_handle = -1;
  StrandHandle strand_handle = -1;
  StrandSegmentHandle strand_segment_handle = -1;
  bool main_child = false;
  bool base = false;

  void SetInitialPosition(const glm::vec2& initial_position);
  [[nodiscard]] glm::vec2 GetInitialPosition() const;
  [[nodiscard]] float GetDistanceToBoundary() const;
  bool enable = true;
  [[nodiscard]] bool IsBoundary() const;
  T data;
  void Update(const UpdateSettings& update_settings);
  void Stop();
  [[nodiscard]] ParticleHandle GetHandle() const;
  [[nodiscard]] glm::vec3 GetColor() const;
  void SetColor(const glm::vec3& color);

  [[nodiscard]] glm::vec2 GetPosition() const;
  void SetPosition(const glm::vec2& position);

  void Move(const glm::vec2& position);

  [[nodiscard]] glm::vec2 GetVelocity(float dt) const;
  void SetVelocity(const glm::vec2& velocity, float dt);

  [[nodiscard]] glm::vec2 GetAcceleration() const;
  void SetAcceleration(const glm::vec2& acceleration);

  [[nodiscard]] glm::vec2 GetPolarPosition() const;
  [[nodiscard]] glm::vec2 GetInitialPolarPosition() const;
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
float Particle2D<T>::GetDistanceToBoundary() const {
  return distance_to_boundary_;
}

template <typename T>
bool Particle2D<T>::IsBoundary() const {
  return boundary_;
}

template <typename T>
void Particle2D<T>::Update(const UpdateSettings& update_settings) {
  const auto lastV = position_ - last_position_ - update_settings.damping * (position_ - last_position_);
  last_position_ = position_;
  auto targetV = lastV + acceleration_ * update_settings.dt * update_settings.dt;
  const auto speed = glm::length(targetV);
  if (speed > glm::epsilon<float>()) {
    targetV = glm::min(update_settings.max_velocity * update_settings.dt, speed) * glm::normalize(targetV);
    position_ = position_ + targetV;
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
}  // namespace eco_sys_lab
