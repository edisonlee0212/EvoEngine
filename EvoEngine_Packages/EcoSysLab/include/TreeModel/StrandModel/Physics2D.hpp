#pragma once
#include "RigidBody2D.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @brief Handle type for referencing RigidBody2D instances.
 */
typedef int RigidBodyHandle;

/**
 * @brief A 2D physics simulation system for rigid bodies.
 * @tparam T The data type used within the RigidBody2D class for physical calculations.
 */
template <typename T>
class Physics2D {
  /**
   * @brief A list of rigid bodies managed by the physics system.
   */
  std::vector<RigidBody2D<T>> rigid_bodies_2d_{};

  /**
   * @brief Resolves contact between two rigid bodies by adjusting their positions.
   * @param p1_handle Handle to the first rigid body.
   * @param p2_handle Handle to the second rigid body.
   */
  void SolveContact(RigidBodyHandle p1_handle, RigidBodyHandle p2_handle);

  /**
   * @brief The time step used for physics simulations.
   */
  float delta_time_ = 0.002f;

  /**
   * @brief Updates the states of all rigid bodies with an optional modification function.
   * @param modify_rigid_body_func Function used to modify each rigid body before physics updates.
   */
  void Update(const std::function<void(RigidBody2D<T>& rigid_body)>& modify_rigid_body_func);

 public:
  /**
   * @brief Allocates a new rigid body and returns its handle.
   * @return Handle to the newly created rigid body.
   */
  [[nodiscard]] RigidBodyHandle AllocateRigidBody();

  /**
   * @brief Retrieves a reference to a rigid body using its handle.
   * @param handle The handle of the rigid body.
   * @return Reference to the rigid body.
   */
  [[nodiscard]] RigidBody2D<T>& RefRigidBody(RigidBodyHandle handle);

  /**
   * @brief Removes a rigid body from the system.
   * @param handle The handle of the rigid body to remove.
   */
  void RemoveRigidBody(RigidBodyHandle handle);

  /**
   * @brief Shifts all rigid bodies by a specified offset.
   * @param offset The amount to move all rigid bodies.
   */
  void Shift(const glm::vec2& offset);

  /**
   * @brief Provides a read-only view of all rigid bodies managed by the system.
   * @return A constant reference to the list of rigid bodies.
   */
  [[nodiscard]] const std::vector<RigidBody2D<T>>& PeekRigidBodies() const;

  /**
   * @brief Provides a modifiable reference to all rigid bodies managed by the system.
   * @return A reference to the list of rigid bodies.
   */
  [[nodiscard]] std::vector<RigidBody2D<T>>& RefRigidBodies();

  /**
   * @brief Simulates the physics system over a duration.
   * @param time The time duration to simulate.
   * @param modify_rigid_body_func Function called to modify each rigid body before updates.
   */
  void Simulate(float time, const std::function<void(RigidBody2D<T>& rigid_body)>& modify_rigid_body_func);
};

template <typename T>
void Physics2D<T>::SolveContact(RigidBodyHandle p1_handle, RigidBodyHandle p2_handle) {
  if (p1_handle == p2_handle)
    return;
  auto& p1 = rigid_bodies_2d_.at(p1_handle);
  auto& p2 = rigid_bodies_2d_.at(p2_handle);
  const auto difference = p1.position_ - p2.position_;
  const auto distance = glm::length(difference);
  const auto min_distance = p1.thickness_ + p2.thickness_;
  if (distance < min_distance) {
    const auto axis = distance < glm::epsilon<float>() ? glm::vec2(1, 0) : difference / distance;
    const auto delta = min_distance - distance;
    p1.position_ += 0.5f * delta * axis;
    p2.position_ -= 0.5f * delta * axis;
  }
}

template <typename T>
void Physics2D<T>::Update(const std::function<void(RigidBody2D<T>& collision_rigid_body)>& modify_rigid_body_func) {
  Jobs::RunParallelFor(rigid_bodies_2d_.size(), [&](size_t i) {
    modify_rigid_body_func(rigid_bodies_2d_[i]);
  });
  for (size_t i = 0; i < rigid_bodies_2d_.size(); i++) {
    for (size_t j = 0; j < rigid_bodies_2d_.size(); j++) {
      SolveContact(i, j);
    }
  }
  Jobs::RunParallelFor(rigid_bodies_2d_.size(), [&](size_t i) {
    rigid_bodies_2d_[i].Update(delta_time_);
  });
}

template <typename T>
RigidBodyHandle Physics2D<T>::AllocateRigidBody() {
  rigid_bodies_2d_.emplace_back();
  return rigid_bodies_2d_.size() - 1;
}

template <typename T>
RigidBody2D<T>& Physics2D<T>::RefRigidBody(RigidBodyHandle handle) {
  return rigid_bodies_2d_[handle];
}

template <typename T>
void Physics2D<T>::RemoveRigidBody(RigidBodyHandle handle) {
  rigid_bodies_2d_[handle] = rigid_bodies_2d_.back();
  rigid_bodies_2d_.pop_back();
}

template <typename T>
void Physics2D<T>::Shift(const glm::vec2& offset) {
  Jobs::RunParallelFor(rigid_bodies_2d_.size(), [&](size_t i) {
    auto& particle = rigid_bodies_2d_[i];
    particle.SetPosition(particle.position_ + offset);
  });
}

template <typename T>
const std::vector<RigidBody2D<T>>& Physics2D<T>::PeekRigidBodies() const {
  return rigid_bodies_2d_;
}

template <typename T>
std::vector<RigidBody2D<T>>& Physics2D<T>::RefRigidBodies() {
  return rigid_bodies_2d_;
}

template <typename T>
void Physics2D<T>::Simulate(const float time,
                            const std::function<void(RigidBody2D<T>& collision_rigid_body)>& modify_rigid_body_func) {
  const auto count = static_cast<size_t>(glm::round(time / delta_time_));
  for (size_t i{count}; i--;) {
    Update(modify_rigid_body_func);
  }
}

}  // namespace eco_sys_lab_package