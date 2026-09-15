#include "Physics2DDemo.hpp"
#include "Application.hpp"

#include <Times.hpp>
using namespace eco_sys_lab_package;

void Physics2DDemo::FixedUpdate() {
  const auto gravity = gravity_direction * gravity_strength;
  physics_2d_.Simulate(ApplicationContext::Get().GetTimes().FixedDeltaTime(), [&](auto& particle) {
    // Apply gravity
    glm::vec2 acceleration = gravity;
    auto friction = -glm::normalize(particle.GetVelocity()) * this->friction;
    if (!glm::any(glm::isnan(friction))) {
      acceleration += friction;
    }
    particle.SetAcceleration(acceleration);
    // Apply constraints
    {
      const auto to_center = particle.GetPosition() - world_center;
      const auto distance = glm::length(to_center);
      if (distance > world_radius - particle.GetRadius()) {
        const auto n = to_center / distance;
        particle.Move(world_center + n * (world_radius - particle.GetRadius()));
      }
    }
  });
}
