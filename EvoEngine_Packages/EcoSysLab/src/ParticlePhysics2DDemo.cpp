#include "ParticlePhysics2DDemo.hpp"
#include "Application.hpp"

#include <Times.hpp>

using namespace eco_sys_lab_package;

void ParticlePhysics2DDemo::FixedUpdate() {
  particle_physics_2d_.Simulate(
      ApplicationContext::Get().GetTimes().TimeStep() / particle_physics_2d_.GetDeltaTime(),
      [&](auto& grid, const bool grid_resized) {
        if (grid_resized || boundaries_updated_)
          grid.ApplyBoundaries(profile_boundaries_);
        boundaries_updated_ = false;
      },
      [&](auto& particle) {
        // Apply constraints
        auto acceleration = glm::vec2(0.f);
        if (!particle_physics_2d_.particle_grid_2d.PeekCells().empty()) {
          const auto& cell = particle_physics_2d_.particle_grid_2d.RefCell(particle.GetPosition());
          if (glm::length(cell.target) > glm::epsilon<float>()) {
            acceleration += gravity_strength * 10.0f * glm::normalize(cell.target);
          }
        }
        particle.SetAcceleration(acceleration);
      });
}
