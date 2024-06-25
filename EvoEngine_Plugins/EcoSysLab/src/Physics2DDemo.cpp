#include "Physics2DDemo.hpp"

#include <Times.hpp>
using namespace eco_sys_lab;

bool Physics2DDemo::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  static bool enable_render = true;
  if (ImGui::Button("Reset")) {
    physics_2d_ = {};
  }
  ImGui::Checkbox("Enable render", &enable_render);
  ImGui::DragFloat2("World center", &world_center.x, 0.01f);
  ImGui::DragFloat("World radius", &world_radius, 0.01f);
  ImGui::DragFloat("Gravity strength", &gravity_strength, 0.01f);
  ImGui::DragFloat("Friction", &friction, 0.1f);
  static float target_damping = 0.1f;
  ImGui::DragFloat("Target damping", &target_damping, 0.01f);
  if (ImGui::Button("Apply damping")) {
    for (auto& particle : physics_2d_.RefRigidBodies()) {
      particle.SetDamping(target_damping);
    }
  }
  if (enable_render) {
    const std::string tag = "Physics2D Scene [" + std::to_string(GetOwner().GetIndex()) + "]";
    if (ImGui::Begin(tag.c_str())) {
      physics_2d_.OnInspect(
          [&](glm::vec2 position) {
            const auto rigid_body_handle = physics_2d_.AllocateRigidBody();
            auto& particle = physics_2d_.RefRigidBody(rigid_body_handle);
            particle.SetColor(glm::vec4(glm::abs(glm::ballRand(1.0f)), 1.0f));
            particle.SetRadius(glm::linearRand(0.1f, 3.0f));
            particle.SetPosition(position);
          },
          [&](const ImVec2 origin, const float zoom_factor, ImDrawList* draw_list) {
            const auto wc = world_center * zoom_factor;
            draw_list->AddCircle(origin + ImVec2(wc.x, wc.y), world_radius * zoom_factor,
                                 IM_COL32(255, 0, 0, 255));
          });
    }
    ImGui::End();
  }
  return changed;
}

void Physics2DDemo::FixedUpdate() {
  const auto gravity = gravity_direction * gravity_strength;
  physics_2d_.Simulate(Times::FixedDeltaTime(), [&](auto& particle) {
    // Apply gravity
    glm::vec2 acceleration = gravity;
    auto friction = -glm::normalize(particle.GetVelocity()) * this->friction;
    if (!glm::any(glm::isnan(friction))) {
      acceleration += friction;
    }
    { particle.SetAcceleration(acceleration); }
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
