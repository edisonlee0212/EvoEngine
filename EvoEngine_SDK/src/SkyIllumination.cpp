#include "SkyIllumination.hpp"

#include "EditorLayer.hpp"

using namespace evo_engine;

bool SkyIllumination::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNodeEx("Atmosphere Settings")) {
    if (ImGui::DragFloat("Earth Radius (km)", &atmosphere.earth_radius, 1.0f, 0.0f,
                         atmosphere.atmosphere_radius - 1.0f)) {
      atmosphere.earth_radius = glm::clamp(atmosphere.earth_radius, 1.0f, atmosphere.atmosphere_radius - 1.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Atmosphere Radius (km)", &atmosphere.atmosphere_radius, 1.0f, atmosphere.earth_radius + 1.0f,
                         100000.0f)) {
      atmosphere.atmosphere_radius =
          glm::clamp(atmosphere.atmosphere_radius, atmosphere.earth_radius + 1.0f, 100000.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Rayleigh scale height (m)", &atmosphere.hr, 1.0f, 0.0f, 100000.0f)) {
      atmosphere.hr = glm::clamp(atmosphere.hr, 0.0f, 10000.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Mie scale height (m)", &atmosphere.hm, 1.0f, 0.0f, 100000.0f)) {
      atmosphere.hm = glm::clamp(atmosphere.hm, 0.0f, 10000.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Mie scattering mean cosine", &atmosphere.g, 0.001f, 0.0f, 0.999f, "%.4f")) {
      atmosphere.g = glm::clamp(atmosphere.g, 0.0f, 0.999f);
      changed = true;
    }
    if (ImGui::DragInt("Samples", &atmosphere.num_samples, 1, 128)) {
      atmosphere.num_samples = glm::clamp(atmosphere.num_samples, 1, 128);
      changed = true;
    }
    if (ImGui::DragInt("Samples light", &atmosphere.num_samples_light, 1, 128)) {
      atmosphere.num_samples_light = glm::clamp(atmosphere.num_samples_light, 1, 128);
      changed = true;
    }

    if (ImGui::DragFloat("Intensity", &atmosphere.intensity, 0.1f, 0.0f, 10.f)) {
      changed = true;
    }
    ImGui::TreePop();
  }
  if (ImGui::DragFloat("Gamma", &gamma, 0.01f, 0.0f, 5.f)) {
    changed = true;
  }
  static glm::vec3 angles = glm::vec3(90, 0, 0);
  if (ImGui::DragFloat3("Sun angle", &angles.x, 1.0f)) {
    sun_direction = glm::quat(glm::radians(angles)) * glm::vec3(0, 0, -1);
    changed = true;
  }

  if (ImGui::ColorEdit3("Ground color", &ground_color.x)) {
    changed = true;
  }

  if (ImGui::DragFloat("Ground transmittance", &ground_transmittance, 0.01f, 0.0f, 1.0f)) {
    ground_transmittance = glm::clamp(ground_transmittance, 0.0f, 1.f);
    changed = true;
  }
  return changed;
}