#include "AssetManager.hpp"
#include "Climate.hpp"
#include "EcoSysLabLayer.hpp"
#include "EcoSysLabObjectInspectors.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorLayer.hpp"
#include "Tree.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;

bool ClimateDescriptorInspector::Inspect(InspectorContext& context, ClimateDescriptor& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (ImGui::Button("Instantiate")) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto climate_entity = scene->CreateEntity(target.GetTitle());
    const auto climate = scene->GetOrSetPrivateComponent<Climate>(climate_entity).lock();
    climate->climate_descriptor_ref = target.GetSelf();
  }
  return changed;
}

bool ClimateInspector::Inspect(InspectorContext& context, Climate& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (editor_layer->DragAndDropButton<ClimateDescriptor>(target.climate_descriptor_ref, "ClimateDescriptor", true)) {
    target.InitializeClimateModel();
    changed = true;
  }

  if (target.climate_descriptor_ref.Get<ClimateDescriptor>()) {
  }
  if (ImGui::TreeNode("Air")) {
    changed |= ImGui::DragFloat("Ambient temperature (C)", &target.air_temperature_c, 0.5f, -50.0f, 100.0f);
    changed |= ImGui::SliderFloat("Ambient relative humidity", &target.air_relative_humidity, 0.0f, 1.0f);
    changed |= ImGui::DragFloat("Ambient pressure (Pa)", &target.air_pressure_pa, 100.0f, 10000.0f, 120000.0f);
    changed |= ImGui::DragFloat("Air coupling (1/s)", &target.air_coupling_rate, 0.1f, 0.0f, 100.0f);
    changed |= ImGui::DragFloat("Air friction (1/s)", &target.air_friction, 0.1f, 0.0f, 100.0f);
    float cell_size = target.air_cell_size;
    if (ImGui::InputFloat("Grid cell size (m)", &cell_size, 0.0f, 0.0f, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue)) {
      target.air_cell_size = glm::clamp(cell_size, Climate::minimum_air_cell_size, 0.2f);
      changed = true;
    }
    ImGui::TextDisabled("0.05-0.2 m; press Enter to rebuild");
    ImGui::TreePop();
  }
  return changed;
}
