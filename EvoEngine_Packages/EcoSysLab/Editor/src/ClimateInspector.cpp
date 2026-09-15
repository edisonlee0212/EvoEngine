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
  return changed;
}
