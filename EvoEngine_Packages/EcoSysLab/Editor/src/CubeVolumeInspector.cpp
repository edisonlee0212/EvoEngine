#include "CubeVolume.hpp"
#include "EcoSysLabAuthoringInspectors.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorFileDialogs.hpp"
#include "EditorLayer.hpp"
#include "SDKInspectionAdapters.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;
bool CubeVolumeInspector::Inspect(InspectorContext& context, CubeVolume& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (ImGui::DragFloat3("Min", &target.min_max_bound.min.x, 0.1f))
    changed = true;
  if (ImGui::DragFloat3("Max", &target.min_max_bound.max.x, 0.1f))
    changed = true;

  if (editor_layer->DragAndDropButton<MeshRenderer>(privateComponentRef, "Target MeshRenderer")) {
    if (const auto mmr = privateComponentRef.Get<MeshRenderer>()) {
      target.ApplyMeshBounds(mmr->mesh.Get<Mesh>());
      privateComponentRef.Clear();
      changed = true;
    }
  }
  return changed;
}
