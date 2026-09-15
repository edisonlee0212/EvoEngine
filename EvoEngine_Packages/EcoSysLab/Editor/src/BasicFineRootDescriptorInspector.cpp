#include "EcoSysLabDescriptorInspectors.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorLayer.hpp"

using namespace eco_sys_lab_package;

bool eco_sys_lab_package::InspectBasicFineRootDescriptor(InspectorContext& context, BasicFineRootDescriptor& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (editor_layer->DragAndDropButton<Material>(target.fine_root_material_ref, "Fine Root Material"))
    changed = true;
  return changed;
}
