#include "DigitalAgricultureInspectorStates.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
using namespace evo_engine;
#include "BtfMaterial.hpp"
#include "EditorLayer.hpp"
#include "Mesh.hpp"
bool digital_agriculture_package::InspectBtfMeshRenderer(InspectorContext& context, BtfMeshRenderer& target) {
  const auto& editor_layer = context.editor_layer;

  bool changed = false;

  if (editor_layer->DragAndDropButton<Mesh>(target.mesh, "Mesh"))
    changed = true;
  if (editor_layer->DragAndDropButton<BtfMaterial>(target.btf, "BtfMaterial"))
    changed = true;

  return changed;
}
