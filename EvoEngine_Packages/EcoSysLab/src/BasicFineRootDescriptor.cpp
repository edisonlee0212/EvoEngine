#include "EcoSysLabSerializationAdapters.hpp"

using namespace eco_sys_lab_package;

void eco_sys_lab_package::SerializeBasicFineRootDescriptor(YAML::Emitter& out, const BasicFineRootDescriptor& target) {
  target.fine_root_material_ref.Save("fine_root_material_ref", out);
}

void eco_sys_lab_package::DeserializeBasicFineRootDescriptor(const YAML::Node& in, BasicFineRootDescriptor& target) {
  target.fine_root_material_ref.Load("fine_root_material_ref", in);
}

bool BasicFineRootDescriptor::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (editor_layer->DragAndDropButton<Material>(fine_root_material_ref, "Fine Root Material"))
    changed = true;
  return changed;
}

void BasicFineRootDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
  if (fine_root_material_ref.Get<Material>())
    list.push_back(fine_root_material_ref);
}

void BasicFineRootDescriptor::PrepareController(FineRootController& foliage_controller) const {
}
