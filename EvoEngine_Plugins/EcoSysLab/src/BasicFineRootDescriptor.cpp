#include "BasicFineRootDescriptor.hpp"

using namespace eco_sys_lab_plugin;
void BasicFineRootDescriptor::Serialize(YAML::Emitter& out) const {
  fine_root_material_ref.Save("fine_root_material_ref", out);
}
void BasicFineRootDescriptor::Deserialize(const YAML::Node& in) {
  fine_root_material_ref.Load("fine_root_material_ref", in);
}
bool BasicFineRootDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
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
