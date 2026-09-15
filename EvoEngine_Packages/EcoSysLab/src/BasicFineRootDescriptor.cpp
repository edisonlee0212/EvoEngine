#include "EcoSysLabSerializationAdapters.hpp"

using namespace eco_sys_lab_package;

void eco_sys_lab_package::SerializeBasicFineRootDescriptor(YAML::Emitter& out, const BasicFineRootDescriptor& target) {
  target.fine_root_material_ref.Save("fine_root_material_ref", out);
}

void eco_sys_lab_package::DeserializeBasicFineRootDescriptor(const YAML::Node& in, BasicFineRootDescriptor& target) {
  target.fine_root_material_ref.Load("fine_root_material_ref", in);
}

void BasicFineRootDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
  if (fine_root_material_ref.Get<Material>())
    list.push_back(fine_root_material_ref);
}

void BasicFineRootDescriptor::PrepareController(FineRootController& foliage_controller) const {
}
