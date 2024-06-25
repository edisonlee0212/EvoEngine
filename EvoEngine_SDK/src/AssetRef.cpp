//
// Created by Bosheng Li on 10/12/2021.
//

#include "AssetRef.hpp"
#include "ProjectManager.hpp"

using namespace evo_engine;
bool AssetRef::Update() {
  if (asset_handle_.GetValue() == 0) {
    value_.reset();
    return false;
  }

  if (!value_) {
    if (const auto ptr = ProjectManager::GetAsset(asset_handle_)) {
      value_ = ptr;
      asset_type_name_ = ptr->GetTypeName();
      return true;
    }
    Clear();
    return false;
  }

  return true;
}

void AssetRef::Clear() {
  value_.reset();
  asset_handle_ = Handle(0);
}
void AssetRef::Set(const AssetRef &target) {
  asset_handle_ = target.asset_handle_;
  Update();
}