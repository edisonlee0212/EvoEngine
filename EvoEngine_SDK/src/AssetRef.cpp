//
// Created by Bosheng Li on 10/12/2021.
//

#include "AssetRef.hpp"

#include "AssetManager.hpp"
#include "FileManager.hpp"

using namespace evo_engine;

void AssetRef::Deserialize(const YAML::Node& in) {
  value_.reset();
  asset_handle_ = Handle(0);
  asset_type_name_.clear();
  if (in["asset_handle_"]) {
    asset_handle_ = Handle(in["asset_handle_"].as<uint64_t>());
  }
  if (in["type_name_"]) {
    asset_type_name_ = in["type_name_"].as<std::string>();
  }
  if (const auto ptr = AssetManager::PeekAssetImpl(asset_handle_)) {
    value_ = ptr;
    asset_type_name_ = ptr->GetTypeName();
  }
}

bool AssetRef::Update() {
  if (!value_) {
    if (asset_handle_.GetValue() == 0) {
      value_.reset();
      return false;
    }
    if (const auto ptr = AssetManager::PeekAssetImpl(asset_handle_)) {
      value_ = ptr;
      asset_type_name_ = ptr->GetTypeName();
      return true;
    }
    if (FileManager::GetFile(asset_handle_)) {
      if (const auto ptr = AssetManager::GetAssetImpl(asset_handle_)) {
        value_ = ptr;
        asset_type_name_ = ptr->GetTypeName();
        return true;
      }
    }
    value_.reset();
    return false;
  }

  asset_handle_ = value_->GetHandle();
  asset_type_name_ = value_->GetTypeName();
  return true;
}

void AssetRef::Clear() {
  value_.reset();
  asset_handle_ = Handle(0);
  asset_type_name_.clear();
}
void AssetRef::Set(const AssetRef& target) {
  value_ = target.value_;
  asset_handle_ = target.GetAssetHandle();
  asset_type_name_ = value_ ? value_->GetTypeName() : target.asset_type_name_;
  if (!value_) {
    Update();
  }
}
