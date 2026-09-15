//
// Created by Bosheng Li on 10/12/2021.
//

#include "AssetRef.hpp"

#include "AssetManager.hpp"
#include "FileManager.hpp"

using namespace evo_engine;

void AssetRef::Deserialize(const YAML::Node& in) {
  value_.reset();
  if (in["asset_handle_"])
    asset_handle_ = Handle(in["asset_handle_"].as<uint64_t>());
  if (in["type_name_"])
    asset_type_name_ = in["type_name_"].as<std::string>();
  Update();
}

bool AssetRef::Update() {
  if (value_)
    return true;
  if (asset_handle_.GetValue() == 0)
    return false;
  auto ptr = AssetManager::PeekAssetImpl(asset_handle_);
  if (!ptr && FileManager::GetFile(asset_handle_))
    ptr = AssetManager::GetAssetImpl(asset_handle_);
  if (!ptr)
    return false;
  value_ = ptr;
  asset_type_name_ = ptr->GetTypeName();
  return true;
}

std::shared_ptr<IAsset> AssetRef::PeekAsset() const {
  if (value_) {
    return value_;
  }
  return asset_handle_.GetValue() == 0 ? nullptr : AssetManager::PeekAssetImpl(asset_handle_);
}

std::string AssetRef::GetAssetTypeName() const {
  if (value_) {
    return value_->GetTypeName();
  }
  if (!asset_type_name_.empty()) {
    return asset_type_name_;
  }
  const auto file = FileManager::GetFile(asset_handle_);
  return file ? file->GetAssetTypeName() : std::string{};
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
