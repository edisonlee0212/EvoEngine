#pragma once
#include "IAsset.hpp"
#include "IHandle.hpp"
namespace evo_engine {
class AssetRef final : public ISerializable {
  friend class Prefab;

  friend class EditorLayer;
  std::shared_ptr<IAsset> value_ = {};
  Handle asset_handle_ = Handle(0);
  std::string asset_type_name_;
  bool Update();

 public:
  void Serialize(YAML::Emitter &out) const override {
    out << YAML::Key << "asset_handle_" << YAML::Value << asset_handle_;
    out << YAML::Key << "asset_type_name_" << YAML::Value << asset_type_name_;
  }
  void Deserialize(const YAML::Node &in) override {
    if (in["asset_handle_"])
      asset_handle_ = Handle(in["asset_handle_"].as<uint64_t>());
    if (in["asset_type_name_"])
      asset_type_name_ = in["asset_type_name_"].as<std::string>();
    Update();
  }
  AssetRef() {
    asset_handle_ = Handle(0);
    asset_type_name_ = "";
    value_.reset();
  }
  ~AssetRef() override {
    asset_handle_ = Handle(0);
    asset_type_name_ = "";
    value_.reset();
  }
  template <typename T = IAsset>
  AssetRef(const std::shared_ptr<T> &other) {
    Set(other);
  }
  template <typename T = IAsset>
  AssetRef &operator=(const std::shared_ptr<T> &other) {
    Set(other);
    return *this;
  }
  template <typename T = IAsset>
  AssetRef &operator=(std::shared_ptr<T> &&other) noexcept {
    Set(other);
    return *this;
  }
  bool operator==(const AssetRef &rhs) const {
    return asset_handle_ == rhs.asset_handle_;
  }
  bool operator!=(const AssetRef &rhs) const {
    return asset_handle_ != rhs.asset_handle_;
  }

  template <typename T = IAsset>
  [[nodiscard]] std::shared_ptr<T> Get() {
    if (Update()) {
      return std::dynamic_pointer_cast<T>(value_);
    }
    return nullptr;
  }
  template <typename T = IAsset>
  void Set(std::shared_ptr<T> target) {
    if (target) {
      auto asset = std::dynamic_pointer_cast<IAsset>(target);
      asset_type_name_ = asset->GetTypeName();
      asset_handle_ = asset->GetHandle();
      value_ = asset;
    } else {
      asset_handle_ = Handle(0);
      value_.reset();
    }
  }
  void Set(const AssetRef &target);
  void Clear();
  [[nodiscard]] Handle GetAssetHandle() const {
    return asset_handle_;
  }
};
}  // namespace evo_engine