
#pragma once
#include "IAsset.hpp"
#include "IHandle.hpp"

namespace evo_engine {

/**
 * @class AssetRef
 * @brief A class representing a reference to an asset in the system.
 */
class AssetRef final : public ISerializable {
  friend class Prefab;
  friend class EditorLayer;

  /**
   * @brief Pointer to the underlying asset.
   */
  std::shared_ptr<IAsset> value_ = {};

  /**
   * @brief Handle to the asset.
   */
  Handle asset_handle_ = Handle(0);

  /**
   * @brief Name of the type of the asset.
   */
  std::string asset_type_name_;

  /**
   * @brief Updates the internal state of the asset reference.
   * @return `true` if the update was successful, `false` otherwise.
   */
  bool Update();

 public:
  /**
   * @brief Serializes the `AssetRef` object to a YAML emitter.
   * @param out The YAML emitter to serialize to.
   */
  void Serialize(YAML::Emitter &out) const override {
    out << YAML::Key << "asset_handle_" << YAML::Value << asset_handle_;
    out << YAML::Key << "type_name_" << YAML::Value << asset_type_name_;
  }

  /**
   * @brief Deserializes the `AssetRef` object from a YAML node.
   * @param in The YAML node to deserialize from.
   */
  void Deserialize(const YAML::Node &in) override {
    if (in["asset_handle_"])
      asset_handle_ = Handle(in["asset_handle_"].as<uint64_t>());
    if (in["type_name_"])
      asset_type_name_ = in["type_name_"].as<std::string>();
    Update();
  }

  /**
   * @brief Default constructor for `AssetRef`.
   */
  AssetRef() {
    asset_handle_ = Handle(0);
    asset_type_name_ = "";
    value_.reset();
  }

  /**
   * @brief Destructor for `AssetRef`.
   */
  ~AssetRef() override {
    asset_handle_ = Handle(0);
    asset_type_name_ = "";
    value_.reset();
  }

  /**
   * @brief Constructor to initialize an `AssetRef` with an asset.
   * @tparam T The asset type, defaults to `IAsset`.
   * @param other Shared pointer to the asset to initialize with.
   */
  template <typename T = IAsset>
  AssetRef(const std::shared_ptr<T> &other) {
    Set(other);
  }

  /**
   * @brief Assignment operator for setting this `AssetRef`.
   * @tparam T The asset type, defaults to `IAsset`.
   * @param other Shared pointer to the asset to set.
   * @return Reference to this `AssetRef` object.
   */
  template <typename T = IAsset>
  AssetRef &operator=(const std::shared_ptr<T> &other) {
    Set(other);
    return *this;
  }

  /**
   * @brief Move assignment operator for setting this `AssetRef`.
   * @tparam T The asset type, defaults to `IAsset`.
   * @param other Rvalue reference to the shared pointer of the asset to set.
   * @return Reference to this `AssetRef` object.
   */
  template <typename T = IAsset>
  AssetRef &operator=(std::shared_ptr<T> &&other) noexcept {
    Set(other);
    return *this;
  }

  /**
   * @brief Equality operator to compare two `AssetRef` objects.
   * @param rhs The `AssetRef` object to compare with.
   * @return `true` if the objects are equal, `false` otherwise.
   */
  bool operator==(const AssetRef &rhs) const {
    return asset_handle_ == rhs.asset_handle_;
  }

  /**
   * @brief Inequality operator to compare two `AssetRef` objects.
   * @param rhs The `AssetRef` object to compare with.
   * @return `true` if the objects are not equal, `false` otherwise.
   */
  bool operator!=(const AssetRef &rhs) const {
    return asset_handle_ != rhs.asset_handle_;
  }

  /**
   * @brief Retrieves the underlying asset.
   * @tparam T The asset type, defaults to `IAsset`.
   * @return Shared pointer to the asset if available, otherwise `nullptr`.
   */
  template <typename T = IAsset>
  [[nodiscard]] std::shared_ptr<T> Get() {
    if (Update()) {
      return std::dynamic_pointer_cast<T>(value_);
    }
    return nullptr;
  }

  /**
   * @brief Retrieves the underlying asset from a const asset reference.
   *
   * AssetRef maintains a lazy cache in Update(); allow const call sites
   * to use the same lookup path.
   *
   * @tparam T The asset type, defaults to `IAsset`.
   * @return Shared pointer to the asset if available, otherwise `nullptr`.
   */
  template <typename T = IAsset>
  [[nodiscard]] std::shared_ptr<T> Get() const {
    return const_cast<AssetRef *>(this)->Get<T>();
  }

  /**
   * @brief Assigns an asset to this `AssetRef`.
   * @tparam T The asset type, defaults to `IAsset`.
   * @param target Shared pointer to the asset to assign.
   */
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

  /**
   * @brief Sets this asset reference to the target asset reference.
   * @param target The target asset reference.
   */
  void Set(const AssetRef &target);

  /**
   * @brief Clears this asset reference, resetting all internal values.
   */
  void Clear();

  /**
   * @brief Retrieves the handle of the referenced asset.
   * @return A `Handle` object representing the asset's handle.
   */
  [[nodiscard]] Handle GetAssetHandle() const {
    return asset_handle_;
  }
};

}  // namespace evo_engine
