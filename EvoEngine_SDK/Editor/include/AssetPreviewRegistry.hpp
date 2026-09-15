#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include "EvoEngineAPI.hpp"
#include "EvoEngineEditorAPI.hpp"
#include "IAsset.hpp"

namespace evo_engine {
class Texture2D;
struct OffscreenPreviewSettings;
class EVOENGINE_EDITOR_API AssetPreviewRegistry final {
 public:
  static AssetPreviewRegistry& GetInstance();
  using AssetPreviewHandler =
      std::function<std::shared_ptr<Texture2D>(const std::shared_ptr<IAsset>&, const OffscreenPreviewSettings&)>;
  struct AssetPreviewHandlerInfo {
    size_t type_id = 0;
    std::string type_name;
    std::string owner_name;
    uint32_t version = 0;
  };
  template <typename T>
  static bool RegisterAssetPreviewHandler(
      std::function<std::shared_ptr<Texture2D>(const std::shared_ptr<T>&, const OffscreenPreviewSettings&)>
          generate_thumbnail_handler,
      const std::string& owner_name = {}, const std::string& type_name = {}, uint32_t version = 0);

  template <typename T>
  static bool UnregisterAssetPreviewHandler();

  template <typename T>
  [[nodiscard]] static bool HasAssetPreviewHandler();

  static bool RegisterAssetPreviewHandler(const size_t& type_id, AssetPreviewHandler generate_thumbnail_handler,
                                          const std::string& owner_name = {}, const std::string& type_name = {},
                                          uint32_t version = 0);
  static bool UnregisterAssetPreviewHandler(const size_t& type_id);
  static size_t UnregisterAssetPreviewHandlersByOwner(const std::string& owner_name);

  [[nodiscard]] static bool HasAssetPreviewHandler(const size_t& type_id);
  [[nodiscard]] static bool HasAssetPreviewHandler(const std::string& type_name);
  [[nodiscard]] static const AssetPreviewHandlerInfo* FindAssetPreviewHandler(const size_t& type_id);

  [[nodiscard]] static std::shared_ptr<Texture2D> GenerateAssetThumbnail(const std::shared_ptr<IAsset>& asset,
                                                                         const OffscreenPreviewSettings& settings);

 private:
  struct AssetPreviewHandlerRecord {
    AssetPreviewHandler generate_thumbnail_handler;
    AssetPreviewHandlerInfo info;
  };

  std::unordered_map<size_t, AssetPreviewHandlerRecord> asset_preview_handlers_{};

  [[nodiscard]] static const AssetPreviewHandlerRecord* FindAssetPreviewRecord(const AssetPreviewRegistry& registry,
                                                                               const size_t& type_id);
};

template <typename T>
bool AssetPreviewRegistry::RegisterAssetPreviewHandler(
    std::function<std::shared_ptr<Texture2D>(const std::shared_ptr<T>&, const OffscreenPreviewSettings&)>
        generate_thumbnail_handler,
    const std::string& owner_name, const std::string& type_name, const uint32_t version) {
  AssetPreviewHandler erased_generate_thumbnail_handler;
  if (generate_thumbnail_handler) {
    erased_generate_thumbnail_handler = [handler = std::move(generate_thumbnail_handler)](
                                            const std::shared_ptr<IAsset>& asset,
                                            const OffscreenPreviewSettings& settings) {
      const auto typed_asset = std::dynamic_pointer_cast<T>(asset);
      return typed_asset ? handler(typed_asset, settings) : nullptr;
    };
  }

  return RegisterAssetPreviewHandler(typeid(T).hash_code(), std::move(erased_generate_thumbnail_handler), owner_name,
                                     type_name.empty() ? typeid(T).name() : type_name, version);
}

template <typename T>
bool AssetPreviewRegistry::UnregisterAssetPreviewHandler() {
  return UnregisterAssetPreviewHandler(typeid(T).hash_code());
}

template <typename T>
bool AssetPreviewRegistry::HasAssetPreviewHandler() {
  return HasAssetPreviewHandler(typeid(T).hash_code());
}

}  // namespace evo_engine
