#include "AssetPreviewRegistry.hpp"
#include "Application.hpp"
#include "PackageManager.hpp"
#include "Serialization.hpp"
#include "Texture2D.hpp"

using namespace evo_engine;

AssetPreviewRegistry &AssetPreviewRegistry::GetInstance() {
  static std::unordered_map<Application *, std::unique_ptr<AssetPreviewRegistry>> registries;
  auto *application = &ApplicationContext::Get();
  auto &registry = registries[application];
  if (!registry) {
    registry = std::make_unique<AssetPreviewRegistry>();
    const auto cleanup = PackageManager::RegisterTypeCleanupCallback([application](const std::string &owner) {
      if (registries.find(application) != registries.end())
        UnregisterAssetPreviewHandlersByOwner(owner);
    });
    static_cast<void>(application->RegisterCleanupFunction([application, cleanup] {
      PackageManager::UnregisterTypeCleanupCallback(cleanup);
      registries.erase(application);
    }));
  }
  return *registry;
}

const AssetPreviewRegistry::AssetPreviewHandlerRecord *AssetPreviewRegistry::FindAssetPreviewRecord(
    const AssetPreviewRegistry &registry, const size_t &type_id) {
  const auto search = registry.asset_preview_handlers_.find(type_id);
  return search == registry.asset_preview_handlers_.end() ? nullptr : &search->second;
}

bool AssetPreviewRegistry::RegisterAssetPreviewHandler(const size_t &type_id,
                                                       AssetPreviewHandler generate_thumbnail_handler,
                                                       const std::string &owner_name, const std::string &type_name,
                                                       const uint32_t version) {
  if (!generate_thumbnail_handler) {
    return false;
  }

  GetInstance().asset_preview_handlers_.insert_or_assign(
      type_id, AssetPreviewHandlerRecord{std::move(generate_thumbnail_handler),
                                         AssetPreviewHandlerInfo{type_id, type_name, owner_name, version}});
  return true;
}

bool AssetPreviewRegistry::UnregisterAssetPreviewHandler(const size_t &type_id) {
  return GetInstance().asset_preview_handlers_.erase(type_id) != 0;
}

size_t AssetPreviewRegistry::UnregisterAssetPreviewHandlersByOwner(const std::string &owner_name) {
  if (owner_name.empty()) {
    return 0;
  }

  auto &registry = GetInstance();
  size_t removed = 0;
  for (auto it = registry.asset_preview_handlers_.begin(); it != registry.asset_preview_handlers_.end();) {
    if (it->second.info.owner_name == owner_name) {
      it = registry.asset_preview_handlers_.erase(it);
      ++removed;
    } else {
      ++it;
    }
  }
  return removed;
}

bool AssetPreviewRegistry::HasAssetPreviewHandler(const size_t &type_id) {
  return FindAssetPreviewRecord(GetInstance(), type_id) != nullptr;
}

bool AssetPreviewRegistry::HasAssetPreviewHandler(const std::string &type_name) {
  const auto &registry = GetInstance();
  if (Serialization::HasSerializableType(type_name) &&
      HasAssetPreviewHandler(Serialization::GetSerializableTypeId(type_name))) {
    return true;
  }
  for (const auto &[type_id, record] : registry.asset_preview_handlers_) {
    if (record.info.type_name == type_name) {
      return true;
    }
  }
  return false;
}

const AssetPreviewRegistry::AssetPreviewHandlerInfo *AssetPreviewRegistry::FindAssetPreviewHandler(
    const size_t &type_id) {
  if (const auto *record = FindAssetPreviewRecord(GetInstance(), type_id)) {
    return &record->info;
  }
  return nullptr;
}

std::shared_ptr<Texture2D> AssetPreviewRegistry::GenerateAssetThumbnail(const std::shared_ptr<IAsset> &asset,
                                                                        const OffscreenPreviewSettings &settings) {
  if (!asset) {
    return {};
  }

  const auto &registry = GetInstance();
  if (const auto *record = FindAssetPreviewRecord(registry, typeid(*asset).hash_code());
      record && record->generate_thumbnail_handler) {
    return record->generate_thumbnail_handler(asset, settings);
  }
  return {};
}
