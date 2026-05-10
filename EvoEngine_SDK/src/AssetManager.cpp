#include "AssetManager.hpp"
#include "EditorLayer.hpp"
#include "FileManager.hpp"
#include "ProjectManager.hpp"
#include "Resources.hpp"
#include "UnknownPrivateComponent.hpp"
using namespace evo_engine;

void AssetManager::Initialize() {
  auto& asset_manager = GetInstance();
  // Start a thread for asset importing.
  asset_manager.initialized = true;
}
void AssetManager::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto& asset_manager = GetInstance();
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("View")) {
      ImGui::Checkbox("Assets", &asset_manager.show_asset_inspector_);
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }
  if (asset_manager.show_asset_inspector_) {
    if (ImGui::Begin("Asset Inspector")) {
      if (editor_layer->inspecting_asset) {
        const auto& asset = editor_layer->inspecting_asset;
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0.5f, 0, 1));
        ImGui::Button(asset->GetTitle().c_str());
        ImGui::PopStyleColor(1);
        editor_layer->DraggableAsset(asset);
        ImGui::SameLine();
        ImGui::Text("Type:");
        ImGui::SameLine();
        ImGui::Text(asset->GetTypeName().c_str());
        if (!asset->IsTemporary()) {
          if (ImGui::Button("Save")) {
            asset->Save();
          }
          ImGui::SameLine();
          if (ImGui::Button("Reload")) {
            asset->Load();
          }
        }
        ImGui::SameLine();
        FileUtils::SaveFile(
            "Export...", asset->GetTypeName(), Serialization::PeekAssetExtensions(asset->GetTypeName()),
            [&](const std::filesystem::path& path) {
              asset->Export(path);
            },
            false);
        ImGui::SameLine();
        FileUtils::OpenFile(
            "Import...", asset->GetTypeName(), Serialization::PeekAssetExtensions(asset->GetTypeName()),
            [&](const std::filesystem::path& path) {
              asset->Import(path);
            },
            false);

        ImGui::Separator();
        if (asset->OnInspect(editor_layer))
          asset->SetUnsaved();
      } else {
        ImGui::Text("None");
      }
    }
    ImGui::End();
  }
}

void AssetManager::OnDestroy() {
  auto& asset_manager = GetInstance();
  Clear();
  // Notify thread to end.

  // Clear asset loading queue.

  // Clear threads.

  asset_manager.initialized = false;
}

size_t AssetManager::RestoreUnknownAssets() {
  auto& asset_manager = GetInstance();
  std::vector<std::pair<Handle, std::shared_ptr<UnknownAsset>>> unknown_assets;
  {
    std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
    for (auto& [handle, weak_asset] : asset_manager.asset_registry_.assets_) {
      const auto asset = weak_asset.lock();
      if (const auto unknown_asset = std::dynamic_pointer_cast<UnknownAsset>(asset)) {
        unknown_assets.emplace_back(handle, unknown_asset);
      }
    }
  }

  size_t restored_count = 0;
  for (const auto& [handle, unknown_asset] : unknown_assets) {
    const auto& original_type_name = unknown_asset->GetOriginalTypeName();
    if (original_type_name.empty() || !Serialization::HasSerializableType(original_type_name)) {
      continue;
    }

    size_t hash_code = 0;
    auto restored_asset =
        std::dynamic_pointer_cast<IAsset>(Serialization::ProduceSerializable(original_type_name, hash_code, handle));
    if (!restored_asset) {
      continue;
    }
    restored_asset->self_ = restored_asset;
    restored_asset->OnCreate();
    restored_asset->Deserialize(unknown_asset->GetSerializedNode());
    {
      std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
      asset_manager.asset_registry_.assets_[handle] = restored_asset;
    }
    ++restored_count;
  }
  return restored_count;
}

void AssetManager::Clear() {
  auto& asset_manager = GetInstance();
  std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
  asset_manager.asset_registry_.assets_.clear();
}

std::shared_ptr<IAsset> AssetManager::GetAsset(const std::string& type_name, const Handle& asset_handle) {
  if (type_name.empty()) {
    throw std::invalid_argument("Empty type name!");
  }
  if (type_name == "Binary") {
    throw std::invalid_argument("Type name is Binary!");
  }
  const auto ret_val = GetAssetImpl(asset_handle);
  if (ret_val->GetTypeName() != type_name) {
    throw std::invalid_argument("Type name mismatch!");
  }
  return ret_val;
}

void AssetManager::RemoveAssetImpl(const Handle& asset_handle) {
  auto& asset_manager = GetInstance();
  std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
  if (asset_manager.initialized &&
      asset_manager.asset_registry_.assets_.find(asset_handle) != asset_manager.asset_registry_.assets_.end())
    asset_manager.asset_registry_.assets_.erase(asset_handle);
}

std::shared_ptr<IAsset> AssetManager::GetAssetImpl(const Handle& asset_handle) {
  // return GetAssetFutureImpl(asset_handle).get();
  if (asset_handle == 0) {
    throw std::invalid_argument("Asset handle is 0!");
  }
  auto& asset_manager = GetInstance();
  {
    std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
    if (const auto search = asset_manager.asset_registry_.assets_.find(asset_handle);
        search != asset_manager.asset_registry_.assets_.end() && !search->second.expired()) {
      return search->second.lock();
    }
  }
  if (const std::shared_ptr<File> file = FileManager::GetFile(asset_handle)) {
    size_t hash_code;
    auto ret_val = std::dynamic_pointer_cast<IAsset>(Serialization::ProduceSerializable(
        Serialization::HasSerializableType(file->asset_type_name_) ? file->asset_type_name_ : "UnknownAsset", hash_code,
        asset_handle));
    if (const auto unknown_asset = std::dynamic_pointer_cast<UnknownAsset>(ret_val)) {
      unknown_asset->SetOriginalTypeName(file->asset_type_name_);
    }
    ret_val->file_record_ = file;
    ret_val->self_ = ret_val;
    ret_val->OnCreate();
    if (const auto absolute_path = file->GetAbsolutePath(); std::filesystem::exists(absolute_path)) {
      ret_val->Load();
    } else {
      ret_val->Save();
    }
    file->asset_ = ret_val;
    // file->GetThumbnail();
    {
      std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
      asset_manager.asset_registry_.assets_[asset_handle] = ret_val;
    }
    return ret_val;
  }
  return Resources::TryGetResource<IAsset>(asset_handle);
}

std::future<std::shared_ptr<IAsset>> AssetManager::GetAssetFutureImpl(const Handle& asset_handle) {
  std::packaged_task asset_loading_task([asset_handle] {
    if (asset_handle == 0) {
      throw std::invalid_argument("Asset handle is 0!");
    }
    auto& asset_manager = GetInstance();
    {
      std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
      if (const auto search = asset_manager.asset_registry_.assets_.find(asset_handle);
          search != asset_manager.asset_registry_.assets_.end() && !search->second.expired()) {
        return search->second.lock();
      }
    }
    if (const std::shared_ptr<File> file = FileManager::GetFile(asset_handle)) {
      size_t hash_code;
      auto ret_val = std::dynamic_pointer_cast<IAsset>(
          Serialization::ProduceSerializable(file->asset_type_name_, hash_code, asset_handle));
      ret_val->file_record_ = file;
      ret_val->self_ = ret_val;
      ret_val->OnCreate();
      if (const auto absolute_path = file->GetAbsolutePath(); std::filesystem::exists(absolute_path)) {
        ret_val->Load();
      } else {
        ret_val->Save();
      }
      file->asset_ = ret_val;
      // file->GetThumbnail();
      {
        std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
        asset_manager.asset_registry_.assets_[asset_handle] = ret_val;
      }
      return ret_val;
    }
    return Resources::TryGetResource<IAsset>(asset_handle);
  });
  asset_loading_task();
  return asset_loading_task.get_future();
}

std::shared_ptr<IAsset> AssetManager::CreateTemporaryAsset(const std::string& type_name) {
  return CreateTemporaryAssetImpl(type_name, Handle());
}

std::shared_ptr<IAsset> AssetManager::GetAsset(const Handle& asset_handle) {
  return GetAssetImpl(asset_handle);
}

std::shared_ptr<IAsset> AssetManager::CreateTemporaryAssetImpl(const std::string& type_name,
                                                               const Handle& asset_handle) {
  size_t hash_code;
  auto ret_val =
      std::dynamic_pointer_cast<IAsset>(Serialization::ProduceSerializable(type_name, hash_code, asset_handle));
  if (!ret_val) {
    return nullptr;
  }
  {
    auto& asset_manager = GetInstance();
    std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
    asset_manager.asset_registry_.assets_[ret_val->GetHandle()] = ret_val;
  }
  ret_val->self_ = ret_val;
  ret_val->OnCreate();
  return ret_val;
}
