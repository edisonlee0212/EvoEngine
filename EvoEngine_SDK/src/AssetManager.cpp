#include "AssetManager.hpp"
#include "EditorLayer.hpp"
#include "FileManager.hpp"
#include "ProjectManager.hpp"
#include "Resources.hpp"
#include <fstream>
#include <yaml-cpp/yaml.h>
using namespace evo_engine;

namespace {

bool OverwriteAssetDefaultsToFile(const std::shared_ptr<IAsset>& asset,
                                  std::filesystem::path* written_path) {
  if (!asset || !asset->SupportsDefaultsOverwrite()) {
    return false;
  }

  const auto defaults_path = asset->ResolveWritableDefaultsPath();
  if (defaults_path.empty()) {
    EVOENGINE_WARNING("Defaults overwrite failed: no writable defaults path for asset type " +
                      asset->GetTypeName());
    return false;
  }

  try {
    std::filesystem::create_directories(defaults_path.parent_path());

    YAML::Emitter out;
    out << YAML::BeginMap;
    asset->Serialize(out);
    out << YAML::EndMap;

    std::ofstream stream(defaults_path.string(), std::ios::out | std::ios::trunc);
    if (!stream.is_open()) {
      EVOENGINE_WARNING("Defaults overwrite failed: cannot open " + defaults_path.string());
      return false;
    }

    stream << out.c_str();
    stream.flush();
    if (!stream.good()) {
      EVOENGINE_WARNING("Defaults overwrite failed while writing " + defaults_path.string());
      return false;
    }

    if (written_path) {
      *written_path = defaults_path;
    }
    return true;
  } catch (const std::exception& e) {
    EVOENGINE_WARNING("Defaults overwrite failed at " + defaults_path.string() + ": " + std::string(e.what()));
    return false;
  }
}

}  // namespace

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

        if (asset->SupportsDefaultsOverwrite()) {
          ImGui::SameLine();
          if (ImGui::Button("Overwrite Descriptor Defaults")) {
            std::filesystem::path written_path;
            if (OverwriteAssetDefaultsToFile(asset, &written_path)) {
              EVOENGINE_LOG("Defaults overwritten for " + asset->GetTypeName() + ": " + written_path.string());
            } else {
              EVOENGINE_WARNING("Failed to overwrite defaults for " + asset->GetTypeName());
            }
          }

          if (ImGui::IsItemHovered()) {
            const auto defaults_path = asset->ResolveWritableDefaultsPath();
            std::string tip =
                "Write this asset's current serialized values as defaults for future assets of the same type.";
            if (!defaults_path.empty()) {
              tip += "\nPath: " + defaults_path.string();
            } else {
              tip += "\nPath resolution failed.";
            }
            ImGui::SetTooltip("%s", tip.c_str());
          }
        }

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
