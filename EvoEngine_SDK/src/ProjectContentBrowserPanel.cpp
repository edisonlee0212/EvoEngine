#include "ProjectContentBrowserPanel.hpp"

#include "Application.hpp"
#include "AssetManager.hpp"
#include "EditorLayer.hpp"
#include "FileManager.hpp"
#include "PathUtils.hpp"
#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "Scene.hpp"
#include "Serialization.hpp"
#include "Utilities.hpp"
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#  include "shellapi.h"
#endif

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <string>
#include <system_error>

using namespace evo_engine;

namespace {
constexpr float kTileLabelHeight = 34.0f;
constexpr float kTileTypeHeight = 18.0f;
constexpr float kTilePadding = 6.0f;
constexpr float kTileRounding = 5.0f;
constexpr const char* kProjectPathPayload = "ProjectPath";

struct BrowserTileInteraction {
  bool clicked = false;
  bool hovered = false;
  bool double_clicked = false;
};

void SetLastItemTooltip(const char* text) {
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("%s", text);
  }
}

ImVec4 BrowserAccentTextColor() {
  return ImGui::GetStyleColorVec4(ImGuiCol_TextLink);
}

ImU32 StyleColor(const ImGuiCol color, const float alpha_multiplier = 1.0f) {
  auto value = ImGui::GetStyleColorVec4(color);
  value.w *= alpha_multiplier;
  return ImGui::GetColorU32(value);
}

std::string TypeLabelForFile(const std::shared_ptr<File>& file) {
  if (file->GetAssetTypeName() != "Binary") {
    return file->GetAssetTypeName();
  }
  return "??? (" + file->GetAssetExtension() + ")";
}

ImVec2 BrowserTileSize(const float thumbnail_size, const float tile_width) {
  return {tile_width, thumbnail_size + kTileTypeHeight + kTileLabelHeight + kTilePadding};
}

bool IsProjectMetadataPath(const std::filesystem::path& path) {
  return path.extension() == ".evefilemeta" || path.extension() == ".evefoldermeta";
}

bool PathsEquivalent(const std::filesystem::path& lhs, const std::filesystem::path& rhs) {
  std::error_code error;
  if (std::filesystem::equivalent(lhs, rhs, error) && !error) {
    return true;
  }
  return path_utils::NormalizePathForContainment(lhs) == path_utils::NormalizePathForContainment(rhs);
}

bool IsActiveProjectFile(const std::filesystem::path& path) {
  const auto project_path = ProjectManager::GetProjectPath();
  std::error_code error;
  return !project_path.empty() && std::filesystem::equivalent(path, project_path, error) && !error;
}

bool IsAssetsFolderPath(const std::filesystem::path& path) {
  const auto assets_path = ProjectManager::GetAssetsFolderPath();
  std::error_code error;
  return !assets_path.empty() && std::filesystem::equivalent(path, assets_path, error) && !error;
}

bool ShouldHideProjectEntry(const std::filesystem::path& path) {
  std::error_code error;
  if (!std::filesystem::exists(path, error)) {
    return true;
  }
  if (IsProjectMetadataPath(path)) {
    return true;
  }
  if (!error && std::filesystem::is_regular_file(path, error) && !error && IsActiveProjectFile(path)) {
    return true;
  }
  return false;
}

std::string ProjectEntryDisplayName(const std::filesystem::path& path, const bool show_extension) {
  if (std::filesystem::is_directory(path)) {
    return path.filename().string();
  }
  return show_extension ? path.filename().string() : path.stem().string();
}

std::string ProjectRootDisplayName() {
  const auto folder_name = ProjectManager::GetProjectFolderPath().filename().string();
  return folder_name.empty() ? ProjectManager::GetProjectName() : folder_name;
}

std::filesystem::path PayloadProjectPath(const ImGuiPayload* payload) {
  if (!payload || payload->DataSize <= 0) {
    return {};
  }
  return std::filesystem::path(
      std::string(static_cast<const char*>(payload->Data), static_cast<size_t>(payload->DataSize) - 1));
}

void SetProjectPathDragDropPayload(const std::filesystem::path& path) {
  const auto path_string = path.string();
  ImGui::SetDragDropPayload(kProjectPathPayload, path_string.c_str(), path_string.size() + 1);
}

void ShowPathInExplorer(const std::filesystem::path& path) {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  const auto path_string = path.string();
  ShellExecuteA(nullptr, "open", path_string.c_str(), nullptr, nullptr, SW_SHOWDEFAULT);
#else
  (void)path;
#endif
}

bool SaveEntityAsPrefab(const std::shared_ptr<Folder>& folder, const Handle& entity_handle) {
  if (!folder) {
    return false;
  }
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return false;
  }
  const auto entity = scene->GetEntity(entity_handle);
  if (!scene->IsEntityValid(entity)) {
    return false;
  }
  auto prefab = AssetManager::CreateTemporaryAsset<Prefab>();
  prefab->FromEntity(entity);
  const auto& prefab_extensions = Serialization::PeekAssetExtensions("Prefab");
  if (prefab_extensions.empty()) {
    return false;
  }
  return ProjectManager::SaveAsset(prefab, folder, scene->GetEntityName(entity), prefab_extensions.front());
}

void DrawBackgroundAssetProgressBar(const AssetManager::AssetLoadSnapshot& snapshot) {
  const auto completed_asset_count = snapshot.completed + snapshot.failed + snapshot.cancelled;
  const auto active_asset_count =
      snapshot.queued + snapshot.loading_cpu + snapshot.waiting_for_finalize + snapshot.gpu_pending;
  const auto total_asset_count = std::max(snapshot.total, completed_asset_count + active_asset_count);
  const float progress = total_asset_count == 0 ? 1.0f : static_cast<float>(completed_asset_count) / total_asset_count;
  const std::string label = std::to_string(static_cast<int>(progress * 100.0f)) + "% " +
                            std::to_string(completed_asset_count) + "/" + std::to_string(total_asset_count);

  const float progress_width = std::min(220.0f, std::max(90.0f, ImGui::GetContentRegionAvail().x));
  ImGui::ProgressBar(progress, ImVec2(progress_width, 0.0f), label.c_str());
  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::TextUnformatted("Background assets");
    ImGui::Text("Progress: %zu/%zu", completed_asset_count, total_asset_count);
    if (!snapshot.active_asset_name.empty()) {
      ImGui::TextWrapped("Asset: %s", snapshot.active_asset_name.c_str());
    }
    if (!snapshot.message.empty()) {
      ImGui::TextWrapped("%s", snapshot.message.c_str());
    }
    if (snapshot.failed != 0 || snapshot.cancelled != 0) {
      ImGui::Text("Failed: %zu  Cancelled: %zu", snapshot.failed, snapshot.cancelled);
    }
    ImGui::EndTooltip();
  }
}

BrowserTileInteraction DrawBrowserTile(const char* id, const std::shared_ptr<Texture2D>& texture,
                                       const std::string& type_label, const std::string& name, const bool selected,
                                       const float thumbnail_size, const float tile_width) {
  ImGui::InvisibleButton(id, BrowserTileSize(thumbnail_size, tile_width));
  BrowserTileInteraction interaction;
  interaction.clicked = ImGui::IsItemClicked(ImGuiMouseButton_Left);
  interaction.hovered = ImGui::IsItemHovered();
  interaction.double_clicked = interaction.hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);

  const ImVec2 min = ImGui::GetItemRectMin();
  const ImVec2 max = ImGui::GetItemRectMax();
  auto* draw_list = ImGui::GetWindowDrawList();
  const auto background_color = selected              ? StyleColor(ImGuiCol_HeaderActive)
                                : interaction.hovered ? StyleColor(ImGuiCol_HeaderHovered, 0.75f)
                                                      : StyleColor(ImGuiCol_FrameBg, 0.45f);
  const auto border_color = selected ? StyleColor(ImGuiCol_TextLink, 0.90f) : StyleColor(ImGuiCol_Border, 0.75f);
  draw_list->AddRectFilled(min, max, background_color, kTileRounding);
  draw_list->AddRect(min, max, border_color, kTileRounding);

  const ImVec2 type_min(min.x, min.y + thumbnail_size);
  const ImVec2 type_max(max.x, type_min.y + kTileTypeHeight);
  draw_list->AddRectFilled(type_min, type_max,
                           selected ? StyleColor(ImGuiCol_TextSelectedBg, 0.85f) : StyleColor(ImGuiCol_Header, 0.70f));

  if (texture) {
    const float image_extent = glm::max(8.0f, thumbnail_size - kTilePadding * 2.0f);
    glm::vec2 image_size(image_extent);
    const glm::vec2 texture_resolution = texture->GetResolution();
    const float max_texture_dimension = glm::max(texture_resolution.x, texture_resolution.y);
    if (max_texture_dimension > 0.0f) {
      image_size = texture_resolution * (image_extent / max_texture_dimension);
    }
    const ImVec2 image_min(min.x + (tile_width - image_size.x) * 0.5f, min.y + (thumbnail_size - image_size.y) * 0.5f);
    const ImVec2 image_max(image_min.x + image_size.x, image_min.y + image_size.y);
    draw_list->AddImage(texture->GetImTextureId(), image_min, image_max, ImVec2(0, 1), ImVec2(1, 0));
  }

  const ImU32 type_color = StyleColor(ImGuiCol_TextDisabled);
  const auto type_size = ImGui::CalcTextSize(type_label.c_str());
  const float available_type_width = glm::max(tile_width - kTilePadding * 2.0f, 1.0f);
  const float type_scale =
      type_size.x > available_type_width ? glm::max(available_type_width / glm::max(type_size.x, 1.0f), 0.7f) : 1.0f;
  const ImVec2 scaled_type_size(type_size.x * type_scale, type_size.y * type_scale);
  const ImVec2 type_text_min(min.x + kTilePadding + glm::max((available_type_width - scaled_type_size.x) * 0.5f, 0.0f),
                             type_min.y + glm::max((kTileTypeHeight - scaled_type_size.y) * 0.5f, 0.0f));
  const ImVec4 type_clip(type_min.x, type_min.y, type_max.x, type_max.y);
  draw_list->AddText(nullptr, ImGui::GetFontSize() * type_scale, type_text_min, type_color, type_label.c_str(), nullptr,
                     0.0f, &type_clip);

  const ImVec2 label_min(min.x + kTilePadding, type_max.y + kTilePadding * 0.5f);
  const ImVec2 label_max(max.x - kTilePadding, max.y - kTilePadding * 0.5f);
  const ImVec4 label_clip(label_min.x, label_min.y, label_max.x, label_max.y);
  draw_list->AddText(nullptr, 0.0f, label_min,
                     selected || interaction.hovered ? StyleColor(ImGuiCol_Text) : StyleColor(ImGuiCol_Text, 0.88f),
                     name.c_str(), nullptr, label_max.x - label_min.x, &label_clip);

  return interaction;
}
}  // namespace

void ProjectContentBrowserPanel::RevealAsset(const Handle& asset_handle) {
  const auto file = FileManager::GetFile(asset_handle);
  if (!file) {
    return;
  }
  const auto folder = file->GetFolder().lock();
  if (!folder) {
    return;
  }

  search_query_.fill('\0');
  NavigateToFolder(folder);
  selected_item_type_ = SelectedItemType::AssetFile;
  selected_item_handle_ = file->GetAssetHandle();
}

void ProjectContentBrowserPanel::RevealFolder(const std::filesystem::path& assets_relative_path) {
  auto folder = ProjectManager::GetInstance().assets_folder_;
  if (!folder) {
    return;
  }
  for (const auto& path_part : assets_relative_path) {
    const auto part = path_part.string();
    if (part.empty() || part == ".") {
      continue;
    }
    std::shared_ptr<Folder> next_folder;
    for (const auto& [_, child] : folder->children_) {
      if (child && child->GetName() == part) {
        next_folder = child;
        break;
      }
    }
    if (!next_folder) {
      return;
    }
    folder = next_folder;
  }

  search_query_.fill('\0');
  NavigateToFolder(folder);
}

void ProjectContentBrowserPanel::SetHierarchyWidth(const float width) {
  hierarchy_width_ = glm::max(width, 32.0f);
}

void ProjectContentBrowserPanel::Draw(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto& project_manager = ProjectManager::GetInstance();
  if (project_manager.show_project_window) {
    if (ImGui::Begin("Project")) {
      if (project_manager.assets_folder_) {
        SyncNavigationHistory();
        const auto current_asset_folder = GetCurrentAssetFolder();
        const auto current_location_path = GetCurrentLocationPath();
        if (ImGui::BeginDragDropTarget()) {
          if (current_asset_folder) {
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kProjectPathPayload)) {
              (void)ProjectManager::CopyProjectItemToAssets(PayloadProjectPath(payload), current_asset_folder);
            }
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
              IM_ASSERT(payload->DataSize == sizeof(Handle));
              Handle handle = *static_cast<Handle*>(payload->Data);
              (void)ProjectManager::MoveAsset(handle, current_asset_folder);
            }
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Folder")) {
              IM_ASSERT(payload->DataSize == sizeof(Handle));
              if (Handle handle = *static_cast<Handle*>(payload->Data); handle.GetValue() != 0) {
                (void)ProjectManager::MoveFolder(handle, current_asset_folder);
              }
            }
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
              IM_ASSERT(payload->DataSize == sizeof(Handle));
              auto entity_handle = *static_cast<Handle*>(payload->Data);
              SaveEntityAsPrefab(current_asset_folder, entity_handle);
            }
          } else if (!current_location_path.empty()) {
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
              IM_ASSERT(payload->DataSize == sizeof(Handle));
              Handle handle = *static_cast<Handle*>(payload->Data);
              (void)ProjectManager::CopyAssetFileToProjectFolder(handle, current_location_path);
            }
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Folder")) {
              IM_ASSERT(payload->DataSize == sizeof(Handle));
              if (Handle handle = *static_cast<Handle*>(payload->Data); handle.GetValue() != 0) {
                (void)ProjectManager::CopyAssetFolderToProjectFolder(handle, current_location_path);
              }
            }
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kProjectPathPayload)) {
              (void)ProjectManager::CopyProjectItemToProjectFolder(PayloadProjectPath(payload), current_location_path);
            }
          }
          ImGui::EndDragDropTarget();
        }
        float cell_size = thumbnail_size_ + thumbnail_padding_;
        static float h = 100;
        auto avail = ImGui::GetContentRegionAvail();
        content_width_ = glm::max(avail.x - hierarchy_width_, cell_size + 8.0f);
        hierarchy_width_ = glm::max(avail.x - content_width_, 32.0f);
        h = avail.y;
        ImGui::Splitter(true, 8.0, hierarchy_width_, content_width_, 32.0f, cell_size + 8.0f, h);
        ImGui::BeginChild("1", ImVec2(hierarchy_width_, h), true);
        ProjectHierarchyHelper(editor_layer, ProjectManager::GetProjectFolderPath());
        hierarchy_reveal_target_.reset();
        project_hierarchy_reveal_target_.reset();
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("2", ImVec2(content_width_ - 5.0f, h), true, ImGuiWindowFlags_AlwaysVerticalScrollbar);
        DrawToolbar();
        ImGui::Separator();
        bool updated = false;
        if (ImGui::BeginPopupContextWindow("NewAssetPopup")) {
          if (ImGui::Button("Show in Explorer...")) {
            ShowPathInExplorer(current_location_path);
          }

          if (current_asset_folder) {
            FileUtils::OpenFile(
                "Import model...", "Model",
                {".eveprefab", ".obj", ".gltf", ".glb", ".blend", ".ply", ".fbx", ".dae", ".x3d"},
                [&](const std::filesystem::path& path) {
                  const auto prefab = AssetManager::CreateTemporaryAsset<Prefab>();
                  if (prefab->Import(path)) {
                    (void)ProjectManager::SaveAsset(prefab, current_asset_folder, path.stem().string(), ".eveprefab",
                                                    false);
                  }
                },
                false);

            if (ImGui::Button("New folder...")) {
              (void)ProjectManager::CreateFolder(current_asset_folder, "New Folder");
            }
            if (ImGui::BeginMenu("New asset...")) {
              for (auto& i : Serialization::GetInstance().asset_extensions_) {
                if (i.first == "IAsset")
                  continue;
                if (ImGui::Button(i.first.c_str())) {
                  (void)ProjectManager::CreateAsset(current_asset_folder, i.first);
                }
              }
              ImGui::EndMenu();
            }
          }
          ImGui::EndPopup();
        }

        if (HasSearchQuery()) {
          DrawSearchResults(editor_layer, updated);
        } else {
          float panel_width = ImGui::GetContentRegionAvail().x;
          int column_count = glm::max(1, static_cast<int>(panel_width / (cell_size + thumbnail_padding_)));
          ImGui::Columns(column_count, nullptr, false);
          if (current_asset_folder) {
            DrawAssetFolderContents(editor_layer, current_asset_folder, cell_size, updated);
          } else {
            DrawProjectFolderContents(cell_size, updated);
          }

          ImGui::Columns(1);
        }
        ImGui::EndChild();
      } else {
        ImGui::Text("No project loaded!");
      }
    }
    ImGui::End();
  }
}

void ProjectContentBrowserPanel::DrawToolbar() {
  const auto draw_icon_button = [](const char* id, const std::shared_ptr<Texture2D>& icon, const char* fallback,
                                   const char* tooltip, const bool enabled) {
    if (!enabled) {
      ImGui::BeginDisabled();
    }
    bool clicked = false;
    if (icon) {
      clicked = ImGui::ImageButton(id, icon->GetImTextureId(), {16, 16}, {0, 1}, {1, 0});
    } else {
      clicked = ImGui::Button(fallback, {24, 24});
    }
    SetLastItemTooltip(tooltip);
    if (!enabled) {
      ImGui::EndDisabled();
    }
    return enabled && clicked;
  };

  if (draw_icon_button("BackHistory", EditorLayer::FindIcon("LeftButton"), "<##BackHistory", "Back",
                       CanNavigateBack())) {
    NavigateHistory(-1);
  }
  ImGui::SameLine();
  if (draw_icon_button("ForwardHistory", EditorLayer::FindIcon("RightButton"), ">##ForwardHistory", "Forward",
                       CanNavigateForward())) {
    NavigateHistory(1);
  }
  ImGui::SameLine();
  const auto current_asset_folder = GetCurrentAssetFolder();
  const bool can_go_up = current_location_.type == BrowserLocationType::AssetFolder ? current_asset_folder != nullptr
                                                                                    : !IsCurrentProjectRoot();
  if (draw_icon_button("ParentFolder", EditorLayer::FindIcon("BackButton"), "^##ParentFolder", "Parent folder",
                       can_go_up)) {
    if (current_location_.type == BrowserLocationType::AssetFolder) {
      if (current_asset_folder && current_asset_folder->GetHandle().GetValue() == 0) {
        NavigateToProjectRoot();
      } else if (current_asset_folder) {
        NavigateToFolder(current_asset_folder->parent_.lock());
      }
    } else {
      const auto parent_path = GetCurrentLocationPath().parent_path();
      if (ProjectManager::GetProjectFolderPath() == parent_path) {
        NavigateToProjectRoot();
      } else {
        NavigateToProjectFolder(parent_path);
      }
    }
  }
  ImGui::SameLine();
  if (draw_icon_button("RefreshProjectBrowser", EditorLayer::FindIcon("RefreshButton"), "R##RefreshProjectBrowser",
                       "Refresh", true)) {
    ProjectManager::DispatchScanAssetsTask();
  }

  ImGui::SameLine();
  ImGui::SetNextItemWidth(220.0f);
  ImGui::InputTextWithHint("##ProjectBrowserSearch", "Search project", search_query_.data(), search_query_.size());
  SetLastItemTooltip(recursive_search_ ? "Searches nested folders and files" : "Searches current folder only");

  ImGui::SameLine();
  if (draw_icon_button("ProjectBrowserSettings", EditorLayer::FindIcon("SceneSettings"), "...##ProjectBrowserSettings",
                       "Browser settings", true)) {
    ImGui::OpenPopup("ProjectBrowserSettings");
  }
  if (ImGui::BeginPopup("ProjectBrowserSettings")) {
    ImGui::Checkbox("Show extensions", &show_extension_);
    ImGui::Checkbox("Recursive search", &recursive_search_);
    ImGui::SetNextItemWidth(160.0f);
    ImGui::SliderFloat("Thumbnail size", &thumbnail_size_, 10.0f, 150.0f, "%.0f");
    ImGui::SetNextItemWidth(160.0f);
    ImGui::SliderFloat("Padding", &thumbnail_padding_, 4.0f, 24.0f, "%.0f");
    ImGui::EndPopup();
  }

  const auto asset_load_snapshot = AssetManager::GetAssetLoadSnapshot();
  if (ProjectManager::GetInstance().start_scene_ && asset_load_snapshot.Active()) {
    ImGui::SameLine();
    DrawBackgroundAssetProgressBar(asset_load_snapshot);
  }

  DrawBreadcrumbs();
}

void ProjectContentBrowserPanel::DrawBreadcrumbs() {
  const auto project_folder = ProjectManager::GetProjectFolderPath();
  const std::string project_label = ProjectRootDisplayName() + "##ProjectRootBreadcrumb";
  if (ImGui::SmallButton(project_label.c_str())) {
    NavigateToProjectRoot();
  }
  if (current_location_.type == BrowserLocationType::ProjectRoot) {
    return;
  }

  ImGui::SameLine();
  ImGui::TextUnformatted(">");
  ImGui::SameLine();
  if (current_location_.type == BrowserLocationType::AssetFolder) {
    const auto root_folder = ProjectManager::GetAssetsFolder();
    if (ImGui::SmallButton("Assets##AssetsBreadcrumb")) {
      NavigateToFolder(root_folder);
    }
    std::vector<std::shared_ptr<Folder>> folders;
    for (auto folder = GetCurrentAssetFolder(); folder && root_folder && folder.get() != root_folder.get();
         folder = folder->parent_.lock()) {
      folders.emplace_back(folder);
    }
    std::reverse(folders.begin(), folders.end());
    for (const auto& folder : folders) {
      ImGui::SameLine();
      ImGui::TextUnformatted(">");
      ImGui::SameLine();
      const std::string label = folder->GetName() + "##Breadcrumb" + std::to_string(folder->GetHandle().GetValue());
      if (ImGui::SmallButton(label.c_str())) {
        NavigateToFolder(folder);
      }
    }
    return;
  }

  auto walker = project_folder;
  const auto relative_path = std::filesystem::relative(GetCurrentLocationPath(), project_folder);
  for (const auto& part : relative_path) {
    if (part.empty() || part == ".") {
      continue;
    }
    walker /= part;
    const std::string label = part.string() + "##ProjectBreadcrumb" + walker.string();
    if (ImGui::SmallButton(label.c_str())) {
      NavigateToProjectFolder(walker);
    }
    ImGui::SameLine();
    ImGui::TextUnformatted(">");
    ImGui::SameLine();
  }
}

bool ProjectContentBrowserPanel::DrawAssetContextMenu(const std::shared_ptr<File>& file, const std::string& tag) {
  if (!ImGui::BeginPopupContextItem(tag.c_str())) {
    return false;
  }
  if (ImGui::Button("Duplicate")) {
    file->GetFolder().lock()->Duplicate(file->GetAssetHandle());
  }
  if (file->GetAssetTypeName() == "GaussianSplat") {
    if (auto asset = AssetManager::GetAssetImpl(file->asset_handle_)) {
      if (ImGui::BeginMenu(("Export" + tag).c_str())) {
        FileUtils::SaveFile(
            "Export PLY...", "GaussianSplat PLY", {".ply"},
            [asset](const std::filesystem::path& path) {
              return asset->Export(path);
            },
            false);
        FileUtils::SaveFile(
            "Export SPLAT...", "GaussianSplat SPLAT", {".splat"},
            [asset](const std::filesystem::path& path) {
              return asset->Export(path);
            },
            false);
        FileUtils::SaveFile(
            "Export KSPLAT...", "GaussianSplat KSPLAT", {".ksplat"},
            [asset](const std::filesystem::path& path) {
              return asset->Export(path);
            },
            false);
        ImGui::EndMenu();
      }
    }
  }
  if (file->GetAssetTypeName() != "Binary" && ImGui::BeginMenu(("Rename" + tag).c_str())) {
    static char new_name[256] = {};
    ImGui::InputText(("New name" + tag).c_str(), new_name, 256);
    if (ImGui::Button(("Confirm" + tag).c_str())) {
      auto asset = AssetManager::GetAssetImpl(file->asset_handle_);
      asset->SetPathAndSave(asset->GetAssetsFolderRelativePath().replace_filename(
          std::string(new_name) + asset->GetFileRecord().lock()->GetAssetExtension()));
      memset(new_name, 0, 256);
    }
    ImGui::EndMenu();
  }
  if (ImGui::Button(("Delete" + tag).c_str())) {
    (void)ProjectManager::DeleteAsset(file->GetAssetHandle());
    ImGui::EndPopup();
    return true;
  }
  ImGui::EndPopup();
  return false;
}

void ProjectContentBrowserPanel::DrawAssetFolderContents(const std::shared_ptr<EditorLayer>& editor_layer,
                                                         const std::shared_ptr<Folder>& folder, const float cell_size,
                                                         bool& updated) {
  if (!folder) {
    return;
  }
  if (!updated) {
    for (auto& i : folder->children_) {
      const std::string icon_tag = "##FolderTile" + std::to_string(i.second->handle_);
      const auto thumbnail_tex = EditorLayer::FindIcon("Folder");
      if (!thumbnail_tex) {
        continue;
      }
      const bool item_selected = selected_item_type_ == SelectedItemType::AssetFolder &&
                                 selected_item_handle_.GetValue() == i.second->GetHandle().GetValue();
      const auto interaction = DrawBrowserTile(icon_tag.c_str(), thumbnail_tex, "Folder", i.second->name_,
                                               item_selected, thumbnail_size_, cell_size);
      if (interaction.clicked) {
        selected_item_type_ = SelectedItemType::AssetFolder;
        selected_item_handle_ = i.second->GetHandle();
        selected_project_path_.clear();
      }

      if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
        ImGui::SetDragDropPayload("Folder", &i.second->handle_, sizeof(Handle));
        ImGui::TextColored(BrowserAccentTextColor(), i.second->name_.c_str());
        ImGui::EndDragDropSource();
      }
      if (i.second->GetHandle() != 0) {
        if (ImGui::BeginPopupContextItem(icon_tag.c_str())) {
          if (ImGui::BeginMenu(("Rename" + icon_tag).c_str())) {
            static char new_name[256] = {};
            ImGui::InputText(("New name" + icon_tag).c_str(), new_name, 256);
            if (ImGui::Button(("Confirm" + icon_tag).c_str())) {
              i.second->Rename(std::string(new_name));
              memset(new_name, 0, 256);
              ImGui::CloseCurrentPopup();
            }
            ImGui::EndMenu();
          }
          if (ImGui::Button(("Remove" + icon_tag).c_str())) {
            updated = ProjectManager::DeleteFolder(i.second->handle_);
            ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
            break;
          }
          ImGui::EndPopup();
        }
      }
      if (ImGui::BeginDragDropTarget()) {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kProjectPathPayload)) {
          (void)ProjectManager::CopyProjectItemToAssets(PayloadProjectPath(payload), i.second);
        }
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Folder")) {
          IM_ASSERT(payload->DataSize == sizeof(Handle));
          if (Handle payload_n = *static_cast<Handle*>(payload->Data); payload_n.GetValue() != 0) {
            (void)ProjectManager::MoveFolder(payload_n, i.second);
          }
        }
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
          IM_ASSERT(payload->DataSize == sizeof(Handle));
          Handle payload_n = *static_cast<Handle*>(payload->Data);
          (void)ProjectManager::MoveAsset(payload_n, i.second);
        }
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Binary")) {
          IM_ASSERT(payload->DataSize == sizeof(Handle));
          Handle payload_n = *static_cast<Handle*>(payload->Data);
          (void)ProjectManager::MoveAsset(payload_n, i.second);
        }
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
          IM_ASSERT(payload->DataSize == sizeof(Handle));
          auto entity_handle = *static_cast<Handle*>(payload->Data);
          SaveEntityAsPrefab(i.second, entity_handle);
        }
        ImGui::EndDragDropTarget();
      }
      if (interaction.double_clicked) {
        NavigateToFolder(i.second);
        updated = true;
        break;
      }
      ImGui::NextColumn();
    }
  }
  if (!updated) {
    for (auto& i : folder->files) {
      auto file_name = i.second->GetAssetsFolderRelativePath().filename();
      if (file_name.string() == ".eveproj" || file_name.extension().string() == ".eveproj")
        continue;
      const std::string icon_tag = "##AssetTile" + std::to_string(i.first.GetValue());
      const bool item_selected =
          selected_item_type_ == SelectedItemType::AssetFile && selected_item_handle_.GetValue() == i.first.GetValue();

      const bool tile_visible = ImGui::IsRectVisible(BrowserTileSize(thumbnail_size_, cell_size));
      const auto thumbnail_tex = i.second->GetThumbnail(tile_visible);
      const auto display_name = show_extension_ ? file_name.string() : file_name.stem().string();
      const auto interaction = DrawBrowserTile(icon_tag.c_str(), thumbnail_tex, TypeLabelForFile(i.second),
                                               display_name, item_selected, thumbnail_size_, cell_size);
      if (interaction.clicked) {
        selected_item_type_ = SelectedItemType::AssetFile;
        selected_item_handle_ = i.first;
        selected_project_path_.clear();
      }
      if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
        ImGui::SetDragDropPayload("Asset", &i.first, sizeof(Handle));
        ImGui::TextColored(BrowserAccentTextColor(), i.second->GetAssetFileName().c_str());
        ImGui::EndDragDropSource();
      }

      if (DrawAssetContextMenu(i.second, icon_tag)) {
        break;
      }

      if (interaction.double_clicked && i.second->GetAssetTypeName() != "Binary") {
        if (auto asset = AssetManager::GetAssetImpl(i.second->asset_handle_))
          editor_layer->OpenAssetInspector(asset);
      }
      ImGui::NextColumn();
    }
  }
}

void ProjectContentBrowserPanel::DrawProjectFolderContents(const float cell_size, bool& updated) {
  const auto folder_path = GetCurrentLocationPath();
  if (folder_path.empty()) {
    return;
  }
  const auto folder_icon = EditorLayer::FindIcon("Folder");
  const auto binary_icon = EditorLayer::FindIcon("Binary");
  if (IsCurrentProjectRoot() && ProjectManager::GetAssetsFolder()) {
    const auto asset_root = ProjectManager::GetAssetsFolder();
    const bool item_selected =
        selected_item_type_ == SelectedItemType::AssetFolder && selected_item_handle_.GetValue() == 0;
    const auto interaction = DrawBrowserTile("##ProjectAssetsTile", folder_icon, "Assets", "Assets", item_selected,
                                             thumbnail_size_, cell_size);
    if (interaction.clicked) {
      selected_item_type_ = SelectedItemType::AssetFolder;
      selected_item_handle_ = 0;
      selected_project_path_.clear();
    }
    if (ImGui::BeginDragDropTarget()) {
      if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kProjectPathPayload)) {
        (void)ProjectManager::CopyProjectItemToAssets(PayloadProjectPath(payload), asset_root);
      }
      if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
        IM_ASSERT(payload->DataSize == sizeof(Handle));
        Handle payload_n = *static_cast<Handle*>(payload->Data);
        (void)ProjectManager::MoveAsset(payload_n, asset_root);
      }
      if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Folder")) {
        IM_ASSERT(payload->DataSize == sizeof(Handle));
        if (Handle payload_n = *static_cast<Handle*>(payload->Data); payload_n.GetValue() != 0) {
          (void)ProjectManager::MoveFolder(payload_n, asset_root);
        }
      }
      if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
        IM_ASSERT(payload->DataSize == sizeof(Handle));
        SaveEntityAsPrefab(asset_root, *static_cast<Handle*>(payload->Data));
      }
      ImGui::EndDragDropTarget();
    }
    if (interaction.double_clicked) {
      NavigateToFolder(asset_root);
      updated = true;
    }
    ImGui::NextColumn();
  }

  std::vector<std::filesystem::path> entries;
  std::error_code error;
  for (const auto& entry : std::filesystem::directory_iterator(folder_path, error)) {
    if (!error && !ShouldHideProjectEntry(entry.path()) &&
        !(IsCurrentProjectRoot() && IsAssetsFolderPath(entry.path()))) {
      entries.emplace_back(entry.path());
    }
  }
  std::sort(entries.begin(), entries.end(), [](const std::filesystem::path& lhs, const std::filesystem::path& rhs) {
    const auto lhs_is_directory = std::filesystem::is_directory(lhs);
    const auto rhs_is_directory = std::filesystem::is_directory(rhs);
    if (lhs_is_directory != rhs_is_directory) {
      return lhs_is_directory;
    }
    return lhs.filename().string() < rhs.filename().string();
  });

  for (const auto& entry_path : entries) {
    const bool is_directory = std::filesystem::is_directory(entry_path);
    const std::string icon_tag = "##ProjectItemTile" + entry_path.string();
    const bool item_selected =
        selected_project_path_ == entry_path &&
        selected_item_type_ == (is_directory ? SelectedItemType::ProjectFolder : SelectedItemType::ProjectFile);
    const auto interaction = DrawBrowserTile(icon_tag.c_str(), is_directory ? folder_icon : binary_icon,
                                             is_directory ? "Folder" : entry_path.extension().string(),
                                             ProjectEntryDisplayName(entry_path, show_extension_), item_selected,
                                             thumbnail_size_, cell_size);
    if (interaction.clicked) {
      selected_item_type_ = is_directory ? SelectedItemType::ProjectFolder : SelectedItemType::ProjectFile;
      selected_item_handle_ = 0;
      selected_project_path_ = entry_path;
    }
    if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
      SetProjectPathDragDropPayload(entry_path);
      ImGui::TextColored(BrowserAccentTextColor(), entry_path.filename().string().c_str());
      ImGui::EndDragDropSource();
    }
    if (is_directory && ImGui::BeginDragDropTarget()) {
      if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
        IM_ASSERT(payload->DataSize == sizeof(Handle));
        (void)ProjectManager::CopyAssetFileToProjectFolder(*static_cast<Handle*>(payload->Data), entry_path);
      }
      if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Folder")) {
        IM_ASSERT(payload->DataSize == sizeof(Handle));
        if (Handle payload_n = *static_cast<Handle*>(payload->Data); payload_n.GetValue() != 0) {
          (void)ProjectManager::CopyAssetFolderToProjectFolder(payload_n, entry_path);
        }
      }
      if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kProjectPathPayload)) {
        (void)ProjectManager::CopyProjectItemToProjectFolder(PayloadProjectPath(payload), entry_path);
      }
      ImGui::EndDragDropTarget();
    }
    if (ImGui::BeginPopupContextItem(icon_tag.c_str())) {
      if (ImGui::BeginMenu(("Rename" + icon_tag).c_str())) {
        static char new_name[256] = {};
        ImGui::InputText(("New name" + icon_tag).c_str(), new_name, 256);
        if (ImGui::Button(("Confirm" + icon_tag).c_str())) {
          if (ProjectManager::RenameProjectItem(entry_path, std::string(new_name))) {
            memset(new_name, 0, 256);
            ImGui::CloseCurrentPopup();
          }
        }
        ImGui::EndMenu();
      }
      if (ImGui::Button(("Delete" + icon_tag).c_str())) {
        updated = ProjectManager::DeleteProjectItem(entry_path);
        ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
        break;
      }
      ImGui::EndPopup();
    }
    if (is_directory && interaction.double_clicked) {
      NavigateToProjectFolder(entry_path);
      updated = true;
      break;
    }
    ImGui::NextColumn();
  }
}

void ProjectContentBrowserPanel::DrawSearchResults(const std::shared_ptr<EditorLayer>& editor_layer, bool& updated) {
  const std::string query = search_query_.data();
  if (query.empty()) {
    return;
  }

  bool has_results = false;
  auto visit_folder = [&](auto&& self, const std::shared_ptr<Folder>& folder) -> void {
    if (!folder || updated) {
      return;
    }

    for (const auto& [_, child] : folder->children_) {
      if (FolderMatchesSearch(child, query)) {
        has_results = true;
        const auto label = "[Folder] " + child->GetAssetsRelativePath().string() + "##SearchFolder" +
                           std::to_string(child->GetHandle().GetValue());
        if (ImGui::Selectable(label.c_str())) {
          selected_item_type_ = SelectedItemType::AssetFolder;
          selected_item_handle_ = child->GetHandle();
          selected_project_path_.clear();
          NavigateToFolder(child);
        }
        if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
          NavigateToFolder(child);
        }
      }
      if (recursive_search_) {
        self(self, child);
      }
    }

    for (const auto& [handle, file] : folder->files) {
      if (!FileMatchesSearch(file, query)) {
        continue;
      }
      has_results = true;
      auto file_name = file->GetAssetsFolderRelativePath();
      const auto label = "[" + file->GetAssetTypeName() + "] " +
                         (show_extension_ ? file_name.string() : file_name.replace_extension("").string()) +
                         "##SearchFile" + std::to_string(handle.GetValue());
      if (ImGui::Selectable(label.c_str())) {
        selected_item_type_ = SelectedItemType::AssetFile;
        selected_item_handle_ = handle;
        selected_project_path_.clear();
      }
      if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0) && file->GetAssetTypeName() != "Binary") {
        if (auto asset = AssetManager::GetAssetImpl(file->asset_handle_)) {
          editor_layer->OpenAssetInspector(asset);
        }
      }
      if (ImGui::BeginPopupContextItem(("SearchFileContext" + std::to_string(handle.GetValue())).c_str())) {
        if (ImGui::Button("Duplicate")) {
          file->GetFolder().lock()->Duplicate(file->GetAssetHandle());
          updated = true;
          ImGui::CloseCurrentPopup();
          ImGui::EndPopup();
          return;
        }
        if (ImGui::Button("Delete")) {
          (void)ProjectManager::DeleteAsset(handle);
          updated = true;
          ImGui::CloseCurrentPopup();
          ImGui::EndPopup();
          return;
        }
        ImGui::EndPopup();
      }
    }
  };

  if (const auto current_asset_folder = GetCurrentAssetFolder()) {
    visit_folder(visit_folder, current_asset_folder);
  } else {
    auto visit_project_folder = [&](auto&& self, const std::filesystem::path& folder) -> void {
      std::error_code error;
      for (const auto& entry : std::filesystem::directory_iterator(folder, error)) {
        if (error || ShouldHideProjectEntry(entry.path()) ||
            (folder == ProjectManager::GetProjectFolderPath() && IsAssetsFolderPath(entry.path()))) {
          continue;
        }
        const bool is_directory = entry.is_directory();
        if (ProjectPathMatchesSearch(entry.path(), query)) {
          has_results = true;
          const auto label = std::string(is_directory ? "[Folder] " : "[File] ") + entry.path().filename().string() +
                             "##ProjectSearch" + entry.path().string();
          if (ImGui::Selectable(label.c_str())) {
            selected_item_type_ = is_directory ? SelectedItemType::ProjectFolder : SelectedItemType::ProjectFile;
            selected_item_handle_ = 0;
            selected_project_path_ = entry.path();
            if (is_directory) {
              NavigateToProjectFolder(entry.path());
            }
          }
          if (is_directory && ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
            NavigateToProjectFolder(entry.path());
          }
        }
        if (recursive_search_ && is_directory) {
          self(self, entry.path());
        }
      }
    };
    visit_project_folder(visit_project_folder, GetCurrentLocationPath());
  }
  if (!has_results) {
    ImGui::TextDisabled("No matching project items.");
  }
}

void ProjectContentBrowserPanel::NavigateToFolder(const std::shared_ptr<Folder>& folder, const bool add_history) {
  if (!folder) {
    return;
  }

  auto& project_manager = ProjectManager::GetInstance();
  project_manager.current_focused_folder_ = folder;
  current_location_.type = BrowserLocationType::AssetFolder;
  current_location_.asset_folder_handle = folder->GetHandle();
  current_location_.project_folder_path.clear();
  RequestHierarchyReveal(folder);
  selected_item_type_ = SelectedItemType::None;
  selected_item_handle_ = 0;
  selected_project_path_.clear();
  if (!add_history) {
    return;
  }

  SyncNavigationHistory();
}

void ProjectContentBrowserPanel::NavigateToProjectRoot(const bool add_history) {
  current_location_ = {};
  selected_item_type_ = SelectedItemType::None;
  selected_item_handle_ = 0;
  selected_project_path_.clear();
  project_hierarchy_reveal_target_ = ProjectManager::GetProjectFolderPath();
  if (add_history) {
    SyncNavigationHistory();
  }
}

void ProjectContentBrowserPanel::NavigateToProjectFolder(const std::filesystem::path& folder, const bool add_history) {
  if (folder.empty() || !std::filesystem::is_directory(folder) || !ProjectManager::IsInProjectFolder(folder) ||
      ProjectManager::IsInAssetsFolder(folder)) {
    return;
  }
  current_location_.type = BrowserLocationType::ProjectFolder;
  current_location_.asset_folder_handle = 0;
  current_location_.project_folder_path = folder;
  selected_item_type_ = SelectedItemType::None;
  selected_item_handle_ = 0;
  selected_project_path_.clear();
  RequestHierarchyReveal(folder);
  if (add_history) {
    SyncNavigationHistory();
  }
}

void ProjectContentBrowserPanel::RequestHierarchyReveal(const std::shared_ptr<Folder>& folder) {
  if (folder) {
    hierarchy_reveal_target_ = folder->GetHandle();
  }
}

void ProjectContentBrowserPanel::RequestHierarchyReveal(const std::filesystem::path& folder) {
  if (!folder.empty()) {
    project_hierarchy_reveal_target_ = folder;
  }
}

void ProjectContentBrowserPanel::NavigateHistory(const int offset) {
  const auto target_index = static_cast<int64_t>(folder_history_index_) + offset;
  if (target_index < 0 || target_index >= static_cast<int64_t>(folder_history_.size())) {
    return;
  }

  folder_history_index_ = static_cast<size_t>(target_index);
  const auto target = folder_history_[folder_history_index_];
  if (target.type == BrowserLocationType::AssetFolder) {
    NavigateToFolder(FileManager::GetFolder(target.asset_folder_handle), false);
  } else if (target.type == BrowserLocationType::ProjectFolder) {
    NavigateToProjectFolder(target.project_folder_path, false);
  } else {
    NavigateToProjectRoot(false);
  }
}

void ProjectContentBrowserPanel::SyncNavigationHistory() {
  if (folder_history_.empty()) {
    folder_history_.emplace_back(current_location_);
    folder_history_index_ = 0;
    return;
  }
  if (folder_history_index_ >= folder_history_.size()) {
    folder_history_index_ = folder_history_.size() - 1;
  }
  if (LocationsEqual(folder_history_[folder_history_index_], current_location_)) {
    return;
  }
  if (folder_history_index_ + 1 < folder_history_.size()) {
    folder_history_.resize(folder_history_index_ + 1);
  }
  folder_history_.emplace_back(current_location_);
  folder_history_index_ = folder_history_.size() - 1;
}

bool ProjectContentBrowserPanel::CanNavigateBack() const {
  return folder_history_index_ > 0 && folder_history_index_ < folder_history_.size();
}

bool ProjectContentBrowserPanel::CanNavigateForward() const {
  return folder_history_index_ + 1 < folder_history_.size();
}

bool ProjectContentBrowserPanel::HasSearchQuery() const {
  return search_query_[0] != '\0';
}

std::filesystem::path ProjectContentBrowserPanel::GetCurrentLocationPath() const {
  switch (current_location_.type) {
    case BrowserLocationType::ProjectRoot:
      return ProjectManager::GetProjectFolderPath();
    case BrowserLocationType::AssetFolder:
      if (const auto folder = FileManager::GetFolder(current_location_.asset_folder_handle)) {
        return folder->GetAbsolutePath();
      }
      return ProjectManager::GetAssetsFolderPath();
    case BrowserLocationType::ProjectFolder:
      return current_location_.project_folder_path;
  }
  return {};
}

std::shared_ptr<Folder> ProjectContentBrowserPanel::GetCurrentAssetFolder() const {
  if (current_location_.type != BrowserLocationType::AssetFolder) {
    return {};
  }
  return FileManager::GetFolder(current_location_.asset_folder_handle);
}

bool ProjectContentBrowserPanel::IsCurrentProjectRoot() const {
  return current_location_.type == BrowserLocationType::ProjectRoot ||
         (current_location_.type == BrowserLocationType::ProjectFolder &&
          current_location_.project_folder_path == ProjectManager::GetProjectFolderPath());
}

bool ProjectContentBrowserPanel::FileMatchesSearch(const std::shared_ptr<File>& file, const std::string& query) {
  return file && (TextContainsCaseInsensitive(file->GetAssetFileName(), query) ||
                  TextContainsCaseInsensitive(file->GetAssetExtension(), query) ||
                  TextContainsCaseInsensitive(file->GetAssetTypeName(), query) ||
                  TextContainsCaseInsensitive(file->GetAssetsFolderRelativePath().string(), query));
}

bool ProjectContentBrowserPanel::FolderMatchesSearch(const std::shared_ptr<Folder>& folder, const std::string& query) {
  return folder && (TextContainsCaseInsensitive(folder->GetName(), query) ||
                    TextContainsCaseInsensitive(folder->GetAssetsRelativePath().string(), query));
}

bool ProjectContentBrowserPanel::ProjectPathMatchesSearch(const std::filesystem::path& path, const std::string& query) {
  return TextContainsCaseInsensitive(path.filename().string(), query) ||
         TextContainsCaseInsensitive(path.string(), query);
}

bool ProjectContentBrowserPanel::TextContainsCaseInsensitive(const std::string& text, const std::string& query) {
  if (query.empty()) {
    return true;
  }
  return std::search(text.begin(), text.end(), query.begin(), query.end(),
                     [](const char text_char, const char query_char) {
                       return std::tolower(static_cast<unsigned char>(text_char)) ==
                              std::tolower(static_cast<unsigned char>(query_char));
                     }) != text.end();
}

bool ProjectContentBrowserPanel::LocationsEqual(const BrowserLocation& lhs, const BrowserLocation& rhs) {
  if (lhs.type != rhs.type) {
    return false;
  }
  if (lhs.type == BrowserLocationType::AssetFolder) {
    return lhs.asset_folder_handle.GetValue() == rhs.asset_folder_handle.GetValue();
  }
  if (lhs.type == BrowserLocationType::ProjectFolder) {
    return lhs.project_folder_path == rhs.project_folder_path;
  }
  return true;
}

void ProjectContentBrowserPanel::FolderHierarchyHelper(const std::shared_ptr<EditorLayer>& editor_layer,
                                                       const std::shared_ptr<Folder>& folder,
                                                       const std::shared_ptr<Folder>& reveal_folder) {
  if (!folder) {
    return;
  }
  const bool reveal_path = reveal_folder && reveal_folder->IsSelfOrAncestor(folder->GetHandle());
  if (reveal_path) {
    ImGui::SetNextItemOpen(true, ImGuiCond_Always);
  }
  const bool folder_selected = current_location_.type == BrowserLocationType::AssetFolder &&
                               current_location_.asset_folder_handle.GetValue() == folder->GetHandle().GetValue();
  const bool opened = ImGui::TreeNodeEx(
      folder->name_.c_str(),
      ImGuiTreeNodeFlags_OpenOnArrow | (folder_selected ? ImGuiTreeNodeFlags_Selected : ImGuiTreeNodeFlags_None));
  if (folder == reveal_folder) {
    ImGui::SetScrollHereY(0.35f);
  }
  if (ImGui::BeginDragDropTarget()) {
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kProjectPathPayload)) {
      (void)ProjectManager::CopyProjectItemToAssets(PayloadProjectPath(payload), folder);
    }
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Folder")) {
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      Handle payload_n = *static_cast<Handle*>(payload->Data);
      if (payload_n.GetValue() != 0) {
        (void)ProjectManager::MoveFolder(payload_n, folder);
      }
    }
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      Handle payload_n = *static_cast<Handle*>(payload->Data);
      (void)ProjectManager::MoveAsset(payload_n, folder);
    }
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Binary")) {
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      Handle payload_n = *static_cast<Handle*>(payload->Data);
      (void)ProjectManager::MoveAsset(payload_n, folder);
    }
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      auto entity_handle = *static_cast<Handle*>(payload->Data);
      SaveEntityAsPrefab(folder, entity_handle);
    }
    ImGui::EndDragDropTarget();
  }
  if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
    NavigateToFolder(folder);
  }
  const std::string tag = "##Folder" + std::to_string(folder->handle_);
  if (folder->GetHandle().GetValue() != 0 && ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
    ImGui::SetDragDropPayload("Folder", &folder->handle_, sizeof(Handle));
    ImGui::TextColored(BrowserAccentTextColor(), folder->name_.c_str());
    ImGui::EndDragDropSource();
  }
  if (folder->GetHandle() != 0) {
    if (ImGui::BeginPopupContextItem(tag.c_str())) {
      if (ImGui::BeginMenu(("Rename" + tag).c_str())) {
        static char new_name[256] = {};
        ImGui::InputText(("New name" + tag).c_str(), new_name, 256);
        if (ImGui::Button(("Confirm" + tag).c_str())) {
          folder->Rename(std::string(new_name));
          memset(new_name, 0, 256);
          ImGui::CloseCurrentPopup();
        }
        ImGui::EndMenu();
      }
      if (ImGui::Button(("Remove" + tag).c_str())) {
        (void)ProjectManager::DeleteFolder(folder->handle_);
        ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
        return;
      }
      ImGui::EndPopup();
    }
  }
  if (opened) {
    for (const auto& i : folder->children_) {
      FolderHierarchyHelper(editor_layer, i.second, reveal_folder);
    }
    for (const auto& i : folder->files) {
      const std::string tag = "##HierarchyAsset" + std::to_string(i.first.GetValue());
      if (ImGui::TreeNodeEx((i.second->GetAssetFileName() + i.second->GetAssetExtension()).c_str(),
                            ImGuiTreeNodeFlags_Bullet)) {
        ImGui::TreePop();
      }
      if (ImGui::IsItemHovered()) {
        if (ImGui::IsMouseDoubleClicked(0) && i.second->GetAssetTypeName() != "Binary") {
          // If it's an asset then inspect.
          if (auto asset = AssetManager::GetAssetImpl(i.second->asset_handle_))
            editor_layer->OpenAssetInspector(asset);
        }
      }
      if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
        ImGui::SetDragDropPayload("Asset", &i.first, sizeof(Handle));
        ImGui::TextColored(BrowserAccentTextColor(), i.second->GetAssetFileName().c_str());
        ImGui::EndDragDropSource();
      }
      if (DrawAssetContextMenu(i.second, tag)) {
        return;
      }
    }
    ImGui::TreePop();
  }
}

void ProjectContentBrowserPanel::ProjectHierarchyHelper(const std::shared_ptr<EditorLayer>& editor_layer,
                                                        const std::filesystem::path& folder) {
  std::error_code error;
  if (folder.empty() || !std::filesystem::is_directory(folder, error) || error) {
    return;
  }

  const auto project_folder = ProjectManager::GetProjectFolderPath();
  const bool is_project_root = PathsEquivalent(folder, project_folder);
  const bool folder_selected = is_project_root ? current_location_.type == BrowserLocationType::ProjectRoot
                                               : (current_location_.type == BrowserLocationType::ProjectFolder &&
                                                  PathsEquivalent(current_location_.project_folder_path, folder));
  const bool reveal_path =
      project_hierarchy_reveal_target_ && (PathsEquivalent(*project_hierarchy_reveal_target_, folder) ||
                                           path_utils::IsSameOrChildPath(*project_hierarchy_reveal_target_, folder));
  if (is_project_root || reveal_path) {
    ImGui::SetNextItemOpen(true, ImGuiCond_Always);
  }

  const auto display_name = is_project_root ? ProjectRootDisplayName() : folder.filename().string();
  const std::string label = display_name + "##ProjectFolder" + folder.string();
  const bool opened =
      ImGui::TreeNodeEx(label.c_str(), ImGuiTreeNodeFlags_OpenOnArrow |
                                           (folder_selected ? ImGuiTreeNodeFlags_Selected : ImGuiTreeNodeFlags_None));
  if (project_hierarchy_reveal_target_ && PathsEquivalent(*project_hierarchy_reveal_target_, folder)) {
    ImGui::SetScrollHereY(0.35f);
  }
  if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
    if (is_project_root) {
      NavigateToProjectRoot();
    } else {
      NavigateToProjectFolder(folder);
    }
  }

  if (!is_project_root && ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
    SetProjectPathDragDropPayload(folder);
    ImGui::TextColored(BrowserAccentTextColor(), folder.filename().string().c_str());
    ImGui::EndDragDropSource();
  }
  if (ImGui::BeginDragDropTarget()) {
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      (void)ProjectManager::CopyAssetFileToProjectFolder(*static_cast<Handle*>(payload->Data), folder);
    }
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Folder")) {
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      if (Handle payload_n = *static_cast<Handle*>(payload->Data); payload_n.GetValue() != 0) {
        (void)ProjectManager::CopyAssetFolderToProjectFolder(payload_n, folder);
      }
    }
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kProjectPathPayload)) {
      (void)ProjectManager::CopyProjectItemToProjectFolder(PayloadProjectPath(payload), folder);
    }
    ImGui::EndDragDropTarget();
  }
  if (!is_project_root) {
    const std::string tag = "##ProjectFolderContext" + folder.string();
    if (ImGui::BeginPopupContextItem(tag.c_str())) {
      if (ImGui::BeginMenu(("Rename" + tag).c_str())) {
        static char new_name[256] = {};
        ImGui::InputText(("New name" + tag).c_str(), new_name, 256);
        if (ImGui::Button(("Confirm" + tag).c_str())) {
          if (ProjectManager::RenameProjectItem(folder, std::string(new_name))) {
            memset(new_name, 0, 256);
            ImGui::CloseCurrentPopup();
          }
        }
        ImGui::EndMenu();
      }
      if (ImGui::Button(("Delete" + tag).c_str())) {
        (void)ProjectManager::DeleteProjectItem(folder);
        ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
        return;
      }
      ImGui::EndPopup();
    }
  }

  if (!opened) {
    return;
  }

  if (is_project_root) {
    const auto reveal_folder = hierarchy_reveal_target_ ? FileManager::GetFolder(*hierarchy_reveal_target_) : nullptr;
    FolderHierarchyHelper(editor_layer, ProjectManager::GetAssetsFolder(), reveal_folder);
  }

  std::vector<std::filesystem::path> child_folders;
  for (const auto& entry : std::filesystem::directory_iterator(folder, error)) {
    if (error) {
      break;
    }
    if (ShouldHideProjectEntry(entry.path()) || (is_project_root && IsAssetsFolderPath(entry.path()))) {
      continue;
    }
    std::error_code entry_error;
    if (entry.is_directory(entry_error) && !entry_error) {
      child_folders.emplace_back(entry.path());
    }
  }
  std::sort(child_folders.begin(), child_folders.end(),
            [](const std::filesystem::path& lhs, const std::filesystem::path& rhs) {
              return lhs.filename().string() < rhs.filename().string();
            });
  for (const auto& child_folder : child_folders) {
    ProjectHierarchyHelper(editor_layer, child_folder);
  }
  ImGui::TreePop();
}
