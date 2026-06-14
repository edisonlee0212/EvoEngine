#include "ProjectContentBrowserPanel.hpp"

#include "Application.hpp"
#include "AssetManager.hpp"
#include "EditorLayer.hpp"
#include "FileManager.hpp"
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

using namespace evo_engine;

namespace {
constexpr float kTileLabelHeight = 34.0f;
constexpr float kTileTypeHeight = 18.0f;
constexpr float kTilePadding = 6.0f;
constexpr float kTileRounding = 5.0f;

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

BrowserTileInteraction DrawBrowserTile(const char* id, const std::shared_ptr<Texture2D>& texture,
                                       const std::string& type_label, const std::string& name, const bool selected,
                                       const float thumbnail_size, const float tile_width) {
  const ImVec2 tile_size(tile_width, thumbnail_size + kTileTypeHeight + kTileLabelHeight + kTilePadding);
  ImGui::InvisibleButton(id, tile_size);
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
  draw_list->AddText(ImVec2(min.x + glm::max((tile_width - type_size.x) * 0.5f, kTilePadding),
                            type_min.y + glm::max((kTileTypeHeight - type_size.y) * 0.5f, 0.0f)),
                     type_color, type_label.c_str());

  const ImVec2 label_min(min.x + kTilePadding, type_max.y + kTilePadding * 0.5f);
  const ImVec2 label_max(max.x - kTilePadding, max.y - kTilePadding * 0.5f);
  const ImVec4 label_clip(label_min.x, label_min.y, label_max.x, label_max.y);
  draw_list->AddText(nullptr, 0.0f, label_min,
                     selected || interaction.hovered ? StyleColor(ImGuiCol_Text) : StyleColor(ImGuiCol_Text, 0.88f),
                     name.c_str(), nullptr, label_max.x - label_min.x, &label_clip);

  return interaction;
}
}  // namespace

void ProjectContentBrowserPanel::Draw(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto& project_manager = ProjectManager::GetInstance();
  if (project_manager.show_project_window) {
    if (ImGui::Begin("Project")) {
      if (project_manager.assets_folder_) {
        auto current_focused_folder = project_manager.current_focused_folder_.lock();
        if (!current_focused_folder) {
          current_focused_folder = project_manager.assets_folder_;
          project_manager.current_focused_folder_ = current_focused_folder;
        }
        SyncNavigationHistory(current_focused_folder);
        auto current_folder_path = current_focused_folder->GetAssetsRelativePath();
        if (ImGui::BeginDragDropTarget()) {
          if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
            IM_ASSERT(payload->DataSize == sizeof(Handle));
            Handle handle = *static_cast<Handle*>(payload->Data);
            if (const auto asset = AssetManager::GetAssetImpl(handle)) {
              if (asset->IsTemporary()) {
                auto file_extension = Serialization::PeekAssetExtensions(asset->GetTypeName()).front();
                auto file_name = "New " + asset->GetTypeName();
                auto file_path = ProjectManager::GenerateNewAssetsRelativePath(
                    (current_focused_folder->GetAssetsRelativePath() / file_name).string(), file_extension);
                asset->SetPathAndSave(file_path);
              } else {
                if (auto file = asset->file_record_.lock();
                    file->GetFolder().lock().get() != current_focused_folder.get()) {
                  auto file_extension = file->GetAssetExtension();
                  auto file_name = file->GetAssetFileName();
                  auto file_path = ProjectManager::GenerateNewAssetsRelativePath(
                      (current_focused_folder->GetAssetsRelativePath() / file_name).string(), file_extension);
                  asset->SetPathAndSave(file_path);
                }
              }
            } else {
              if (const auto file = FileManager::GetFile(handle)) {
                auto folder = file->GetFolder().lock();
                if (folder.get() != current_focused_folder.get()) {
                  folder->MoveAsset(file->GetAssetHandle(), current_focused_folder);
                }
              }
            }
          }

          if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
            IM_ASSERT(payload->DataSize == sizeof(Handle));
            auto prefab = AssetManager::CreateTemporaryAsset<Prefab>();
            auto entity_handle = *static_cast<Handle*>(payload->Data);
            auto scene = ApplicationContext::Get().GetActiveScene();
            if (auto entity = scene->GetEntity(entity_handle); scene->IsEntityValid(entity)) {
              prefab->FromEntity(entity);
              // If current folder doesn't contain file with same name
              auto file_name = scene->GetEntityName(entity);
              auto file_extension = Serialization::PeekAssetExtensions("Prefab").front();
              auto file_path = ProjectManager::GenerateNewAssetsRelativePath((current_folder_path / file_name).string(),
                                                                             file_extension);
              prefab->SetPathAndSave(file_path);
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
        FolderHierarchyHelper(editor_layer, project_manager.assets_folder_);
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("2", ImVec2(content_width_ - 5.0f, h), true, ImGuiWindowFlags_AlwaysVerticalScrollbar);
        DrawToolbar(current_focused_folder);
        ImGui::Separator();
        bool updated = false;
        if (ImGui::BeginPopupContextWindow("NewAssetPopup")) {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
          if (ImGui::Button("Show in Explorer...")) {
            const auto folder_path = current_focused_folder->GetAbsolutePath().string();
            ShellExecuteA(nullptr, "open", folder_path.c_str(), nullptr, nullptr, SW_SHOWDEFAULT);
          }
#else
#endif

          FileUtils::OpenFile(
              "Import model...", "Model",
              {".eveprefab", ".obj", ".gltf", ".glb", ".blend", ".ply", ".fbx", ".dae", ".x3d"},
              [&](const std::filesystem::path& path) {
                const auto prefab = AssetManager::CreateTemporaryAsset<Prefab>();
                if (prefab->Import(path)) {
                  prefab->SetPathAndSave(current_focused_folder->GetAssetsRelativePath() /
                                         path.filename().replace_extension(".eveprefab"));
                }
              },
              false);

          if (ImGui::Button("New folder...")) {
            auto new_path = ProjectManager::GenerateNewAssetsRelativePath(
                (current_focused_folder->GetAssetsRelativePath() / "New Folder").string(), "");
            ProjectManager::GetOrCreateFolder(new_path);
          }
          if (ImGui::BeginMenu("New asset...")) {
            for (auto& i : Serialization::GetInstance().asset_extensions_) {
              if (i.first == "IAsset")
                continue;
              if (ImGui::Button(i.first.c_str())) {
                std::string new_file_name = "New " + i.first;
                std::filesystem::path new_path = ProjectManager::GenerateNewAssetsRelativePath(
                    (current_focused_folder->GetAssetsRelativePath() / new_file_name).string(), i.second.front());
                current_focused_folder->GetOrCreateAsset(new_path.stem().string(), new_path.extension().string());
              }
            }
            ImGui::EndMenu();
          }
          ImGui::EndPopup();
        }

        if (HasSearchQuery()) {
          DrawSearchResults(editor_layer, current_focused_folder, updated);
        } else {
          float panel_width = ImGui::GetContentRegionAvail().x;
          int column_count = glm::max(1, static_cast<int>(panel_width / (cell_size + thumbnail_padding_)));
          ImGui::Columns(column_count, nullptr, false);
          if (!updated) {
            for (auto& i : current_focused_folder->children_) {
              const std::string icon_tag = "##FolderTile" + std::to_string(i.second->handle_);
              const auto thumbnail_tex = EditorLayer::FindIcon("Folder");
              if (!thumbnail_tex) {
                continue;
              }
              const bool item_selected = selected_item_type_ == SelectedItemType::Folder &&
                                         selected_item_handle_.GetValue() == i.second->GetHandle().GetValue();
              const auto interaction = DrawBrowserTile(icon_tag.c_str(), thumbnail_tex, "Folder", i.second->name_,
                                                       item_selected, thumbnail_size_, cell_size);
              if (interaction.clicked) {
                selected_item_type_ = SelectedItemType::Folder;
                selected_item_handle_ = i.second->GetHandle();
              }

              if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
                ImGui::SetDragDropPayload("Folder", &i.second->handle_, sizeof(Handle));
                ImGui::TextColored(BrowserAccentTextColor(), i.second->name_.c_str());
                ImGui::EndDragDropSource();
              }
              if (i.second->GetHandle() != 0) {
                if (ImGui::BeginPopupContextItem(icon_tag.c_str())) {
                  if (ImGui::BeginMenu(("Rename" + icon_tag).c_str())) {
                    static char new_name[256] = {0};
                    ImGui::InputText(("New name" + icon_tag).c_str(), new_name, 256);
                    if (ImGui::Button(("Confirm" + icon_tag).c_str())) {
                      i.second->Rename(std::string(new_name));
                      memset(new_name, 0, 256);
                      ImGui::CloseCurrentPopup();
                    }
                    ImGui::EndMenu();
                  }
                  if (ImGui::Button(("Remove" + icon_tag).c_str())) {
                    i.second->parent_.lock()->DeleteChild(i.second->handle_);
                    updated = true;
                    ImGui::CloseCurrentPopup();
                    ImGui::EndPopup();
                    break;
                  }
                  ImGui::EndPopup();
                }
              }
              if (ImGui::BeginDragDropTarget()) {
                if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Folder")) {
                  IM_ASSERT(payload->DataSize == sizeof(Handle));
                  if (Handle payload_n = *static_cast<Handle*>(payload->Data); payload_n.GetValue() != 0) {
                    if (auto received_folder = FileManager::GetFolder(payload_n)) {
                      if (!i.second->IsSelfOrAncestor(received_folder->handle_) &&
                          received_folder->parent_.lock().get() != i.second.get()) {
                        received_folder->parent_.lock()->MoveChild(received_folder->GetHandle(), i.second);
                      }
                    }
                  }
                }
                if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
                  IM_ASSERT(payload->DataSize == sizeof(Handle));
                  Handle payload_n = *static_cast<Handle*>(payload->Data);
                  if (const auto asset = AssetManager::GetAssetImpl(payload_n)) {
                    if (asset->IsTemporary()) {
                      auto file_extension = Serialization::PeekAssetExtensions(asset->GetTypeName()).front();
                      auto file_name = "New " + asset->GetTypeName();
                      auto file_path = ProjectManager::GenerateNewAssetsRelativePath(
                          (i.second->GetAssetsRelativePath() / file_name).string(), file_extension);
                      asset->SetPathAndSave(file_path);
                    } else {
                      if (auto asset_record = asset->file_record_.lock();
                          asset_record->GetFolder().lock().get() != i.second.get()) {
                        auto file_extension = asset_record->GetAssetExtension();
                        auto file_name = asset_record->GetAssetFileName();
                        auto file_path = ProjectManager::GenerateNewAssetsRelativePath(
                            (i.second->GetAssetsRelativePath() / file_name).string(), file_extension);
                        asset->SetPathAndSave(file_path);
                      }
                    }
                  } else {
                    if (const auto file = FileManager::GetFile(payload_n)) {
                      auto folder = file->GetFolder().lock();
                      if (folder.get() != i.second.get()) {
                        folder->MoveAsset(file->GetAssetHandle(), i.second);
                      }
                    }
                  }
                }

                if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Binary")) {
                  IM_ASSERT(payload->DataSize == sizeof(Handle));
                  Handle payload_n = *static_cast<Handle*>(payload->Data);
                  if (const auto file = FileManager::GetFile(payload_n))
                    file->GetFolder().lock()->MoveAsset(payload_n, i.second);
                }

                if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
                  IM_ASSERT(payload->DataSize == sizeof(Handle));
                  auto prefab = AssetManager::CreateTemporaryAsset<Prefab>();
                  auto entity_handle = *static_cast<Handle*>(payload->Data);
                  auto scene = ApplicationContext::Get().GetActiveScene();
                  if (auto entity = scene->GetEntity(entity_handle); scene->IsEntityValid(entity)) {
                    prefab->FromEntity(entity);
                    // If current folder doesn't contain file with same name
                    auto file_name = scene->GetEntityName(entity);
                    auto file_extension = Serialization::PeekAssetExtensions("Prefab").front();
                    auto file_path = ProjectManager::GenerateNewAssetsRelativePath(
                        (i.second->GetAssetsRelativePath() / file_name).string(), file_extension);
                    prefab->SetPathAndSave(file_path);
                  }
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
            for (auto& i : current_focused_folder->files) {
              auto file_name = i.second->GetAssetsFolderRelativePath().filename();
              if (file_name.string() == ".eveproj" || file_name.extension().string() == ".eveproj")
                continue;
              const std::string icon_tag = "##AssetTile" + std::to_string(i.first.GetValue());
              const bool item_selected = selected_item_type_ == SelectedItemType::File &&
                                         selected_item_handle_.GetValue() == i.first.GetValue();

              const auto thumbnail_tex = i.second->GetThumbnail();
              const auto display_name = show_extension_ ? file_name.string() : file_name.stem().string();
              const auto interaction = DrawBrowserTile(icon_tag.c_str(), thumbnail_tex, TypeLabelForFile(i.second),
                                                       display_name, item_selected, thumbnail_size_, cell_size);
              if (interaction.clicked) {
                selected_item_type_ = SelectedItemType::File;
                selected_item_handle_ = i.first;
              }
              if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
                ImGui::SetDragDropPayload("Asset", &i.first, sizeof(Handle));
                ImGui::TextColored(BrowserAccentTextColor(), i.second->GetAssetFileName().c_str());
                ImGui::EndDragDropSource();
              }

              if (ImGui::BeginPopupContextItem(icon_tag.c_str())) {
                if (ImGui::Button("Duplicate")) {
                  i.second->GetFolder().lock()->Duplicate(i.second->GetAssetHandle());
                }
                if (i.second->GetAssetTypeName() != "Binary" && ImGui::BeginMenu(("Rename" + icon_tag).c_str())) {
                  static char new_name[256] = {};
                  ImGui::InputText(("New name" + icon_tag).c_str(), new_name, 256);
                  if (ImGui::Button(("Confirm" + icon_tag).c_str())) {
                    auto ptr = AssetManager::GetAssetImpl(i.second->asset_handle_);
                    ptr->SetPathAndSave(ptr->GetAssetsFolderRelativePath().replace_filename(
                        std::string(new_name) + ptr->GetFileRecord().lock()->GetAssetExtension()));
                    memset(new_name, 0, 256);
                  }
                  ImGui::EndMenu();
                }
                if (ImGui::Button(("Delete" + icon_tag).c_str())) {
                  current_focused_folder->RemoveFile(i.first);
                  ImGui::EndPopup();
                  break;
                }

                ImGui::EndPopup();
              }

              if (interaction.double_clicked && i.second->GetAssetTypeName() != "Binary") {
                // If it's an asset then inspect.
                if (auto asset = AssetManager::GetAssetImpl(i.second->asset_handle_))
                  editor_layer->OpenAssetInspector(asset);
              }
              ImGui::NextColumn();
            }
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

  const auto asset_load_snapshot = AssetManager::GetAssetLoadSnapshot();
  if (project_manager.scan_assets_pending) {
    ImGui::OpenPopup("Scanning assets...");
  } else if (asset_load_snapshot.Active()) {
    ImGui::OpenPopup("Loading assets...");
  } else if (!project_manager.new_project_path_.empty()) {
    ImGui::OpenPopup("Loading Project...");
  }
  if (ImGui::BeginPopupModal("Loading Project...", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("Busy...");
    if (project_manager.new_project_path_.empty()) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("Scanning assets...", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("Busy...");
    if (!project_manager.scan_assets_pending) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("Loading assets...", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("Progress: ");
    const auto completed_asset_count =
        asset_load_snapshot.completed + asset_load_snapshot.failed + asset_load_snapshot.cancelled;
    const auto active_asset_count = asset_load_snapshot.queued + asset_load_snapshot.loading_cpu +
                                    asset_load_snapshot.waiting_for_finalize + asset_load_snapshot.gpu_pending;
    auto total_asset_count = asset_load_snapshot.total;
    total_asset_count = std::max(total_asset_count, completed_asset_count + active_asset_count);
    total_asset_count = std::max(total_asset_count, project_manager.pending_asset_size);
    const float fraction = total_asset_count == 0
                               ? 1.0f
                               : static_cast<float>(completed_asset_count) / static_cast<float>(total_asset_count);
    const std::string text = std::to_string(static_cast<int>(fraction * 100.0f)) + "% - " +
                             std::to_string(completed_asset_count) + "/" + std::to_string(total_asset_count);
    ImGui::ProgressBar(fraction, ImVec2(240, 0), text.c_str());
    if (!asset_load_snapshot.active_asset_name.empty()) {
      ImGui::Text("Asset: %s", asset_load_snapshot.active_asset_name.c_str());
    }
    if (!asset_load_snapshot.message.empty()) {
      ImGui::Text("%s", asset_load_snapshot.message.c_str());
    }
    if (asset_load_snapshot.failed != 0 || asset_load_snapshot.cancelled != 0) {
      ImGui::Text("Failed: %zu  Cancelled: %zu", asset_load_snapshot.failed, asset_load_snapshot.cancelled);
    }
    ImGui::SetItemDefaultFocus();
    if (!asset_load_snapshot.Active()) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

void ProjectContentBrowserPanel::DrawToolbar(const std::shared_ptr<Folder>& current_folder) {
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
  const auto root_folder = ProjectManager::GetInstance().assets_folder_;
  const bool can_go_up = current_folder && root_folder && current_folder.get() != root_folder.get();
  if (draw_icon_button("ParentFolder", EditorLayer::FindIcon("BackButton"), "^##ParentFolder", "Parent folder",
                       can_go_up)) {
    NavigateToFolder(current_folder->parent_.lock());
  }
  ImGui::SameLine();
  if (draw_icon_button("RefreshProjectBrowser", EditorLayer::FindIcon("RefreshButton"), "R##RefreshProjectBrowser",
                       "Refresh", true)) {
    ProjectManager::DispatchScanAssetsTask();
  }

  ImGui::SameLine();
  ImGui::SetNextItemWidth(220.0f);
  ImGui::InputTextWithHint("##ProjectBrowserSearch", "Search assets", search_query_.data(), search_query_.size());
  SetLastItemTooltip(recursive_search_ ? "Searches nested folders and assets" : "Searches current folder only");

  ImGui::SameLine();
  if (ImGui::Button("...##ProjectBrowserSettings", {28, 24})) {
    ImGui::OpenPopup("ProjectBrowserSettings");
  }
  SetLastItemTooltip("Browser settings");
  if (ImGui::BeginPopup("ProjectBrowserSettings")) {
    ImGui::Checkbox("Show extensions", &show_extension_);
    ImGui::Checkbox("Recursive search", &recursive_search_);
    ImGui::SetNextItemWidth(160.0f);
    ImGui::SliderFloat("Thumbnail size", &thumbnail_size_, 10.0f, 150.0f, "%.0f");
    ImGui::SetNextItemWidth(160.0f);
    ImGui::SliderFloat("Padding", &thumbnail_padding_, 4.0f, 24.0f, "%.0f");
    ImGui::EndPopup();
  }

  DrawBreadcrumbs(current_folder);
}

void ProjectContentBrowserPanel::DrawBreadcrumbs(const std::shared_ptr<Folder>& current_folder) {
  std::vector<std::shared_ptr<Folder>> folders;
  for (auto folder = current_folder; folder; folder = folder->parent_.lock()) {
    folders.emplace_back(folder);
  }
  std::reverse(folders.begin(), folders.end());

  for (size_t folder_index = 0; folder_index < folders.size(); ++folder_index) {
    if (folder_index != 0) {
      ImGui::SameLine();
      ImGui::TextUnformatted(">");
      ImGui::SameLine();
    }
    const auto& folder = folders[folder_index];
    const std::string label = folder->GetName() + "##Breadcrumb" + std::to_string(folder->GetHandle().GetValue());
    if (ImGui::SmallButton(label.c_str())) {
      NavigateToFolder(folder);
    }
  }
}

void ProjectContentBrowserPanel::DrawSearchResults(const std::shared_ptr<EditorLayer>& editor_layer,
                                                   const std::shared_ptr<Folder>& root_folder, bool& updated) {
  const std::string query = search_query_.data();
  if (query.empty() || !root_folder) {
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
          selected_item_type_ = SelectedItemType::Folder;
          selected_item_handle_ = child->GetHandle();
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
        selected_item_type_ = SelectedItemType::File;
        selected_item_handle_ = handle;
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
          file->GetFolder().lock()->RemoveFile(handle);
          updated = true;
          ImGui::CloseCurrentPopup();
          ImGui::EndPopup();
          return;
        }
        ImGui::EndPopup();
      }
    }
  };

  visit_folder(visit_folder, root_folder);
  if (!has_results) {
    ImGui::TextDisabled("No matching assets.");
  }
}

void ProjectContentBrowserPanel::NavigateToFolder(const std::shared_ptr<Folder>& folder, const bool add_history) {
  if (!folder) {
    return;
  }

  auto& project_manager = ProjectManager::GetInstance();
  project_manager.current_focused_folder_ = folder;
  selected_item_type_ = SelectedItemType::None;
  selected_item_handle_ = 0;
  if (!add_history) {
    return;
  }

  const auto handle = folder->GetHandle();
  if (!folder_history_.empty() && folder_history_index_ < folder_history_.size() &&
      folder_history_[folder_history_index_].GetValue() == handle.GetValue()) {
    return;
  }
  if (folder_history_index_ + 1 < folder_history_.size()) {
    folder_history_.resize(folder_history_index_ + 1);
  }
  folder_history_.emplace_back(handle);
  folder_history_index_ = folder_history_.size() - 1;
}

void ProjectContentBrowserPanel::NavigateHistory(const int offset) {
  const auto target_index = static_cast<int64_t>(folder_history_index_) + offset;
  if (target_index < 0 || target_index >= static_cast<int64_t>(folder_history_.size())) {
    return;
  }

  if (const auto folder = FileManager::GetFolder(folder_history_[static_cast<size_t>(target_index)])) {
    folder_history_index_ = static_cast<size_t>(target_index);
    NavigateToFolder(folder, false);
  }
}

void ProjectContentBrowserPanel::SyncNavigationHistory(const std::shared_ptr<Folder>& current_folder) {
  if (!current_folder) {
    return;
  }
  const auto handle = current_folder->GetHandle();
  if (folder_history_.empty()) {
    folder_history_.emplace_back(handle);
    folder_history_index_ = 0;
    return;
  }
  if (folder_history_index_ >= folder_history_.size()) {
    folder_history_index_ = folder_history_.size() - 1;
  }
  if (folder_history_[folder_history_index_].GetValue() == handle.GetValue()) {
    return;
  }
  if (folder_history_index_ + 1 < folder_history_.size()) {
    folder_history_.resize(folder_history_index_ + 1);
  }
  folder_history_.emplace_back(handle);
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

void ProjectContentBrowserPanel::FolderHierarchyHelper(const std::shared_ptr<EditorLayer>& editor_layer,
                                                       const std::shared_ptr<Folder>& folder) {
  auto& project_manager = ProjectManager::GetInstance();
  auto focus_folder = project_manager.current_focused_folder_.lock();
  const bool opened = ImGui::TreeNodeEx(
      folder->name_.c_str(), ImGuiTreeNodeFlags_OpenOnArrow |
                                 (folder == focus_folder ? ImGuiTreeNodeFlags_Selected : ImGuiTreeNodeFlags_None));
  if (ImGui::BeginDragDropTarget()) {
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Folder")) {
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      Handle payload_n = *static_cast<Handle*>(payload->Data);
      if (payload_n.GetValue() != 0) {
        if (const auto received_folder = FileManager::GetFolder(payload_n)) {
          if (!folder->IsSelfOrAncestor(received_folder->handle_) &&
              received_folder->parent_.lock().get() != folder.get()) {
            received_folder->parent_.lock()->MoveChild(received_folder->GetHandle(), folder);
          }
        }
      }
    }
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      Handle payload_n = *static_cast<Handle*>(payload->Data);
      if (const auto asset = AssetManager::GetAssetImpl(payload_n)) {
        if (asset->IsTemporary()) {
          auto file_extension = Serialization::PeekAssetExtensions(asset->GetTypeName()).front();
          auto file_name = "New " + asset->GetTypeName();
          auto file_path = ProjectManager::GenerateNewAssetsRelativePath(
              (folder->GetAssetsRelativePath() / file_name).string(), file_extension);
          asset->SetPathAndSave(file_path);
        } else {
          if (auto asset_record = asset->file_record_.lock(); asset_record->GetFolder().lock().get() != folder.get()) {
            auto file_extension = asset_record->GetAssetExtension();
            auto file_name = asset_record->GetAssetFileName();
            auto file_path = ProjectManager::GenerateNewAssetsRelativePath(
                (folder->GetAssetsRelativePath() / file_name).string(), file_extension);
            asset->SetPathAndSave(file_path);
          }
        }
      } else {
        if (const auto file = FileManager::GetFile(payload_n)) {
          auto previous_folder = file->GetFolder().lock();
          if (folder && previous_folder.get() != folder.get()) {
            previous_folder->MoveAsset(file->GetAssetHandle(), folder);
          }
        }
      }
    }
    ImGui::EndDragDropTarget();
  }
  if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
    NavigateToFolder(folder);
  }
  const std::string tag = "##Folder" + std::to_string(folder->handle_);
  if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
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
        folder->parent_.lock()->DeleteChild(folder->handle_);
        ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
        return;
      }
      ImGui::EndPopup();
    }
  }
  if (opened) {
    for (const auto& i : folder->children_) {
      FolderHierarchyHelper(editor_layer, i.second);
    }
    for (const auto& i : folder->files) {
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
    }
    ImGui::TreePop();
  }
}
