#include "ProjectManager.hpp"
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "Prefab.hpp"
#include "Scene.hpp"
#include "TransformGraph.hpp"
#include "WindowLayer.hpp"
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#  include "shellapi.h"
#endif

using namespace evo_engine;

std::weak_ptr<Folder> ProjectManager::GetOrCreateFolder(const std::filesystem::path& assets_relative_path) {
  const auto& project_manager = GetInstance();
  if (!assets_relative_path.is_relative()) {
    EVOENGINE_ERROR("Path not relative!")
    return {};
  }
  auto dir_path = project_manager.assets_folder_->GetAbsolutePath() / assets_relative_path;
  std::shared_ptr<Folder> ret_val = project_manager.assets_folder_;
  for (auto it = assets_relative_path.begin(); it != assets_relative_path.end(); ++it) {
    if (it == assets_relative_path.begin() && it->filename().string() == ".")
      continue;
    ret_val = ret_val->GetOrCreateChild(it->filename().string()).lock();
  }
  return ret_val;
}
std::shared_ptr<IAsset> ProjectManager::GetOrCreateAsset(const std::filesystem::path& assets_relative_path) {
  if (std::filesystem::is_directory(assets_relative_path)) {
    EVOENGINE_ERROR("Path is directory!")
    return {};
  }
  const auto folder = GetOrCreateFolder(assets_relative_path.parent_path()).lock();
  auto stem = assets_relative_path.stem().string();
  const auto file_name = assets_relative_path.filename().string();
  auto extension = assets_relative_path.extension().string();
  if (file_name == stem) {
    stem = "";
    extension = file_name;
  }
  return folder->GetOrCreateAsset(stem, extension);
}

void ProjectManager::SetupDefaultScene() {
  auto& project_manager = GetInstance();
  auto project_absolute_path = std::filesystem::absolute(project_manager.new_project_path_);
  bool found_scene = false;
  std::shared_ptr<Scene> scene;
  if (std::filesystem::exists(project_absolute_path)) {
    std::ifstream stream(project_absolute_path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    YAML::Node in = YAML::Load(string_stream.str());
    uint64_t scene_handle = 0;
    if (in["start_scene_handle"])
      scene_handle = in["start_scene_handle"].as<uint64_t>();
    if (in["EditorLayer"]) {
      if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
        editor_layer->Deserialize(in["EditorLayer"]);
      }
    }
    if (in["Layers"]) {
      const auto& layers = in["Layers"];
      for (const auto& layer : Application::GetLayers()) {
        const auto layer_name = layer->GetLayerName();
        if (layers[layer_name]) {
          layer->Deserialize(layers[layer_name]);
        }
      }
    }
    if (auto temp = AssetManager::GetAssetImpl(scene_handle)) {
      scene = std::dynamic_pointer_cast<Scene>(temp);
      SetStartScene(scene);
      SaveProject();
      Application::Attach(scene);
      found_scene = true;
    }
    EVOENGINE_LOG("Found and loaded project")
    if (found_scene && project_manager.scene_post_load_function_.has_value()) {
      project_manager.scene_post_load_function_.value()(scene);
      TransformGraph::CalculateTransformGraphs(scene);
    }
  }
  if (!found_scene) {
    scene = AssetManager::CreateTemporaryAsset<Scene>();
    if (std::filesystem::path new_scene_relative_path = GenerateNewAssetsRelativePath("New Scene", ".evescene");
        scene->SetPathAndSave(new_scene_relative_path)) {
      EVOENGINE_LOG("Created new start scene!")
    }
    SetStartScene(scene);
    SaveProject();
    Application::Attach(scene);

    if (project_manager.new_scene_customizer_.has_value()) {
      project_manager.new_scene_customizer_.value()(scene);
      TransformGraph::CalculateTransformGraphs(scene);
    }
  }

  project_manager.new_project_path_ = "";
}

void ProjectManager::PreUpdate() {
  const auto window_layer = Application::GetLayer<WindowLayer>();
  if (window_layer && Platform::GetFrameCount() < 4)
    return;
  auto& project_manager = GetInstance();

  if (project_manager.scan_assets_pending) {
    ScanAssets();
  } else if (!project_manager.pending_assets.empty()) {
    const auto handle = *project_manager.pending_assets.begin();
    project_manager.pending_assets.erase(handle);
    AssetManager::GetAssetImpl(handle);
  } else if (!project_manager.new_project_path_.empty()) {
    SetupDefaultScene();
  }

  if (project_manager.pending_assets.empty()) {
    project_manager.pending_asset_size = 0;
  } else {
  }
}

void ProjectManager::LoadAllPendingAssets() {
  auto& project_manager = GetInstance();
  if (project_manager.pending_assets.empty()) {
    return;
  }
  for (const auto& i : project_manager.pending_assets) {
    AssetManager::GetAssetImpl(i);
  }
  project_manager.pending_assets.clear();
  project_manager.pending_asset_size = 0;
}

void ProjectManager::SaveProject() {
  const auto& project_manager = GetInstance();
  if (const auto directory = project_manager.project_path_.parent_path(); !std::filesystem::exists(directory)) {
    std::filesystem::create_directories(directory);
  }
  if (const auto active_scene = Application::GetActiveScene(); active_scene && !active_scene->IsTemporary()) {
    active_scene->Save();
  }
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "start_scene_handle" << YAML::Value << project_manager.start_scene_->GetHandle();
  if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
    out << YAML::Key << "EditorLayer" << YAML::Value << YAML::BeginMap;
    editor_layer->Serialize(out);
    out << YAML::EndMap;
  }
  out << YAML::Key << "Layers" << YAML::Value << YAML::BeginMap;
  for (const auto& layer : Application::GetLayers()) {
    out << YAML::Key << layer->GetLayerName() << YAML::Value << YAML::BeginMap;
    layer->Serialize(out);
    out << YAML::EndMap;
  }
  out << YAML::EndMap;
  out << YAML::EndMap;
  std::ofstream file_out(project_manager.project_path_.string());
  file_out << out.c_str();
  file_out.flush();
}
std::filesystem::path ProjectManager::GetProjectPath() {
  auto& project_manager = GetInstance();
  return project_manager.project_path_;
}

std::filesystem::path ProjectManager::GetAssetsFolderPath() {
  auto& project_manager = GetInstance();
  return project_manager.assets_folder_path;
}

std::string ProjectManager::GetProjectName() {
  const auto& project_manager = GetInstance();
  return project_manager.project_path_.stem().string();
}
std::weak_ptr<Folder> ProjectManager::GetCurrentFocusedFolder() {
  auto& project_manager = GetInstance();
  return project_manager.current_focused_folder_;
}

std::shared_ptr<Folder> ProjectManager::GetAssetsFolder() {
  auto& project_manager = GetInstance();
  return project_manager.assets_folder_;
}

bool ProjectManager::IsInAssetsFolder(const std::filesystem::path& absolute_path) {
  if (!absolute_path.is_absolute()) {
    EVOENGINE_ERROR("Not absolute path!")
    return false;
  }
  const auto& project_manager = GetInstance();
  const auto project_folder_path = project_manager.assets_folder_path;
  const auto absolute_path_string = absolute_path.string();
  const auto project_folder_path_string = project_folder_path.string();
  return std::search(absolute_path_string.begin(), absolute_path_string.end(), project_folder_path_string.begin(),
                     project_folder_path_string.end()) != absolute_path_string.end();
}
bool ProjectManager::IsValidAssetFileName(const std::filesystem::path& path) {
  auto stem = path.stem().string();
  const auto file_name = path.filename().string();
  auto extension = path.extension().string();
  if (file_name == stem) {
    stem = "";
    extension = file_name;
  }
  return Serialization::GetAssetTypeName(extension) == "Binary";
}

void ProjectManager::GetOrCreateProject(const std::filesystem::path& path) {
  auto& project_manager = GetInstance();
  project_manager.new_project_path_ = path;
  auto project_absolute_path = std::filesystem::absolute(project_manager.new_project_path_);
  if (std::filesystem::is_directory(project_absolute_path)) {
    EVOENGINE_ERROR("Path is directory!")
    return;
  }
  if (!project_absolute_path.is_absolute()) {
    EVOENGINE_ERROR("Path not absolute!")
    return;
  }
  if (project_absolute_path.extension() != ".eveproj") {
    EVOENGINE_ERROR("Wrong extension!")
    return;
  }
  project_manager.project_path_ = project_absolute_path;
  project_manager.assets_folder_path = project_absolute_path.parent_path() / "Assets";
  AssetManager::Clear();
  FileManager::Clear();
  Application::Reset();

  auto& file_manager = FileManager::GetInstance();
  project_manager.current_focused_folder_ = project_manager.assets_folder_ = std::make_shared<Folder>();
  project_manager.assets_folder_->name_ = "Assets";
  file_manager.file_registry_mutex.lock();
  file_manager.folder_registry_[0] = project_manager.assets_folder_;
  file_manager.file_registry_mutex.unlock();
  project_manager.assets_folder_->self_ = project_manager.assets_folder_;

  if (const auto window_layer = Application::GetLayer<WindowLayer>()) {
    DispatchScanAssetsTask();
  } else {
    ScanAssets();
    LoadAllPendingAssets();
    SetupDefaultScene();
  }
}

void ProjectManager::DispatchScanAssetsTask() {
  auto& project_manager = GetInstance();
  project_manager.scan_assets_pending = true;
}

std::filesystem::path ProjectManager::GenerateNewAssetsRelativePath(const std::string& relative_stem,
                                                                    const std::string& postfix) {
  assert(std::filesystem::path(relative_stem + postfix).is_relative());
  const auto& project_manager = GetInstance();
  const auto assets_path = project_manager.assets_folder_path;
  std::filesystem::path test_path = assets_path / (relative_stem + postfix);
  int i = 0;
  while (std::filesystem::exists(test_path)) {
    i++;
    test_path = assets_path / (relative_stem + " (" + std::to_string(i) + ")" + postfix);
  }
  if (i == 0)
    return relative_stem + postfix;
  return relative_stem + " (" + std::to_string(i) + ")" + postfix;
}

std::filesystem::path ProjectManager::GenerateNewAbsolutePath(const std::string& absolute_stem,
                                                              const std::string& postfix) {
  std::filesystem::path test_path = absolute_stem + postfix;
  int i = 0;
  while (std::filesystem::exists(test_path)) {
    i++;
    test_path = absolute_stem + " (" + std::to_string(i) + ")" + postfix;
  }
  if (i == 0)
    return absolute_stem + postfix;
  return absolute_stem + " (" + std::to_string(i) + ")" + postfix;
}

void ProjectManager::SetActionAfterSceneLoad(const std::function<void(const std::shared_ptr<Scene>&)>& actions) {
  auto& project_manager = GetInstance();
  project_manager.scene_post_load_function_ = actions;
}

void ProjectManager::SetActionAfterNewScene(const std::function<void(const std::shared_ptr<Scene>&)>& actions) {
  auto& project_manager = GetInstance();
  project_manager.new_scene_customizer_ = actions;
}

void ProjectManager::ScanAssets() {
  auto& project_manager = GetInstance();
  project_manager.scan_assets_pending = false;
  if (!project_manager.assets_folder_)
    return;
  if (!std::filesystem::exists(project_manager.assets_folder_->GetAbsolutePath())) {
    std::filesystem::create_directories(project_manager.assets_folder_->GetAbsolutePath());
  }
  auto& file_manager = FileManager::GetInstance();
  file_manager.file_registry_mutex.lock();
  project_manager.assets_folder_->handle_ = 0;
  std::vector<Handle> missing_asset_handles;
  project_manager.assets_folder_->Refresh(missing_asset_handles);
  file_manager.file_registry_mutex.unlock();
  project_manager.pending_assets.clear();
  project_manager.pending_asset_size = missing_asset_handles.size();
  for (const auto& i : missing_asset_handles) {
    project_manager.pending_assets.emplace(i);
  }
}

void ProjectManager::Initialize() {
  auto& project_manager = GetInstance();
  project_manager.initialized = true;
}

void ProjectManager::OnDestroy() {
  auto& project_manager = GetInstance();

  project_manager.assets_folder_.reset();
  project_manager.new_scene_customizer_.reset();
  project_manager.current_focused_folder_.reset();
  project_manager.start_scene_.reset();

  project_manager.initialized = false;
}

void ProjectManager::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto& project_manager = GetInstance();
  auto& asset_manager = AssetManager::GetInstance();
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("Project")) {
      ImGui::Text(("Current Project path: " + project_manager.project_path_.string()).c_str());

      FileUtils::SaveFile(
          "Create or load New Project", "Project", {".eveproj"},
          [](const std::filesystem::path& file_path) {
            try {
              GetOrCreateProject(file_path);
            } catch (const std::exception& e) {
              EVOENGINE_ERROR(std::string(e.what()) + ": Failed to create/load from " + file_path.string())
            }
          },
          false);

      if (ImGui::Button("Save")) {
        SaveProject();
      }
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View")) {
      ImGui::Checkbox("Project", &project_manager.show_project_window);
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }
  if (project_manager.show_project_window) {
    if (ImGui::Begin("Project")) {
      if (project_manager.assets_folder_) {
        auto current_focused_folder = project_manager.current_focused_folder_.lock();
        auto current_folder_path = current_focused_folder->GetAssetsRelativePath();
        if (ImGui::BeginDragDropTarget()) {
          if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
            IM_ASSERT(payload->DataSize == sizeof(Handle));
            Handle handle = *static_cast<Handle*>(payload->Data);
            if (const auto asset = AssetManager::GetAssetImpl(handle)) {
              if (asset->IsTemporary()) {
                auto file_extension = Serialization::PeekAssetExtensions(asset->GetTypeName()).front();
                auto file_name = "New " + asset->GetTypeName();
                auto file_path = GenerateNewAssetsRelativePath(
                    (current_focused_folder->GetAssetsRelativePath() / file_name).string(), file_extension);
                asset->SetPathAndSave(file_path);
              } else {
                if (auto file = asset->file_record_.lock();
                    file->GetFolder().lock().get() != current_focused_folder.get()) {
                  auto file_extension = file->GetAssetExtension();
                  auto file_name = file->GetAssetFileName();
                  auto file_path = GenerateNewAssetsRelativePath(
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
            auto scene = Application::GetActiveScene();
            if (auto entity = scene->GetEntity(entity_handle); scene->IsEntityValid(entity)) {
              prefab->FromEntity(entity);
              // If current folder doesn't contain file with same name
              auto file_name = scene->GetEntityName(entity);
              auto file_extension = Serialization::PeekAssetExtensions("Prefab").front();
              auto file_path =
                  GenerateNewAssetsRelativePath((current_folder_path / file_name).string(), file_extension);
              prefab->SetPathAndSave(file_path);
            }
          }

          ImGui::EndDragDropTarget();
        }
        static glm::vec2 thumbnail_size_padding = {75.0f, 8.0f};
        float cell_size = thumbnail_size_padding.x + thumbnail_size_padding.y;
        static float size1 = 200;
        static float size2 = 200;
        static float h = 100;
        auto avail = ImGui::GetContentRegionAvail();
        size2 = glm::max(avail.x - size1, cell_size + 8.0f);
        size1 = glm::max(avail.x - size2, 32.0f);
        h = avail.y;
        ImGui::Splitter(true, 8.0, size1, size2, 32.0f, cell_size + 8.0f, h);
        ImGui::BeginChild("1", ImVec2(size1, h), true);
        FolderHierarchyHelper(editor_layer, project_manager.assets_folder_);
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("2", ImVec2(size2 - 5.0f, h), true, ImGuiWindowFlags_AlwaysVerticalScrollbar);
        if (ImGui::ImageButton(editor_layer->editor_icons_["RefreshButton"]->GetImTextureId(), {16, 16}, {0, 1},
                               {1, 0})) {
          DispatchScanAssetsTask();
        }
        ImGui::SameLine();
        if (current_focused_folder != project_manager.assets_folder_) {
          if (ImGui::ImageButton(editor_layer->editor_icons_["BackButton"]->GetImTextureId(), {16, 16}, {0, 1},
                                 {1, 0})) {
            project_manager.current_focused_folder_ = current_focused_folder->parent_;
          }
        } else {
          ImGui::BeginDisabled();
          if (ImGui::ImageButton(editor_layer->editor_icons_["BackButton"]->GetImTextureId(), {16, 16}, {0, 1},
                                 {1, 0})) {
            project_manager.current_focused_folder_ = current_focused_folder->parent_;
          }
          ImGui::EndDisabled();
        }
        static bool show_extension = false;
        ImGui::SameLine();
        ImGui::Checkbox("Ext", &show_extension);
        ImGui::SameLine();
        ImGui::Text(current_focused_folder->GetAssetsRelativePath().string().c_str());

        ImGui::SameLine();

        ImGui::PushItemWidth(60);
        ImGui::SliderFloat("##Thumbnail size", &thumbnail_size_padding.x, 10, 150, "%.0f", ImGuiSliderFlags_None);
        ImGui::PopItemWidth();

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
            auto new_path = GenerateNewAssetsRelativePath(
                (current_focused_folder->GetAssetsRelativePath() / "New Folder").string(), "");
            GetOrCreateFolder(new_path);
          }
          if (ImGui::BeginMenu("New asset...")) {
            for (auto& i : Serialization::GetInstance().asset_extensions_) {
              if (i.first == "IAsset")
                continue;
              if (ImGui::Button(i.first.c_str())) {
                std::string new_file_name = "New " + i.first;
                std::filesystem::path new_path = GenerateNewAssetsRelativePath(
                    (current_focused_folder->GetAssetsRelativePath() / new_file_name).string(), i.second.front());
                current_focused_folder->GetOrCreateAsset(new_path.stem().string(), new_path.extension().string());
              }
            }
            ImGui::EndMenu();
          }
          ImGui::EndPopup();
        }

        float panel_width = ImGui::GetContentRegionAvail().x;
        int column_count = glm::max(1, static_cast<int>(panel_width / (cell_size + thumbnail_size_padding.y)));
        ImGui::Columns(column_count, nullptr, false);
        if (!updated) {
          for (auto& i : current_focused_folder->children_) {
            const std::string tag = "##Folder" + std::to_string(i.second->handle_);
            const auto& thumbnail_tex = editor_layer->editor_icons_["Folder"];
            glm::vec2 resolution = thumbnail_tex->GetResolution();
            resolution *= thumbnail_size_padding.x / glm::max(resolution.x, resolution.y);
            ImGui::ImageButton(tag.c_str(), thumbnail_tex->GetImTextureId(), {resolution.x, resolution.y}, {0, 1},
                               {1, 0});

            if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
              ImGui::SetDragDropPayload("Folder", &i.second->handle_, sizeof(Handle));
              ImGui::TextColored(ImVec4(0, 0, 1, 1), ("Folder" + tag).c_str());
              ImGui::EndDragDropSource();
            }
            if (i.second->GetHandle() != 0) {
              if (ImGui::BeginPopupContextItem(tag.c_str())) {
                if (ImGui::BeginMenu(("Rename" + tag).c_str())) {
                  static char new_name[256] = {0};
                  ImGui::InputText(("New name" + tag).c_str(), new_name, 256);
                  if (ImGui::Button(("Confirm" + tag).c_str())) {
                    i.second->Rename(std::string(new_name));
                    memset(new_name, 0, 256);
                    ImGui::CloseCurrentPopup();
                  }
                  ImGui::EndMenu();
                }
                if (ImGui::Button(("Remove" + tag).c_str())) {
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
                    auto file_path = GenerateNewAssetsRelativePath(
                        (i.second->GetAssetsRelativePath() / file_name).string(), file_extension);
                    asset->SetPathAndSave(file_path);
                  } else {
                    if (auto asset_record = asset->file_record_.lock();
                        asset_record->GetFolder().lock().get() != i.second.get()) {
                      auto file_extension = asset_record->GetAssetExtension();
                      auto file_name = asset_record->GetAssetFileName();
                      auto file_path = GenerateNewAssetsRelativePath(
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
                auto scene = Application::GetActiveScene();
                if (auto entity = scene->GetEntity(entity_handle); scene->IsEntityValid(entity)) {
                  // If current folder doesn't contain file with same name
                  auto file_name = scene->GetEntityName(entity);
                  auto file_extension = Serialization::PeekAssetExtensions("Prefab").front();
                  auto file_path = GenerateNewAssetsRelativePath(
                      (i.second->GetAssetsRelativePath() / file_name).string(), file_extension);
                  prefab->SetPathAndSave(file_path);
                }
              }

              ImGui::EndDragDropTarget();
            }
            bool item_hovered = false;
            if (ImGui::IsItemHovered()) {
              item_hovered = true;
              if (ImGui::IsMouseDoubleClicked(0)) {
                project_manager.current_focused_folder_ = i.second;
                updated = true;
                break;
              }
            }
            if (item_hovered)
              ImGui::PushStyleColor(ImGuiCol_Text, {1, 1, 0, 1});
            ImGui::BeginDisabled();
            ImGui::ButtonEx("Folder", {thumbnail_size_padding.x + 8, 20});
            ImGui::EndDisabled();
            ImGui::TextWrapped(i.second->name_.c_str());
            if (item_hovered)
              ImGui::PopStyleColor(1);
            ImGui::NextColumn();
          }
        }
        if (!updated) {
          for (auto& i : current_focused_folder->files) {
            auto file_name = i.second->GetAssetsFolderRelativePath().filename();
            if (file_name.string() == ".eveproj" || file_name.extension().string() == ".eveproj")
              continue;
            static Handle focused_asset_handle;
            bool item_focused = false;
            if (focused_asset_handle == i.first.GetValue()) {
              item_focused = true;
            }
            const std::string tag = "##" + i.second->GetAssetTypeName() + std::to_string(i.first.GetValue());

            const auto thumbnail_tex = i.second->GetThumbnail();
            glm::vec2 resolution = {thumbnail_size_padding.x, thumbnail_size_padding.x};
            if (thumbnail_tex) {
              resolution = thumbnail_tex->GetResolution();
              const float max_dim = glm::max(resolution.x, resolution.y);
              if (max_dim > 0.0f) {
                resolution *= thumbnail_size_padding.x / max_dim;
              }
            }
            ImTextureID thumbnail_id = thumbnail_tex ? thumbnail_tex->GetImTextureId() : nullptr;
            if (!thumbnail_id) {
              if (const auto fallback_tex = EditorLayer::FindIcon("Binary")) {
                thumbnail_id = fallback_tex->GetImTextureId();
                const auto fallback_resolution = fallback_tex->GetResolution();
                resolution = glm::vec2(fallback_resolution) *
                             (thumbnail_size_padding.x / glm::max(fallback_resolution.x, fallback_resolution.y));
              }
            }
            if (thumbnail_id) {
              ImGui::ImageButton(tag.c_str(), thumbnail_id, {resolution.x, resolution.y}, {0, 1}, {1, 0});
            } else {
              ImGui::Button(tag.c_str(), {resolution.x, resolution.y});
            }
            bool item_hovered = false;
            if (ImGui::IsItemHovered()) {
              item_hovered = true;
              if (ImGui::IsMouseDoubleClicked(0) && i.second->GetAssetTypeName() != "Binary") {
                // If it's an asset then inspect.
                if (auto asset = AssetManager::GetAssetImpl(i.second->asset_handle_))
                  editor_layer->inspecting_asset = asset;
              }
            }
            if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
              ImGui::SetDragDropPayload("Asset", &i.first, sizeof(Handle));
              ImGui::TextColored(ImVec4(0, 0, 1, 1), i.second->GetAssetFileName().c_str());
              ImGui::EndDragDropSource();
            }

            if (ImGui::BeginPopupContextItem(tag.c_str())) {
              if (ImGui::Button("Duplicate")) {
                i.second->GetFolder().lock()->Duplicate(i.second->GetAssetHandle());
              }
              if (i.second->GetAssetTypeName() != "Binary" && ImGui::BeginMenu(("Rename" + tag).c_str())) {
                static char new_name[256] = {};
                ImGui::InputText(("New name" + tag).c_str(), new_name, 256);
                if (ImGui::Button(("Confirm" + tag).c_str())) {
                  auto ptr = AssetManager::GetAssetImpl(i.second->asset_handle_);
                  ptr->SetPathAndSave(ptr->GetAssetsFolderRelativePath().replace_filename(
                      std::string(new_name) + ptr->GetFileRecord().lock()->GetAssetExtension()));
                  memset(new_name, 0, 256);
                }
                ImGui::EndMenu();
              }
              if (ImGui::Button(("Delete" + tag).c_str())) {
                current_focused_folder->RemoveFile(i.first);
                ImGui::EndPopup();
                break;
              }

              ImGui::EndPopup();
            }

            if (item_focused)
              ImGui::PushStyleColor(ImGuiCol_Text, {1, 0, 0, 1});
            else if (item_hovered)
              ImGui::PushStyleColor(ImGuiCol_Text, {1, 1, 0, 1});
            ImGui::BeginDisabled();
            std::string type_text = i.second->GetAssetTypeName();
            if (type_text == "Binary") {
              type_text = "??? (";
              type_text.append(i.second->asset_extension_);
              type_text.append(")");
            }
            ImGui::ButtonEx(type_text.c_str(), {thumbnail_size_padding.x + 8, 20});
            ImGui::EndDisabled();
            if (show_extension) {
              ImGui::TextWrapped(file_name.string().c_str());
            } else {
              ImGui::TextWrapped(file_name.stem().string().c_str());
            }
            if (item_focused || item_hovered)
              ImGui::PopStyleColor(1);
            ImGui::NextColumn();
          }
        }

        ImGui::Columns(1);
        // ImGui::SliderFloat("Thumbnail Size", &thumbnailSizePadding.x, 16, 512);
        ImGui::EndChild();
      } else {
        ImGui::Text("No project loaded!");
      }
    }
    ImGui::End();
  }

  if (project_manager.scan_assets_pending) {
    ImGui::OpenPopup("Scanning files...");
  } else if (project_manager.pending_asset_size != 0) {
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
    const float fraction =
        1.0f - static_cast<float>(project_manager.pending_assets.size()) / project_manager.pending_asset_size;
    const std::string text =
        std::to_string(static_cast<int>(fraction * 100.0f)) + "% - " +
        std::to_string(project_manager.pending_asset_size - project_manager.pending_assets.size()) + "/" +
        std::to_string(project_manager.pending_asset_size);
    ImGui::ProgressBar(fraction, ImVec2(240, 0), text.c_str());
    ImGui::SetItemDefaultFocus();
    if (project_manager.pending_asset_size == 0) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

void ProjectManager::FolderHierarchyHelper(const std::shared_ptr<EditorLayer>& editor_layer,
                                           const std::shared_ptr<Folder>& folder) {
  auto& project_manager = GetInstance();
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
          auto file_path =
              GenerateNewAssetsRelativePath((folder->GetAssetsRelativePath() / file_name).string(), file_extension);
          asset->SetPathAndSave(file_path);
        } else {
          if (auto asset_record = asset->file_record_.lock(); asset_record->GetFolder().lock().get() != folder.get()) {
            auto file_extension = asset_record->GetAssetExtension();
            auto file_name = asset_record->GetAssetFileName();
            auto file_path =
                GenerateNewAssetsRelativePath((folder->GetAssetsRelativePath() / file_name).string(), file_extension);
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
    project_manager.current_focused_folder_ = folder;
  }
  const std::string tag = "##Folder" + std::to_string(folder->handle_);
  if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
    ImGui::SetDragDropPayload("Folder", &folder->handle_, sizeof(Handle));
    ImGui::TextColored(ImVec4(0, 0, 1, 1), ("Folder" + tag).c_str());
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
            editor_layer->inspecting_asset = asset;
        }
      }
      if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
        ImGui::SetDragDropPayload("Asset", &i.first, sizeof(Handle));
        ImGui::TextColored(ImVec4(0, 0, 1, 1), i.second->GetAssetFileName().c_str());
        ImGui::EndDragDropSource();
      }
    }
    ImGui::TreePop();
  }
}

std::weak_ptr<Scene> ProjectManager::GetStartScene() {
  auto& project_manager = GetInstance();
  return project_manager.start_scene_;
}
void ProjectManager::SetStartScene(const std::shared_ptr<Scene>& scene) {
  auto& project_manager = GetInstance();
  project_manager.start_scene_ = scene;
}

std::filesystem::path ProjectManager::GetAssetsRelativePath(const std::filesystem::path& absolute_path) {
  const auto& project_manager = GetInstance();
  if (!project_manager.assets_folder_)
    return {};
  if (!absolute_path.is_absolute())
    return {};
  if (!IsInAssetsFolder(absolute_path))
    return {};
  return std::filesystem::relative(absolute_path, project_manager.assets_folder_path);
}