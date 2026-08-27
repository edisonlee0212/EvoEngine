#pragma once

#include "EditorPanel.hpp"
#include "IHandle.hpp"

#include <array>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace evo_engine {
class EVOENGINE_API File;
class EVOENGINE_API Folder;

class EVOENGINE_API ProjectContentBrowserPanel final : public EditorPanel {
 public:
  void Draw(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void RevealAsset(const Handle& asset_handle);
  void RevealFolder(const std::filesystem::path& assets_relative_path);
  void SetHierarchyWidth(float width);

 private:
  enum class SelectedItemType { None, AssetFolder, AssetFile, ProjectFolder, ProjectFile };
  enum class BrowserLocationType { ProjectRoot, AssetFolder, ProjectFolder };

  struct BrowserLocation {
    BrowserLocationType type = BrowserLocationType::ProjectRoot;
    Handle asset_folder_handle = 0;
    std::filesystem::path project_folder_path;
  };

  void FolderHierarchyHelper(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Folder>& folder,
                             const std::shared_ptr<Folder>& reveal_folder);
  void ProjectHierarchyHelper(const std::shared_ptr<EditorLayer>& editor_layer, const std::filesystem::path& folder);
  void DrawToolbar();
  void DrawBreadcrumbs();
  void DrawAssetFolderContents(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Folder>& folder,
                               float cell_size, bool& updated);
  void DrawProjectFolderContents(float cell_size, bool& updated);
  void DrawSearchResults(const std::shared_ptr<EditorLayer>& editor_layer, bool& updated);
  bool DrawAssetContextMenu(const std::shared_ptr<File>& file, const std::string& tag);
  void NavigateToFolder(const std::shared_ptr<Folder>& folder, bool add_history = true);
  void NavigateToProjectRoot(bool add_history = true);
  void NavigateToProjectFolder(const std::filesystem::path& folder, bool add_history = true);
  void RequestHierarchyReveal(const std::shared_ptr<Folder>& folder);
  void RequestHierarchyReveal(const std::filesystem::path& folder);
  void NavigateHistory(int offset);
  void SyncNavigationHistory();

  [[nodiscard]] bool CanNavigateBack() const;
  [[nodiscard]] bool CanNavigateForward() const;
  [[nodiscard]] bool HasSearchQuery() const;
  [[nodiscard]] std::filesystem::path GetCurrentLocationPath() const;
  [[nodiscard]] std::shared_ptr<Folder> GetCurrentAssetFolder() const;
  [[nodiscard]] bool IsCurrentProjectRoot() const;

  [[nodiscard]] static bool FileMatchesSearch(const std::shared_ptr<File>& file, const std::string& query);
  [[nodiscard]] static bool FolderMatchesSearch(const std::shared_ptr<Folder>& folder, const std::string& query);
  [[nodiscard]] static bool ProjectPathMatchesSearch(const std::filesystem::path& path, const std::string& query);
  [[nodiscard]] static bool TextContainsCaseInsensitive(const std::string& text, const std::string& query);
  [[nodiscard]] static bool LocationsEqual(const BrowserLocation& lhs, const BrowserLocation& rhs);

  std::array<char, 128> search_query_{};
  std::vector<BrowserLocation> folder_history_;
  size_t folder_history_index_ = 0;
  float thumbnail_size_ = 75.0f;
  float thumbnail_padding_ = 8.0f;
  float hierarchy_width_ = 200.0f;
  float content_width_ = 200.0f;
  std::optional<Handle> hierarchy_reveal_target_;
  std::optional<std::filesystem::path> project_hierarchy_reveal_target_;
  BrowserLocation current_location_;
  SelectedItemType selected_item_type_ = SelectedItemType::None;
  Handle selected_item_handle_ = 0;
  std::filesystem::path selected_project_path_;
  bool show_extension_ = false;
  bool recursive_search_ = true;
};
}  // namespace evo_engine
