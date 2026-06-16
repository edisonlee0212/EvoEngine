#pragma once

#include "EditorPanel.hpp"
#include "IHandle.hpp"

#include <array>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace evo_engine {
class File;
class Folder;

class ProjectContentBrowserPanel final : public EditorPanel {
 public:
  void Draw(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void RevealAsset(const Handle& asset_handle);
  void RevealFolder(const std::filesystem::path& assets_relative_path);
  void SetHierarchyWidth(float width);

 private:
  enum class SelectedItemType { None, Folder, File };

  void FolderHierarchyHelper(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Folder>& folder);
  void DrawToolbar(const std::shared_ptr<Folder>& current_folder);
  void DrawBreadcrumbs(const std::shared_ptr<Folder>& current_folder);
  void DrawSearchResults(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Folder>& root_folder,
                         bool& updated);
  void NavigateToFolder(const std::shared_ptr<Folder>& folder, bool add_history = true);
  void NavigateHistory(int offset);
  void SyncNavigationHistory(const std::shared_ptr<Folder>& current_folder);

  [[nodiscard]] bool CanNavigateBack() const;
  [[nodiscard]] bool CanNavigateForward() const;
  [[nodiscard]] bool HasSearchQuery() const;

  [[nodiscard]] static bool FileMatchesSearch(const std::shared_ptr<File>& file, const std::string& query);
  [[nodiscard]] static bool FolderMatchesSearch(const std::shared_ptr<Folder>& folder, const std::string& query);
  [[nodiscard]] static bool TextContainsCaseInsensitive(const std::string& text, const std::string& query);

  std::array<char, 128> search_query_{};
  std::vector<Handle> folder_history_;
  size_t folder_history_index_ = 0;
  float thumbnail_size_ = 75.0f;
  float thumbnail_padding_ = 8.0f;
  float hierarchy_width_ = 200.0f;
  float content_width_ = 200.0f;
  SelectedItemType selected_item_type_ = SelectedItemType::None;
  Handle selected_item_handle_ = 0;
  bool show_extension_ = false;
  bool recursive_search_ = true;
};
}  // namespace evo_engine
