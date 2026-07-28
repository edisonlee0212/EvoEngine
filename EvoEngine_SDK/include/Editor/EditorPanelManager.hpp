#pragma once

#include "EditorPanel.hpp"

#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

struct ImGuiContext;
struct ImGuiSettingsHandler;
struct ImGuiTextBuffer;

namespace evo_engine {

enum class EditorPanelCategory { View };

class EditorPanelManager {
 public:
  ~EditorPanelManager();

  void Clear();
  void RegisterPanel(EditorPanelCategory category, std::string id, std::string title, bool& open,
                     std::shared_ptr<EditorPanel> panel);
  void RegisterSettingsHandler();
  void UnregisterSettingsHandler();
  void DrawMenuItems(EditorPanelCategory category);
  void ResetPanelOpenStatesToDefaults();
  void Draw(EditorPanelCategory category, const std::shared_ptr<EditorLayer>& editor_layer);
  void DrawExcept(EditorPanelCategory category, const std::shared_ptr<EditorLayer>& editor_layer,
                  std::string_view excluded_id);
  void DrawOnly(EditorPanelCategory category, const std::shared_ptr<EditorLayer>& editor_layer, std::string_view id);

 private:
  struct PanelRecord {
    EditorPanelCategory category = EditorPanelCategory::View;
    std::string id;
    std::string title;
    bool* open = nullptr;
    bool default_open = false;
    bool last_open = false;
    std::shared_ptr<EditorPanel> panel;
  };

  void SetPanelOpen(const std::string& id, bool open);
  void SyncPanelOpenState(PanelRecord& panel) const;
  void WriteSettings(ImGuiTextBuffer* out) const;
  static void* ReadSettingsOpen(ImGuiContext* context, ImGuiSettingsHandler* handler, const char* name);
  static void ReadSettingsLine(ImGuiContext* context, ImGuiSettingsHandler* handler, void* entry, const char* line);
  static void WriteSettings(ImGuiContext* context, ImGuiSettingsHandler* handler, ImGuiTextBuffer* out);

  std::vector<PanelRecord> panels_;
  bool settings_handler_registered_ = false;
};

}  // namespace evo_engine
