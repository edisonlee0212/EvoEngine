#include "EditorPanelManager.hpp"

#include "imgui.h"
#include "imgui_internal.h"

#include <charconv>
#include <cstring>
#include <utility>

using namespace evo_engine;

namespace {
constexpr auto kEditorPanelSettingsType = "EditorPanel";
constexpr auto kEditorPanelSettingsEntry = "State";
}  // namespace

EditorPanelManager::~EditorPanelManager() {
  UnregisterSettingsHandler();
}

void EditorPanelManager::Clear() {
  panels_.clear();
}

void EditorPanelManager::RegisterPanel(EditorPanelCategory category, std::string id, std::string title, bool& open,
                                       std::shared_ptr<EditorPanel> panel) {
  panels_.emplace_back(PanelRecord{category, std::move(id), std::move(title), &open, open, open, std::move(panel)});
}

void EditorPanelManager::RegisterSettingsHandler() {
  if (settings_handler_registered_ || !ImGui::GetCurrentContext()) {
    return;
  }

  if (ImGui::FindSettingsHandler(kEditorPanelSettingsType)) {
    return;
  }

  ImGuiSettingsHandler handler;
  handler.TypeName = kEditorPanelSettingsType;
  handler.TypeHash = ImHashStr(kEditorPanelSettingsType);
  handler.ReadOpenFn = &EditorPanelManager::ReadSettingsOpen;
  handler.ReadLineFn = &EditorPanelManager::ReadSettingsLine;
  handler.WriteAllFn = &EditorPanelManager::WriteSettings;
  handler.UserData = this;
  ImGui::AddSettingsHandler(&handler);
  settings_handler_registered_ = true;
}

void EditorPanelManager::UnregisterSettingsHandler() {
  if (!settings_handler_registered_ || !ImGui::GetCurrentContext()) {
    settings_handler_registered_ = false;
    return;
  }

  ImGui::RemoveSettingsHandler(kEditorPanelSettingsType);
  settings_handler_registered_ = false;
}

void EditorPanelManager::DrawMenuItems(const EditorPanelCategory category) {
  for (auto& panel : panels_) {
    if (panel.category == category && panel.open) {
      if (ImGui::MenuItem(panel.title.c_str(), nullptr, panel.open)) {
        SyncPanelOpenState(panel);
      }
    }
  }
}

void EditorPanelManager::ResetPanelOpenStatesToDefaults() {
  for (auto& panel : panels_) {
    if (panel.open) {
      *panel.open = panel.default_open;
      panel.last_open = panel.default_open;
    }
  }
  if (ImGui::GetCurrentContext()) {
    ImGui::MarkIniSettingsDirty();
  }
}

void EditorPanelManager::OnInspect(const EditorPanelCategory category,
                                   const std::shared_ptr<EditorLayer>& editor_layer) {
  for (auto& panel : panels_) {
    SyncPanelOpenState(panel);
    if (panel.category == category && panel.open && *panel.open && panel.panel) {
      panel.panel->OnInspect(editor_layer);
      SyncPanelOpenState(panel);
    }
  }
}

void EditorPanelManager::SetPanelOpen(const std::string& id, const bool open) {
  for (auto& panel : panels_) {
    if (panel.id == id && panel.open) {
      *panel.open = open;
      panel.last_open = open;
      return;
    }
  }
}

void EditorPanelManager::SyncPanelOpenState(PanelRecord& panel) const {
  if (!panel.open || panel.last_open == *panel.open) {
    return;
  }

  panel.last_open = *panel.open;
  if (ImGui::GetCurrentContext()) {
    ImGui::MarkIniSettingsDirty();
  }
}

void EditorPanelManager::WriteSettings(ImGuiTextBuffer* out) const {
  out->appendf("[%s][%s]\n", kEditorPanelSettingsType, kEditorPanelSettingsEntry);
  for (const auto& panel : panels_) {
    out->appendf("%s=%d\n", panel.id.c_str(), panel.open && *panel.open ? 1 : 0);
  }
  out->append("\n");
}

void* EditorPanelManager::ReadSettingsOpen(ImGuiContext*, ImGuiSettingsHandler* handler, const char* name) {
  if (std::strcmp(name, kEditorPanelSettingsEntry) != 0) {
    return nullptr;
  }
  return handler->UserData;
}

void EditorPanelManager::ReadSettingsLine(ImGuiContext*, ImGuiSettingsHandler*, void* entry, const char* line) {
  auto* panel_manager = static_cast<EditorPanelManager*>(entry);
  const auto* equals = std::strchr(line, '=');
  if (!panel_manager || !equals) {
    return;
  }

  int open = 0;
  const auto* value_begin = equals + 1;
  const auto* value_end = value_begin + std::strlen(value_begin);
  if (std::from_chars(value_begin, value_end, open).ec != std::errc()) {
    return;
  }

  panel_manager->SetPanelOpen(std::string(line, equals), open != 0);
}

void EditorPanelManager::WriteSettings(ImGuiContext*, ImGuiSettingsHandler* handler, ImGuiTextBuffer* out) {
  const auto* panel_manager = static_cast<EditorPanelManager*>(handler->UserData);
  if (panel_manager) {
    panel_manager->WriteSettings(out);
  }
}
