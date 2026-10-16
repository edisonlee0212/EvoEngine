#include "AssetManager.hpp"
#include "EditorLayer.hpp"
#include "RenderLayer.hpp"

#include <algorithm>

using namespace evo_engine;

float EditorLayer::DrawStatusBar() {
  const auto viewport = ImGui::GetMainViewport();
  const auto& style = ImGui::GetStyle();
  const float height = ImGui::GetFrameHeight();
  ImGui::SetNextWindowPos({viewport->Pos.x, viewport->Pos.y + viewport->Size.y - height});
  ImGui::SetNextWindowSize({viewport->Size.x, height});
  ImGui::SetNextWindowViewport(viewport->ID);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(style.FramePadding.x, 0));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowMinSize, ImVec2(0, 0));
  ImGui::PushStyleColor(ImGuiCol_WindowBg, style.Colors[ImGuiCol_MenuBarBg]);
  constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoDocking |
                                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove |
                                     ImGuiWindowFlags_NoScrollWithMouse;
  if (ImGui::Begin("##EditorStatusBar", nullptr, flags)) {
    const auto renderer = GetApplication().GetLayer<RenderLayer>();
    const auto shaders = renderer ? renderer->GetShaderPreparationProgress() : ShaderPreparationProgress{};
    const auto assets = AssetManager::GetAssetLoadSnapshot();
    const auto& project = ProjectManager::GetInstance();
    const bool compiling = shaders.completed < shaders.total;
    const bool loading = assets.Active();
    const bool scanning = project.scan_assets_pending;
    const size_t activities = size_t(compiling) + size_t(loading) + size_t(scanning);
    std::string error = shaders.error;
    if (assets.failed)
      error += (error.empty() ? "" : "\n") + std::to_string(assets.failed) + " assets failed to load.";
    if (error.empty())
      acknowledged_background_error_.clear();
    const bool unacknowledged_error = !error.empty() && error != acknowledged_background_error_;
    const char* activity = unacknowledged_error ? "Background activity failed"
                           : compiling          ? "Compiling shaders"
                           : loading            ? "Loading assets"
                           : scanning           ? "Scanning assets"
                                                : "Ready";
    {
      const std::lock_guard lock(console_message_mutex_);
      if (status_console_revision_ != console_message_revision_) {
        status_warning_count_ = status_error_count_ = 0;
        for (const auto& message : console_messages_) {
          status_warning_count_ += message.m_type == ConsoleMessageType::Warning;
          status_error_count_ += message.m_type == ConsoleMessageType::Error;
        }
        status_console_revision_ = console_message_revision_;
      }
    }

    const auto start = ImGui::GetCursorScreenPos();
    const float width = ImGui::GetContentRegionAvail().x;
    const float gap = style.ItemSpacing.x;
    const bool compact = width < 600.0f;
    const std::string warnings = (compact ? "W " : "Warnings ") + std::to_string(status_warning_count_);
    const std::string errors = (compact ? "E " : "Errors ") + std::to_string(status_error_count_);
    const std::string tasks = "Background (" + std::to_string(activities) + ")";
    const float counts_width = ImGui::CalcTextSize(warnings.c_str()).x + ImGui::CalcTextSize(errors.c_str()).x +
                               4.0f * style.FramePadding.x + gap;
    const float tasks_width = ImGui::CalcTextSize(tasks.c_str()).x + 2.0f * style.FramePadding.x;
    const bool show_tasks = width >= counts_width + tasks_width + 2.0f * gap;
    const float right_width = counts_width + (show_tasks ? tasks_width + gap : 0.0f);
    const float left_width = std::max(0.0f, width - right_width - gap);
    const bool show_progress = activities && left_width >= 340.0f && !unacknowledged_error;
    if (left_width > 0.0f) {
      const float progress_width = show_progress ? 128.0f + gap : 0.0f;
      const float activity_width =
          std::min(ImGui::CalcTextSize(activity).x + 2.0f * style.FramePadding.x, left_width - progress_width);
      ImGui::SetCursorScreenPos({start.x + left_width - activity_width - progress_width, start.y});
      const std::string label = std::string(activity) + "###StatusActivity";
      ImGui::PushStyleVar(ImGuiStyleVar_SelectableTextAlign, ImVec2(1, 0.5f));
      if (ImGui::Selectable(label.c_str(), false, 0, {activity_width, height}))
        ImGui::OpenPopup("Background activity");
      ImGui::PopStyleVar();
      if (ImGui::IsItemHovered()) {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 40.0f);
        ImGui::TextUnformatted(activity);
        if (!shaders.current.empty() && compiling)
          ImGui::TextWrapped("%s", shaders.current.c_str());
        if (!assets.active_asset_name.empty() && loading)
          ImGui::TextWrapped("%s", assets.active_asset_name.c_str());
        if (unacknowledged_error)
          ImGui::TextWrapped("%s", error.c_str());
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
      }
      if (show_progress) {
        ImGui::SameLine();
        const size_t total = compiling ? shaders.total : loading ? assets.total : 0;
        const size_t completed = compiling ? shaders.completed : loading ? assets.completed : 0;
        const std::string progress = total ? std::to_string(completed) + " / " + std::to_string(total) : "Scanning";
        ImGui::ProgressBar(total ? float(completed) / float(total) : -float(ImGui::GetTime()), {128.0f, height},
                           progress.c_str());
      }
    }
    ImGui::SetCursorScreenPos({start.x + std::max(0.0f, width - right_width), start.y});
    if (show_tasks) {
      if (ImGui::Button((tasks + "###StatusTasks").c_str(), {tasks_width, height}))
        ImGui::OpenPopup("Background activity");
      ImGui::SameLine();
    }
    if (ImGui::Button((warnings + "###StatusWarnings").c_str(), {0, height})) {
      SetConsoleMessageFilters(false, true, false);
      show_console_window = console_focus_requested_ = true;
    }
    ImGui::SetItemTooltip("Open Console warnings");
    ImGui::SameLine();
    if (ImGui::Button((errors + "###StatusErrors").c_str(), {0, height})) {
      SetConsoleMessageFilters(false, false, true);
      show_console_window = console_focus_requested_ = true;
    }
    ImGui::SetItemTooltip("Open Console errors");

    const float popup_width = std::min(560.0f, viewport->Size.x);
    ImGui::SetNextWindowPos({viewport->Pos.x, viewport->Pos.y + viewport->Size.y - height}, ImGuiCond_Appearing,
                            {0, 1});
    ImGui::SetNextWindowSizeConstraints({popup_width, 0}, {popup_width, std::max(1.0f, viewport->Size.y - height)});
    if (ImGui::BeginPopup("Background activity", ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::TextUnformatted("Background activity");
      ImGui::Separator();
      if (!activities)
        ImGui::TextUnformatted("No background activity.");
      if (shaders.total) {
        ImGui::Text("Shaders: %zu / %zu builds", shaders.completed, shaders.total);
        if (compiling)
          ImGui::TextWrapped("%s", shaders.current.c_str());
      }
      if (loading || assets.failed) {
        ImGui::Text("Assets: %zu / %zu complete, %zu failed", assets.completed, assets.total, assets.failed);
        ImGui::TextWrapped("%s", assets.active_asset_name.c_str());
        ImGui::TextWrapped("%s", assets.message.c_str());
      }
      if (scanning) {
        ImGui::TextUnformatted("Scanning assets...");
        ImGui::TextWrapped("%s", project.loading_status_.c_str());
      }
      if (!error.empty()) {
        ImGui::Separator();
        ImGui::TextWrapped("%s", error.c_str());
        if (unacknowledged_error && ImGui::Button("Acknowledge"))
          acknowledged_background_error_ = error;
      }
      ImGui::EndPopup();
    }
  }
  ImGui::End();
  ImGui::PopStyleColor();
  ImGui::PopStyleVar(3);
  return height;
}
