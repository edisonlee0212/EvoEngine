#include "EditorLayer.hpp"
#include "Application.hpp"
#include "AssetManager.hpp"
#include "Cubemap.hpp"
#include "EditorTheme.hpp"
#include "EnvironmentalMap.hpp"
#include "ILayer.hpp"
#include "InspectorRegistry.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "PackageManager.hpp"
#include "Platform.hpp"
#include "PostProcessingStack.hpp"
#include "Prefab.hpp"
#include "ProjectContentBrowserPanel.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "SDKInspectionAdapters.hpp"
#include "Scene.hpp"
#include "Serialization.hpp"
#include "StrandsRenderer.hpp"
#include "Times.hpp"
#include "WindowLayer.hpp"

#include "imgui_internal.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <functional>
#include <unordered_map>

using namespace evo_engine;

namespace {
constexpr size_t kMaxRuntimePackageBuildOutputSize = 128 * 1024;
constexpr float kCustomTitleBarHeight = 57.0f;
constexpr float kTitleBarMenuY = 4.0f;
constexpr float kTitleBarTextY = 8.0f;
constexpr float kTitleBarLogoSize = 38.0f;
constexpr float kTitleBarLogoX = 10.0f;
constexpr float kTitleBarMenuX = 61.0f;
constexpr float kTitleBarButtonsAreaWidth = 94.0f;
constexpr float kTitleBarButtonSize = 14.0f;
constexpr float kTitleBarSearchMinWidth = 250.0f;
constexpr float kTitleBarSearchMaxWidth = 430.0f;
constexpr float kTitleBarSearchHeight = 28.0f;
constexpr size_t kTitleBarSearchMaxResults = 18;
constexpr ImU32 kTitleBarColor = IM_COL32(21, 21, 21, 255);
constexpr ImU32 kTitleBarMenuAccent = IM_COL32(236, 158, 36, 255);
constexpr ImU32 kTitleBarMenuPopupBg = IM_COL32(50, 50, 50, 255);
constexpr ImU32 kTitleBarMenuPopupBorder = IM_COL32(55, 55, 55, 255);
constexpr ImU32 kTitleBarMenuItemHovered = IM_COL32(0, 0, 0, 80);
constexpr ImU32 kTitleBarMenuActiveText = IM_COL32(26, 26, 26, 255);
constexpr ImU32 kTitleBarAccentPlaying = IM_COL32(18, 88, 30, 255);
constexpr ImU32 kTitleBarAccentPaused = IM_COL32(236, 158, 36, 255);
constexpr ImU32 kTitleBarText = IM_COL32(192, 192, 192, 255);
constexpr ImU32 kTitleBarTextDarker = IM_COL32(128, 128, 128, 255);
constexpr ImU32 kTitleBarMuted = IM_COL32(77, 77, 77, 255);
constexpr ImU32 kTitleBarSearchBg = IM_COL32(30, 30, 30, 255);
constexpr ImU32 kTitleBarSearchHovered = IM_COL32(38, 38, 38, 255);
constexpr ImU32 kTitleBarSearchActive = IM_COL32(44, 44, 44, 255);
constexpr ImU32 kTitleBarSearchBorder = IM_COL32(70, 70, 70, 255);
const std::array<std::string, 4> kCMakeConfigs = {"RelWithDebInfo", "Debug", "Release", "MinSizeRel"};

enum class TitleBarSearchResultType { Entity, Layer, Asset };

struct TitleBarSearchResult {
  TitleBarSearchResultType type = TitleBarSearchResultType::Entity;
  std::string label;
  std::string detail;
  Entity entity;
  Handle asset_handle = Handle(0);
  std::shared_ptr<ILayer> layer;
};

bool CurrentTitleBarAccent(ImU32& accent) {
  switch (ApplicationContext::Get().GetApplicationStatus()) {
    case Application::ExecutionStatus::Playing:
    case Application::ExecutionStatus::Step:
      accent = kTitleBarAccentPlaying;
      return true;
    case Application::ExecutionStatus::Pause:
      accent = kTitleBarAccentPaused;
      return true;
    case Application::ExecutionStatus::NotPlaying:
    case Application::ExecutionStatus::Uninitialized:
    case Application::ExecutionStatus::OnDestroy:
      return false;
  }
  return false;
}

bool TextContainsCaseInsensitive(const std::string& text, const std::string& query) {
  if (query.empty()) {
    return true;
  }
  return std::search(text.begin(), text.end(), query.begin(), query.end(),
                     [](const char text_char, const char query_char) {
                       return std::tolower(static_cast<unsigned char>(text_char)) ==
                              std::tolower(static_cast<unsigned char>(query_char));
                     }) != text.end();
}

const char* TitleBarSearchResultTypeName(const TitleBarSearchResultType type) {
  switch (type) {
    case TitleBarSearchResultType::Entity:
      return "Entity";
    case TitleBarSearchResultType::Layer:
      return "Layer";
    case TitleBarSearchResultType::Asset:
      return "Asset";
  }
  return "";
}

std::unordered_map<EditorLayer*, std::array<char, 128>>& TitleBarSearchBuffers() {
  static std::unordered_map<EditorLayer*, std::array<char, 128>> buffers;
  return buffers;
}

std::array<char, 128>& TitleBarSearchBuffer(EditorLayer* editor_layer) {
  return TitleBarSearchBuffers()[editor_layer];
}

class CallbackEditorPanel final : public EditorPanel {
 public:
  explicit CallbackEditorPanel(std::function<void(const std::shared_ptr<EditorLayer>&)> draw) : draw_(std::move(draw)) {
  }

  void Draw(const std::shared_ptr<EditorLayer>& editor_layer) override {
    draw_(editor_layer);
  }

 private:
  std::function<void(const std::shared_ptr<EditorLayer>&)> draw_;
};

struct RuntimePackageCMakeBuildRequest {
  std::string package_name;
  std::string target_name;
  std::filesystem::path build_dir;
  std::filesystem::path build_package_dir;
  std::filesystem::path runtime_package_dir;
  std::optional<std::string> config;
};

struct RuntimePackageCMakeBuildResult {
  bool success = false;
  int exit_code = -1;
  std::string command;
  std::string output;
  std::string error;
};

void DrawThemeMenuItems() {
  const auto current_theme = editor_theme::GetCurrentTheme();
  if (ImGui::MenuItem("Dark", nullptr, current_theme == editor_theme::Theme::Dark)) {
    editor_theme::Apply(editor_theme::Theme::Dark);
  }
  if (ImGui::MenuItem("Light", nullptr, current_theme == editor_theme::Theme::Light)) {
    editor_theme::Apply(editor_theme::Theme::Light);
  }
}

bool UsesLightBackground() {
  const auto background = ImGui::GetStyleColorVec4(ImGuiCol_WindowBg);
  return background.x * 0.299f + background.y * 0.587f + background.z * 0.114f > 0.5f;
}

ImVec4 WarningTextColor() {
  return UsesLightBackground() ? ImVec4(0.70f, 0.38f, 0.04f, 1.0f) : ImVec4(0.95f, 0.67f, 0.24f, 1.0f);
}

ImVec4 ErrorTextColor() {
  return UsesLightBackground() ? ImVec4(0.74f, 0.13f, 0.13f, 1.0f) : ImVec4(1.0f, 0.35f, 0.35f, 1.0f);
}

ImU32 MultiplyColor(const ImU32 color, const float multiplier) {
  ImVec4 value = ImGui::ColorConvertU32ToFloat4(color);
  value.x = std::clamp(value.x * multiplier, 0.0f, 1.0f);
  value.y = std::clamp(value.y * multiplier, 0.0f, 1.0f);
  value.z = std::clamp(value.z * multiplier, 0.0f, 1.0f);
  return ImGui::ColorConvertFloat4ToU32(value);
}

ImU32 ColorWithSaturation(const ImU32 color, const float saturation) {
  ImVec4 value = ImGui::ColorConvertU32ToFloat4(color);
  float hue;
  float current_saturation;
  float brightness;
  ImGui::ColorConvertRGBtoHSV(value.x, value.y, value.z, hue, current_saturation, brightness);
  ImGui::ColorConvertHSVtoRGB(hue, std::clamp(saturation, 0.0f, 1.0f), brightness, value.x, value.y, value.z);
  return ImGui::ColorConvertFloat4ToU32(value);
}

std::shared_ptr<Texture2D> FindIconInMap(const std::unordered_map<std::string, std::shared_ptr<Texture2D>>& icons,
                                         const std::string& name) {
  if (const auto search = icons.find(name); search != icons.end()) {
    return search->second;
  }
  return {};
}

void DrawFittedImage(ImDrawList* draw_list, const std::shared_ptr<Texture2D>& icon, const ImRect& rect,
                     const ImU32 tint) {
  if (!icon) {
    return;
  }
  const glm::uvec2 resolution = icon->GetResolution();
  if (resolution.x == 0 || resolution.y == 0) {
    return;
  }

  const ImVec2 bounds = rect.GetSize();
  const float scale =
      std::min(bounds.x / static_cast<float>(resolution.x), bounds.y / static_cast<float>(resolution.y));
  const ImVec2 size(static_cast<float>(resolution.x) * scale, static_cast<float>(resolution.y) * scale);
  const ImVec2 min(rect.Min.x + (bounds.x - size.x) * 0.5f, rect.Min.y + (bounds.y - size.y) * 0.5f);
  draw_list->AddImage(icon->GetImTextureId(), min, ImVec2(min.x + size.x, min.y + size.y), ImVec2(0, 1), ImVec2(1, 0),
                      tint);
}

void DrawFittedImage(const std::shared_ptr<Texture2D>& icon, const ImRect& rect, const ImU32 tint) {
  DrawFittedImage(ImGui::GetWindowDrawList(), icon, rect, tint);
}

bool DrawTitleBarImageButton(const char* id, const std::shared_ptr<Texture2D>& icon, const ImVec2 screen_position,
                             const bool close_button) {
  ImGui::SetCursorScreenPos(screen_position);
  const ImRect button_rect(screen_position,
                           ImVec2(screen_position.x + kTitleBarButtonSize, screen_position.y + kTitleBarButtonSize));
  const bool clicked = ImGui::InvisibleButton(id, ImVec2(kTitleBarButtonSize, kTitleBarButtonSize));
  ImU32 tint = close_button ? kTitleBarText : MultiplyColor(kTitleBarText, 0.9f);
  if (ImGui::IsItemActive()) {
    tint = kTitleBarTextDarker;
  } else if (ImGui::IsItemHovered()) {
    tint = close_button ? MultiplyColor(kTitleBarText, 1.4f) : MultiplyColor(kTitleBarText, 1.2f);
  }
  DrawFittedImage(icon, button_rect, tint);
  return clicked;
}

bool BeginTitleBarMenuBar(const ImRect& bar_rectangle) {
  ImGuiWindow* window = ImGui::GetCurrentWindow();
  if (window->SkipItems) {
    return false;
  }

  IM_ASSERT(!window->DC.MenuBarAppending);
  ImGui::BeginGroup();
  ImGui::PushID("##titlebar_menu");

  const ImRect bar_rect(ImVec2(bar_rectangle.Min.x, bar_rectangle.Min.y + window->WindowPadding.y),
                        ImVec2(bar_rectangle.Max.x, bar_rectangle.Max.y + window->WindowPadding.y));
  ImRect clip_rect(IM_ROUND(window->Pos.x + bar_rect.Min.x), IM_ROUND(window->Pos.y + bar_rect.Min.y),
                   IM_ROUND(window->Pos.x + bar_rect.Max.x), IM_ROUND(window->Pos.y + bar_rect.Max.y));
  clip_rect.ClipWith(window->OuterRectClipped);
  ImGui::PushClipRect(clip_rect.Min, clip_rect.Max, false);

  window->DC.CursorPos = window->DC.CursorMaxPos =
      ImVec2(window->Pos.x + bar_rect.Min.x, window->Pos.y + bar_rect.Min.y);
  window->DC.LayoutType = ImGuiLayoutType_Horizontal;
  window->DC.NavLayerCurrent = ImGuiNavLayer_Menu;
  window->DC.MenuBarAppending = true;
  ImGui::AlignTextToFramePadding();
  return true;
}

void EndTitleBarMenuBar() {
  ImGuiWindow* window = ImGui::GetCurrentWindow();
  if (window->SkipItems) {
    return;
  }

  IM_ASSERT(window->DC.MenuBarAppending);
  ImGui::PopClipRect();
  ImGui::PopID();
  window->DC.MenuBarOffset.x = window->DC.CursorPos.x - window->Pos.x;
  GImGui->GroupStack.back().EmitItem = false;
  ImGui::EndGroup();
  window->DC.LayoutType = ImGuiLayoutType_Vertical;
  window->DC.NavLayerCurrent = ImGuiNavLayer_Main;
  window->DC.MenuBarAppending = false;
}

void PushTitleBarMenuStyle() {
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f, 5.0f));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
  ImGui::PushStyleVar(ImGuiStyleVar_PopupRounding, 4.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_PopupBorderSize, 1.0f);
  ImGui::PushStyleColor(ImGuiCol_PopupBg, ImGui::ColorConvertU32ToFloat4(kTitleBarMenuPopupBg));
  ImGui::PushStyleColor(ImGuiCol_Border, ImGui::ColorConvertU32ToFloat4(kTitleBarMenuPopupBorder));
}

void PopTitleBarMenuStyle() {
  ImGui::PopStyleColor(2);
  ImGui::PopStyleVar(5);
}

void PushTitleBarMenuActiveHighlight() {
  const ImU32 active_color = ColorWithSaturation(kTitleBarMenuAccent, 0.5f);
  ImGui::PushStyleColor(ImGuiCol_Header, ImGui::ColorConvertU32ToFloat4(active_color));
  ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImGui::ColorConvertU32ToFloat4(active_color));
  ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImGui::ColorConvertU32ToFloat4(active_color));
}

bool BeginTitleBarMenu(const char* label, bool& menu_open) {
  bool pushed_active_text = false;
  if (menu_open && ImGui::IsPopupOpen(label)) {
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(kTitleBarMenuActiveText));
    pushed_active_text = true;
  }

  if (ImGui::BeginMenu(label)) {
    if (menu_open) {
      ImGui::PopStyleColor(pushed_active_text ? 4 : 3);
      menu_open = false;
      pushed_active_text = false;
    }
    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImGui::ColorConvertU32ToFloat4(kTitleBarMenuItemHovered));
    ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImGui::ColorConvertU32ToFloat4(kTitleBarMenuItemHovered));
    return true;
  }

  if (pushed_active_text) {
    ImGui::PopStyleColor();
  }
  return false;
}

void EndTitleBarMenu() {
  ImGui::PopStyleColor(2);
  ImGui::EndMenu();
}

bool DrawToolbarImageButton(const char* id, const std::shared_ptr<Texture2D>& icon, const char* tooltip) {
  constexpr ImVec2 button_size(23.0f, 23.0f);
  const ImVec2 button_min = ImGui::GetCursorScreenPos();
  const ImRect button_rect(button_min, ImVec2(button_min.x + button_size.x, button_min.y + button_size.y));
  const bool clicked = ImGui::InvisibleButton(id, button_size);
  const ImU32 tint = ImGui::IsItemActive() ? kTitleBarTextDarker : kTitleBarText;
  DrawFittedImage(ImGui::GetWindowDrawList(), icon, button_rect, tint);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("%s", tooltip);
  }
  return clicked;
}

int PlaybackControlCount(const Application::ExecutionStatus status) {
  switch (status) {
    case Application::ExecutionStatus::NotPlaying:
    case Application::ExecutionStatus::Playing:
      return 2;
    case Application::ExecutionStatus::Pause:
      return 3;
    case Application::ExecutionStatus::Uninitialized:
    case Application::ExecutionStatus::Step:
    case Application::ExecutionStatus::OnDestroy:
      return 0;
  }
  return 0;
}

void AcceptEntityExplorerRootDrop(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return;
  }

  const ImVec2 drop_min = ImGui::GetCursorScreenPos();
  const ImVec2 window_pos = ImGui::GetWindowPos();
  const ImVec2 content_max = ImGui::GetWindowContentRegionMax();
  const ImVec2 drop_max(window_pos.x + content_max.x, window_pos.y + content_max.y);
  if (drop_max.x <= drop_min.x || drop_max.y <= drop_min.y) {
    return;
  }

  if (ImGui::BeginDragDropTargetCustom(ImRect(drop_min, drop_max), ImGui::GetID("##EntityExplorerRootDropTarget"))) {
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      const auto entity = scene->GetEntity(*static_cast<Handle*>(payload->Data));
      if (scene->IsEntityValid(entity)) {
        if (const auto parent = scene->GetParent(entity); parent.GetIndex() != 0) {
          scene->RemoveChild(entity, parent);
        }
      }
    }
    ImGui::EndDragDropTarget();
  }
}

void AppendCappedOutput(std::string& output, const char* data, const size_t size) {
  if (size >= kMaxRuntimePackageBuildOutputSize) {
    output.assign(data + size - kMaxRuntimePackageBuildOutputSize, kMaxRuntimePackageBuildOutputSize);
    return;
  }
  if (output.size() + size > kMaxRuntimePackageBuildOutputSize) {
    output.erase(0, output.size() + size - kMaxRuntimePackageBuildOutputSize);
  }
  output.append(data, size);
}

std::string QuoteCommandArgument(const std::string& value) {
  std::string quoted = "\"";
  for (const char c : value) {
    if (c == '"') {
      quoted += '\\';
    }
    quoted += c;
  }
  quoted += '"';
  return quoted;
}

std::string BuildCMakeCommandText(const RuntimePackageCMakeBuildRequest& request) {
  auto command = "cmake --build " + QuoteCommandArgument(request.build_dir.string());
  if (request.config.has_value()) {
    command += " --config " + QuoteCommandArgument(*request.config);
  }
  command += " --target " + QuoteCommandArgument(request.target_name);
  return command;
}

#ifdef EVOENGINE_WINDOWS
std::wstring QuoteWindowsCommandArgument(const std::wstring& value) {
  std::wstring quoted = L"\"";
  size_t backslash_count = 0;
  for (const wchar_t c : value) {
    if (c == L'\\') {
      ++backslash_count;
      continue;
    }
    if (c == L'"') {
      quoted.append(backslash_count * 2 + 1, L'\\');
      quoted += c;
      backslash_count = 0;
      continue;
    }
    quoted.append(backslash_count, L'\\');
    backslash_count = 0;
    quoted += c;
  }
  quoted.append(backslash_count * 2, L'\\');
  quoted += L"\"";
  return quoted;
}
std::wstring BuildWindowsCMakeCommandLine(const RuntimePackageCMakeBuildRequest& request) {
  auto command = L"cmake --build " + QuoteWindowsCommandArgument(request.build_dir.wstring());
  if (request.config.has_value()) {
    command +=
        L" --config " + QuoteWindowsCommandArgument(std::wstring(request.config->begin(), request.config->end()));
  }
  command +=
      L" --target " + QuoteWindowsCommandArgument(std::wstring(request.target_name.begin(), request.target_name.end()));
  return command;
}
#else
std::string QuoteShellCommandArgument(const std::string& value) {
  std::string quoted = "'";
  for (const char c : value) {
    if (c == '\'') {
      quoted += "'\\''";
    } else {
      quoted += c;
    }
  }
  quoted += "'";
  return quoted;
}

std::string BuildShellCMakeCommandLine(const RuntimePackageCMakeBuildRequest& request) {
  auto command = "cmake --build " + QuoteShellCommandArgument(request.build_dir.string());
  if (request.config.has_value()) {
    command += " --config " + QuoteShellCommandArgument(*request.config);
  }
  command += " --target " + QuoteShellCommandArgument(request.target_name) + " 2>&1";
  return command;
}
#endif

std::optional<RuntimePackageCMakeBuildRequest> CreateRuntimePackageBuildRequest(
    const std::string& package_name, const std::filesystem::path& package_runtime_path, std::string& error) {
  if (package_runtime_path.empty()) {
    error = "Package runtime path is empty.";
    return {};
  }

  const auto package_directory = package_runtime_path.parent_path();
  if (package_directory.filename() != "Packages") {
    error = "Build is only available for packages in a build-tree Packages directory.";
    return {};
  }

  RuntimePackageCMakeBuildRequest request;
  request.package_name = package_name;
  request.target_name = package_name + "Package";
  request.runtime_package_dir = package_directory;
  const auto app_or_config_dir = package_directory.parent_path();
  if (app_or_config_dir.filename() == "EvoEngine_App") {
    request.build_dir = app_or_config_dir.parent_path();
    request.build_package_dir = package_directory;
  } else {
    const auto app_dir = app_or_config_dir.parent_path();
    if (app_dir.filename() == "EvoEngine_App") {
      request.config = app_or_config_dir.filename().string();
      request.build_dir = app_dir.parent_path();
      request.build_package_dir = package_directory;
    } else {
      const auto bin_dir = package_directory.parent_path();
      const auto install_preset_dir = bin_dir.parent_path();
      const auto install_dir = install_preset_dir.parent_path();
      const auto out_dir = install_dir.parent_path();
      if (bin_dir.filename() != "bin" || install_dir.filename() != "install" || out_dir.filename() != "out") {
        error =
            "Build is only available for build-tree packages under EvoEngine_App/Packages or "
            "EvoEngine_App/<Config>/Packages, or installed packages under out/install/<preset>/bin/Packages.";
        return {};
      }
      request.build_dir = out_dir / "build" / install_preset_dir.filename();
      std::filesystem::file_time_type newest_time{};
      bool found_package_output = false;
      const auto build_tree_packages = request.build_dir / "EvoEngine_App" / "Packages";
      if (std::filesystem::exists(build_tree_packages / (package_name + ".evepackage"))) {
        request.build_package_dir = build_tree_packages;
        found_package_output = true;
      }
      for (const auto& config : kCMakeConfigs) {
        const auto package_dir = request.build_dir / "EvoEngine_App" / config / "Packages";
        const auto manifest_path = package_dir / (package_name + ".evepackage");
        if (!std::filesystem::exists(manifest_path)) {
          continue;
        }
        std::error_code time_ec;
        const auto write_time = std::filesystem::last_write_time(manifest_path, time_ec);
        if (!found_package_output || (!time_ec && write_time > newest_time)) {
          newest_time = write_time;
          request.config = config;
          request.build_package_dir = package_dir;
          found_package_output = true;
        }
      }
      if (!found_package_output) {
        error = "No matching build-tree package output was found under " + request.build_dir.string() + ".";
        return {};
      }
    }
  }
  std::error_code ec;
  if (!std::filesystem::exists(request.build_dir / "CMakeCache.txt", ec)) {
    error = "CMakeCache.txt was not found in " + request.build_dir.string() + ".";
    return {};
  }
  return request;
}

std::string ReadPackageLibraryName(const std::filesystem::path& manifest_path) {
  std::ifstream manifest_file(manifest_path);
  std::string line;
  while (std::getline(manifest_file, line)) {
    constexpr const char* prefix = "library:";
    constexpr size_t prefix_length = 8;
    if (line.rfind(prefix, 0) != 0) {
      continue;
    }
    auto library_name = line.substr(prefix_length);
    const auto first = library_name.find_first_not_of(" \t");
    if (first == std::string::npos) {
      return {};
    }
    const auto last = library_name.find_last_not_of(" \t");
    return library_name.substr(first, last - first + 1);
  }
  return {};
}

bool CopyPackageBuildOutputToRuntimeDir(const RuntimePackageCMakeBuildRequest& request, std::string& error) {
  if (request.build_package_dir == request.runtime_package_dir) {
    return true;
  }

  std::error_code ec;
  std::filesystem::create_directories(request.runtime_package_dir, ec);
  if (ec) {
    error = "Failed to create runtime package directory: " + ec.message();
    return false;
  }

  const auto source_manifest = request.build_package_dir / (request.package_name + ".evepackage");
  const auto target_manifest = request.runtime_package_dir / source_manifest.filename();
  std::filesystem::copy_file(source_manifest, target_manifest, std::filesystem::copy_options::overwrite_existing, ec);
  if (ec) {
    error = "Failed to copy package manifest: " + ec.message();
    return false;
  }

  const auto library_name = ReadPackageLibraryName(source_manifest);
  if (library_name.empty()) {
    error = "Failed to read package library name from " + source_manifest.string() + ".";
    return false;
  }

  const auto source_library = request.build_package_dir / library_name;
  const auto target_library = request.runtime_package_dir / library_name;
  std::filesystem::copy_file(source_library, target_library, std::filesystem::copy_options::overwrite_existing, ec);
  if (ec) {
    error = "Failed to copy package library: " + ec.message();
    return false;
  }

  const auto source_debug_symbols = source_library.parent_path() / (source_library.stem().string() + ".pdb");
  if (std::filesystem::exists(source_debug_symbols, ec)) {
    const auto target_debug_symbols = target_library.parent_path() / source_debug_symbols.filename();
    std::filesystem::copy_file(source_debug_symbols, target_debug_symbols,
                               std::filesystem::copy_options::overwrite_existing, ec);
    if (ec) {
      error = "Failed to copy package debug symbols: " + ec.message();
      return false;
    }
  }
  return true;
}

void BuildDefaultEditorDockLayout(const ImGuiID dock_space_id, const ImVec2& dock_size) {
  ImGui::DockBuilderRemoveNode(dock_space_id);
  ImGui::DockBuilderAddNode(dock_space_id, ImGuiDockNodeFlags_DockSpace);
  ImGui::DockBuilderSetNodeSize(dock_space_id, dock_size);

  ImGuiID center_node = dock_space_id;
  const ImGuiID left_node = ImGui::DockBuilderSplitNode(center_node, ImGuiDir_Left, 0.22f, nullptr, &center_node);
  const ImGuiID right_node = ImGui::DockBuilderSplitNode(center_node, ImGuiDir_Right, 0.28f, nullptr, &center_node);
  const ImGuiID bottom_node = ImGui::DockBuilderSplitNode(center_node, ImGuiDir_Down, 0.30f, nullptr, &center_node);

  ImGui::DockBuilderDockWindow("Scene", center_node);
  ImGui::DockBuilderDockWindow("Camera", center_node);
  ImGui::DockBuilderDockWindow("Entity Explorer", left_node);
  ImGui::DockBuilderDockWindow("Entity Inspector", right_node);
  ImGui::DockBuilderDockWindow("Scene Camera Debug", right_node);
  ImGui::DockBuilderDockWindow("Project", bottom_node);
  ImGui::DockBuilderDockWindow("Console", bottom_node);
  ImGui::DockBuilderDockWindow("Resources", bottom_node);
  ImGui::DockBuilderDockWindow("Runtime Package Manager", bottom_node);
  ImGui::DockBuilderFinish(dock_space_id);
}

RuntimePackageCMakeBuildResult RunRuntimePackageBuild(const RuntimePackageCMakeBuildRequest& request) {
  RuntimePackageCMakeBuildResult result;
  result.command = BuildCMakeCommandText(request);

#ifdef EVOENGINE_WINDOWS
  SECURITY_ATTRIBUTES security_attributes{};
  security_attributes.nLength = sizeof(SECURITY_ATTRIBUTES);
  security_attributes.bInheritHandle = TRUE;

  HANDLE output_read = nullptr;
  HANDLE output_write = nullptr;
  if (!CreatePipe(&output_read, &output_write, &security_attributes, 0)) {
    result.error = "Failed to create package build output pipe.";
    return result;
  }
  SetHandleInformation(output_read, HANDLE_FLAG_INHERIT, 0);

  STARTUPINFOW startup_info{};
  startup_info.cb = sizeof(startup_info);
  startup_info.dwFlags = STARTF_USESTDHANDLES;
  startup_info.hStdOutput = output_write;
  startup_info.hStdError = output_write;
  startup_info.hStdInput = GetStdHandle(STD_INPUT_HANDLE);

  PROCESS_INFORMATION process_info{};
  auto command_line_text = BuildWindowsCMakeCommandLine(request);
  std::vector<wchar_t> command_line(command_line_text.begin(), command_line_text.end());
  command_line.emplace_back(L'\0');
  const auto working_directory = request.build_dir.wstring();
  if (!CreateProcessW(nullptr, command_line.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr,
                      working_directory.c_str(), &startup_info, &process_info)) {
    CloseHandle(output_read);
    CloseHandle(output_write);
    result.error = "Failed to launch cmake.";
    return result;
  }
  CloseHandle(output_write);

  std::array<char, 4096> buffer{};
  DWORD bytes_read = 0;
  while (ReadFile(output_read, buffer.data(), static_cast<DWORD>(buffer.size()), &bytes_read, nullptr) &&
         bytes_read > 0) {
    AppendCappedOutput(result.output, buffer.data(), bytes_read);
  }
  CloseHandle(output_read);

  WaitForSingleObject(process_info.hProcess, INFINITE);
  DWORD exit_code = 1;
  GetExitCodeProcess(process_info.hProcess, &exit_code);
  CloseHandle(process_info.hProcess);
  CloseHandle(process_info.hThread);
  result.exit_code = static_cast<int>(exit_code);
#else
  const auto command_line = BuildShellCMakeCommandLine(request);
  auto* pipe = popen(command_line.c_str(), "r");
  if (!pipe) {
    result.error = "Failed to launch cmake.";
    return result;
  }
  std::array<char, 4096> buffer{};
  while (const auto bytes_read = std::fread(buffer.data(), 1, buffer.size(), pipe)) {
    AppendCappedOutput(result.output, buffer.data(), bytes_read);
  }
  result.exit_code = pclose(pipe);
#endif

  result.success = result.exit_code == 0;
  if (result.success) {
    std::string copy_error;
    result.success = CopyPackageBuildOutputToRuntimeDir(request, copy_error);
    if (!copy_error.empty()) {
      result.error = copy_error;
    }
  }
  return result;
}
}  // namespace

void EditorLayer::OnCreate() {
  enable_inspection = false;
  const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>();
  if (!window_layer) {
    throw std::runtime_error("WindowLayer not present!");
  }
  editor_theme::ApplyDefault();

  basic_entity_archetype_ = Entities::CreateEntityArchetype("General", GlobalTransform(), Transform());
  RegisterComponentDataInspector<GlobalTransform>([](Entity, IDataComponent* data, bool) {
    const auto* ltw = reinterpret_cast<GlobalTransform*>(data);
    glm::vec3 er;
    glm::vec3 t;
    glm::vec3 s;
    ltw->Decompose(t, er, s);
    er = glm::degrees(er);
    ImGui::DragFloat3("Position##Global", &t.x, 0.1f, 0, 0, "%.3f", ImGuiSliderFlags_ReadOnly);
    ImGui::DragFloat3("Rotation##Global", &er.x, 0.1f, 0, 0, "%.3f", ImGuiSliderFlags_ReadOnly);
    ImGui::DragFloat3("Scale##Global", &s.x, 0.1f, 0, 0, "%.3f", ImGuiSliderFlags_ReadOnly);
    return false;
  });
  RegisterComponentDataInspector<Transform>([&](const Entity entity, IDataComponent* data, bool) {
    auto* ltp = static_cast<Transform*>(static_cast<void*>(data));
    bool edited = false;
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto status = scene->GetDataComponent<TransformUpdateFlag>(entity);
    const bool reload = previous_transform_inspection_entity_ != entity ||
                        previously_stored_transform_.value != ltp->value || status.transform_modified ||
                        status.global_transform_modified;
    if (reload) {
      previous_transform_inspection_entity_ = entity;
      ltp->Decompose(previously_stored_position_, previously_stored_rotation_, previously_stored_scale_);
      previously_stored_rotation_ = glm::degrees(previously_stored_rotation_);
      // local_position_selected_ = true;
      // local_rotation_selected_ = false;
      // local_scale_selected_ = false;

      previously_stored_transform_ = *ltp;
    }
    if (ImGui::DragFloat3("##LocalPosition", &previously_stored_position_.x, 0.01f, 0, 0, "%.3f",
                          reload ? ImGuiSliderFlags_ReadOnly : 0))
      edited = true;
    ImGui::SameLine();
    if (ImGui::Selectable("Position##Local", &local_position_selected_) && local_position_selected_) {
      local_rotation_selected_ = false;
      local_scale_selected_ = false;
    }
    if (ImGui::DragFloat3("##LocalRotation", &previously_stored_rotation_.x, 1.0f, 0, 0, "%.3f",
                          reload ? ImGuiSliderFlags_ReadOnly : 0))
      edited = true;
    ImGui::SameLine();
    if (ImGui::Selectable("Rotation##Local", &local_rotation_selected_) && local_rotation_selected_) {
      local_position_selected_ = false;
      local_scale_selected_ = false;
    }
    if (ImGui::DragFloat3("##LocalScale", &previously_stored_scale_.x, 0.01f, 0, 0, "%.3f",
                          reload ? ImGuiSliderFlags_ReadOnly : 0))
      edited = true;
    ImGui::SameLine();
    if (ImGui::Selectable("Scale##Local", &local_scale_selected_) && local_scale_selected_) {
      local_rotation_selected_ = false;
      local_position_selected_ = false;
    }
    if (edited) {
      ltp->value = glm::translate(previously_stored_position_) *
                   glm::mat4_cast(glm::quat(glm::radians(previously_stored_rotation_))) *
                   glm::scale(previously_stored_scale_);
      previously_stored_transform_ = *ltp;
    }
    transform_read_only = false;
    return edited;
  });

  RegisterComponentDataInspector<Ray>([&](Entity, IDataComponent* data, bool) {
    auto* ray = static_cast<Ray*>(static_cast<void*>(data));
    bool changed = false;
    if (ImGui::InputFloat3("Start", &ray->start.x))
      changed = true;
    if (ImGui::InputFloat3("Direction", &ray->direction.x))
      changed = true;
    if (ImGui::InputFloat("Length", &ray->length))
      changed = true;
    return changed;
  });

  LoadIcons();
  RegisterEditorPanels();

  VkBufferCreateInfo entity_index_read_buffer{};
  entity_index_read_buffer.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  switch (Platform::Constants::texture_2d) {
    case VK_FORMAT_R32G32B32A32_SFLOAT: {
      entity_index_read_buffer.size = sizeof(float) * 4;
      break;
    }
    case VK_FORMAT_R16G16B16A16_SFLOAT: {
      entity_index_read_buffer.size = sizeof(glm::detail::hdata) * 4;
      break;
    }
  }

  entity_index_read_buffer.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  entity_index_read_buffer.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo entity_index_read_buffer_create_info{};
  entity_index_read_buffer_create_info.usage = VMA_MEMORY_USAGE_AUTO;
  entity_index_read_buffer_create_info.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
  entity_index_read_buffer_ = std::make_unique<Buffer>(entity_index_read_buffer, entity_index_read_buffer_create_info);
  vmaMapMemory(Platform::GetVmaAllocator(), entity_index_read_buffer_->GetVmaAllocation(),
               static_cast<void**>(static_cast<void*>(&mapped_entity_index_data_)));

  const auto scene_camera = Serialization::ProduceSerializable<Camera>();
  scene_camera->camera_settings.clear_color = glm::vec4(59.0f / 255.0f, 85 / 255.0f, 143 / 255.f, 1.f);
  scene_camera->camera_settings.use_clear_color = false;
  scene_camera->OnCreate();
  scene_camera->post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
  RegisterEditorCamera(scene_camera);
  scene_camera_handle_ = scene_camera->GetHandle();
  auto& editor_camera = editor_cameras_[scene_camera_handle_];
  editor_camera.position = default_scene_camera_position;
  editor_camera.rotation = default_scene_camera_rotation;
}

void EditorLayer::OnDestroy() {
  TitleBarSearchBuffers().erase(this);
  if (ImGui::GetCurrentContext() && ImGui::GetFrameCount() > 0) {
    const auto* ini_filename = ImGui::GetIO().IniFilename;
    if (ini_filename) {
      ImGui::SaveIniSettingsToDisk(ini_filename);
    }
  }
  editor_panel_manager_.UnregisterSettingsHandler();
  editor_cameras_.clear();
  gizmo_mesh_tasks_.clear();
  gizmo_instanced_mesh_tasks_.clear();
  gizmo_strands_tasks_.clear();
  vmaUnmapMemory(Platform::GetVmaAllocator(), entity_index_read_buffer_->GetVmaAllocation());
  entity_index_read_buffer_.reset();
}

void EditorLayer::PreUpdate() {
  const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>();
  const bool use_custom_title_bar = window_layer && window_layer->UsesCustomTitleBar();
  DrawDockspace(use_custom_title_bar ? kCustomTitleBarHeight : 0.0f);
  if (use_custom_title_bar) {
    DrawCustomTitleBar();
  } else {
    DrawMainMenuBar();
  }

  const auto scene = ApplicationContext::Get().GetActiveScene();
  UpdateCameraTransition();
  PrepareFrameState();
  CaptureSceneWindowMousePosition();
  CaptureMainCameraWindowMousePosition();
  UpdateSceneState(scene);

  const auto editor_layer = std::dynamic_pointer_cast<EditorLayer>(GetSelf());
  HandleSceneDeleteShortcut(scene);
  editor_panel_manager_.Draw(EditorPanelCategory::View, editor_layer);
  DrawAssetInspectorWindows();
  DrawLayerInspectionWindows(scene, editor_layer);
}

void EditorLayer::OpenAssetInspector(const std::shared_ptr<IAsset>& asset) {
  if (!asset || asset->GetTypeName() == "Binary") {
    return;
  }

  const auto asset_handle = asset->GetHandle().GetValue();
  for (auto& inspector_window : inspecting_assets_) {
    if (inspector_window.asset && inspector_window.asset->GetHandle().GetValue() == asset_handle) {
      inspector_window.asset = asset;
      inspector_window.open = true;
      inspector_window.focus_requested = true;
      return;
    }
  }
  inspecting_assets_.push_back({asset, true, true});
}

void EditorLayer::ClearAssetInspectors() {
  inspecting_assets_.clear();
}

void EditorLayer::DrawAssetInspectorWindows() {
  const auto editor_layer = std::dynamic_pointer_cast<EditorLayer>(GetSelf());
  for (size_t i = 0; i < inspecting_assets_.size(); ++i) {
    const auto asset = inspecting_assets_[i].asset;
    if (!asset) {
      inspecting_assets_[i].open = false;
      continue;
    }

    const auto asset_handle = asset->GetHandle().GetValue();
    bool open = inspecting_assets_[i].open;
    if (inspecting_assets_[i].focus_requested) {
      ImGui::SetNextWindowFocus();
      inspecting_assets_[i].focus_requested = false;
    }

    const auto window_title =
        "Asset Inspector - " + asset->GetTitle() + "###AssetInspector_" + std::to_string(asset_handle);
    if (ImGui::Begin(window_title.c_str(), &open)) {
      AssetManager::DrawAssetInspectorContent(editor_layer, asset);
    }
    ImGui::End();

    if (i < inspecting_assets_.size() && inspecting_assets_[i].asset &&
        inspecting_assets_[i].asset->GetHandle().GetValue() == asset_handle) {
      inspecting_assets_[i].open = open;
    }
  }

  inspecting_assets_.erase(std::remove_if(inspecting_assets_.begin(), inspecting_assets_.end(),
                                          [](const AssetInspectorWindow& inspector) {
                                            return !inspector.open || !inspector.asset;
                                          }),
                           inspecting_assets_.end());
}

void EditorLayer::RegisterEditorPanels() {
  editor_panel_manager_.Clear();

  auto register_panel = [this](const std::string& id, const std::string& title, bool& open,
                               std::function<void(const std::shared_ptr<EditorLayer>&)> draw) {
    editor_panel_manager_.RegisterPanel(EditorPanelCategory::View, id, title, open,
                                        std::make_shared<CallbackEditorPanel>(std::move(draw)));
  };

  register_panel("scene_window", "Scene Window", show_scene_window, [this](const std::shared_ptr<EditorLayer>&) {
    SceneCameraWindow();
  });
  register_panel("main_camera_window", "Main Camera Window", show_camera_window,
                 [this](const std::shared_ptr<EditorLayer>&) {
                   MainCameraWindow();
                 });
  register_panel("scene_camera_debug", "Scene Camera Debug", show_scene_camera_debug,
                 [this](const std::shared_ptr<EditorLayer>&) {
                   DrawSceneCameraDebugWindow(ApplicationContext::Get().GetActiveScene());
                 });
  register_panel("entity_explorer", "Entity Explorer", show_entity_explorer_window,
                 [this](const std::shared_ptr<EditorLayer>&) {
                   DrawEntityExplorerWindow(ApplicationContext::Get().GetActiveScene());
                 });
  register_panel("entity_inspector", "Entity Inspector", show_entity_inspector_window,
                 [this](const std::shared_ptr<EditorLayer>& editor_layer) {
                   DrawEntityInspectorWindow(ApplicationContext::Get().GetActiveScene(), editor_layer);
                 });
  register_panel("console", "Console", show_console_window, [this](const std::shared_ptr<EditorLayer>&) {
    DrawConsoleWindow();
  });
  register_panel("runtime_packages", "Runtime Packages", show_package_manager_window,
                 [this](const std::shared_ptr<EditorLayer>&) {
                   DrawRuntimePackageManagerWindow();
                 });
  register_panel("resources", "Resources", Resources::GetInstance().show_resources_,
                 [](const std::shared_ptr<EditorLayer>& editor_layer) {
                   Resources::Draw(editor_layer);
                 });
  editor_panel_manager_.RegisterPanel(EditorPanelCategory::View, "project", "Project",
                                      ProjectManager::GetInstance().show_project_window,
                                      std::make_shared<ProjectContentBrowserPanel>());
  editor_panel_manager_.RegisterSettingsHandler();
}

void EditorLayer::UpdateCameraTransition() {
  if (!lock_camera) {
    return;
  }
  auto& [sceneCameraRotation, sceneCameraPosition, sceneCamera] = editor_cameras_.at(scene_camera_handle_);
  const float elapsed_time = static_cast<float>(ApplicationContext::Get().GetTimes().Now()) - transition_timer_;
  float a = 1.0f - glm::pow(1.0 - elapsed_time / transition_time_, 4.0f);
  if (elapsed_time >= transition_time_)
    a = 1.0f;
  sceneCameraRotation = glm::mix(previous_rotation_, target_rotation_, a);
  sceneCameraPosition = glm::mix(previous_position_, target_position_, a);
  if (a >= 1.0f) {
    lock_camera = false;
    sceneCameraRotation = target_rotation_;
    sceneCameraPosition = target_position_;
    // Camera::ReverseAngle(target_rotation_, m_sceneCameraPitchAngle, m_sceneCameraYawAngle);
  }
}

void EditorLayer::PrepareFrameState() {
  gizmo_mesh_tasks_.clear();
  gizmo_instanced_mesh_tasks_.clear();
  gizmo_strands_tasks_.clear();
  main_camera_focus_override = false;
  scene_camera_focus_override = false;
}

void EditorLayer::CaptureSceneWindowMousePosition() {
  mouse_scene_window_position_ = glm::vec2(FLT_MAX, -FLT_MAX);
  if (show_scene_window) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
    if (ImGui::Begin("Scene")) {
      if (ImGui::BeginChild("SceneCameraRenderer", ImVec2(0, 0), false)) {
        // Using a Child allow to fill all the space of the window.
        // It also allows customization
        if (scene_camera_window_focused_) {
          const auto mp = ImGui::GetMousePos();
          const auto wp = ImGui::GetWindowPos();
          mouse_scene_window_position_ = glm::vec2(mp.x - wp.x, mp.y - wp.y);
        }
      }
      ImGui::EndChild();
    }
    ImGui::End();
    ImGui::PopStyleVar();
  }
}

void EditorLayer::CaptureMainCameraWindowMousePosition() {
  mouse_camera_window_position_ = glm::vec2(FLT_MAX, -FLT_MAX);
  if (show_camera_window) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
    if (ImGui::Begin("Camera")) {
      if (ImGui::BeginChild("MainCameraRenderer", ImVec2(0, 0), false)) {
        // Using a Child allow to fill all the space of the window.
        // It also allows customization
        if (main_camera_window_focused_) {
          auto mp = ImGui::GetMousePos();
          auto wp = ImGui::GetWindowPos();
          mouse_camera_window_position_ = glm::vec2(mp.x - wp.x, mp.y - wp.y);
        }
      }
      ImGui::EndChild();
    }
    ImGui::End();
    ImGui::PopStyleVar();
  }
}

void EditorLayer::UpdateSceneState(const std::shared_ptr<Scene>& scene) {
  if (scene && show_scene_window)
    ResizeCameras();

  if (scene && !main_camera_window_focused_) {
    auto& pressed_keys = scene->pressed_keys_;
    pressed_keys.clear();
  }

  if (scene && apply_transform_to_main_camera && !ApplicationContext::Get().IsPlaying()) {
    if (const auto camera = scene->main_camera.Get<Camera>(); camera && scene->IsEntityValid(camera->GetOwner())) {
      auto& [sceneCameraRotation, sceneCameraPosition, sceneCamera] = editor_cameras_.at(scene_camera_handle_);
      GlobalTransform global_transform;
      global_transform.SetPosition(sceneCameraPosition);
      global_transform.SetRotation(sceneCameraRotation);
      scene->SetDataComponent(camera->GetOwner(), global_transform);
    }
  }

  if (scene && !scene->IsEntityValid(selected_entity_)) {
    SetSelectedEntity(Entity());
  }
  if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
      render_layer && render_layer->need_fade_ != 0 && selection_alpha_ < 256) {
    selection_alpha_ += static_cast<int>(static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime()) * 1280);
  }

  selection_alpha_ = glm::clamp(selection_alpha_, 0, 256);
}

void EditorLayer::DrawEntityExplorerWindow(const std::shared_ptr<Scene>& scene) {
  if (!show_entity_explorer_window) {
    return;
  }
  ImGui::Begin("Entity Explorer");
  if (scene) {
    if (ImGui::BeginPopupContextWindow("NewEntityPopup")) {
      if (ImGui::Button("Create new entity")) {
        scene->CreateEntity(basic_entity_archetype_);
      }
      ImGui::EndPopup();
    }
    const char* hierarchy_display_mode[]{"Archetype", "Hierarchy"};

    ImGui::Combo("Display mode", &selected_hierarchy_display_mode, hierarchy_display_mode,
                 IM_ARRAYSIZE(hierarchy_display_mode));
    if (selected_hierarchy_display_mode == 0) {
      scene->UnsafeForEachEntityStorage([&](size_t i, const std::string& name, const DataComponentStorage& storage) {
        if (i == 0)
          return;
        ImGui::Separator();
        const std::string title1 = std::to_string(i) + ". " + name;
        if (ImGui::TreeNode(title1.c_str())) {
          for (size_t j = 0; j < storage.entity_alive_count; j++) {
            Entity entity = storage.chunk_array.entity_array.at(j);
            std::string title2 = std::to_string(entity.GetIndex()) + ": ";
            title2 += scene->GetEntityName(entity);
            const bool enabled = scene->IsEntityEnabled(entity);
            if (enabled) {
              ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_Text));
            }
            ImGui::TreeNodeEx(
                title2.c_str(),
                ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoAutoOpenOnLog |
                    (selected_entity_ == entity ? ImGuiTreeNodeFlags_Framed : ImGuiTreeNodeFlags_FramePadding));
            if (enabled) {
              ImGui::PopStyleColor();
            }
            DrawEntityMenu(enabled, entity);
            if (!lock_entity_selection_ && ImGui::IsItemHovered() && ImGui::IsMouseClicked(0)) {
              SetSelectedEntity(entity, false);
            }
          }
          ImGui::TreePop();
        }
      });
    } else if (selected_hierarchy_display_mode == 1) {
      scene->ForAllEntities([&](size_t, const Entity entity) {
        if (scene->GetParent(entity).GetIndex() == 0)
          DrawEntityNode(entity, 0);
      });
      selected_entity_hierarchy_list_.clear();
    }
    AcceptEntityExplorerRootDrop(scene);
  } else {
    ImGui::Text("No Scene!");
  }
  ImGui::End();
}

void EditorLayer::DrawEntityInspectorWindow(const std::shared_ptr<Scene>& scene,
                                            const std::shared_ptr<EditorLayer>& editor_layer) {
  if (!show_entity_inspector_window) {
    return;
  }
  ImGui::Begin("Entity Inspector");
  if (scene) {
    ImGui::Text("Selection:");
    ImGui::SameLine();
    ImGui::Checkbox("Lock", &lock_entity_selection_);
    ImGui::SameLine();
    ImGui::Checkbox("Focus", &highlight_selection_);
    ImGui::SameLine();
    ImGui::Checkbox("Gizmos", &enable_gizmos);
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
      SetSelectedEntity({});
    }
    ImGui::Separator();
    if (scene->IsEntityValid(selected_entity_)) {
      std::string title = std::to_string(selected_entity_.GetIndex()) + ": ";
      title += scene->GetEntityName(selected_entity_);
      bool enabled = scene->IsEntityEnabled(selected_entity_);
      if (ImGui::Checkbox((title + "##EnabledCheckbox").c_str(), &enabled)) {
        if (scene->IsEntityEnabled(selected_entity_) != enabled) {
          scene->SetEnable(selected_entity_, enabled);
        }
      }
      ImGui::SameLine();
      bool is_static = scene->IsEntityStatic(selected_entity_);
      if (ImGui::Checkbox("Static##StaticCheckbox", &is_static)) {
        if (scene->IsEntityStatic(selected_entity_) != is_static) {
          scene->SetEntityStatic(selected_entity_, enabled);
        }
      }

      if (const bool deleted = DrawEntityMenu(scene->IsEntityEnabled(selected_entity_), selected_entity_); !deleted) {
        if (ImGui::CollapsingHeader("Data components", ImGuiTreeNodeFlags_DefaultOpen)) {
          if (ImGui::BeginPopupContextItem("DataComponentInspectorPopup")) {
            ImGui::Text("Add data component: ");
            ImGui::Separator();

            for (const auto& i : Serialization::GetInstance().data_component_ids_) {
              const auto id = i.second;
              const auto name = i.first;
              if (id == typeid(Transform).hash_code() || id == typeid(GlobalTransform).hash_code() ||
                  id == typeid(TransformUpdateFlag).hash_code())
                continue;

              if (!scene->HasDataComponent(selected_entity_, id) && ImGui::Button(name.c_str())) {
                scene->AddDataComponent(selected_entity_, id);
              }
            }
            ImGui::Separator();
            ImGui::EndPopup();
          }
          bool skip = false;
          int i = 0;
          scene->UnsafeForEachDataComponent(selected_entity_, [&](const DataComponentType& type, void* data) {
            if (skip)
              return;
            std::string info = type.type_name;
            if (info == "TransformUpdateFlag" || info == "GlobalTransform")
              return;
            info += " Size: " + std::to_string(type.type_size);
            ImGui::Text(info.c_str());
            ImGui::PushID(i);
            if (ImGui::BeginPopupContextItem(("DataComponentDeletePopup" + std::to_string(i)).c_str())) {
              if (ImGui::Button("Remove")) {
                skip = true;
                scene->RemoveDataComponent(selected_entity_, type.type_index);
              }
              ImGui::EndPopup();
            }
            ImGui::PopID();
            InspectComponentData(selected_entity_, static_cast<IDataComponent*>(data), type,
                                 scene->GetParent(selected_entity_).GetIndex() != 0);
            ImGui::Separator();
            i++;
          });
        }

        if (ImGui::CollapsingHeader("Private components", ImGuiTreeNodeFlags_DefaultOpen)) {
          if (ImGui::BeginPopupContextItem("PrivateComponentInspectorPopup")) {
            ImGui::Text("Add private component: ");
            ImGui::Separator();
            for (const auto& i : Serialization::GetInstance().private_component_ids_) {
              const auto id = i.second;
              const auto name = i.first;
              if (!scene->HasPrivateComponent(selected_entity_, id) && ImGui::Button(name.c_str())) {
                scene->AddPrivateComponent(selected_entity_, id);
              }
            }
            ImGui::Separator();
            ImGui::EndPopup();
          }

          int i = 0;
          bool skip = false;
          scene->ForEachPrivateComponent(selected_entity_, [&](const PrivateComponentElement& data) {
            if (skip)
              return;
            ImGui::Checkbox(data.private_component_data->GetTypeName().c_str(), &data.private_component_data->enabled_);
            DraggablePrivateComponent(data.private_component_data);
            const std::string tag = "##" + data.private_component_data->GetTypeName() +
                                    std::to_string(data.private_component_data->GetHandle());
            if (ImGui::BeginPopupContextItem(tag.c_str())) {
              if (ImGui::Button(("Remove" + tag).c_str())) {
                skip = true;
                scene->RemovePrivateComponent(selected_entity_, data.type_index);
              }
              ImGui::EndPopup();
            }
            if (!skip) {
              if (ImGui::TreeNodeEx(("Component Settings##" + std::to_string(i)).c_str(),
                                    ImGuiTreeNodeFlags_DefaultOpen)) {
                InspectorContext context;
                context.editor_layer = editor_layer;
                context.scene = scene;
                if (InspectorRegistry::GetInstance().Inspect(context, *data.private_component_data))
                  scene->SetUnsaved();
                ImGui::TreePop();
              }
            }
            ImGui::Separator();
            i++;
          });
        }
      }
    } else {
      SetSelectedEntity(Entity());
    }
  } else {
    ImGui::Text("No Scene!");
  }
  ImGui::End();
}

void EditorLayer::DrawConsoleWindow() {
  if (!show_console_window) {
    return;
  }
  if (ImGui::Begin("Console")) {
    ImGui::Checkbox("Log", &enable_console_logs_);
    ImGui::SameLine();
    ImGui::Checkbox("Warning", &enable_console_warnings_);
    ImGui::SameLine();
    ImGui::Checkbox("Error", &enable_console_errors_);
    ImGui::SameLine();
    if (ImGui::Button("Clear all")) {
      console_messages_.clear();
    }
    int i = 0;
    for (auto msg = console_messages_.rbegin(); msg != console_messages_.rend(); ++msg) {
      if (i > 999)
        break;
      i++;
      switch (msg->m_type) {
        case ConsoleMessageType::Log:
          if (enable_console_logs_) {
            ImGui::TextColored(ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled), "%.2f: ", msg->m_time);
            ImGui::SameLine();
            ImGui::TextColored(ImGui::GetStyleColorVec4(ImGuiCol_Text), msg->m_value.c_str());
            ImGui::Separator();
          }
          break;
        case ConsoleMessageType::Warning:
          if (enable_console_warnings_) {
            ImGui::TextColored(ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled), "%.2f: ", msg->m_time);
            ImGui::SameLine();
            ImGui::TextColored(WarningTextColor(), msg->m_value.c_str());
            ImGui::Separator();
          }
          break;
        case ConsoleMessageType::Error:
          if (enable_console_errors_) {
            ImGui::TextColored(ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled), "%.2f: ", msg->m_time);
            ImGui::SameLine();
            ImGui::TextColored(ErrorTextColor(), msg->m_value.c_str());
            ImGui::Separator();
          }
          break;
      }
    }
  }
  ImGui::End();
}

void EditorLayer::DrawRuntimePackageManagerWindow() {
  if (!show_package_manager_window) {
    return;
  }
  if (!runtime_package_manager_scanned_) {
    PackageManager::ScanAvailablePackages();
    runtime_package_manager_scanned_ = true;
  }
  PollRuntimePackageBuildJobs();

  bool package_manager_open = show_package_manager_window;
  if (ImGui::Begin("Runtime Package Manager", &package_manager_open)) {
    if (ImGui::Button("Scan")) {
      PackageManager::ScanAvailablePackages();
      runtime_package_manager_scanned_ = true;
    }
    ImGui::SameLine();
    const bool can_modify_packages = PackageManager::CanModifyPackages();
    const bool runtime_package_build_active = HasActiveRuntimePackageBuild();
    ImGui::BeginDisabled(!can_modify_packages || selected_runtime_package_names_.empty());
    if (ImGui::Button("Load Selected")) {
      std::vector<std::string> selected_packages(selected_runtime_package_names_.begin(),
                                                 selected_runtime_package_names_.end());
      selected_runtime_package_names_.clear();
      ApplicationContext::Get().QueueEndOfLoopAction([selected_packages]() {
        for (const auto& package_name : selected_packages) {
          PackageManager::Load(package_name);
        }
      });
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!can_modify_packages);
    if (ImGui::Button("Load All")) {
      ApplicationContext::Get().QueueEndOfLoopAction([]() {
        PackageManager::LoadAll();
      });
    }
    ImGui::EndDisabled();
    if (!can_modify_packages) {
      ImGui::TextUnformatted("Package changes are disabled while the application is playing, stepping, or paused.");
    }

    const auto search_paths = PackageManager::GetSearchPaths();
    const auto available_packages = PackageManager::GetAvailablePackages();
    const auto loaded_packages = PackageManager::GetLoadedPackages();
    const auto available_count = static_cast<size_t>(
        std::count_if(available_packages.begin(), available_packages.end(), [](const AvailablePackageInfo& package) {
          return !package.loaded;
        }));

    ImGui::Text("Available: %zu", available_count);
    ImGui::SameLine();
    ImGui::Text("Loaded: %zu", loaded_packages.size());

    std::unordered_map<std::string, const AvailablePackageInfo*> available_by_name;
    std::unordered_map<std::string, const LoadedPackageInfo*> loaded_by_name;
    available_by_name.reserve(available_packages.size());
    loaded_by_name.reserve(loaded_packages.size());
    for (const auto& package : available_packages) {
      available_by_name[package.name] = &package;
      if (package.loaded || !package.library_exists) {
        selected_runtime_package_names_.erase(package.name);
      }
    }
    for (const auto& package : loaded_packages) {
      loaded_by_name[package.name] = &package;
    }

    const auto find_available_package = [&](const std::string& package_name) -> const AvailablePackageInfo* {
      const auto search = available_by_name.find(package_name);
      if (search == available_by_name.end() || search->second->loaded) {
        return nullptr;
      }
      return search->second;
    };
    const auto find_loaded_package = [&](const std::string& package_name) -> const LoadedPackageInfo* {
      const auto search = loaded_by_name.find(package_name);
      return search == loaded_by_name.end() ? nullptr : search->second;
    };
    const auto select_available_package = [&](const std::string& package_name) {
      inspected_runtime_package_source_ = RuntimePackageInspectionSource::Available;
      inspected_runtime_package_name_ = package_name;
    };
    const auto select_loaded_package = [&](const std::string& package_name) {
      inspected_runtime_package_source_ = RuntimePackageInspectionSource::Loaded;
      inspected_runtime_package_name_ = package_name;
    };
    const auto select_first_package = [&]() {
      for (const auto& package : available_packages) {
        if (!package.loaded) {
          select_available_package(package.name);
          return;
        }
      }
      if (!loaded_packages.empty()) {
        select_loaded_package(loaded_packages.front().name);
        return;
      }
      inspected_runtime_package_name_.clear();
    };

    const auto start_runtime_package_build = [&](const RuntimePackageCMakeBuildRequest& request) {
      RuntimePackageBuildJob job;
      job.future = std::async(std::launch::async, [request]() {
        const auto cmake_result = RunRuntimePackageBuild(request);
        RuntimePackageBuildResult result;
        result.success = cmake_result.success;
        result.exit_code = cmake_result.exit_code;
        result.command = cmake_result.command;
        result.output = cmake_result.output;
        result.error = cmake_result.error;
        return result;
      });
      runtime_package_build_jobs_[request.package_name] = std::move(job);
    };

    const auto draw_runtime_package_build_controls = [&](const std::string& package_name,
                                                         const std::filesystem::path& package_runtime_path) {
      std::string build_error;
      const auto build_request = CreateRuntimePackageBuildRequest(package_name, package_runtime_path, build_error);
      const auto build_job = runtime_package_build_jobs_.find(package_name);
      const bool package_building =
          build_job != runtime_package_build_jobs_.end() && !build_job->second.result.has_value();

      if (package_building) {
        ImGui::TextUnformatted("Build: running");
      } else if (build_job != runtime_package_build_jobs_.end() && build_job->second.result.has_value()) {
        const auto& result = *build_job->second.result;
        ImGui::Text("Build: %s", result.success ? "succeeded" : "failed");
        if (!result.command.empty()) {
          ImGui::TextWrapped("Command: %s", result.command.c_str());
        }
        if (!result.success) {
          ImGui::Text("Exit code: %d", result.exit_code);
        }
        if (!result.error.empty()) {
          ImGui::TextWrapped("%s", result.error.c_str());
        }
        if (!result.output.empty() && ImGui::TreeNode("Build output")) {
          ImGui::BeginChild("BuildOutput", ImVec2(0.0f, 140.0f), true);
          ImGui::TextUnformatted(result.output.c_str());
          ImGui::EndChild();
          ImGui::TreePop();
        }
      }

      ImGui::BeginDisabled(package_building || runtime_package_build_active || !build_request.has_value());
      if (ImGui::Button("Build")) {
        start_runtime_package_build(*build_request);
      }
      ImGui::EndDisabled();
      if (!build_request.has_value()) {
        ImGui::TextWrapped("Build unavailable: %s", build_error.c_str());
      } else if (runtime_package_build_active && !package_building) {
        ImGui::TextUnformatted("Build disabled while another package is building.");
      }
    };

    const auto draw_available_package_row = [&](const AvailablePackageInfo& package) {
      const bool can_load = !package.loaded && package.library_exists;
      bool selected = selected_runtime_package_names_.find(package.name) != selected_runtime_package_names_.end();
      ImGui::PushID(package.name.c_str());
      ImGui::BeginDisabled(!can_modify_packages || !can_load);
      if (ImGui::Checkbox("##Select", &selected)) {
        if (selected) {
          selected_runtime_package_names_.insert(package.name);
        } else {
          selected_runtime_package_names_.erase(package.name);
        }
      }
      ImGui::EndDisabled();
      ImGui::SameLine();

      auto package_label = package.name;
      if (!package.library_exists) {
        package_label += " (missing library)";
      }

      const bool inspected = inspected_runtime_package_source_ == RuntimePackageInspectionSource::Available &&
                             inspected_runtime_package_name_ == package.name;
      if (ImGui::Selectable(package_label.c_str(), inspected)) {
        select_available_package(package.name);
      }
      ImGui::PopID();
    };

    const auto loaded_dependent_name = [&](const std::string& package_name) {
      for (const auto& loaded_package : loaded_packages) {
        if (loaded_package.name == package_name) {
          continue;
        }
        if (std::find(loaded_package.dependencies.begin(), loaded_package.dependencies.end(), package_name) !=
            loaded_package.dependencies.end()) {
          return loaded_package.name;
        }
      }
      return std::string();
    };

    const auto draw_loaded_package_row = [&](const LoadedPackageInfo& package) {
      ImGui::PushID(package.name.c_str());
      const bool inspected = inspected_runtime_package_source_ == RuntimePackageInspectionSource::Loaded &&
                             inspected_runtime_package_name_ == package.name;
      if (ImGui::Selectable(package.name.c_str(), inspected)) {
        select_loaded_package(package.name);
      }
      ImGui::PopID();
    };

    const auto draw_path = [](const char* label, const std::filesystem::path& path) {
      const auto path_string = path.string();
      ImGui::TextUnformatted(label);
      ImGui::SameLine();
      ImGui::TextWrapped("%s", path_string.c_str());
    };
    const auto draw_available_dependencies = [&](const std::vector<std::string>& dependencies) {
      ImGui::TextUnformatted("Dependencies:");
      if (dependencies.empty()) {
        ImGui::BulletText("%s", "None");
        return;
      }
      for (const auto& dependency : dependencies) {
        std::string status;
        if (const auto search = available_by_name.find(dependency); search != available_by_name.end()) {
          if (search->second->loaded) {
            status = " (loaded)";
          } else if (!search->second->library_exists) {
            status = " (missing library)";
          }
        } else {
          status = " (missing manifest)";
        }
        ImGui::BulletText("%s%s", dependency.c_str(), status.c_str());
      }
    };
    const auto draw_loaded_dependencies = [&](const std::vector<std::string>& dependencies) {
      ImGui::TextUnformatted("Dependencies:");
      if (dependencies.empty()) {
        ImGui::BulletText("%s", "None");
        return;
      }
      for (const auto& dependency : dependencies) {
        const auto loaded_search = loaded_by_name.find(dependency);
        ImGui::BulletText("%s%s", dependency.c_str(),
                          loaded_search == loaded_by_name.end() ? " (not loaded)" : " (loaded)");
      }
    };
    const auto draw_type_list = [](const char* label, const std::vector<std::string>& type_names) {
      if (type_names.empty()) {
        return;
      }
      if (ImGui::TreeNode(label)) {
        for (const auto& type_name : type_names) {
          ImGui::BulletText("%s", type_name.c_str());
        }
        ImGui::TreePop();
      }
    };

    const auto draw_available_package_inspection = [&](const AvailablePackageInfo& package) {
      const bool can_load = !package.loaded && package.library_exists;
      ImGui::PushID(package.name.c_str());
      ImGui::Text("Name: %s", package.name.c_str());
      ImGui::Text("Status: %s", package.library_exists ? "Available" : "Missing library");
      ImGui::Text("Version: %s", package.version.empty() ? "Unknown" : package.version.c_str());
      if (!package.description.empty()) {
        ImGui::TextWrapped("%s", package.description.c_str());
      }
      ImGui::Separator();
      draw_available_dependencies(package.dependencies);
      ImGui::Separator();
      draw_path("Manifest:", package.manifest_path);
      draw_path("Library:", package.library_path);
      ImGui::Separator();
      draw_runtime_package_build_controls(package.name, package.manifest_path);
      ImGui::BeginDisabled(!can_modify_packages || !can_load);
      if (ImGui::Button("Load")) {
        const auto package_name = package.name;
        selected_runtime_package_names_.erase(package.name);
        ApplicationContext::Get().QueueEndOfLoopAction([package_name]() {
          PackageManager::Load(package_name);
        });
      }
      ImGui::EndDisabled();
      ImGui::PopID();
    };

    const auto draw_loaded_package_inspection = [&](const LoadedPackageInfo& package) {
      const auto dependent_package_name = loaded_dependent_name(package.name);
      ImGui::PushID(package.name.c_str());
      ImGui::Text("Name: %s", package.name.c_str());
      ImGui::Text("Version: %s", package.version.empty() ? "Unknown" : package.version.c_str());
      ImGui::Text("Live objects: %zu", package.live_object_count);
      if (!dependent_package_name.empty()) {
        ImGui::TextWrapped("Reload and unload are disabled because %s depends on this package.",
                           dependent_package_name.c_str());
      }
      if (!package.description.empty()) {
        ImGui::TextWrapped("%s", package.description.c_str());
      }
      ImGui::Separator();
      draw_loaded_dependencies(package.dependencies);
      ImGui::Separator();
      draw_path("Original path:", package.original_path);
      draw_path("Loaded path:", package.loaded_path);
      ImGui::Separator();
      draw_type_list("Private components", package.private_component_types);
      draw_type_list("Assets", package.asset_types);
      draw_type_list("Data components", package.data_component_types);
      draw_type_list("Systems", package.system_types);
      draw_type_list("Layers", package.layer_types);
      ImGui::Separator();
      draw_runtime_package_build_controls(package.name, package.original_path);
      const bool can_reload_or_unload = can_modify_packages && dependent_package_name.empty();
      ImGui::BeginDisabled(!can_reload_or_unload);
      if (ImGui::Button("Reload")) {
        const auto package_name = package.name;
        ApplicationContext::Get().QueueEndOfLoopAction([package_name]() {
          PackageManager::Reload(package_name);
        });
      }
      ImGui::SameLine();
      if (ImGui::Button("Unload")) {
        const auto package_name = package.name;
        ApplicationContext::Get().QueueEndOfLoopAction([package_name]() {
          PackageManager::Unload(package_name);
        });
      }
      ImGui::EndDisabled();
      ImGui::PopID();
    };

    const AvailablePackageInfo* inspected_available_package = nullptr;
    const LoadedPackageInfo* inspected_loaded_package = nullptr;
    const auto resolve_inspected_package = [&]() {
      inspected_available_package = nullptr;
      inspected_loaded_package = nullptr;
      if (inspected_runtime_package_name_.empty()) {
        return;
      }
      if (inspected_runtime_package_source_ == RuntimePackageInspectionSource::Available) {
        inspected_available_package = find_available_package(inspected_runtime_package_name_);
      } else {
        inspected_loaded_package = find_loaded_package(inspected_runtime_package_name_);
      }
    };
    if (inspected_runtime_package_name_.empty()) {
      select_first_package();
    }
    resolve_inspected_package();
    if (!inspected_available_package && !inspected_loaded_package && !inspected_runtime_package_name_.empty()) {
      const auto previous_package_name = inspected_runtime_package_name_;
      if (const auto loaded_package = find_loaded_package(previous_package_name)) {
        select_loaded_package(loaded_package->name);
      } else if (const auto available_package = find_available_package(previous_package_name)) {
        select_available_package(available_package->name);
      } else {
        select_first_package();
      }
      resolve_inspected_package();
    }

    ImGui::Separator();
    const auto content_region = ImGui::GetContentRegionAvail();
    const float package_list_width = std::min(420.0f, std::max(260.0f, content_region.x * 0.36f));
    if (ImGui::BeginChild("RuntimePackageListPanel", ImVec2(package_list_width, 0.0f), true)) {
      ImGui::TextUnformatted("Package List");
      ImGui::Separator();
      ImGui::Text("Available (%zu)", available_count);
      if (available_count == 0) {
        ImGui::TextUnformatted("No unloaded package manifests found.");
      }
      ImGui::PushID("AvailablePackages");
      for (const auto& package : available_packages) {
        if (package.loaded) {
          continue;
        }
        draw_available_package_row(package);
      }
      ImGui::PopID();
      ImGui::Spacing();
      ImGui::Separator();
      ImGui::Text("Loaded (%zu)", loaded_packages.size());
      if (loaded_packages.empty()) {
        ImGui::TextUnformatted("No runtime packages loaded.");
      }
      ImGui::PushID("LoadedPackages");
      for (const auto& package : loaded_packages) {
        draw_loaded_package_row(package);
      }
      ImGui::PopID();
    }
    ImGui::EndChild();
    ImGui::SameLine();
    if (ImGui::BeginChild("RuntimePackageInspectionPanel", ImVec2(0.0f, 0.0f), true)) {
      ImGui::TextUnformatted("Package Inspection");
      ImGui::Separator();
      if (inspected_available_package) {
        draw_available_package_inspection(*inspected_available_package);
      } else if (inspected_loaded_package) {
        draw_loaded_package_inspection(*inspected_loaded_package);
      } else {
        ImGui::TextUnformatted("Select a package from the list.");
        if (!search_paths.empty()) {
          ImGui::Spacing();
          ImGui::TextUnformatted("Search paths:");
          for (const auto& path : search_paths) {
            ImGui::BulletText("%s", path.string().c_str());
          }
        }
      }
    }
    ImGui::EndChild();
  }
  show_package_manager_window = package_manager_open;
  ImGui::End();
}

void EditorLayer::PollRuntimePackageBuildJobs() {
  bool refresh_packages = false;
  for (auto& [package_name, job] : runtime_package_build_jobs_) {
    if (job.result.has_value() || !job.future.valid()) {
      continue;
    }
    if (job.future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      continue;
    }
    try {
      job.result = job.future.get();
    } catch (const std::exception& e) {
      RuntimePackageBuildResult result;
      result.error = e.what();
      job.result = std::move(result);
    }
    if (job.result->success) {
      refresh_packages = true;
      EVOENGINE_LOG("Runtime package build succeeded: " + package_name)
    } else {
      EVOENGINE_ERROR("Runtime package build failed: " + package_name)
    }
  }

  if (refresh_packages) {
    PackageManager::ScanAvailablePackages();
    runtime_package_manager_scanned_ = true;
  }
}

bool EditorLayer::HasActiveRuntimePackageBuild() const {
  for (const auto& [_, job] : runtime_package_build_jobs_) {
    if (!job.result.has_value() && job.future.valid()) {
      return true;
    }
  }
  return false;
}

void EditorLayer::HandleSceneDeleteShortcut(const std::shared_ptr<Scene>& scene) {
  if (scene && scene_camera_window_focused_ && Input::GetKey(GLFW_KEY_DELETE) == Input::KeyActionType::Press) {
    if (scene->IsEntityValid(selected_entity_)) {
      scene->DeleteEntity(selected_entity_);
    }
  }
}

void EditorLayer::DrawSceneCameraDebugWindow(const std::shared_ptr<Scene>& scene) {
  if (scene && show_scene_camera_debug) {
    if (ImGui::Begin("Scene Camera Debug")) {
      static float debug_scale = 0.25f;
      ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 1.0f);
      debug_scale = glm::clamp(debug_scale, 0.1f, 1.0f);
      auto& [sceneCameraRotation, sceneCameraPosition, sceneCamera] = editor_cameras_.at(scene_camera_handle_);
      DrawCameraDebugViews(*sceneCamera, debug_scale);
    }
    ImGui::End();
  }
}

void EditorLayer::DrawLayerInspectionWindows(const std::shared_ptr<Scene>& scene,
                                             const std::shared_ptr<EditorLayer>& editor_layer) {
  const auto layers = ApplicationContext::Get().GetLayers();
  for (const auto& layer : layers) {
    if (!layer->enable_inspection) {
      continue;
    }
    InspectorContext context;
    context.editor_layer = editor_layer;
    context.scene = scene;
    const auto& inspector_registry = InspectorRegistry::GetInstance();
    if (inspector_registry.FindInspector(typeid(*layer))) {
      inspector_registry.Inspect(context, *layer);
    }
  }
}

void EditorLayer::DrawLayerSettingsWindow(const std::shared_ptr<EditorLayer>& editor_layer) {
  const auto window_title = GetLayerName();
  bool open = enable_inspection;
  if (!ImGui::Begin(window_title.c_str(), &open)) {
    ImGui::End();
    enable_inspection = open;
    return;
  }
  ImGui::Checkbox("Scene Window", &show_scene_window);
  if (show_scene_window) {
    ImGui::Checkbox("Scene Camera Debug Window", &show_scene_camera_debug);
    ImGui::Checkbox("Scene Window Info", &show_scene_info);
  }
  ImGui::Checkbox("Main Camera Window", &show_camera_window);
  if (show_camera_window) {
    ImGui::Checkbox("Main Camera Window Info", &show_camera_info);
  }
  ImGui::Checkbox("Entity Explorer", &show_entity_explorer_window);
  ImGui::Checkbox("Entity Inspector", &show_entity_inspector_window);
  ImGui::Checkbox("Console", &show_console_window);
  ImGui::Checkbox("Runtime Packages", &show_package_manager_window);

  if (ImGui::TreeNode("Scene camera settings")) {
    ImGui::Checkbox("View Gizmos", &enable_view_gizmos);
    if (ImGui::Button("Reset camera")) {
      MoveCamera(default_scene_camera_rotation, default_scene_camera_position);
    }
    auto& [sceneCameraRotation, sceneCameraPosition, sceneCamera] = editor_cameras_.at(scene_camera_handle_);
    if (ImGui::Button("Set default camera position")) {
      default_scene_camera_position = sceneCameraPosition;
      default_scene_camera_rotation = sceneCameraRotation;
    }
    ImGui::DragFloat("Speed", &velocity, 0.1f, 0, 0, "%.1f");
    ImGui::DragFloat("Sensitivity", &sensitivity, 0.1f, 0, 0, "%.1f");
    ImGui::Checkbox("Copy Transform", &apply_transform_to_main_camera);
    ImGui::DragFloat("Resolution", &scene_camera_resolution_multiplier, 0.1f, 0.1f, 4.0f);

    if (ImGui::TreeNode("Camera settings")) {
      InspectorContext context;
      context.editor_layer = editor_layer;
      context.scene = ApplicationContext::Get().GetActiveScene();
      InspectorRegistry::GetInstance().Inspect(context, *sceneCamera);
      ImGui::TreePop();
    }

    ImGui::TreePop();
  }
  ImGui::End();
  enable_inspection = open;
}

void EditorLayer::DrawMainMenuBar() {
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5, 5));
  if (ImGui::BeginMainMenuBar()) {
    DrawMainMenuItems();
    ImGui::EndMainMenuBar();
  }
  ImGui::PopStyleVar();
}

float EditorLayer::DrawTitleBarSearch(const ImVec2& titlebar_min, const float controls_x, const float drag_start_x,
                                      std::vector<glm::vec4>& drag_regions) {
  const float window_width = ImGui::GetWindowWidth();
  const float left_limit = titlebar_min.x + drag_start_x + 18.0f;
  const float right_limit = titlebar_min.x + controls_x - 16.0f;
  const float available_width = right_limit - left_limit;
  if (available_width < kTitleBarSearchMinWidth) {
    drag_regions.emplace_back(drag_start_x, 0.0f, controls_x - drag_start_x, kCustomTitleBarHeight);
    return titlebar_min.x + window_width * 0.5f;
  }

  const float desired_width = std::max(kTitleBarSearchMinWidth, window_width * 0.24f);
  const float search_width = std::min(kTitleBarSearchMaxWidth, std::min(desired_width, available_width));
  float search_x = titlebar_min.x + (window_width - search_width) * 0.5f;
  search_x = std::max(left_limit, std::min(search_x, right_limit - search_width));
  const float search_y = titlebar_min.y + (kCustomTitleBarHeight - kTitleBarSearchHeight) * 0.5f;

  ImGui::SetCursorScreenPos(ImVec2(search_x, search_y));
  ImGui::SetNextItemWidth(search_width);
  const float vertical_padding = std::max(2.0f, (kTitleBarSearchHeight - ImGui::GetFontSize()) * 0.5f);
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(12.0f, vertical_padding));
  ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 4.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
  ImGui::PushStyleColor(ImGuiCol_FrameBg, ImGui::ColorConvertU32ToFloat4(kTitleBarSearchBg));
  ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImGui::ColorConvertU32ToFloat4(kTitleBarSearchHovered));
  ImGui::PushStyleColor(ImGuiCol_FrameBgActive, ImGui::ColorConvertU32ToFloat4(kTitleBarSearchActive));
  ImGui::PushStyleColor(ImGuiCol_Border, ImGui::ColorConvertU32ToFloat4(kTitleBarSearchBorder));
  ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(kTitleBarText));
  ImGui::PushStyleColor(ImGuiCol_TextDisabled, ImGui::ColorConvertU32ToFloat4(kTitleBarTextDarker));
  auto& search_buffer = TitleBarSearchBuffer(this);
  ImGui::InputTextWithHint("##TitleBarSearch", "Search entities, layers, assets", search_buffer.data(),
                           search_buffer.size());
  const ImVec2 search_min = ImGui::GetItemRectMin();
  const ImVec2 search_max = ImGui::GetItemRectMax();
  ImGui::PopStyleColor(6);
  ImGui::PopStyleVar(3);

  const float search_left = search_min.x - titlebar_min.x;
  const float search_right = search_max.x - titlebar_min.x;
  drag_regions.emplace_back(drag_start_x, 0.0f, search_left - drag_start_x - 6.0f, kCustomTitleBarHeight);
  drag_regions.emplace_back(search_right + 6.0f, 0.0f, controls_x - search_right - 6.0f, kCustomTitleBarHeight);

  const bool has_query = search_buffer[0] != '\0';
  if (!has_query) {
    return search_max.x;
  }

  ImGui::SetNextWindowPos(ImVec2(search_min.x, search_max.y + 6.0f), ImGuiCond_Always);
  ImGui::SetNextWindowSizeConstraints(ImVec2(search_width, 0.0f), ImVec2(search_width, 360.0f));
  ImGui::SetNextWindowSize(ImVec2(search_width, 0.0f), ImGuiCond_Always);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 8.0f));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
  ImGui::PushStyleColor(ImGuiCol_WindowBg, ImGui::ColorConvertU32ToFloat4(kTitleBarMenuPopupBg));
  ImGui::PushStyleColor(ImGuiCol_Border, ImGui::ColorConvertU32ToFloat4(kTitleBarMenuPopupBorder));
  ImGui::PushStyleColor(ImGuiCol_Header, ImGui::ColorConvertU32ToFloat4(kTitleBarMenuItemHovered));
  ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImGui::ColorConvertU32ToFloat4(kTitleBarMenuItemHovered));
  ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImGui::ColorConvertU32ToFloat4(kTitleBarMenuItemHovered));
  constexpr ImGuiWindowFlags search_result_flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoDocking |
                                                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove |
                                                   ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoFocusOnAppearing |
                                                   ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_AlwaysAutoResize;
  if (ImGui::Begin("##TitleBarSearchResults", nullptr, search_result_flags)) {
    const std::string query(search_buffer.data());
    std::vector<TitleBarSearchResult> results;
    results.reserve(kTitleBarSearchMaxResults);

    auto add_result = [&](const TitleBarSearchResult& result) {
      if (results.size() < kTitleBarSearchMaxResults) {
        results.push_back(result);
      }
    };

    if (const auto scene = GetScene()) {
      scene->ForAllEntities([&](const size_t, const Entity entity) {
        if (results.size() >= kTitleBarSearchMaxResults) {
          return;
        }
        const std::string entity_name = scene->GetEntityName(entity);
        const std::string entity_handle = std::to_string(scene->GetEntityHandle(entity).GetValue());
        if (!TextContainsCaseInsensitive(entity_name, query) && !TextContainsCaseInsensitive(entity_handle, query)) {
          return;
        }
        TitleBarSearchResult result;
        result.type = TitleBarSearchResultType::Entity;
        result.label = entity_name.empty() ? "Unnamed Entity" : entity_name;
        result.detail = "Handle " + entity_handle;
        result.entity = entity;
        add_result(result);
      });
    }

    const auto& inspector_registry = InspectorRegistry::GetInstance();
    for (const auto& layer : ApplicationContext::Get().GetLayers()) {
      if (!layer || results.size() >= kTitleBarSearchMaxResults) {
        continue;
      }
      const auto* inspector = inspector_registry.FindInspector(typeid(*layer));
      if (!inspector) {
        continue;
      }
      const std::string layer_name = layer->GetLayerName();
      if (!TextContainsCaseInsensitive(layer_name, query) &&
          !TextContainsCaseInsensitive(inspector->type_name, query)) {
        continue;
      }
      TitleBarSearchResult result;
      result.type = TitleBarSearchResultType::Layer;
      result.label = layer_name;
      result.detail = inspector->type_name.empty() ? "Inspectable layer" : inspector->type_name;
      result.layer = layer;
      add_result(result);
    }

    if (ProjectManager::HasProject()) {
      std::function<void(const std::shared_ptr<Folder>&)> add_folder_assets =
          [&](const std::shared_ptr<Folder>& folder) {
            if (!folder || results.size() >= kTitleBarSearchMaxResults) {
              return;
            }
            for (const auto& i : folder->files) {
              if (results.size() >= kTitleBarSearchMaxResults) {
                return;
              }
              const auto& file = i.second;
              if (!file || file->GetAssetTypeName() == "Binary") {
                continue;
              }
              const std::string file_name = file->GetAssetFileName() + file->GetAssetExtension();
              const std::string relative_path = file->GetAssetsFolderRelativePath().string();
              const std::string type_name = file->GetAssetTypeName();
              if (!TextContainsCaseInsensitive(file_name, query) &&
                  !TextContainsCaseInsensitive(relative_path, query) &&
                  !TextContainsCaseInsensitive(type_name, query)) {
                continue;
              }
              TitleBarSearchResult result;
              result.type = TitleBarSearchResultType::Asset;
              result.label = file_name;
              result.detail = type_name + " - " + relative_path;
              result.asset_handle = file->GetAssetHandle();
              add_result(result);
            }
            for (const auto& i : folder->children_) {
              add_folder_assets(i.second);
              if (results.size() >= kTitleBarSearchMaxResults) {
                return;
              }
            }
          };
      add_folder_assets(ProjectManager::GetAssetsFolder());
    }

    auto activate_result = [&](const TitleBarSearchResult& result) {
      switch (result.type) {
        case TitleBarSearchResultType::Entity: {
          if (const auto scene = GetScene(); scene && scene->IsEntityValid(result.entity)) {
            show_entity_explorer_window = true;
            show_entity_inspector_window = true;
            SetSelectedEntity(result.entity);
          }
          break;
        }
        case TitleBarSearchResultType::Layer:
          if (result.layer) {
            result.layer->enable_inspection = true;
          }
          break;
        case TitleBarSearchResultType::Asset:
          if (const auto asset = AssetManager::GetAssetImpl(result.asset_handle)) {
            OpenAssetInspector(asset);
          }
          break;
      }
      search_buffer.fill('\0');
    };

    if (results.empty()) {
      ImGui::TextDisabled("No results");
    } else {
      const float row_height = 36.0f;
      for (size_t i = 0; i < results.size(); ++i) {
        const auto& result = results[i];
        ImGui::PushID(static_cast<int>(i));
        const ImVec2 row_min = ImGui::GetCursorScreenPos();
        if (ImGui::Selectable("##TitleBarSearchResult", false, ImGuiSelectableFlags_None, ImVec2(0.0f, row_height))) {
          activate_result(result);
          ImGui::PopID();
          break;
        }
        const ImVec2 label_pos(row_min.x + 8.0f, row_min.y + 4.0f);
        const ImVec2 detail_pos(row_min.x + 8.0f, row_min.y + 20.0f);
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        draw_list->AddText(label_pos, kTitleBarText, result.label.c_str());
        const std::string detail = std::string(TitleBarSearchResultTypeName(result.type)) + " - " + result.detail;
        draw_list->AddText(detail_pos, kTitleBarTextDarker, detail.c_str());
        ImGui::PopID();
      }
      if (results.size() == kTitleBarSearchMaxResults) {
        ImGui::TextDisabled("Keep typing to narrow results");
      }
    }
  }
  ImGui::End();
  ImGui::PopStyleColor(5);
  ImGui::PopStyleVar(4);

  return search_max.x;
}

void EditorLayer::DrawCustomTitleBar() {
  const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>();
  if (!window_layer) {
    return;
  }

  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->Pos);
  ImGui::SetNextWindowSize(ImVec2(viewport->Size.x, kCustomTitleBarHeight));
  ImGui::SetNextWindowViewport(viewport->ID);
  constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoDocking |
                                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove |
                                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar |
                                     ImGuiWindowFlags_NoScrollWithMouse;

  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6.0f, 5.0f));
  ImGui::PushStyleColor(ImGuiCol_WindowBg, ImGui::ColorConvertU32ToFloat4(kTitleBarColor));
  ImGui::PushStyleColor(ImGuiCol_MenuBarBg, ImGui::ColorConvertU32ToFloat4(kTitleBarColor));
  ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(kTitleBarText));
  if (ImGui::Begin("Editor Custom Title Bar", nullptr, flags)) {
    const auto draw_list = ImGui::GetWindowDrawList();
    const ImVec2 titlebar_min = ImGui::GetWindowPos();
    const ImVec2 titlebar_max(titlebar_min.x + ImGui::GetWindowWidth(), titlebar_min.y + kCustomTitleBarHeight);
    draw_list->AddRectFilled(titlebar_min, titlebar_max, kTitleBarColor);
    ImU32 titlebar_accent = 0;
    if (CurrentTitleBarAccent(titlebar_accent)) {
      draw_list->AddRectFilledMultiColor(titlebar_min, ImVec2(titlebar_min.x + 380.0f, titlebar_max.y), titlebar_accent,
                                         kTitleBarColor, kTitleBarColor, titlebar_accent);
    }

    const ImVec2 logo_min(titlebar_min.x + kTitleBarLogoX,
                          titlebar_min.y + (kCustomTitleBarHeight - kTitleBarLogoSize) * 0.5f);
    DrawFittedImage(FindIconInMap(editor_icons_, "TitleBarLogo"),
                    ImRect(logo_min, ImVec2(logo_min.x + kTitleBarLogoSize, logo_min.y + kTitleBarLogoSize)),
                    IM_COL32_WHITE);

    float menu_right = kTitleBarMenuX;
    const float controls_x = ImGui::GetWindowWidth() - kTitleBarButtonsAreaWidth;
    PushTitleBarMenuStyle();
    const ImRect menu_bar_rect(ImVec2(kTitleBarMenuX, kTitleBarMenuY),
                               ImVec2(controls_x, kTitleBarMenuY + ImGui::GetFrameHeightWithSpacing()));
    if (BeginTitleBarMenuBar(menu_bar_rect)) {
      DrawMainMenuItems(true);
      menu_right = std::max(menu_right, ImGui::GetItemRectMax().x - titlebar_min.x);
      EndTitleBarMenuBar();
    }
    PopTitleBarMenuStyle();

    float drag_start_x = menu_right + 24.0f;
    const auto scene = GetScene();
    const std::string scene_name = scene ? scene->GetTitle() : std::string();
    if (scene && !scene_name.empty()) {
      const auto& style = ImGui::GetStyle();
      const ImVec2 scene_text_size = ImGui::CalcTextSize(scene_name.c_str());
      const float scene_button_width = scene_text_size.x + style.FramePadding.x * 2.0f;
      const float scene_x = menu_right + 50.0f;
      if (scene_x + scene_button_width + 24.0f < ImGui::GetWindowWidth() * 0.5f) {
        const ImVec2 scene_pos(titlebar_min.x + scene_x, titlebar_min.y + 6.0f);
        const float separator_y = scene_pos.y + (ImGui::GetFrameHeight() - scene_text_size.y) * 0.5f;
        draw_list->AddRectFilled(ImVec2(scene_pos.x - 8.0f, separator_y - 1.0f),
                                 ImVec2(scene_pos.x - 6.0f, separator_y + scene_text_size.y + 1.0f), kTitleBarMuted,
                                 2.0f);
        ImGui::SetCursorScreenPos(scene_pos);
        ImGui::PushID("TitleBarSceneAsset");
        ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_Header));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyleColorVec4(ImGuiCol_HeaderHovered));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetStyleColorVec4(ImGuiCol_HeaderActive));
        ImGui::Button((scene_name + "##SceneTitle").c_str(), ImVec2(scene_button_width, 0.0f));
        ImGui::PopStyleColor(3);
        if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
          OpenAssetInspector(scene);
        }
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.0f, 8.0f));
        if (ImGui::BeginDragDropSource()) {
          const Handle scene_handle = scene->GetHandle();
          ImGui::SetDragDropPayload("Asset", &scene_handle, sizeof(Handle));
          ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_Header));
          ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyleColorVec4(ImGuiCol_HeaderHovered));
          ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetStyleColorVec4(ImGuiCol_HeaderActive));
          ImGui::Button((scene_name + "##SceneTitleDragPreview").c_str(), ImVec2(scene_button_width, 0.0f));
          ImGui::PopStyleColor(3);
          ImGui::EndDragDropSource();
        }
        ImGui::PopStyleVar();
        ImGui::PopID();
        drag_start_x = std::max(drag_start_x, scene_x + scene_button_width + 24.0f);
      }
    }

    std::vector<glm::vec4> drag_regions;
    const float search_reserved_right = DrawTitleBarSearch(titlebar_min, controls_x, drag_start_x, drag_regions);

    const std::string project_name = ProjectManager::HasProject() ? ProjectManager::GetProjectName() : std::string();
    if (!project_name.empty() && ImGui::GetWindowWidth() > 760.0f) {
      const ImVec2 project_size = ImGui::CalcTextSize(project_name.c_str());
      const float right_offset = ImGui::GetWindowWidth() / 5.0f;
      const ImVec2 project_pos(titlebar_min.x + ImGui::GetWindowWidth() - right_offset - project_size.x,
                               titlebar_min.y + kTitleBarTextY);
      if (project_pos.x > search_reserved_right + 24.0f) {
        draw_list->AddText(project_pos, kTitleBarTextDarker, project_name.c_str());
        draw_list->AddRect(ImVec2(project_pos.x - 12.0f, project_pos.y - 5.0f),
                           ImVec2(project_pos.x + project_size.x + 12.0f, project_pos.y + project_size.y + 5.0f),
                           IM_COL32(40, 40, 40, 255), 3.0f);
      }
    }

    if (!drag_regions.empty()) {
      window_layer->SetCustomTitleBarDragRegions(drag_regions);
    } else {
      window_layer->ClearCustomTitleBarDragRegion();
    }

    ImGui::PushClipRect(titlebar_min, titlebar_max, false);
    const float button_y = titlebar_min.y + 8.0f;
    float button_x = titlebar_min.x + ImGui::GetWindowWidth() - 18.0f - kTitleBarButtonSize;
    if (DrawTitleBarImageButton("Close##Editor", FindIconInMap(editor_icons_, "WindowClose"),
                                ImVec2(button_x, button_y), true)) {
      ApplicationContext::Get().End();
    }
    button_x -= 15.0f + kTitleBarButtonSize;
    if (DrawTitleBarImageButton(
            window_layer->IsWindowMaximized() ? "Restore##Editor" : "Maximize##Editor",
            FindIconInMap(editor_icons_, window_layer->IsWindowMaximized() ? "WindowRestore" : "WindowMaximize"),
            ImVec2(button_x, button_y), false)) {
      ApplicationContext::Get().QueueEndOfLoopAction([window_layer]() {
        window_layer->ToggleMaximized();
      });
    }
    button_x -= 17.0f + kTitleBarButtonSize;
    if (DrawTitleBarImageButton("Minimize##Editor", FindIconInMap(editor_icons_, "WindowMinimize"),
                                ImVec2(button_x, button_y), false)) {
      ApplicationContext::Get().QueueEndOfLoopAction([window_layer]() {
        window_layer->MinimizeWindow();
      });
    }
    ImGui::PopClipRect();
  }
  ImGui::End();
  ImGui::PopStyleColor(3);
  ImGui::PopStyleVar(4);
}

void EditorLayer::DrawMainMenuItems(const bool title_bar_style) {
  bool menu_open = title_bar_style && ImGui::IsPopupOpen("##titlebar_menu", ImGuiPopupFlags_AnyPopupId);
  if (menu_open) {
    PushTitleBarMenuActiveHighlight();
  }
  auto begin_menu = [&](const char* label) {
    return title_bar_style ? BeginTitleBarMenu(label, menu_open) : ImGui::BeginMenu(label);
  };
  auto end_menu = [&]() {
    if (title_bar_style) {
      EndTitleBarMenu();
    } else {
      ImGui::EndMenu();
    }
  };
  auto panel_menu_item = [](const char* label, bool& open) {
    ImGui::MenuItem(label, nullptr, &open);
  };
  auto draw_layer_inspection_menu = []() {
    if (ImGui::BeginMenu("Layer Inspection")) {
      for (const auto& layer : ApplicationContext::Get().GetLayers()) {
        ImGui::Checkbox(layer->layer_name_.c_str(), &layer->enable_inspection);
      }
      ImGui::EndMenu();
    }
  };

  if (begin_menu("File")) {
    ProjectManager::DrawProjectMenuItems();
    ImGui::Separator();
    if (ImGui::MenuItem("Exit")) {
      ApplicationContext::Get().End();
    }
    end_menu();
  }
  if (begin_menu("Window")) {
    if (ImGui::BeginMenu("Scene")) {
      panel_menu_item("Scene Window", show_scene_window);
      panel_menu_item("Main Camera Window", show_camera_window);
      panel_menu_item("Scene Camera Debug", show_scene_camera_debug);
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Entity")) {
      panel_menu_item("Entity Explorer", show_entity_explorer_window);
      panel_menu_item("Entity Inspector", show_entity_inspector_window);
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Content")) {
      panel_menu_item("Project", ProjectManager::GetInstance().show_project_window);
      panel_menu_item("Resources", Resources::GetInstance().show_resources_);
      ImGui::EndMenu();
    }
    panel_menu_item("Console", show_console_window);
    panel_menu_item("Runtime Packages", show_package_manager_window);
    ImGui::Separator();
    if (ImGui::BeginMenu("Layout")) {
      if (ImGui::MenuItem("Reset Default Layout")) {
        RequestDefaultEditorLayout();
      }
      ImGui::EndMenu();
    }
    end_menu();
  }
  if (begin_menu("View")) {
    if (ImGui::BeginMenu("Theme")) {
      DrawThemeMenuItems();
      ImGui::EndMenu();
    }
    ImGui::Separator();
    draw_layer_inspection_menu();
    end_menu();
  }
  if (menu_open) {
    ImGui::PopStyleColor(3);
  }
}

bool EditorLayer::DrawPlayControls() {
  if (!show_play_buttons) {
    return false;
  }
  bool toolbar_interacted = false;
  auto draw_button = [&](const char* id, const char* icon_name, const char* tooltip) {
    const bool clicked = DrawToolbarImageButton(id, FindIconInMap(editor_icons_, icon_name), tooltip);
    toolbar_interacted = toolbar_interacted || ImGui::IsItemHovered() || ImGui::IsItemActive();
    return clicked;
  };

  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 0.0f));
  switch (ApplicationContext::Get().GetApplicationStatus()) {
    case Application::ExecutionStatus::NotPlaying: {
      if (draw_button("PlayButton", "PlayButton", "Play")) {
        ApplicationContext::Get().Play();
      }
      ImGui::SameLine();
      if (draw_button("StepButton", "StepButton", "Step")) {
        ApplicationContext::Get().Step();
      }
      break;
    }
    case Application::ExecutionStatus::Playing: {
      if (draw_button("PauseButton", "PauseButton", "Pause")) {
        ApplicationContext::Get().Pause();
      }
      ImGui::SameLine();
      if (draw_button("StopButton", "StopButton", "Stop")) {
        ApplicationContext::Get().Stop();
      }
      break;
    }
    case Application::ExecutionStatus::Pause: {
      if (draw_button("PlayButton", "PlayButton", "Resume")) {
        ApplicationContext::Get().Play();
      }
      ImGui::SameLine();
      if (draw_button("StepButton", "StepButton", "Step")) {
        ApplicationContext::Get().Step();
      }
      ImGui::SameLine();
      if (draw_button("StopButton", "StopButton", "Stop")) {
        ApplicationContext::Get().Stop();
      }
      break;
    }
    case Application::ExecutionStatus::Uninitialized:
    case Application::ExecutionStatus::Step:
    case Application::ExecutionStatus::OnDestroy:
      break;
  }
  ImGui::PopStyleVar();
  return toolbar_interacted;
}

void EditorLayer::DrawScenePlaybackToolbar(const ImVec2& overlay_pos, const ImVec2& view_port_size) {
  const int button_count = PlaybackControlCount(ApplicationContext::Get().GetApplicationStatus());
  if (!show_play_buttons || button_count == 0) {
    return;
  }

  constexpr float button_size = 23.0f;
  constexpr float horizontal_padding = 12.0f;
  constexpr float item_spacing = 8.0f;
  constexpr float background_height = 31.0f;
  const float background_width = horizontal_padding * 2.0f + button_size * static_cast<float>(button_count) +
                                 item_spacing * static_cast<float>(button_count - 1);
  const ImVec2 background_min(overlay_pos.x + (view_port_size.x - background_width) * 0.5f, overlay_pos.y + 4.0f);
  const ImVec2 background_max(background_min.x + background_width, background_min.y + background_height);
  ImGui::GetWindowDrawList()->AddRectFilled(background_min, background_max, IM_COL32(15, 15, 15, 127), 4.0f);

  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(item_spacing, 0.0f));
  ImGui::SetCursorScreenPos(
      ImVec2(background_min.x + horizontal_padding, background_min.y + (background_height - button_size) * 0.5f));
  if (DrawPlayControls()) {
    scene_camera_window_focused_ = false;
  }
  ImGui::PopStyleVar();
}

void EditorLayer::RequestDefaultEditorLayout() {
  editor_panel_manager_.ResetPanelOpenStatesToDefaults();
  dock_layout_reset_pending_ = true;
}

void EditorLayer::DrawDockspace(const float top_offset) {
#pragma region Dock
  static bool opt_fullscreen_persistent = true;
  const bool opt_fullscreen = opt_fullscreen_persistent;
  static ImGuiDockNodeFlags dock_space_flags = ImGuiDockNodeFlags_None;

  // We are using the ImGuiWindowFlags_NoDocking flag to make the parent window not dockable into,
  // because it would be confusing to have two docking targets within each others.
  ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDocking;
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  if (opt_fullscreen) {
    const ImVec2 pos = top_offset > 0.0f ? ImVec2(viewport->Pos.x, viewport->Pos.y + top_offset) : viewport->WorkPos;
    const ImVec2 size = top_offset > 0.0f ? ImVec2(viewport->Size.x, std::max(1.0f, viewport->Size.y - top_offset))
                                          : viewport->WorkSize;
    ImGui::SetNextWindowPos(pos);
    ImGui::SetNextWindowSize(size);
    ImGui::SetNextWindowViewport(viewport->ID);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    window_flags |=
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
  }

  // When using ImGuiDockNodeFlags_PassthruCentralNode, DockSpace() will render our background
  // and handle the pass-thru hole, so we ask Begin() to not render a background.
  if (dock_space_flags & ImGuiDockNodeFlags_PassthruCentralNode)
    window_flags |= ImGuiWindowFlags_NoBackground;

  // Important: note that we proceed even if Begin() returns false (aka window is collapsed).
  // This is because we want to keep our DockSpace() active. If a DockSpace() is inactive,
  // all active windows docked into it will lose their parent and become undocked.
  // We cannot preserve the docking relationship between an active window and an inactive docking, otherwise
  // any change of dock space/settings would lead to windows being stuck in limbo and never being visible.

  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
  static bool open_dock = true;
  ImGui::Begin("Root DockSpace", &open_dock, window_flags);
  ImGui::PopStyleVar();
  if (opt_fullscreen)
    ImGui::PopStyleVar(2);
  dock_space_id = ImGui::GetID("MyDockSpace");
  const ImVec2 dock_size = ImGui::GetContentRegionAvail();
  if (dock_layout_reset_pending_ && dock_size.x > 1.0f && dock_size.y > 1.0f) {
    BuildDefaultEditorDockLayout(dock_space_id, dock_size);
    dock_layout_reset_pending_ = false;
    ImGui::MarkIniSettingsDirty();
  }
  ImGui::DockSpace(dock_space_id, ImVec2(0.0f, 0.0f), dock_space_flags);
  ImGui::End();
#pragma endregion
}

bool EditorLayer::DrawEntityMenu(const bool& enabled, const Entity& entity) const {
  bool deleted = false;
  if (ImGui::BeginPopupContextItem(std::to_string(entity.GetIndex()).c_str())) {
    const auto scene = GetScene();
    ImGui::Text(("Handle: " + std::to_string(scene->GetEntityHandle(entity).GetValue())).c_str());
    if (ImGui::Button("Delete")) {
      scene->DeleteEntity(entity);
      deleted = true;
    }
    if (!deleted && ImGui::Button(enabled ? "Disable" : "Enable")) {
      if (enabled) {
        scene->SetEnable(entity, false);
      } else {
        scene->SetEnable(entity, true);
      }
    }
    if (const std::string tag = "##Entity" + std::to_string(scene->GetEntityHandle(entity));
        !deleted && ImGui::BeginMenu(("Rename" + tag).c_str())) {
      static char new_name[256];
      ImGui::InputText("New name", new_name, 256);
      if (ImGui::Button("Confirm")) {
        scene->SetEntityName(entity, std::string(new_name));
        memset(new_name, 0, 256);
      }
      ImGui::EndMenu();
    }
    ImGui::EndPopup();
  }
  return deleted;
}

void EditorLayer::DrawEntityNode(const Entity& entity, const unsigned& hierarchy_level) {
  const auto scene = GetScene();
  std::string title = std::to_string(entity.GetIndex()) + ": ";
  title += scene->GetEntityName(entity);
  const bool enabled = scene->IsEntityEnabled(entity);
  if (enabled) {
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_Text));
  }
  if (const int index = selected_entity_hierarchy_list_.size() - hierarchy_level - 1;
      !selected_entity_hierarchy_list_.empty() && index >= 0 && index < selected_entity_hierarchy_list_.size() &&
      selected_entity_hierarchy_list_[index] == entity) {
    ImGui::SetNextItemOpen(true);
  }
  const bool opened = ImGui::TreeNodeEx(
      title.c_str(), ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_OpenOnArrow |
                         ImGuiTreeNodeFlags_NoAutoOpenOnLog |
                         (selected_entity_ == entity ? ImGuiTreeNodeFlags_Framed : ImGuiTreeNodeFlags_FramePadding));
  if (ImGui::BeginDragDropSource()) {
    const auto handle = scene->GetEntityHandle(entity);
    ImGui::SetDragDropPayload("Entity", &handle, sizeof(Handle));
    ImGui::TextColored(ImGui::GetStyleColorVec4(ImGuiCol_TextLink), title.c_str());
    ImGui::EndDragDropSource();
  }
  if (ImGui::BeginDragDropTarget()) {
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      scene->SetParent(scene->GetEntity(*static_cast<Handle*>(payload->Data)), entity, true);
    }
    ImGui::EndDragDropTarget();
  }
  if (enabled) {
    ImGui::PopStyleColor();
  }
  if (!lock_entity_selection_ && ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
    SetSelectedEntity(entity, false);
  }
  if (const bool deleted = DrawEntityMenu(enabled, entity); opened && !deleted) {
    ImGui::TreePush(title.c_str());
    scene->ForEachChild(entity, [=](const Entity child) {
      DrawEntityNode(child, hierarchy_level + 1);
    });
    ImGui::TreePop();
  }
}

void EditorLayer::InspectComponentData(const Entity entity, IDataComponent* data, const DataComponentType& type,
                                       const bool is_root) {
  if (component_data_inspector_map_.find(type.type_index) != component_data_inspector_map_.end()) {
    if (component_data_inspector_map_.at(type.type_index)(entity, data, is_root)) {
      const auto scene = GetScene();
      scene->SetUnsaved();
    }
  }
}

void EditorLayer::SceneCameraWindow() {
  const auto scene = GetScene();
  auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>();
  const auto& graphics = Platform::GetInstance();
  auto& [sceneCameraRotation, sceneCameraPosition, scene_camera] = editor_cameras_.at(scene_camera_handle_);
#pragma region Scene Window

  scene_camera_window_focused_ = false;
  if (ImGui::Begin("Scene")) {
    if (scene) {
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
      ImVec2 view_port_size;
      // Using a Child allow to fill all the space of the window.
      // It also allows customization
      static int corner = 1;
      if (ImGui::BeginChild("SceneCameraRenderer", ImVec2(0, 0), false)) {
        if (ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows)) {
          scene_camera_window_focused_ = true;
        }
        view_port_size = ImGui::GetWindowSize();
        scene_camera_resolution_x_ = static_cast<int>(view_port_size.x * scene_camera_resolution_multiplier);
        scene_camera_resolution_y_ = static_cast<int>(view_port_size.y * scene_camera_resolution_multiplier);
        const ImVec2 overlay_pos = ImGui::GetWindowPos();
        if (scene_camera && scene_camera->Rendered()) {
          // Because I use the texture from OpenGL, I need to invert the V from the UV.
          ImGui::Image(scene_camera->GetRenderTexture()->GetColorImTextureId(),
                       ImVec2(view_port_size.x, view_port_size.y), ImVec2(0, 1), ImVec2(1, 0));
          CameraWindowDragAndDrop();
        } else {
          ImGui::Text("No active scene camera!");
        }
        DrawScenePlaybackToolbar(overlay_pos, view_port_size);
        const auto window_pos = ImVec2((corner & 1) ? (overlay_pos.x + view_port_size.x) : (overlay_pos.x),
                                       (corner & 2) ? (overlay_pos.y + view_port_size.y) : (overlay_pos.y));

        if (show_scene_info) {
          const auto window_pos_pivot = ImVec2((corner & 1) ? 1.0f : 0.0f, (corner & 2) ? 1.0f : 0.0f);
          ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
          ImGui::SetNextWindowBgAlpha(0.35f);
          constexpr ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoDocking |
                                                    ImGuiWindowFlags_NoSavedSettings |
                                                    ImGuiWindowFlags_NoFocusOnAppearing;
          if (constexpr ImGuiChildFlags child_flags = ImGuiChildFlags_None;
              ImGui::BeginChild("Info", ImVec2(150, 150), child_flags, window_flags)) {
            ImGui::Text("Info:");
            ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
            std::string draw_call_info = {};
            const auto current_frame_index = Platform::GetCurrentFrameIndex();
            if (graphics.prim_count[current_frame_index] < 999)
              draw_call_info += std::to_string(graphics.prim_count[current_frame_index]);
            else if (graphics.prim_count[current_frame_index] < 999999)
              draw_call_info += std::to_string(static_cast<int>(graphics.prim_count[current_frame_index] / 1000)) + "K";
            else
              draw_call_info +=
                  std::to_string(static_cast<int>(graphics.prim_count[current_frame_index] / 1000000)) + "M";
            draw_call_info += " tris";
            ImGui::Text(draw_call_info.c_str());
            ImGui::Text("%d drawcall", graphics.draw_call[current_frame_index]);
            ImGui::Text("Idle: %.3f", graphics.cpu_wait_time);
            ImGui::Separator();
            if (ImGui::IsMousePosValid()) {
              const auto pos = Input::GetMousePosition();
              ImGui::Text("Mouse: [%.0f,%.0f]", pos.x, pos.y);
            } else {
              ImGui::Text("Mouse: <invalid>");
            }
            uint32_t mode = static_cast<uint32_t>(scene_camera->camera_render_mode);
            if (ImGui::Combo("Render Mode", {"Rasterization", "Ray Tracing"}, mode)) {
              scene_camera->camera_render_mode = static_cast<Camera::CameraRenderMode>(mode);
              scene_camera->ResetFrameCount();
            }
          }
          ImGui::EndChild();
        }
        if (scene_camera_window_focused_) {
#pragma region Scene Camera Controller
          static bool is_dragging_previously = false;
          bool mouse_drag = true;
          if (mouse_scene_window_position_.x < 0 || mouse_scene_window_position_.y < 0 ||
              mouse_scene_window_position_.x > view_port_size.x || mouse_scene_window_position_.y > view_port_size.y ||
              Input::GetKey(GLFW_MOUSE_BUTTON_RIGHT) != Input::KeyActionType::Hold) {
            mouse_drag = false;
          }
          static float prev_x = 0;
          static float prev_y = 0;
          if (mouse_drag && !is_dragging_previously) {
            prev_x = mouse_scene_window_position_.x;
            prev_y = mouse_scene_window_position_.y;
          }
          const float x_offset = mouse_scene_window_position_.x - prev_x;
          const float y_offset = mouse_scene_window_position_.y - prev_y;
          prev_x = mouse_scene_window_position_.x;
          prev_y = mouse_scene_window_position_.y;
          is_dragging_previously = mouse_drag;

          if (mouse_drag && !lock_camera) {
            glm::vec3 front = sceneCameraRotation * glm::vec3(0, 0, -1);
            const glm::vec3 right = sceneCameraRotation * glm::vec3(1, 0, 0);
            if (Input::GetKey(GLFW_KEY_W) == Input::KeyActionType::Hold) {
              sceneCameraPosition +=
                  front * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime()) * velocity;
            }
            if (Input::GetKey(GLFW_KEY_S) == Input::KeyActionType::Hold) {
              sceneCameraPosition -=
                  front * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime()) * velocity;
            }
            if (Input::GetKey(GLFW_KEY_A) == Input::KeyActionType::Hold) {
              sceneCameraPosition -=
                  right * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime()) * velocity;
            }
            if (Input::GetKey(GLFW_KEY_D) == Input::KeyActionType::Hold) {
              sceneCameraPosition +=
                  right * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime()) * velocity;
            }
            if (Input::GetKey(GLFW_KEY_LEFT_SHIFT) == Input::KeyActionType::Hold) {
              sceneCameraPosition.y += velocity * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime());
            }
            if (Input::GetKey(GLFW_KEY_LEFT_CONTROL) == Input::KeyActionType::Hold) {
              sceneCameraPosition.y -= velocity * static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime());
            }
            if (x_offset != 0.0f || y_offset != 0.0f) {
              front = glm::rotate(front, glm::radians(-x_offset * sensitivity), glm::vec3(0, 1, 0));
              const glm::vec3 right = glm::normalize(glm::cross(front, glm::vec3(0.0f, 1.0f, 0.0f)));
              if ((front.y < 0.99f && y_offset < 0.0f) || (front.y > -0.99f && y_offset > 0.0f)) {
                front = glm::rotate(front, glm::radians(-y_offset * sensitivity), right);
              }
              const glm::vec3 up = glm::normalize(glm::cross(right, front));
              sceneCameraRotation = glm::quatLookAt(front, up);
            }
#pragma endregion
          }
        }
      }
#pragma region Gizmos and Entity Selection
      gizmo_using_ = false;
      gizmo_displaying_ = false;
      if (enable_gizmos) {
        ImGuizmo::SetOrthographic(false);
        ImGuizmo::SetDrawlist();
        float view_manipulate_left = ImGui::GetWindowPos().x;
        float view_manipulate_top = ImGui::GetWindowPos().y;
        ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, view_port_size.x, view_port_size.y);
        glm::mat4 camera_view = glm::inverse(glm::translate(sceneCameraPosition) * glm::mat4_cast(sceneCameraRotation));
        glm::mat4 camera_projection = scene_camera->GetProjection();
        const auto op = local_position_selected_   ? ImGuizmo::OPERATION::TRANSLATE
                        : local_rotation_selected_ ? ImGuizmo::OPERATION::ROTATE
                                                   : ImGuizmo::OPERATION::SCALE;
        if (scene->IsEntityValid(selected_entity_)) {
          auto transform = scene->GetDataComponent<Transform>(selected_entity_);
          GlobalTransform parent_global_transform;
          if (Entity parent_entity = scene->GetParent(selected_entity_); parent_entity.GetIndex() != 0) {
            parent_global_transform = scene->GetDataComponent<GlobalTransform>(scene->GetParent(selected_entity_));
          }
          auto global_transform = scene->GetDataComponent<GlobalTransform>(selected_entity_);

          ImGuizmo::Manipulate(glm::value_ptr(camera_view), glm::value_ptr(camera_projection), op, ImGuizmo::LOCAL,
                               glm::value_ptr(global_transform.value));
          gizmo_displaying_ = true;
          if (ImGuizmo::IsUsing()) {
            transform.value = glm::inverse(parent_global_transform.value) * global_transform.value;
            scene->SetDataComponent(selected_entity_, transform);
            transform.Decompose(previously_stored_position_, previously_stored_rotation_, previously_stored_scale_);
            previously_stored_rotation_ = glm::degrees(previously_stored_rotation_);
            gizmo_using_ = true;
          }
        }
        if (enable_view_gizmos) {
          ImGuizmo::ViewManipulate(glm::value_ptr(camera_view), 1.0f, ImVec2(view_manipulate_left, view_manipulate_top),
                                   ImVec2(96, 96), 0);
          GlobalTransform gl;
          gl.value = glm::inverse(camera_view);
          sceneCameraRotation = gl.GetRotation();
        }
      }
#pragma endregion

      ImGui::EndChild();
      scene_camera->SetRequireRendering(
          !(ImGui::GetCurrentWindowRead()->Hidden && !ImGui::GetCurrentWindowRead()->Collapsed));
      ImGui::PopStyleVar();
    } else {
      ImGui::Text("No Scene!");
    }
  }
  ImGui::End();

#pragma endregion
}
bool EditorLayer::IsGizmosDisplaying() const {
  return gizmo_displaying_;
}

bool EditorLayer::IsGizmosUsing() const {
  return gizmo_using_;
}
void EditorLayer::MainCameraWindow() {
  if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>(); !render_layer)
    return;
  const auto& graphics = Platform::GetInstance();
  const auto scene = GetScene();
#pragma region Window
  main_camera_window_focused_ = false;
  if (ImGui::Begin("Camera")) {
    if (scene) {
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
      static int corner = 1;
      // Using a Child allow to fill all the space of the window.
      // It also allows customization
      if (ImGui::BeginChild("MainCameraRenderer", ImVec2(0, 0), false)) {
        if (ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows)) {
          main_camera_window_focused_ = true;
        }
        const ImVec2 view_port_size = ImGui::GetWindowSize();
        main_camera_resolution_x = static_cast<int>(view_port_size.x * main_camera_resolution_multiplier_);
        main_camera_resolution_y = static_cast<int>(view_port_size.y * main_camera_resolution_multiplier_);
        //  Get the size of the child (i.e. the whole draw size of the windows).
        const ImVec2 overlay_pos = ImGui::GetWindowPos();
        // Because I use the texture from OpenGL, I need to invert the V from the UV.
        const auto main_camera = scene->main_camera.Get<Camera>();
        if (main_camera && main_camera->Rendered()) {
          ImGui::Image(main_camera->GetRenderTexture()->GetColorImTextureId(),
                       ImVec2(view_port_size.x, view_port_size.y), ImVec2(0, 1), ImVec2(1, 0));
          CameraWindowDragAndDrop();
        } else {
          ImGui::Text("No active main camera!");
        }

        const auto window_pos = ImVec2((corner & 1) ? (overlay_pos.x + view_port_size.x) : (overlay_pos.x),
                                       (corner & 2) ? (overlay_pos.y + view_port_size.y) : (overlay_pos.y));
        if (show_camera_info) {
          const auto window_pos_pivot = ImVec2((corner & 1) ? 1.0f : 0.0f, (corner & 2) ? 1.0f : 0.0f);
          ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
          ImGui::SetNextWindowBgAlpha(0.35f);
          constexpr ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoDocking |
                                                    ImGuiWindowFlags_NoSavedSettings |
                                                    ImGuiWindowFlags_NoFocusOnAppearing;
          if (constexpr ImGuiChildFlags child_flags = ImGuiChildFlags_None;
              ImGui::BeginChild("Render Info", ImVec2(300, 150), child_flags, window_flags)) {
            ImGui::Text("Info & Settings");
            ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
            ImGui::PushItemWidth(100);
            ImGui::Checkbox("Auto resize", &main_camera_allow_auto_resize);
            if (main_camera_allow_auto_resize) {
              ImGui::DragFloat("Resolution multiplier", &main_camera_resolution_multiplier_, 0.1f, 0.1f, 4.0f);
            }
            ImGui::PopItemWidth();
            std::string draw_call_info = {};
            const auto current_frame_index = Platform::GetCurrentFrameIndex();
            if (graphics.prim_count[current_frame_index] < 999)
              draw_call_info += std::to_string(graphics.prim_count[current_frame_index]);
            else if (graphics.prim_count[current_frame_index] < 999999)
              draw_call_info += std::to_string(static_cast<int>(graphics.prim_count[current_frame_index] / 1000)) + "K";
            else
              draw_call_info +=
                  std::to_string(static_cast<int>(graphics.prim_count[current_frame_index] / 1000000)) + "M";
            draw_call_info += " tris";
            ImGui::Text(draw_call_info.c_str());
            ImGui::Text("%d drawcall", graphics.draw_call[current_frame_index]);
            ImGui::Separator();
            if (ImGui::IsMousePosValid()) {
              const auto pos = Input::GetMousePosition();
              ImGui::Text("Mouse Pos: (%.1f,%.1f)", pos.x, pos.y);
            } else {
              ImGui::Text("Mouse Pos: <invalid>");
            }
            uint32_t mode = static_cast<uint32_t>(main_camera->camera_render_mode);
            if (ImGui::Combo("Render Mode", {"Rasterization", "Ray Tracing"}, mode)) {
              main_camera->camera_render_mode = static_cast<Camera::CameraRenderMode>(mode);
              main_camera->ResetFrameCount();
            }
          }
          ImGui::EndChild();
        }

        if (main_camera_window_focused_ && !lock_entity_selection_ &&
            Input::GetKey(GLFW_KEY_ESCAPE) == Input::KeyActionType::Press) {
          SetSelectedEntity(Entity());
        }
        if (!ApplicationContext::Get().IsPlaying() && main_camera_window_focused_ && !lock_entity_selection_ &&
            Input::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press &&
            !(mouse_camera_window_position_.x < 0 || mouse_camera_window_position_.y < 0 ||
              mouse_camera_window_position_.x > view_port_size.x ||
              mouse_camera_window_position_.y > view_port_size.y)) {
          if (const auto focused_entity = MouseEntitySelection(main_camera, mouse_camera_window_position_);
              focused_entity == Entity()) {
            SetSelectedEntity(Entity());
          } else {
            Entity walker = focused_entity;
            bool found = false;
            while (walker.GetIndex() != 0) {
              if (walker == selected_entity_) {
                found = true;
                break;
              }
              walker = scene->GetParent(walker);
            }
            if (found) {
              walker = scene->GetParent(walker);
              if (walker.GetIndex() == 0) {
                SetSelectedEntity(focused_entity);
              } else {
                SetSelectedEntity(walker);
              }
            } else {
              SetSelectedEntity(focused_entity);
            }
          }
        }
      }

      ImGui::EndChild();
      if (const auto main_camera = scene->main_camera.Get<Camera>()) {
        main_camera->SetRequireRendering(!ImGui::GetCurrentWindowRead()->Hidden &&
                                         !ImGui::GetCurrentWindowRead()->Collapsed);
      }
      ImGui::PopStyleVar();
    } else {
      ImGui::Text("No Scene!");
    }
  }
  ImGui::End();

#pragma endregion
}

void EditorLayer::OnInputEvent(const Input::InputEvent& input_event) {
  // If main camera is focused, we pass the event to the scene.
  if (main_camera_window_focused_ && ApplicationContext::Get().IsPlaying()) {
    const auto active_scene = ApplicationContext::Get().GetActiveScene();
    auto& pressed_keys = active_scene->pressed_keys_;
    if (input_event.key_action == Input::KeyActionType::Press) {
      if (const auto search = pressed_keys.find(input_event.key); search != active_scene->pressed_keys_.end()) {
        // Dispatch hold if the key is already pressed.
        search->second = Input::KeyActionType::Hold;
      } else {
        // Dispatch press if the key is previously released.
        pressed_keys.insert({input_event.key, Input::KeyActionType::Press});
      }
    } else if (input_event.key_action == Input::KeyActionType::Release) {
      if (pressed_keys.find(input_event.key) != pressed_keys.end()) {
        // Dispatch hold if the key is already pressed.
        pressed_keys.erase(input_event.key);
      }
    }
  }
}

void EditorLayer::ResizeCameras() {
  if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>(); !render_layer)
    return;
  const auto& scene_camera = GetSceneCamera();
  if (const auto resolution = scene_camera->GetSize();
      scene_camera_resolution_x_ != 0 && scene_camera_resolution_y_ != 0 &&
      (resolution.x != scene_camera_resolution_x_ || resolution.y != scene_camera_resolution_y_)) {
    scene_camera->Resize({scene_camera_resolution_x_, scene_camera_resolution_y_});
  }
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (const std::shared_ptr<Camera> main_camera = scene->main_camera.Get<Camera>()) {
    if (main_camera_allow_auto_resize)
      main_camera->Resize({main_camera_resolution_x, main_camera_resolution_y});
  }
}

std::shared_ptr<Texture2D> EditorLayer::FindIcon(const std::string& name) {
  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    if (const auto search = editor_layer->editor_icons_.find(name); search != editor_layer->editor_icons_.end())
      return search->second;
  }
  return {};
}

std::vector<ConsoleMessage>& EditorLayer::GetConsoleMessages() {
  return console_messages_;
}

bool EditorLayer::SceneCameraWindowFocused() const {
  return scene_camera_window_focused_;
}

bool EditorLayer::MainCameraWindowFocused() const {
  return main_camera_window_focused_;
}

void EditorLayer::RegisterEditorCamera(const std::shared_ptr<Camera>& camera) {
  if (editor_cameras_.find(camera->GetHandle()) == editor_cameras_.end()) {
    editor_cameras_[camera->GetHandle()] = {};
    editor_cameras_.at(camera->GetHandle()).camera = camera;
  }
}

glm::vec3& EditorLayer::RefEditorCameraPosition(const Handle& handle) {
  return editor_cameras_.at(handle).position;
}

glm::quat& EditorLayer::RefEditorCameraRotation(const Handle& handle) {
  return editor_cameras_.at(handle).rotation;
}

glm::vec2 EditorLayer::GetMouseSceneCameraPosition() const {
  return mouse_scene_window_position_;
}

Input::KeyActionType EditorLayer::GetKey(const int key) {
  return Input::GetKey(key);
}

std::shared_ptr<Camera> EditorLayer::GetSceneCamera() {
  return editor_cameras_.at(scene_camera_handle_).camera;
}

glm::vec3 EditorLayer::GetSceneCameraPosition() const {
  return editor_cameras_.at(scene_camera_handle_).position;
}

glm::quat EditorLayer::GetSceneCameraRotation() const {
  return editor_cameras_.at(scene_camera_handle_).rotation;
}

void EditorLayer::SetSceneCameraPosition(const glm::vec3& target_position) {
  editor_cameras_.at(scene_camera_handle_).position = target_position;
}

void EditorLayer::SetSceneCameraRotation(const glm::quat& target_rotation) {
  editor_cameras_.at(scene_camera_handle_).rotation = target_rotation;
}

void EditorLayer::UpdateTextureId(ImTextureID& target, const VkSampler image_sampler, const VkImageView image_view,
                                  const VkImageLayout image_layout) {
  if (!ImGui::GetCurrentContext())
    return;
  if (target != 0)
    ImGui_ImplVulkan_RemoveTexture(reinterpret_cast<VkDescriptorSet>(target));
  target = reinterpret_cast<ImTextureID>(ImGui_ImplVulkan_AddTexture(image_sampler, image_view, image_layout));
}

Entity EditorLayer::GetSelectedEntity() const {
  return selected_entity_;
}

void EditorLayer::SetSelectedEntity(const Entity& entity, const bool open_menu) {
  if (entity == selected_entity_)
    return;
  selected_entity_hierarchy_list_.clear();
  const auto scene = GetScene();
  const auto previous_descendants = scene->GetDescendants(selected_entity_);
  for (const auto& i : previous_descendants) {
    scene->GetEntityMetadata(i).ancestor_selected = false;
  }
  if (scene->IsEntityValid(selected_entity_))
    scene->GetEntityMetadata(selected_entity_).ancestor_selected = false;
  if (entity.GetIndex() == 0) {
    selected_entity_ = Entity();
    lock_entity_selection_ = false;
    selection_alpha_ = 0;
    return;
  }

  if (!scene->IsEntityValid(entity))
    return;
  selected_entity_ = entity;
  const auto descendants = scene->GetDescendants(selected_entity_);

  for (const auto& i : descendants) {
    scene->GetEntityMetadata(i).ancestor_selected = true;
  }
  scene->GetEntityMetadata(selected_entity_).ancestor_selected = true;
  if (!open_menu)
    return;
  auto walker = entity;
  while (walker.GetIndex() != 0) {
    selected_entity_hierarchy_list_.push_back(walker);
    walker = scene->GetParent(walker);
  }
}

bool EditorLayer::GetLockEntitySelection() const {
  return lock_entity_selection_;
}

void EditorLayer::SetLockEntitySelection(const bool value) {
  const auto scene = GetScene();
  if (!value)
    lock_entity_selection_ = false;
  else if (scene->IsEntityValid(selected_entity_)) {
    lock_entity_selection_ = true;
  }
}

bool EditorLayer::UnsafeDroppableAsset(AssetRef& target, const std::vector<std::string>& type_names) {
  bool status_changed = false;
  if (ImGui::BeginDragDropTarget()) {
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Asset")) {
      const std::shared_ptr<IAsset> ptr = target.Get<IAsset>();
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      const Handle payload_n = *static_cast<Handle*>(payload->Data);
      if (!ptr || payload_n.GetValue() != target.GetAssetHandle().GetValue()) {
        const auto asset = AssetManager::GetAssetImpl(payload_n);
        for (const auto& type_name : type_names) {
          if (asset && asset->GetTypeName() == type_name) {
            target.Clear();
            target.asset_handle_ = payload_n;
            target.Update();
            status_changed = true;
            break;
          }
        }
      }
    }
    ImGui::EndDragDropTarget();
  }
  return status_changed;
}

bool EditorLayer::UnsafeDroppablePrivateComponent(PrivateComponentRef& target,
                                                  const std::vector<std::string>& type_names) {
  bool status_changed = false;
  if (ImGui::BeginDragDropTarget()) {
    const auto current_scene = ApplicationContext::Get().GetActiveScene();
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      const auto payload_n = *static_cast<Handle*>(payload->Data);
      const auto entity = current_scene->GetEntity(payload_n);
      if (current_scene->IsEntityValid(entity)) {
        for (const auto& type_name : type_names) {
          if (current_scene->HasPrivateComponent(entity, type_name)) {
            const auto ptr = target.Get<IPrivateComponent>();
            const auto new_private_component = current_scene->GetPrivateComponent(entity, type_name).lock();
            target = new_private_component;
            status_changed = true;
            break;
          }
        }
      }
    } else if (const ImGuiPayload* payload2 = ImGui::AcceptDragDropPayload("PrivateComponent")) {
      IM_ASSERT(payload2->DataSize == sizeof(Handle));
      const auto payload_n = *static_cast<Handle*>(payload2->Data);
      const auto entity = current_scene->GetEntity(payload_n);
      for (const auto& type_name : type_names) {
        if (current_scene->HasPrivateComponent(entity, type_name)) {
          target = current_scene->GetPrivateComponent(entity, type_name).lock();
          status_changed = true;
          break;
        }
      }
    }
    ImGui::EndDragDropTarget();
  }
  return status_changed;
}

bool EditorLayer::DragAndDropButton(EntityRef& entity_ref, const std::string& name, bool modifiable) {
  ImGui::Text(name.c_str());
  ImGui::SameLine();
  bool status_changed = false;
  if (const auto entity = entity_ref.Get(); entity.GetIndex() != 0) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    ImGui::Button(scene->GetEntityName(entity).c_str());
    Draggable(entity_ref);
    if (modifiable) {
      status_changed = Rename(entity_ref);
      status_changed = Remove(entity_ref) || status_changed;
    }
    if (!status_changed && ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
      selected_entity_ = entity;
    }
  } else {
    ImGui::Button("none");
  }
  status_changed = Droppable(entity_ref) || status_changed;
  return status_changed;
}
bool EditorLayer::Droppable(EntityRef& entity_ref) {
  bool status_changed = false;
  if (ImGui::BeginDragDropTarget()) {
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("Entity")) {
      const auto scene = ApplicationContext::Get().GetActiveScene();
      IM_ASSERT(payload->DataSize == sizeof(Handle));
      const auto payload_n = *static_cast<Handle*>(payload->Data);
      if (const auto new_entity = scene->GetEntity(payload_n); scene->IsEntityValid(new_entity)) {
        entity_ref = new_entity;
        status_changed = true;
      }
    }
    ImGui::EndDragDropTarget();
  }
  return status_changed;
}
void EditorLayer::Draggable(EntityRef& entity_ref) {
  const auto entity = entity_ref.Get();
  if (entity.GetIndex() != 0) {
    DraggableEntity(entity);
  }
}
void EditorLayer::DraggableEntity(const Entity& entity) {
  if (ImGui::BeginDragDropSource()) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto handle = scene->GetEntityHandle(entity);
    ImGui::SetDragDropPayload("Entity", &handle, sizeof(Handle));
    ImGui::TextColored(ImGui::GetStyleColorVec4(ImGuiCol_TextLink), scene->GetEntityName(entity).c_str());
    ImGui::EndDragDropSource();
  }
}
bool EditorLayer::Rename(EntityRef& entity_ref) {
  const auto entity = entity_ref.Get();
  const bool status_changed = RenameEntity(entity);
  return status_changed;
}
bool EditorLayer::Remove(EntityRef& entity_ref) {
  bool status_changed = false;
  const auto entity = entity_ref.Get();
  if (const auto scene = ApplicationContext::Get().GetActiveScene(); scene->IsEntityValid(entity)) {
    const std::string tag = "##Entity" + std::to_string(scene->GetEntityHandle(entity));
    if (ImGui::BeginPopupContextItem(tag.c_str())) {
      if (ImGui::Button(("Remove" + tag).c_str())) {
        entity_ref.Clear();
        status_changed = true;
      }
      ImGui::EndPopup();
    }
  }
  return status_changed;
}

void EditorLayer::MouseEntitySelection() {
  const auto scene = GetScene();
  auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>();
  auto& [sceneCameraRotation, sceneCameraPosition, sceneCamera] = editor_cameras_.at(scene_camera_handle_);
#pragma region Scene Window
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
  if (ImGui::Begin("Scene")) {
    ImVec2 view_port_size;
    // Using a Child allow to fill all the space of the window.
    // It also allows customization
    if (ImGui::BeginChild("SceneCameraRenderer", ImVec2(0, 0), false)) {
      view_port_size = ImGui::GetWindowSize();
    }
#pragma region Gizmos and Entity Selection
    if (scene_camera_window_focused_ && !lock_entity_selection_ &&
        Input::GetKey(GLFW_KEY_ESCAPE) == Input::KeyActionType::Press) {
      SetSelectedEntity(Entity());
    }
    if (scene_camera_window_focused_ && !lock_entity_selection_ && !gizmo_using_ &&
        Input::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press &&
        !(mouse_scene_window_position_.x < 0 || mouse_scene_window_position_.y < 0 ||
          mouse_scene_window_position_.x > view_port_size.x || mouse_scene_window_position_.y > view_port_size.y)) {
      if (const auto focused_entity = MouseEntitySelection(sceneCamera, mouse_scene_window_position_);
          focused_entity == Entity()) {
        SetSelectedEntity(Entity());
      } else {
        Entity walker = focused_entity;
        bool found = false;
        while (walker.GetIndex() != 0) {
          if (walker == selected_entity_) {
            found = true;
            break;
          }
          walker = scene->GetParent(walker);
        }
        if (found) {
          walker = scene->GetParent(walker);
          if (walker.GetIndex() == 0) {
            SetSelectedEntity(focused_entity);
          } else {
            SetSelectedEntity(walker);
          }
        } else {
          SetSelectedEntity(focused_entity);
        }
      }
    }
#pragma endregion
    ImGui::EndChild();
  }
  ImGui::End();
  ImGui::PopStyleVar();
#pragma endregion
}

Entity EditorLayer::MouseEntitySelection(const std::shared_ptr<Camera>& target_camera,
                                         const glm::vec2& mouse_position) const {
  Entity ret_val;
  const auto& g_buffer_normal = target_camera->GetGBufferNormalImage();
  const glm::vec2 resolution = target_camera->GetSize();
  glm::vec2 point = resolution;
  point.x = mouse_position.x;
  point.y -= mouse_position.y;
  if (point.x >= 0 && point.x < resolution.x && point.y >= 0 && point.y < resolution.y) {
    VkBufferImageCopy image_copy;
    image_copy.bufferOffset = 0;
    image_copy.bufferRowLength = 0;
    image_copy.bufferImageHeight = 0;
    image_copy.imageSubresource.layerCount = 1;
    image_copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    image_copy.imageSubresource.baseArrayLayer = 0;
    image_copy.imageSubresource.mipLevel = 0;
    image_copy.imageExtent.width = 1;
    image_copy.imageExtent.height = 1;
    image_copy.imageExtent.depth = 1;
    image_copy.imageOffset.x = static_cast<int32_t>(point.x);
    image_copy.imageOffset.y = static_cast<int32_t>(point.y);
    image_copy.imageOffset.z = 0;
    entity_index_read_buffer_->CopyFromImage(*g_buffer_normal, image_copy);
    float val = -1;
    switch (Platform::Constants::texture_2d) {
      case VK_FORMAT_R32G32B32A32_SFLOAT: {
        const auto* ptr = static_cast<float*>(mapped_entity_index_data_);
        val = glm::round(ptr[3]);
        break;
      }
      case VK_FORMAT_R16G16B16A16_SFLOAT: {
        const auto* ptr = static_cast<glm::detail::hdata*>(mapped_entity_index_data_);
        val = glm::round(glm::detail::toFloat32(ptr[3]));
        break;
      }
    }
    if (const int32_t instance_index = static_cast<int>(val); instance_index > 0) {
      const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
      const auto scene = GetScene();
      if (const auto handle = render_layer->GetCurrentRenderInstanceStorage()->GetInstanceEntityHandle(instance_index);
          handle != 0)
        ret_val = scene->GetEntity(handle);
    }
  }
  return ret_val;
}

bool EditorLayer::RenameEntity(const Entity& entity) {
  constexpr bool status_changed = false;
  if (const auto scene = ApplicationContext::Get().GetActiveScene(); scene->IsEntityValid(entity)) {
    const std::string tag = "##Entity" + std::to_string(scene->GetEntityHandle(entity));
    if (ImGui::BeginPopupContextItem(tag.c_str())) {
      if (ImGui::BeginMenu(("Rename" + tag).c_str())) {
        static char new_name[256];
        ImGui::InputText(("New name" + tag).c_str(), new_name, 256);
        if (ImGui::Button(("Confirm" + tag).c_str())) {
          scene->SetEntityName(entity, std::string(new_name));
          memset(new_name, 0, 256);
        }
        ImGui::EndMenu();
      }
      ImGui::EndPopup();
    }
  }
  return status_changed;
}

bool EditorLayer::DragAndDropButton(AssetRef& target, const std::string& name,
                                    const std::vector<std::string>& acceptable_type_names, bool modifiable) {
  ImGui::Text(name.c_str());
  ImGui::SameLine();
  const auto ptr = target.Get<IAsset>();
  bool status_changed = false;
  ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_Header));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyleColorVec4(ImGuiCol_HeaderHovered));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetStyleColorVec4(ImGuiCol_HeaderActive));
  if (ptr) {
    const auto title = ptr->GetTitle();
    ImGui::Button(title.c_str());
    DraggableAsset(ptr);
    if (modifiable) {
      status_changed = Rename(target);
      status_changed = Remove(target) || status_changed;
    }
    if (!status_changed && ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
      OpenAssetInspector(ptr);
    }
  } else {
    ImGui::Button("none");
  }
  ImGui::PopStyleColor(3);
  status_changed = UnsafeDroppableAsset(target, acceptable_type_names) || status_changed;
  return status_changed;
}
bool EditorLayer::DragAndDropButton(PrivateComponentRef& target, const std::string& name,
                                    const std::vector<std::string>& acceptable_type_names, const bool modifiable) {
  ImGui::Text(name.c_str());
  ImGui::SameLine();
  bool status_changed = false;
  const auto ptr = target.Get<IPrivateComponent>();
  ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_Button));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyleColorVec4(ImGuiCol_HeaderHovered));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetStyleColorVec4(ImGuiCol_HeaderActive));
  if (ptr) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    ImGui::Button(scene->GetEntityName(ptr->GetOwner()).c_str());
    const std::string tag = "##" + ptr->GetTypeName() + std::to_string(ptr->GetHandle());
    DraggablePrivateComponent(ptr);
    if (modifiable) {
      status_changed = Remove(target);
    }
  } else {
    ImGui::Button("none");
  }
  ImGui::PopStyleColor(3);
  status_changed = UnsafeDroppablePrivateComponent(target, acceptable_type_names) || status_changed;
  return status_changed;
}

void EditorLayer::LoadIcons() {
  const auto default_resources = std::filesystem::path("./DefaultResources");
  auto load_icon = [&](const std::string& name, const std::filesystem::path& path) {
    auto icon = AssetManager::CreateTemporaryAsset<Texture2D>();
    Serialization::LoadAsset(*icon, path);
    editor_icons_[name] = std::move(icon);
  };

  load_icon("Scene", default_resources / "Editor/Assets/Scene.png");
  load_icon("Binary", default_resources / "Editor/Assets/Binary.png");
  load_icon("Folder", default_resources / "Editor/Assets/Folder.png");
  load_icon("Material", default_resources / "Editor/Assets/Material.png");
  load_icon("Mesh", default_resources / "Editor/Assets/Mesh.png");
  load_icon("Prefab", default_resources / "Editor/Assets/Prefab.png");
  load_icon("Texture2D", default_resources / "Editor/Assets/Texture2D.png");
  load_icon("TitleBarLogo", default_resources / "Editor/TitleBar/EvoEngine64White.png");
  load_icon("WindowMinimize", default_resources / "Editor/Window/Minimize.png");
  load_icon("WindowMaximize", default_resources / "Editor/Window/Maximize.png");
  load_icon("WindowRestore", default_resources / "Editor/Window/Restore.png");
  load_icon("WindowClose", default_resources / "Editor/Window/Close.png");
  load_icon("PlayButton", default_resources / "Editor/Viewport/Play.png");
  load_icon("PauseButton", default_resources / "Editor/Viewport/Pause.png");
  load_icon("StopButton", default_resources / "Editor/Viewport/Stop.png");
  load_icon("StepButton", default_resources / "Editor/Viewport/Simulate.png");
  load_icon("BackButton", default_resources / "Editor/Navigation/back.png");
  load_icon("LeftButton", default_resources / "Editor/Navigation/left.png");
  load_icon("RightButton", default_resources / "Editor/Navigation/right.png");
  load_icon("RefreshButton", default_resources / "Editor/Navigation/refresh.png");
  load_icon("InfoButton", default_resources / "Editor/Console/InfoButton.png");
  load_icon("ErrorButton", default_resources / "Editor/Console/ErrorButton.png");
  load_icon("WarningButton", default_resources / "Editor/Console/WarningButton.png");
}

void EditorLayer::CameraWindowDragAndDrop() const {
  if (AssetRef asset_ref;
      UnsafeDroppableAsset(asset_ref, {"Scene", "Prefab", "Mesh", "Strands", "Cubemap", "EnvironmentalMap"})) {
    const auto scene = GetScene();
    if (const auto asset = asset_ref.Get<IAsset>();
        !ApplicationContext::Get().IsPlaying() && asset->GetTypeName() == "Scene") {
      const auto new_scene = std::dynamic_pointer_cast<Scene>(asset);
      ProjectManager::SetStartScene(new_scene);
      ProjectManager::SaveProject();
      ApplicationContext::Get().Attach(new_scene);
    } else if (asset->GetTypeName() == "Prefab") {
      const auto entity = std::dynamic_pointer_cast<Prefab>(asset)->ToEntity(scene, true, true);
      scene->SetEntityName(entity, asset->GetTitle());
    } else if (asset->GetTypeName() == "Mesh") {
      const auto entity = scene->CreateEntity(asset->GetTitle());
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
      mesh_renderer->mesh.Set<Mesh>(std::dynamic_pointer_cast<Mesh>(asset));
      const auto material = AssetManager::CreateTemporaryAsset<Material>();
      mesh_renderer->material.Set<Material>(material);
    } else if (asset->GetTypeName() == "Strands") {
      const auto entity = scene->CreateEntity(asset->GetTitle());
      const auto strands_renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(entity).lock();
      strands_renderer->strands.Set<Strands>(std::dynamic_pointer_cast<Strands>(asset));
      const auto material = AssetManager::CreateTemporaryAsset<Material>();
      strands_renderer->material.Set<Material>(material);
    } else if (asset->GetTypeName() == "EnvironmentalMap") {
      scene->environment.environmental_map = std::dynamic_pointer_cast<EnvironmentalMap>(asset);
    } else if (asset->GetTypeName() == "Cubemap") {
      const auto main_camera = scene->main_camera.Get<Camera>();
      main_camera->skybox = std::dynamic_pointer_cast<Cubemap>(asset);
    }
  }
}

void EditorLayer::MoveCamera(const glm::quat& target_rotation, const glm::vec3& target_position,
                             const float& transition_time) {
  auto& [sceneCameraRotation, sceneCameraPosition, sceneCamera] = editor_cameras_.at(scene_camera_handle_);
  previous_rotation_ = sceneCameraRotation;
  previous_position_ = sceneCameraPosition;
  transition_time_ = transition_time;
  transition_timer_ = static_cast<float>(ApplicationContext::Get().GetTimes().Now());
  target_rotation_ = target_rotation;
  target_position_ = target_position;
  lock_camera = true;
}

bool EditorLayer::LocalPositionSelected() const {
  return local_position_selected_;
}

bool EditorLayer::LocalRotationSelected() const {
  return local_rotation_selected_;
}

bool EditorLayer::LocalScaleSelected() const {
  return local_scale_selected_;
}

glm::vec3& EditorLayer::UnsafeGetPreviouslyStoredPosition() {
  return previously_stored_position_;
}

glm::vec3& EditorLayer::UnsafeGetPreviouslyStoredRotation() {
  return previously_stored_rotation_;
}

glm::vec3& EditorLayer::UnsafeGetPreviouslyStoredScale() {
  return previously_stored_scale_;
}
