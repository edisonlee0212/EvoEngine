#include "EditorLayer.hpp"
#include "Application.hpp"
#include "AssetManager.hpp"
#include "Cubemap.hpp"
#include "EditorTheme.hpp"
#include "EntityBatchInspector.hpp"
#include "EnvironmentalLighting.hpp"
#include "EnvironmentalMap.hpp"
#include "FileManager.hpp"
#include "GaussianSplat.hpp"
#include "GaussianSplatRenderer.hpp"
#include "ILayer.hpp"
#include "InspectorRegistry.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "PackageManager.hpp"
#include "Platform.hpp"
#include "PostProcessingStack.hpp"
#include "Prefab.hpp"
#include "Profiler.hpp"
#include "ProfilerPanelModel.hpp"
#include "ProjectContentBrowserPanel.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "SDKInspectionAdapters.hpp"
#include "Scene.hpp"
#include "Serialization.hpp"
#include "StrandsRenderer.hpp"
#include "Times.hpp"
#include "UnknownPrivateComponent.hpp"
#include "Utilities.hpp"
#include "WindowLayer.hpp"

#include "imgui_internal.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace evo_engine {
struct ProfilerBreakdownSeries {
  std::string key;
  std::string name;
  std::vector<float> values;
};

struct CpuProfilerSeries {
  std::string key;
  std::string name;
  profiler_panel_detail::CpuExecutorGroup group = profiler_panel_detail::CpuExecutorGroup::Other;
  std::vector<float> values;
  double average_milliseconds = 0.0;
};

struct ProfilerPanelState {
  struct CpuNode {
    std::string key;
    std::string name;
    std::string category;
    std::vector<CpuNode> children;
  };
  struct CpuThread {
    uint64_t id = 0;
    std::string name;
    std::vector<CpuNode> roots;
  };
  struct GpuPass {
    std::string key;
    GpuTimestampScopeMetadata metadata;
  };
  struct GpuGroup {
    std::string name;
    std::vector<GpuPass> passes;
  };

  std::vector<CpuThread> cpu_threads;
  std::vector<GpuGroup> gpu_groups;
  std::vector<GpuTimestampFrameSnapshot> gpu_frames;
  std::unordered_set<std::string> cataloged_cpu_frames;
  std::unordered_set<std::string> cataloged_gpu_frames;
  std::unordered_set<std::string> pinned_gpu_passes;
  std::unordered_set<std::string> hidden_gpu_passes;
  std::unordered_set<std::string> pinned_cpu_scopes;
  std::unordered_set<std::string> hidden_cpu_scopes;
  std::array<char, 128> gpu_filter{};
  std::array<char, 128> cpu_plot_filter{};
  int gpu_top_series_count = 8;
  int cpu_top_series_count = 8;
  ProfilerHistoryStats breakdown_cpu_history;
  GpuTimestampHistoryStats breakdown_gpu_history;
  std::vector<GpuTimestampFrameSnapshot> breakdown_aligned_gpu_frames;
  std::vector<ProfilerBreakdownSeries> breakdown_cpu_series;
  std::vector<ProfilerBreakdownSeries> breakdown_gpu_series;
  std::chrono::steady_clock::time_point breakdown_next_refresh{};
  int breakdown_selected_frame_index = -1;
  std::optional<ProfilerFrameStats> diagnostics_cpu_frame;
  std::optional<GpuTimestampFrameSnapshot> diagnostics_gpu_frame;
  bool diagnostics_follow_latest = false;
};
}  // namespace evo_engine

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

struct KeyBindingCaptureState {
  ImGuiID id = 0;
  int start_frame = -1;
};

std::string KeyboardKeyName(const int key) {
  if (key >= GLFW_KEY_F1 && key <= GLFW_KEY_F25)
    return "F" + std::to_string(key - GLFW_KEY_F1 + 1);
  if (key >= GLFW_KEY_KP_0 && key <= GLFW_KEY_KP_9)
    return "Keypad " + std::to_string(key - GLFW_KEY_KP_0);
  switch (key) {
    case GLFW_KEY_SPACE:
      return "Space";
    case GLFW_KEY_ESCAPE:
      return "Escape";
    case GLFW_KEY_ENTER:
      return "Enter";
    case GLFW_KEY_TAB:
      return "Tab";
    case GLFW_KEY_BACKSPACE:
      return "Backspace";
    case GLFW_KEY_INSERT:
      return "Insert";
    case GLFW_KEY_DELETE:
      return "Delete";
    case GLFW_KEY_RIGHT:
      return "Right Arrow";
    case GLFW_KEY_LEFT:
      return "Left Arrow";
    case GLFW_KEY_DOWN:
      return "Down Arrow";
    case GLFW_KEY_UP:
      return "Up Arrow";
    case GLFW_KEY_PAGE_UP:
      return "Page Up";
    case GLFW_KEY_PAGE_DOWN:
      return "Page Down";
    case GLFW_KEY_HOME:
      return "Home";
    case GLFW_KEY_END:
      return "End";
    case GLFW_KEY_CAPS_LOCK:
      return "Caps Lock";
    case GLFW_KEY_SCROLL_LOCK:
      return "Scroll Lock";
    case GLFW_KEY_NUM_LOCK:
      return "Num Lock";
    case GLFW_KEY_PRINT_SCREEN:
      return "Print Screen";
    case GLFW_KEY_PAUSE:
      return "Pause";
    case GLFW_KEY_KP_DECIMAL:
      return "Keypad Decimal";
    case GLFW_KEY_KP_DIVIDE:
      return "Keypad Divide";
    case GLFW_KEY_KP_MULTIPLY:
      return "Keypad Multiply";
    case GLFW_KEY_KP_SUBTRACT:
      return "Keypad Subtract";
    case GLFW_KEY_KP_ADD:
      return "Keypad Add";
    case GLFW_KEY_KP_ENTER:
      return "Keypad Enter";
    case GLFW_KEY_KP_EQUAL:
      return "Keypad Equal";
    case GLFW_KEY_LEFT_SHIFT:
      return "Left Shift";
    case GLFW_KEY_LEFT_CONTROL:
      return "Left Control";
    case GLFW_KEY_LEFT_ALT:
      return "Left Alt";
    case GLFW_KEY_LEFT_SUPER:
      return "Left Super";
    case GLFW_KEY_RIGHT_SHIFT:
      return "Right Shift";
    case GLFW_KEY_RIGHT_CONTROL:
      return "Right Control";
    case GLFW_KEY_RIGHT_ALT:
      return "Right Alt";
    case GLFW_KEY_RIGHT_SUPER:
      return "Right Super";
    case GLFW_KEY_MENU:
      return "Menu";
    default:
      break;
  }
  if (key >= GLFW_KEY_SPACE && key <= GLFW_KEY_LAST) {
    if (const char* name = glfwGetKeyName(key, 0)) {
      std::string result = name;
      if (result.size() == 1)
        result[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(result[0])));
      return result;
    }
  }
  return "Unknown (" + std::to_string(key) + ")";
}

std::string MouseButtonName(const int button) {
  switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT:
      return "Mouse Left";
    case GLFW_MOUSE_BUTTON_RIGHT:
      return "Mouse Right";
    case GLFW_MOUSE_BUTTON_MIDDLE:
      return "Mouse Middle";
    default:
      return button >= 0 && button <= GLFW_MOUSE_BUTTON_LAST ? "Mouse " + std::to_string(button + 1)
                                                             : "Unknown (" + std::to_string(button) + ")";
  }
}

void DrawKeyBinding(const char* label, int& binding, const bool mouse,
                    const std::unordered_map<int, Input::KeyActionType>& pressed_keys) {
  static KeyBindingCaptureState capture;
  ImGui::PushID(label);
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(label);
  ImGui::SameLine(150.0f);
  const ImGuiID id = ImGui::GetID("Binding");
  const bool capturing = capture.id == id;
  const std::string button_text = (capturing ? "Press a key..."
                                   : mouse   ? MouseButtonName(binding)
                                             : KeyboardKeyName(binding)) +
                                  "###Binding";
  if (ImGui::Button(button_text.c_str(), ImVec2(150.0f, 0.0f))) {
    capture.id = id;
    capture.start_frame = ImGui::GetFrameCount();
  }
  if (capture.id == id && ImGui::GetFrameCount() > capture.start_frame) {
    for (const auto& [key, action] : pressed_keys) {
      if (action != Input::KeyActionType::Press)
        continue;
      if (key == GLFW_KEY_ESCAPE) {
        capture.id = 0;
        break;
      }
      const bool valid =
          mouse ? key >= 0 && key <= GLFW_MOUSE_BUTTON_LAST : key >= GLFW_KEY_SPACE && key <= GLFW_KEY_LAST;
      if (valid) {
        binding = key;
        capture.id = 0;
        break;
      }
    }
  }
  ImGui::PopID();
}

float ClampFiniteNonnegative(const float value, const float fallback) {
  return std::isfinite(value) && value >= 0.0f ? value : fallback;
}

float MoveTowards(const float current, const float target, const float max_delta) {
  const float delta = target - current;
  if (std::abs(delta) <= max_delta) {
    return target;
  }
  return current + std::copysign(max_delta, delta);
}

glm::vec3 MoveTowards(const glm::vec3& current, const glm::vec3& target, const float max_delta) {
  const glm::vec3 delta = target - current;
  const float distance = glm::length(delta);
  if (!std::isfinite(distance) || distance <= glm::epsilon<float>() || distance <= max_delta) {
    return target;
  }
  return current + delta / distance * max_delta;
}

void ResetEditorCameraFreeFlyState(EditorCameraFreeFlyState& state) {
  state.was_dragging = false;
  state.previous_mouse_x = 0.0f;
  state.previous_mouse_y = 0.0f;
  state.smoothed_move_velocity = glm::vec3(0.0f);
  state.look_response = 0.0f;
}

float CameraControlResponseStep(const float response_time, const float delta_time) {
  if (response_time <= glm::epsilon<float>()) {
    return 1.0f;
  }
  return std::clamp(delta_time / response_time, 0.0f, 1.0f);
}

glm::vec3 SanitizeVelocity(const glm::vec3& velocity) {
  return std::isfinite(velocity.x) && std::isfinite(velocity.y) && std::isfinite(velocity.z) ? velocity
                                                                                             : glm::vec3(0.0f);
}

bool MatricesNear(const glm::mat4& lhs, const glm::mat4& rhs) {
  for (glm::length_t column = 0; column < 4; ++column) {
    for (glm::length_t row = 0; row < 4; ++row) {
      if (glm::abs(lhs[column][row] - rhs[column][row]) > 1.0e-4f) {
        return false;
      }
    }
  }
  return true;
}

bool IsFiniteVector(const glm::vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

struct AspectFitRect {
  ImVec2 offset = {0.0f, 0.0f};
  ImVec2 size = {0.0f, 0.0f};
};

AspectFitRect CalculateAspectFitRect(const ImVec2& container_size, const glm::uvec2& texture_size) {
  AspectFitRect rect;
  if (container_size.x <= 0.0f || container_size.y <= 0.0f || texture_size.x == 0u || texture_size.y == 0u) {
    return rect;
  }
  const auto container_ratio = container_size.x / container_size.y;
  const auto texture_ratio = static_cast<float>(texture_size.x) / static_cast<float>(texture_size.y);
  if (container_ratio > texture_ratio) {
    rect.size.y = container_size.y;
    rect.size.x = rect.size.y * texture_ratio;
    rect.offset.x = (container_size.x - rect.size.x) * 0.5f;
  } else {
    rect.size.x = container_size.x;
    rect.size.y = rect.size.x / texture_ratio;
    rect.offset.y = (container_size.y - rect.size.y) * 0.5f;
  }
  return rect;
}

bool TryMapAspectFitMouseToTexture(const AspectFitRect& rect, const glm::uvec2& texture_size,
                                   const glm::vec2& panel_mouse_position, glm::vec2& texture_mouse_position) {
  const glm::vec2 local_position(panel_mouse_position.x - rect.offset.x, panel_mouse_position.y - rect.offset.y);
  if (rect.size.x <= 0.0f || rect.size.y <= 0.0f || local_position.x < 0.0f || local_position.y < 0.0f ||
      local_position.x >= rect.size.x || local_position.y >= rect.size.y) {
    return false;
  }
  texture_mouse_position.x = local_position.x / rect.size.x * static_cast<float>(texture_size.x);
  texture_mouse_position.y = local_position.y / rect.size.y * static_cast<float>(texture_size.y);
  return texture_mouse_position.x >= 0.0f && texture_mouse_position.y >= 0.0f &&
         texture_mouse_position.x < static_cast<float>(texture_size.x) &&
         texture_mouse_position.y < static_cast<float>(texture_size.y);
}

std::string FormatRenderCounter(const size_t value) {
  if (value < 999) {
    return std::to_string(value);
  }
  if (value < 999999) {
    return std::to_string(static_cast<int>(value / 1000)) + "K";
  }
  return std::to_string(static_cast<int>(value / 1000000)) + "M";
}

void DrawRenderCounterSummary(const Platform& graphics, const uint32_t current_frame_index) {
  const auto prim_count =
      current_frame_index < graphics.prim_count.size() ? graphics.prim_count[current_frame_index] : 0u;
  const auto draw_call_count =
      current_frame_index < graphics.draw_call.size() ? graphics.draw_call[current_frame_index] : 0u;
  ImGui::Text("%s tris", FormatRenderCounter(prim_count).c_str());
  ImGui::Text("%llu draw submissions", static_cast<unsigned long long>(draw_call_count));
}

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

ImU32 ProfilerCategoryColor(const std::string& category) {
  const auto hash = std::hash<std::string>{}(category);
  const float hue = static_cast<float>(hash % 360) / 360.0f;
  float r = 0.0f;
  float g = 0.0f;
  float b = 0.0f;
  ImGui::ColorConvertHSVtoRGB(hue, 0.55f, 0.90f, r, g, b);
  return ImGui::GetColorU32(ImVec4(r, g, b, 0.85f));
}

std::filesystem::path DefaultProfilerTracePath() {
  if (ProjectManager::HasProject()) {
    return ProjectManager::GetProjectPath().parent_path() / "ProfilerTrace.json";
  }
  return std::filesystem::current_path() / "ProfilerTrace.json";
}

bool TextContainsCaseInsensitive(const std::string& text, const std::string& query);

struct ProfilerHistorySummary {
  double average_ms = 0.0;
  double maximum_ms = 0.0;
  double p95_ms = 0.0;
  size_t frame_count = 0;
};

using ProfilerHistorySummaries = std::unordered_map<std::string, ProfilerHistorySummary>;

std::string ProfilerNodePath(const uint64_t thread_id, const std::string& parent_path,
                             const ProfilerHierarchyNode& node) {
  return std::to_string(thread_id) + "\x1f" + parent_path + "\x1f" + node.category + "\x1f" + node.name;
}

void UpdateProfilerCpuCatalogNodes(const uint64_t thread_id, const std::vector<ProfilerHierarchyNode>& nodes,
                                   const std::string& parent_path,
                                   std::vector<ProfilerPanelState::CpuNode>& catalog_nodes) {
  for (const auto& node : nodes) {
    const auto key = ProfilerNodePath(thread_id, parent_path, node);
    auto& catalog = profiler_panel_detail::AppendFirstSeen(
        catalog_nodes, key,
        [](const auto& entry) -> const std::string& {
          return entry.key;
        },
        ProfilerPanelState::CpuNode{key, node.name, node.category});
    UpdateProfilerCpuCatalogNodes(thread_id, node.children, key, catalog.children);
  }
}

void UpdateProfilerCpuCatalog(const std::vector<ProfilerFrameStats>& frames, ProfilerPanelState& state) {
  for (const auto& frame : frames) {
    const auto frame_key = std::to_string(frame.capture_session_index) + "\x1f" + std::to_string(frame.frame_index);
    if (!state.cataloged_cpu_frames.emplace(frame_key).second)
      continue;
    for (const auto& lane : frame.thread_lanes) {
      auto& thread = profiler_panel_detail::AppendFirstSeen(
          state.cpu_threads, lane.thread_id,
          [](const auto& entry) {
            return entry.id;
          },
          ProfilerPanelState::CpuThread{lane.thread_id, lane.thread_name});
      UpdateProfilerCpuCatalogNodes(lane.thread_id, lane.hierarchy, "", thread.roots);
    }
  }
}

void CollectProfilerCurrentNodes(const uint64_t thread_id, const std::vector<ProfilerHierarchyNode>& nodes,
                                 const std::string& parent_path,
                                 std::unordered_map<std::string, const ProfilerHierarchyNode*>& current_nodes) {
  for (const auto& node : nodes) {
    const auto path = ProfilerNodePath(thread_id, parent_path, node);
    current_nodes[path] = &node;
    CollectProfilerCurrentNodes(thread_id, node.children, path, current_nodes);
  }
}

void CollectProfilerHistorySamples(const uint64_t thread_id, const std::vector<ProfilerHierarchyNode>& nodes,
                                   const std::string& parent_path,
                                   std::unordered_map<std::string, std::vector<double>>& samples) {
  for (const auto& node : nodes) {
    const auto path = ProfilerNodePath(thread_id, parent_path, node);
    samples[path].emplace_back(node.inclusive_ms);
    CollectProfilerHistorySamples(thread_id, node.children, path, samples);
  }
}

ProfilerHistorySummaries BuildProfilerHistorySummaries(const std::vector<ProfilerFrameStats>& frames) {
  std::unordered_map<std::string, std::vector<double>> samples;
  for (const auto& frame : frames) {
    for (const auto& lane : frame.thread_lanes) {
      CollectProfilerHistorySamples(lane.thread_id, lane.hierarchy, "", samples);
    }
  }

  ProfilerHistorySummaries summaries;
  for (auto& [path, values] : samples) {
    auto& summary = summaries[path];
    const auto rolling = profiler_panel_detail::SummarizeWithMissingZeros(std::move(values), frames.size());
    summary.average_ms = rolling.average;
    summary.maximum_ms = rolling.maximum;
    summary.p95_ms = rolling.p95;
    summary.frame_count = rolling.observed_count;
  }
  return summaries;
}

bool ProfilerHierarchyMatchesFilter(const ProfilerHierarchyNode& node, const std::string& filter) {
  if (TextContainsCaseInsensitive(node.name, filter) || TextContainsCaseInsensitive(node.category, filter)) {
    return true;
  }
  return std::any_of(node.children.begin(), node.children.end(), [&](const auto& child) {
    return ProfilerHierarchyMatchesFilter(child, filter);
  });
}

bool ProfilerHierarchyMatchesFilter(const ProfilerPanelState::CpuNode& node, const std::string& filter) {
  if (TextContainsCaseInsensitive(node.name, filter) || TextContainsCaseInsensitive(node.category, filter)) {
    return true;
  }
  return std::any_of(node.children.begin(), node.children.end(), [&](const auto& child) {
    return ProfilerHierarchyMatchesFilter(child, filter);
  });
}

std::string ProfilerScopeDisplayName(const std::string& name) {
  if (name == "Application::Loop")
    return "Application Loop";
  if (name.rfind("Application::", 0) == 0)
    return name.substr(13);
  return name;
}

void DrawProfilerHierarchyNode(const ProfilerPanelState::CpuNode& catalog_node,
                               const std::unordered_map<std::string, const ProfilerHierarchyNode*>& current_nodes,
                               const double parent_ms, const double frame_ms, const ProfilerHistorySummaries& summaries,
                               const std::string& filter, const bool ancestor_matches = false) {
  const bool node_matches = TextContainsCaseInsensitive(catalog_node.name, filter) ||
                            TextContainsCaseInsensitive(catalog_node.category, filter);
  if (!ancestor_matches && !node_matches && !ProfilerHierarchyMatchesFilter(catalog_node, filter)) {
    return;
  }

  const auto current_search = current_nodes.find(catalog_node.key);
  const auto* node = current_search == current_nodes.end() ? nullptr : current_search->second;
  const auto summary_search = summaries.find(catalog_node.key);
  const ProfilerHistorySummary summary =
      summary_search == summaries.end() ? ProfilerHistorySummary{} : summary_search->second;
  const bool has_children = !catalog_node.children.empty();
  ImGui::TableNextRow();
  ImGui::TableNextColumn();
  ImGui::PushID(catalog_node.key.c_str());
  ImGuiTreeNodeFlags flags =
      ImGuiTreeNodeFlags_SpanFullWidth | ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;
  if (!filter.empty())
    flags |= ImGuiTreeNodeFlags_DefaultOpen;
  if (!has_children)
    flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
  const auto display_name = ProfilerScopeDisplayName(catalog_node.name);
  const bool open = ImGui::TreeNodeEx("Scope", flags, "%s", display_name.c_str());
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("%s\nCategory: %s%s", catalog_node.name.c_str(), catalog_node.category.c_str(),
                      node ? "" : "\nNot present in the selected frame");
  }
  ImGui::TableNextColumn();
  ImGui::Text("%.3f", node ? node->inclusive_ms : 0.0);
  ImGui::TableNextColumn();
  ImGui::Text("%.3f", node ? node->self_ms : 0.0);
  ImGui::TableNextColumn();
  ImGui::Text("%u", node ? node->count : 0);
  ImGui::TableNextColumn();
  ImGui::Text("%.1f%%", node && parent_ms > 0.0 ? node->inclusive_ms / parent_ms * 100.0 : 0.0);
  ImGui::TableNextColumn();
  ImGui::Text("%.1f%%", node && frame_ms > 0.0 ? node->inclusive_ms / frame_ms * 100.0 : 0.0);
  ImGui::TableNextColumn();
  ImGui::Text("%.3f", summary.average_ms);
  ImGui::TableNextColumn();
  ImGui::Text("%.3f", summary.maximum_ms);
  ImGui::TableNextColumn();
  ImGui::Text("%.3f", summary.p95_ms);
  if (open && has_children) {
    for (const auto& child : catalog_node.children) {
      DrawProfilerHierarchyNode(child, current_nodes, node ? node->inclusive_ms : 0.0, frame_ms, summaries, filter,
                                ancestor_matches || node_matches);
    }
    ImGui::TreePop();
  }
  ImGui::PopID();
}

std::string GpuProfilerPassKey(const GpuTimestampScopeMetadata& metadata) {
  const auto& pass_id = metadata.stable_pass_id.empty() ? metadata.display_name : metadata.stable_pass_id;
  return metadata.group + "\x1f" + pass_id;
}

std::string GpuProfilerFrameKey(const uint64_t session_index, const uint64_t frame_index) {
  return std::to_string(session_index) + "\x1f" + std::to_string(frame_index);
}

struct GpuProfilerHistorySummary {
  GpuTimestampScopeMetadata metadata{};
  GpuTimestampStats stats{};
};

using GpuProfilerHistorySummaries = std::unordered_map<std::string, GpuProfilerHistorySummary>;

ImU32 GpuProfilerSeriesColor(const std::string& key);

void CatalogCpuProfilerSeries(const profiler_panel_detail::CpuExecutorGroup group,
                              const std::vector<ProfilerHierarchyNode>& nodes, const std::string& parent_path,
                              std::vector<CpuProfilerSeries>& series,
                              std::unordered_map<std::string, size_t>& series_indices) {
  for (const auto& node : nodes) {
    const auto key = profiler_panel_detail::StableHierarchyKey(group, parent_path, node.category, node.name);
    if (series_indices.emplace(key, series.size()).second) {
      auto& item = series.emplace_back();
      item.key = key;
      item.name = ProfilerScopeDisplayName(node.name);
      item.group = group;
    }
    CatalogCpuProfilerSeries(group, node.children, key, series, series_indices);
  }
}

void AccumulateCpuProfilerSeries(const profiler_panel_detail::CpuExecutorGroup group,
                                 const std::vector<ProfilerHierarchyNode>& nodes, const std::string& parent_path,
                                 std::unordered_map<std::string, double>& values) {
  for (const auto& node : nodes) {
    const auto key = profiler_panel_detail::StableHierarchyKey(group, parent_path, node.category, node.name);
    values[key] += node.inclusive_ms;
    AccumulateCpuProfilerSeries(group, node.children, key, values);
  }
}

std::vector<CpuProfilerSeries> BuildCpuProfilerSeries(const std::vector<ProfilerFrameStats>& frames) {
  std::vector<CpuProfilerSeries> series;
  std::unordered_map<std::string, size_t> series_indices;
  for (const auto& frame : frames) {
    for (const auto& lane : frame.thread_lanes) {
      CatalogCpuProfilerSeries(profiler_panel_detail::ClassifyCpuExecutor(lane.thread_name), lane.hierarchy, "", series,
                               series_indices);
    }
  }
  for (auto& item : series)
    item.values.assign(frames.size(), 0.0f);
  for (size_t frame_index = 0; frame_index < frames.size(); ++frame_index) {
    std::unordered_map<std::string, double> values;
    for (const auto& lane : frames[frame_index].thread_lanes) {
      AccumulateCpuProfilerSeries(profiler_panel_detail::ClassifyCpuExecutor(lane.thread_name), lane.hierarchy, "",
                                  values);
    }
    for (const auto& [key, value] : values) {
      if (const auto found = series_indices.find(key); found != series_indices.end())
        series[found->second].values[frame_index] = static_cast<float>(value);
    }
  }
  for (auto& item : series) {
    for (const float value : item.values)
      item.average_milliseconds += value;
    if (!item.values.empty())
      item.average_milliseconds /= static_cast<double>(item.values.size());
  }
  return series;
}

void DrawCpuProfilerHistoryPlot(const char* id, const std::vector<CpuProfilerSeries>& series,
                                const std::vector<size_t>& visible_series,
                                const std::vector<ProfilerFrameStats>& frames, int& selected_frame_index) {
  const ImVec2 size(std::max(320.0f, ImGui::GetContentRegionAvail().x), 180.0f);
  ImGui::InvisibleButton(id, size);
  const auto minimum = ImGui::GetItemRectMin();
  const auto maximum = ImGui::GetItemRectMax();
  auto* draw_list = ImGui::GetWindowDrawList();
  draw_list->AddRectFilled(minimum, maximum, ImGui::GetColorU32(ImGuiCol_FrameBg), 4.0f);
  float plot_max = 16.667f;
  for (const auto index : visible_series) {
    for (const auto value : series[index].values)
      plot_max = std::max(plot_max, value);
  }
  const auto x_for_frame = [&](const size_t index) {
    return minimum.x +
           (frames.size() <= 1 ? 0.0f : static_cast<float>(index) / static_cast<float>(frames.size() - 1) * size.x);
  };
  const auto y_for_value = [&](const float value) {
    return maximum.y - value / plot_max * size.y;
  };
  for (const float budget : {16.667f, 33.333f}) {
    if (budget > plot_max)
      continue;
    const float y = y_for_value(budget);
    draw_list->AddLine(ImVec2(minimum.x, y), ImVec2(maximum.x, y), IM_COL32(180, 180, 180, 80));
  }
  for (const auto series_index : visible_series) {
    const auto color = GpuProfilerSeriesColor(series[series_index].key);
    for (size_t frame_index = 1; frame_index < frames.size(); ++frame_index) {
      if (frames[frame_index - 1].capture_session_index != frames[frame_index].capture_session_index)
        continue;
      draw_list->AddLine(
          ImVec2(x_for_frame(frame_index - 1), y_for_value(series[series_index].values[frame_index - 1])),
          ImVec2(x_for_frame(frame_index), y_for_value(series[series_index].values[frame_index])), color, 1.5f);
    }
  }
  if (selected_frame_index >= 0 && selected_frame_index < static_cast<int>(frames.size())) {
    const float x = x_for_frame(static_cast<size_t>(selected_frame_index));
    draw_list->AddLine(ImVec2(x, minimum.y), ImVec2(x, maximum.y), ImGui::GetColorU32(ImGuiCol_PlotLinesHovered));
  }
  draw_list->AddRect(minimum, maximum, ImGui::GetColorU32(ImGuiCol_Border), 4.0f);
  if (ImGui::IsItemHovered() && !frames.empty()) {
    const float fraction = std::clamp((ImGui::GetIO().MousePos.x - minimum.x) / size.x, 0.0f, 1.0f);
    const size_t index = static_cast<size_t>(std::round(fraction * static_cast<float>(frames.size() - 1)));
    ImGui::BeginTooltip();
    ImGui::Text("Session %llu  Frame %llu", static_cast<unsigned long long>(frames[index].capture_session_index),
                static_cast<unsigned long long>(frames[index].frame_index));
    for (const auto series_index : visible_series) {
      const auto& item = series[series_index];
      const float value = item.values[index];
      if (value > 0.0f)
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(GpuProfilerSeriesColor(item.key)), "%s / %s  %.3f ms",
                           profiler_panel_detail::CpuExecutorGroupName(item.group), item.name.c_str(), value);
    }
    ImGui::EndTooltip();
    if (ImGui::IsItemClicked())
      selected_frame_index = static_cast<int>(index);
  }
}

void UpdateGpuProfilerCatalog(const std::vector<GpuTimestampFrameSnapshot>& frames, ProfilerPanelState& state) {
  for (const auto& frame : frames) {
    if (!frame.results_available)
      continue;
    if (!state.cataloged_gpu_frames
             .emplace(GpuProfilerFrameKey(frame.capture_session_index, frame.application_frame_index))
             .second)
      continue;
    for (const auto& aggregate : BuildGpuTimestampPassAggregates(frame)) {
      auto& group = profiler_panel_detail::AppendFirstSeen(
          state.gpu_groups, aggregate.metadata.group,
          [](const auto& entry) -> const std::string& {
            return entry.name;
          },
          ProfilerPanelState::GpuGroup{aggregate.metadata.group});
      const auto key = GpuProfilerPassKey(aggregate.metadata);
      profiler_panel_detail::AppendFirstSeen(
          group.passes, key,
          [](const auto& pass) -> const std::string& {
            return pass.key;
          },
          ProfilerPanelState::GpuPass{key, aggregate.metadata});
    }
  }
}

GpuProfilerHistorySummaries BuildGpuProfilerHistorySummaries(const std::vector<GpuTimestampFrameSnapshot>& frames,
                                                             const ProfilerPanelState& state) {
  GpuProfilerHistorySummaries summaries;
  for (const auto& group : state.gpu_groups) {
    for (const auto& pass : group.passes) {
      auto& summary = summaries[pass.key];
      summary.metadata = pass.metadata;
      summary.stats.name = pass.metadata.display_name;
    }
  }
  for (const auto& frame : frames) {
    if (!frame.results_available)
      continue;
    std::unordered_map<std::string, double> frame_values;
    for (const auto& aggregate : BuildGpuTimestampPassAggregates(frame)) {
      frame_values[GpuProfilerPassKey(aggregate.metadata)] = aggregate.total_milliseconds;
    }
    for (auto& [key, summary] : summaries) {
      const auto value = frame_values.find(key);
      summary.stats.AddSample(value == frame_values.end() ? 0.0 : value->second);
    }
  }
  return summaries;
}

struct GpuProfilerSeries {
  std::string key{};
  GpuTimestampScopeMetadata metadata{};
  std::vector<float> values{};
  double average_milliseconds = 0.0;
};

std::vector<GpuProfilerSeries> BuildGpuProfilerSeries(const std::vector<ProfilerFrameStats>& cpu_frames,
                                                      const std::vector<GpuTimestampFrameSnapshot>& gpu_frames,
                                                      const ProfilerPanelState& state) {
  std::unordered_map<std::string, const GpuTimestampFrameSnapshot*> gpu_by_frame;
  for (const auto& gpu_frame : gpu_frames) {
    gpu_by_frame[GpuProfilerFrameKey(gpu_frame.capture_session_index, gpu_frame.application_frame_index)] = &gpu_frame;
  }

  std::vector<GpuProfilerSeries> series;
  std::unordered_map<std::string, size_t> series_indices;
  for (const auto& group : state.gpu_groups) {
    for (const auto& pass : group.passes) {
      if (!pass.metadata.contributes_to_frame_total)
        continue;
      series_indices[pass.key] = series.size();
      auto& pass_series = series.emplace_back();
      pass_series.key = pass.key;
      pass_series.metadata = pass.metadata;
      pass_series.values.resize(cpu_frames.size(), std::numeric_limits<float>::quiet_NaN());
    }
  }

  for (size_t frame_index = 0; frame_index < cpu_frames.size(); ++frame_index) {
    const auto& cpu_frame = cpu_frames[frame_index];
    const auto gpu_search =
        gpu_by_frame.find(GpuProfilerFrameKey(cpu_frame.capture_session_index, cpu_frame.application_frame_index));
    if (gpu_search == gpu_by_frame.end() || !gpu_search->second->results_available)
      continue;
    for (auto& pass_series : series)
      pass_series.values[frame_index] = 0.0f;
    for (const auto& aggregate : BuildGpuTimestampPassAggregates(*gpu_search->second)) {
      if (!aggregate.metadata.contributes_to_frame_total)
        continue;
      const auto series_search = series_indices.find(GpuProfilerPassKey(aggregate.metadata));
      if (series_search != series_indices.end())
        series[series_search->second].values[frame_index] = static_cast<float>(aggregate.total_milliseconds);
    }
  }

  for (auto& pass_series : series) {
    double total = 0.0;
    size_t count = 0;
    for (const float value : pass_series.values) {
      if (!std::isfinite(value))
        continue;
      total += value;
      ++count;
    }
    pass_series.average_milliseconds = count == 0 ? 0.0 : total / static_cast<double>(count);
  }
  return series;
}

ImU32 GpuProfilerSeriesColor(const std::string& key) {
  const float hue = static_cast<float>(std::hash<std::string>{}(key) % 360) / 360.0f;
  float red = 0.0f;
  float green = 0.0f;
  float blue = 0.0f;
  ImGui::ColorConvertHSVtoRGB(hue, 0.70f, 0.95f, red, green, blue);
  return ImGui::GetColorU32(ImVec4(red, green, blue, 1.0f));
}

const char* GpuTimestampQueueName(const GpuTimestampQueue queue) {
  switch (queue) {
    case GpuTimestampQueue::Graphics:
      return "Graphics";
    case GpuTimestampQueue::Compute:
      return "Compute";
    case GpuTimestampQueue::Transfer:
      return "Transfer";
    case GpuTimestampQueue::RayTracing:
      return "Ray tracing";
    case GpuTimestampQueue::Immediate:
      return "Immediate";
  }
  return "Unknown";
}

void DrawGpuProfilerHistoryPlot(const char* id, const std::vector<GpuProfilerSeries>& series,
                                const std::vector<size_t>& visible_series,
                                const std::vector<ProfilerFrameStats>& frames, int& selected_frame_index) {
  const ImVec2 size(std::max(320.0f, ImGui::GetContentRegionAvail().x), 180.0f);
  ImGui::InvisibleButton(id, size);
  const auto minimum = ImGui::GetItemRectMin();
  const auto maximum = ImGui::GetItemRectMax();
  auto* draw_list = ImGui::GetWindowDrawList();
  draw_list->AddRectFilled(minimum, maximum, ImGui::GetColorU32(ImGuiCol_FrameBg), 4.0f);
  draw_list->PushClipRect(minimum, maximum, true);
  float plot_max = 16.667f;
  for (const auto series_index : visible_series) {
    for (const float value : series[series_index].values) {
      if (std::isfinite(value))
        plot_max = std::max(plot_max, value);
    }
  }
  const auto x_for_frame = [&](const size_t frame_index) {
    return minimum.x + (frames.size() <= 1
                            ? 0.0f
                            : static_cast<float>(frame_index) / static_cast<float>(frames.size() - 1) * size.x);
  };
  const auto y_for_value = [&](const float value) {
    return maximum.y - value / plot_max * size.y;
  };
  for (const float budget : {16.667f, 33.333f}) {
    if (budget > plot_max)
      continue;
    const float y = y_for_value(budget);
    draw_list->AddLine(ImVec2(minimum.x, y), ImVec2(maximum.x, y), IM_COL32(180, 180, 180, 80), 1.0f);
    draw_list->AddText(ImVec2(minimum.x + 4.0f, y + 2.0f), IM_COL32(180, 180, 180, 160),
                       budget < 20.0f ? "16.7 ms" : "33.3 ms");
  }
  for (size_t frame_index = 1; frame_index < frames.size(); ++frame_index) {
    if (frames[frame_index - 1].capture_session_index == frames[frame_index].capture_session_index)
      continue;
    const float gap_x = (x_for_frame(frame_index - 1) + x_for_frame(frame_index)) * 0.5f;
    draw_list->AddLine(ImVec2(gap_x, minimum.y), ImVec2(gap_x, maximum.y), IM_COL32(255, 190, 70, 180), 2.0f);
  }
  for (const auto series_index : visible_series) {
    const auto& pass_series = series[series_index];
    const auto color = GpuProfilerSeriesColor(pass_series.key);
    for (size_t frame_index = 1; frame_index < frames.size(); ++frame_index) {
      const float previous = pass_series.values[frame_index - 1];
      const float current = pass_series.values[frame_index];
      if (!std::isfinite(previous) || !std::isfinite(current) ||
          frames[frame_index - 1].capture_session_index != frames[frame_index].capture_session_index)
        continue;
      draw_list->AddLine(ImVec2(x_for_frame(frame_index - 1), y_for_value(previous)),
                         ImVec2(x_for_frame(frame_index), y_for_value(current)), color, 1.5f);
    }
  }
  if (selected_frame_index >= 0 && selected_frame_index < static_cast<int>(frames.size())) {
    const float selected_x = x_for_frame(static_cast<size_t>(selected_frame_index));
    draw_list->AddLine(ImVec2(selected_x, minimum.y), ImVec2(selected_x, maximum.y),
                       ImGui::GetColorU32(ImGuiCol_PlotLinesHovered), 1.0f);
  }
  draw_list->PopClipRect();
  draw_list->AddRect(minimum, maximum, ImGui::GetColorU32(ImGuiCol_Border), 4.0f);
  if (ImGui::IsItemHovered() && !frames.empty()) {
    const float fraction = std::clamp((ImGui::GetIO().MousePos.x - minimum.x) / size.x, 0.0f, 1.0f);
    const auto hovered_index = static_cast<size_t>(std::round(fraction * static_cast<float>(frames.size() - 1)));
    ImGui::BeginTooltip();
    ImGui::Text("Frame %llu", static_cast<unsigned long long>(frames[hovered_index].frame_index));
    for (const auto series_index : visible_series) {
      const float value = series[series_index].values[hovered_index];
      if (std::isfinite(value) && value > 0.0f)
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(GpuProfilerSeriesColor(series[series_index].key)),
                           "%s  %.3f ms", series[series_index].metadata.display_name.c_str(), value);
    }
    ImGui::EndTooltip();
    if (ImGui::IsItemClicked())
      selected_frame_index = static_cast<int>(hovered_index);
  }
}

void DrawGpuProfilerStackedPlot(const char* id, const std::vector<GpuProfilerSeries>& series,
                                const std::vector<size_t>& visible_series,
                                const std::vector<ProfilerFrameStats>& frames, int& selected_frame_index) {
  const ImVec2 size(std::max(320.0f, ImGui::GetContentRegionAvail().x), 150.0f);
  ImGui::InvisibleButton(id, size);
  const auto minimum = ImGui::GetItemRectMin();
  const auto maximum = ImGui::GetItemRectMax();
  auto* draw_list = ImGui::GetWindowDrawList();
  draw_list->AddRectFilled(minimum, maximum, ImGui::GetColorU32(ImGuiCol_FrameBg), 4.0f);
  float maximum_total = 16.667f;
  std::vector<float> totals(frames.size(), 0.0f);
  for (const auto& pass_series : series) {
    for (size_t frame_index = 0; frame_index < frames.size(); ++frame_index) {
      if (std::isfinite(pass_series.values[frame_index]))
        totals[frame_index] += pass_series.values[frame_index];
    }
  }
  for (const float total : totals)
    maximum_total = std::max(maximum_total, total);
  const float frame_width = frames.empty() ? size.x : size.x / static_cast<float>(frames.size());
  for (size_t frame_index = 0; frame_index < frames.size(); ++frame_index) {
    float accumulated = 0.0f;
    for (const auto series_index : visible_series) {
      const float value = series[series_index].values[frame_index];
      if (!std::isfinite(value) || value <= 0.0f)
        continue;
      const float lower_y = maximum.y - accumulated / maximum_total * size.y;
      accumulated += value;
      const float upper_y = maximum.y - accumulated / maximum_total * size.y;
      draw_list->AddRectFilled(ImVec2(minimum.x + frame_width * static_cast<float>(frame_index), upper_y),
                               ImVec2(minimum.x + frame_width * static_cast<float>(frame_index + 1), lower_y),
                               GpuProfilerSeriesColor(series[series_index].key));
    }
    const float other = std::max(0.0f, totals[frame_index] - accumulated);
    if (other > 0.0f) {
      const float lower_y = maximum.y - accumulated / maximum_total * size.y;
      accumulated += other;
      const float upper_y = maximum.y - accumulated / maximum_total * size.y;
      draw_list->AddRectFilled(ImVec2(minimum.x + frame_width * static_cast<float>(frame_index), upper_y),
                               ImVec2(minimum.x + frame_width * static_cast<float>(frame_index + 1), lower_y),
                               IM_COL32(110, 110, 110, 220));
    }
  }
  for (size_t frame_index = 1; frame_index < frames.size(); ++frame_index) {
    if (frames[frame_index - 1].capture_session_index == frames[frame_index].capture_session_index)
      continue;
    const float gap_x = minimum.x + frame_width * static_cast<float>(frame_index);
    draw_list->AddLine(ImVec2(gap_x, minimum.y), ImVec2(gap_x, maximum.y), IM_COL32(255, 190, 70, 220), 2.0f);
  }
  draw_list->AddRect(minimum, maximum, ImGui::GetColorU32(ImGuiCol_Border), 4.0f);
  if (ImGui::IsItemHovered() && !frames.empty()) {
    const size_t hovered_index = std::min(
        frames.size() - 1, static_cast<size_t>(std::max(0.0f, ImGui::GetIO().MousePos.x - minimum.x) / frame_width));
    float visible_total = 0.0f;
    ImGui::BeginTooltip();
    ImGui::Text("Session %llu  Frame %llu%s",
                static_cast<unsigned long long>(frames[hovered_index].capture_session_index),
                static_cast<unsigned long long>(frames[hovered_index].frame_index),
                selected_frame_index == static_cast<int>(hovered_index) ? "  (selected)" : "");
    for (const auto series_index : visible_series) {
      const float value = series[series_index].values[hovered_index];
      if (!std::isfinite(value) || value <= 0.0f)
        continue;
      visible_total += value;
      ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(GpuProfilerSeriesColor(series[series_index].key)),
                         "%s  %.3f ms", series[series_index].metadata.display_name.c_str(), value);
    }
    const float other = std::max(0.0f, totals[hovered_index] - visible_total);
    if (other > 0.0f)
      ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(IM_COL32(110, 110, 110, 220)), "Other  %.3f ms", other);
    ImGui::Separator();
    ImGui::Text("Summed pass work  %.3f ms", totals[hovered_index]);
    ImGui::EndTooltip();
    if (ImGui::IsItemClicked())
      selected_frame_index = static_cast<int>(hovered_index);
  }
}

double ProfilerFrameSynchronizationMilliseconds(const ProfilerFrameStats& frame) {
  std::vector<profiler_panel_detail::TimingInterval> intervals;
  for (const auto& lane : frame.thread_lanes) {
    if (lane.thread_name != "MainThread")
      continue;
    for (const auto& event : lane.events) {
      if (event.category != "Synchronization")
        continue;
      const double start = std::clamp(event.start_ms, 0.0, frame.duration_ms);
      const double end = std::clamp(event.start_ms + event.duration_ms, start, frame.duration_ms);
      intervals.emplace_back(profiler_panel_detail::TimingInterval{start, end - start});
    }
  }
  return profiler_panel_detail::IntervalUnionMilliseconds(std::move(intervals));
}

std::vector<profiler_panel_detail::FrameOverviewSample> BuildProfilerOverviewSamples(
    const std::vector<ProfilerFrameStats>& cpu_frames, const std::vector<GpuTimestampFrameSnapshot>& gpu_frames) {
  std::unordered_map<std::string, const GpuTimestampFrameSnapshot*> gpu_by_frame;
  for (const auto& frame : gpu_frames)
    gpu_by_frame[GpuProfilerFrameKey(frame.capture_session_index, frame.application_frame_index)] = &frame;
  std::vector<profiler_panel_detail::FrameOverviewSample> samples;
  samples.reserve(cpu_frames.size());
  for (const auto& frame : cpu_frames) {
    const auto gpu = gpu_by_frame.find(GpuProfilerFrameKey(frame.capture_session_index, frame.application_frame_index));
    std::optional<double> gpu_span;
    if (gpu != gpu_by_frame.end() && gpu->second->results_available)
      gpu_span = gpu->second->span_milliseconds;
    samples.emplace_back(profiler_panel_detail::BuildFrameOverviewSample(
        frame.duration_ms, ProfilerFrameSynchronizationMilliseconds(frame), gpu_span));
  }
  return samples;
}

void DrawProfilerOverviewPlot(const char* id, const std::vector<profiler_panel_detail::FrameOverviewSample>& samples,
                              const std::vector<ProfilerFrameStats>& frames, int& selected_frame_index) {
  const ImVec2 size(std::max(320.0f, ImGui::GetContentRegionAvail().x), 220.0f);
  ImGui::InvisibleButton(id, size);
  const auto minimum = ImGui::GetItemRectMin();
  const auto maximum = ImGui::GetItemRectMax();
  auto* draw_list = ImGui::GetWindowDrawList();
  draw_list->AddRectFilled(minimum, maximum, ImGui::GetColorU32(ImGuiCol_FrameBg), 4.0f);
  float plot_max = 16.667f;
  for (const auto& sample : samples)
    plot_max = std::max(plot_max, static_cast<float>(sample.total_ms));
  const float frame_width = frames.empty() ? size.x : size.x / static_cast<float>(frames.size());
  const auto y_for_value = [&](const double value) {
    return maximum.y - static_cast<float>(value) / plot_max * size.y;
  };
  for (const float budget : {16.667f, 33.333f}) {
    if (budget > plot_max)
      continue;
    const float y = y_for_value(budget);
    draw_list->AddLine(ImVec2(minimum.x, y), ImVec2(maximum.x, y), IM_COL32(180, 180, 180, 80));
    draw_list->AddText(ImVec2(minimum.x + 4.0f, y + 2.0f), IM_COL32(180, 180, 180, 160),
                       budget < 20.0f ? "16.7 ms" : "33.3 ms");
  }
  const ImU32 cpu_color = IM_COL32(65, 155, 235, 220);
  const ImU32 sync_color = IM_COL32(245, 170, 55, 230);
  const ImU32 gpu_color = IM_COL32(85, 210, 120, 255);
  const ImU32 total_color = IM_COL32(235, 235, 235, 230);
  for (size_t index = 0; index < samples.size(); ++index) {
    const float left = minimum.x + frame_width * static_cast<float>(index);
    const float right = minimum.x + frame_width * static_cast<float>(index + 1);
    const auto& sample = samples[index];
    draw_list->AddRectFilled(ImVec2(left, y_for_value(sample.cpu_active_ms)), ImVec2(right, maximum.y), cpu_color);
    draw_list->AddRectFilled(ImVec2(left, y_for_value(sample.cpu_wall_ms)),
                             ImVec2(right, y_for_value(sample.cpu_active_ms)), sync_color);
  }
  for (size_t index = 1; index < samples.size(); ++index) {
    if (frames[index - 1].capture_session_index != frames[index].capture_session_index)
      continue;
    const float previous_x = minimum.x + frame_width * (static_cast<float>(index) - 0.5f);
    const float current_x = minimum.x + frame_width * (static_cast<float>(index) + 0.5f);
    if (samples[index - 1].gpu_available && samples[index].gpu_available)
      draw_list->AddLine(ImVec2(previous_x, y_for_value(samples[index - 1].gpu_ms)),
                         ImVec2(current_x, y_for_value(samples[index].gpu_ms)), gpu_color, 1.5f);
    draw_list->AddLine(ImVec2(previous_x, y_for_value(samples[index - 1].total_ms)),
                       ImVec2(current_x, y_for_value(samples[index].total_ms)), total_color, 1.0f);
  }
  for (size_t index = 1; index < frames.size(); ++index) {
    if (frames[index - 1].capture_session_index == frames[index].capture_session_index)
      continue;
    const float x = minimum.x + frame_width * static_cast<float>(index);
    draw_list->AddLine(ImVec2(x, minimum.y), ImVec2(x, maximum.y), IM_COL32(255, 190, 70, 220), 2.0f);
  }
  if (selected_frame_index >= 0 && selected_frame_index < static_cast<int>(frames.size())) {
    const float x = minimum.x + frame_width * (static_cast<float>(selected_frame_index) + 0.5f);
    draw_list->AddLine(ImVec2(x, minimum.y), ImVec2(x, maximum.y), ImGui::GetColorU32(ImGuiCol_PlotLinesHovered));
  }
  draw_list->AddRect(minimum, maximum, ImGui::GetColorU32(ImGuiCol_Border), 4.0f);
  if (ImGui::IsItemHovered() && !frames.empty()) {
    const size_t index = std::min(
        frames.size() - 1, static_cast<size_t>(std::max(0.0f, ImGui::GetIO().MousePos.x - minimum.x) / frame_width));
    const auto& sample = samples[index];
    ImGui::BeginTooltip();
    ImGui::Text("Session %llu  Frame %llu", static_cast<unsigned long long>(frames[index].capture_session_index),
                static_cast<unsigned long long>(frames[index].frame_index));
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(cpu_color), "CPU active  %.3f ms", sample.cpu_active_ms);
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(sync_color), "Synchronization  %.3f ms",
                       sample.synchronization_ms);
    ImGui::Text("CPU wall  %.3f ms", sample.cpu_wall_ms);
    if (sample.gpu_available)
      ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(gpu_color), "GPU span  %.3f ms", sample.gpu_ms);
    else
      ImGui::TextDisabled("GPU span  pending/unavailable");
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(total_color), "Total  %.3f ms", sample.total_ms);
    ImGui::EndTooltip();
    if (ImGui::IsItemClicked())
      selected_frame_index = static_cast<int>(index);
  }
}

const ProfilerHierarchyNode* FindApplicationLoop(const ProfilerFrameStats& frame) {
  const auto main_thread = std::find_if(frame.thread_lanes.begin(), frame.thread_lanes.end(), [](const auto& lane) {
    return lane.thread_name == "MainThread";
  });
  if (main_thread == frame.thread_lanes.end())
    return nullptr;
  const auto loop = std::find_if(main_thread->hierarchy.begin(), main_thread->hierarchy.end(), [](const auto& node) {
    return node.name == "Application::Loop";
  });
  return loop == main_thread->hierarchy.end() ? nullptr : &*loop;
}

const ProfilerThreadHistory* FindMainThreadHistory(const ProfilerHistoryStats& history) {
  const auto thread = std::find_if(history.threads.begin(), history.threads.end(), [](const auto& candidate) {
    return candidate.thread_name == "MainThread";
  });
  return thread == history.threads.end() ? nullptr : &*thread;
}

const ProfilerCounterHistory* FindProfilerCounter(const ProfilerHistoryStats& history, const std::string& name) {
  const auto counter = std::find_if(history.counters.begin(), history.counters.end(), [&](const auto& candidate) {
    return candidate.name == name;
  });
  return counter == history.counters.end() ? nullptr : &*counter;
}

std::vector<GpuTimestampFrameSnapshot> AlignGpuProfilerFrames(
    const std::vector<ProfilerFrameStats>& cpu_frames, const std::vector<GpuTimestampFrameSnapshot>& gpu_frames) {
  std::unordered_map<std::string, const GpuTimestampFrameSnapshot*> gpu_by_frame;
  for (const auto& frame : gpu_frames)
    gpu_by_frame[GpuProfilerFrameKey(frame.capture_session_index, frame.application_frame_index)] = &frame;
  std::vector<GpuTimestampFrameSnapshot> aligned;
  aligned.reserve(cpu_frames.size());
  for (const auto& frame : cpu_frames) {
    const auto search =
        gpu_by_frame.find(GpuProfilerFrameKey(frame.capture_session_index, frame.application_frame_index));
    if (search == gpu_by_frame.end()) {
      aligned.emplace_back();
    } else {
      aligned.emplace_back(*search->second);
    }
  }
  return aligned;
}

std::vector<ProfilerBreakdownSeries> BuildCpuBreakdownSeries(const std::vector<ProfilerFrameStats>& frames) {
  std::vector<ProfilerBreakdownSeries> series;
  for (const auto& frame : frames) {
    if (const auto* loop = FindApplicationLoop(frame)) {
      for (const auto& child : loop->children) {
        profiler_panel_detail::AppendFirstSeen(
            series, child.name,
            [](const auto& entry) -> const std::string& {
              return entry.key;
            },
            ProfilerBreakdownSeries{child.name, ProfilerScopeDisplayName(child.name), {}});
      }
    }
  }
  series.emplace_back(ProfilerBreakdownSeries{"ApplicationLoopSelf", "Loop self", {}});
  series.emplace_back(ProfilerBreakdownSeries{"FrameOverhead", "Frame overhead", {}});
  for (auto& item : series)
    item.values.reserve(frames.size());
  for (const auto& frame : frames) {
    const auto* loop = FindApplicationLoop(frame);
    for (auto& item : series) {
      double value = 0.0;
      if (loop && item.key == "ApplicationLoopSelf") {
        value = loop->self_ms;
      } else if (item.key == "FrameOverhead") {
        value = std::max(0.0, frame.duration_ms - (loop ? loop->inclusive_ms : 0.0));
      } else if (loop) {
        const auto child = std::find_if(loop->children.begin(), loop->children.end(), [&](const auto& candidate) {
          return candidate.name == item.key;
        });
        if (child != loop->children.end())
          value = child->inclusive_ms;
      }
      item.values.emplace_back(static_cast<float>(value));
    }
  }
  return series;
}

std::vector<ProfilerBreakdownSeries> BuildGpuGroupBreakdownSeries(
    const std::vector<GpuTimestampFrameSnapshot>& aligned_frames) {
  std::vector<ProfilerBreakdownSeries> series;
  for (const auto& frame : aligned_frames) {
    if (!frame.results_available)
      continue;
    for (const auto& aggregate : BuildGpuTimestampPassAggregates(frame)) {
      profiler_panel_detail::AppendFirstSeen(
          series, aggregate.metadata.group,
          [](const auto& entry) -> const std::string& {
            return entry.key;
          },
          ProfilerBreakdownSeries{aggregate.metadata.group, aggregate.metadata.group, {}});
    }
  }
  for (auto& item : series)
    item.values.reserve(aligned_frames.size());
  for (const auto& frame : aligned_frames) {
    std::unordered_map<std::string, double> values;
    if (frame.results_available) {
      for (const auto& aggregate : BuildGpuTimestampPassAggregates(frame)) {
        if (aggregate.metadata.contributes_to_frame_total)
          values[aggregate.metadata.group] += aggregate.total_milliseconds;
      }
    }
    for (auto& item : series)
      item.values.emplace_back(static_cast<float>(values[item.key]));
  }
  return series;
}

void DrawProfilerBreakdownStackedPlot(const char* id, const std::vector<ProfilerBreakdownSeries>& series,
                                      const std::vector<ProfilerFrameStats>& frames, int& selected_frame_index) {
  const ImVec2 size(std::max(320.0f, ImGui::GetContentRegionAvail().x), 140.0f);
  ImGui::InvisibleButton(id, size);
  const auto minimum = ImGui::GetItemRectMin();
  const auto maximum = ImGui::GetItemRectMax();
  auto* draw_list = ImGui::GetWindowDrawList();
  draw_list->AddRectFilled(minimum, maximum, ImGui::GetColorU32(ImGuiCol_FrameBg), 4.0f);
  std::vector<float> totals(frames.size(), 0.0f);
  float maximum_total = 16.667f;
  for (const auto& item : series) {
    for (size_t frame_index = 0; frame_index < item.values.size(); frame_index++)
      totals[frame_index] += item.values[frame_index];
  }
  for (const auto total : totals)
    maximum_total = std::max(maximum_total, total);
  const float frame_width = frames.empty() ? size.x : size.x / static_cast<float>(frames.size());
  for (size_t frame_index = 0; frame_index < frames.size(); frame_index++) {
    float accumulated = 0.0f;
    for (const auto& item : series) {
      const float value = item.values[frame_index];
      if (value <= 0.0f)
        continue;
      const float lower_y = maximum.y - accumulated / maximum_total * size.y;
      accumulated += value;
      const float upper_y = maximum.y - accumulated / maximum_total * size.y;
      draw_list->AddRectFilled(ImVec2(minimum.x + frame_width * static_cast<float>(frame_index), upper_y),
                               ImVec2(minimum.x + frame_width * static_cast<float>(frame_index + 1), lower_y),
                               GpuProfilerSeriesColor(item.key));
    }
  }
  if (selected_frame_index >= 0 && selected_frame_index < static_cast<int>(frames.size())) {
    const float selected_x = minimum.x + frame_width * (static_cast<float>(selected_frame_index) + 0.5f);
    draw_list->AddLine(ImVec2(selected_x, minimum.y), ImVec2(selected_x, maximum.y),
                       ImGui::GetColorU32(ImGuiCol_PlotLinesHovered), 1.0f);
  }
  draw_list->AddRect(minimum, maximum, ImGui::GetColorU32(ImGuiCol_Border), 4.0f);
  if (ImGui::IsItemHovered() && !frames.empty()) {
    const size_t hovered_index = std::min(
        frames.size() - 1, static_cast<size_t>(std::max(0.0f, ImGui::GetIO().MousePos.x - minimum.x) / frame_width));
    ImGui::BeginTooltip();
    ImGui::Text("Frame %llu  %.3f ms", static_cast<unsigned long long>(frames[hovered_index].frame_index),
                totals[hovered_index]);
    for (const auto& item : series) {
      if (item.values[hovered_index] > 0.0f)
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(GpuProfilerSeriesColor(item.key)), "%s  %.3f ms",
                           item.name.c_str(), item.values[hovered_index]);
    }
    ImGui::EndTooltip();
    if (ImGui::IsItemClicked())
      selected_frame_index = static_cast<int>(hovered_index);
  }
}

ProfilerDurationSummary SummarizeBreakdownSeries(const ProfilerBreakdownSeries& series,
                                                 const size_t selected_frame_index) {
  ProfilerDurationSummary summary;
  summary.frame_count = series.values.size();
  summary.observed_frame_count = series.values.size();
  if (series.values.empty())
    return summary;
  std::vector<double> sorted;
  sorted.reserve(series.values.size());
  for (const auto value : series.values) {
    summary.average_ms += value;
    summary.maximum_ms = std::max(summary.maximum_ms, static_cast<double>(value));
    sorted.emplace_back(value);
  }
  summary.selected_ms = series.values[std::min(selected_frame_index, series.values.size() - 1)];
  summary.average_ms /= static_cast<double>(series.values.size());
  std::sort(sorted.begin(), sorted.end());
  summary.median_ms = sorted[(sorted.size() - 1) / 2];
  summary.p95_ms = sorted[static_cast<size_t>(std::ceil(static_cast<double>(sorted.size()) * 0.95)) - 1];
  return summary;
}

void DrawProfilerBreakdownCpuSummaryRow(const char* name, const ProfilerDurationSummary& summary,
                                        const double frame_ms) {
  ImGui::TableNextRow();
  ImGui::TableNextColumn();
  ImGui::TreeNodeEx(name,
                    ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_SpanFullWidth);
  ImGui::TableNextColumn();
  ImGui::Text("%.3f", summary.selected_ms);
  ImGui::TableNextColumn();
  ImGui::Text("%.3f", summary.selected_ms);
  ImGui::TableNextColumn();
  ImGui::Text("%.3f", summary.average_ms);
  ImGui::TableNextColumn();
  ImGui::Text("%.3f", summary.median_ms);
  ImGui::TableNextColumn();
  ImGui::Text("%.3f", summary.p95_ms);
  ImGui::TableNextColumn();
  ImGui::Text("%.1f%%", frame_ms > 0.0 ? summary.selected_ms / frame_ms * 100.0 : 0.0);
  ImGui::TableNextColumn();
  ImGui::Text("%zu/%zu", summary.observed_frame_count, summary.frame_count);
}

void DrawProfilerBreakdownCpuNode(const ProfilerHistoryNode& node, const double frame_ms,
                                  const std::string& parent_path = {}) {
  ImGui::TableNextRow();
  ImGui::TableNextColumn();
  const auto stable_path = profiler_panel_detail::StableHierarchyKey(
      profiler_panel_detail::CpuExecutorGroup::MainThread, parent_path, node.category, node.name);
  ImGui::PushID(stable_path.c_str());
  ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_SpanFullWidth | ImGuiTreeNodeFlags_OpenOnArrow;
  if (node.name == "Application::Loop")
    flags |= ImGuiTreeNodeFlags_DefaultOpen;
  if (node.children.empty())
    flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
  const auto display_name = ProfilerScopeDisplayName(node.name);
  const bool open = ImGui::TreeNodeEx("BreakdownScope", flags, "%s", display_name.c_str());
  ImGui::TableNextColumn();
  ImGui::Text("%.3f", node.inclusive.selected_ms);
  ImGui::TableNextColumn();
  ImGui::Text("%.3f", node.self.selected_ms);
  ImGui::TableNextColumn();
  ImGui::Text("%.3f", node.inclusive.average_ms);
  ImGui::TableNextColumn();
  ImGui::Text("%.3f", node.inclusive.median_ms);
  ImGui::TableNextColumn();
  ImGui::Text("%.3f", node.inclusive.p95_ms);
  ImGui::TableNextColumn();
  ImGui::Text("%.1f%%", frame_ms > 0.0 ? node.inclusive.selected_ms / frame_ms * 100.0 : 0.0);
  ImGui::TableNextColumn();
  ImGui::Text("%zu/%zu", node.inclusive.observed_frame_count, node.inclusive.frame_count);
  if (open && !node.children.empty()) {
    for (const auto& child : node.children)
      DrawProfilerBreakdownCpuNode(child, frame_ms, stable_path);
    ImGui::TreePop();
  }
  ImGui::PopID();
}

std::string ProfilerThreadDisplayName(const std::string& name) {
  if (profiler_panel_detail::ClassifyCpuExecutor(name) == profiler_panel_detail::CpuExecutorGroup::GpuSubmission)
    return "GPU Submission (CPU) / " + name;
  return name;
}

void DrawProfilerCpuTimeline(const ProfilerFrameStats& frame) {
  const float timeline_width = std::max(320.0f, ImGui::GetContentRegionAvail().x - 180.0f);
  const float scale = timeline_width / std::max(0.001f, static_cast<float>(frame.duration_ms));
  auto* draw_list = ImGui::GetWindowDrawList();
  for (const auto& lane : frame.thread_lanes) {
    const auto row_origin = ImGui::GetCursorScreenPos();
    const auto display_name = ProfilerThreadDisplayName(lane.thread_name);
    ImGui::TextUnformatted(display_name.c_str());
    const float timeline_x = row_origin.x + 170.0f;
    const float timeline_y = row_origin.y;
    draw_list->AddRectFilled(ImVec2(timeline_x, timeline_y), ImVec2(timeline_x + timeline_width, timeline_y + 20.0f),
                             ImGui::GetColorU32(ImGuiCol_FrameBg), 3.0f);
    for (const auto& event : lane.events) {
      const float event_x = timeline_x + static_cast<float>(event.start_ms) * scale;
      const float event_width = std::max(1.0f, static_cast<float>(event.duration_ms) * scale);
      const float event_y = timeline_y + 2.0f + static_cast<float>(event.depth % 3) * 4.0f;
      const ImVec2 event_min(event_x, event_y);
      const ImVec2 event_max(event_x + event_width, event_y + 12.0f);
      draw_list->AddRectFilled(event_min, event_max, ProfilerCategoryColor(event.category), 2.0f);
      const auto mouse = ImGui::GetIO().MousePos;
      if (ImGui::IsWindowHovered() && mouse.x >= event_min.x && mouse.x <= event_max.x && mouse.y >= event_min.y &&
          mouse.y <= event_max.y) {
        ImGui::BeginTooltip();
        ImGui::TextUnformatted(event.name.c_str());
        ImGui::Text("Category: %s", event.category.c_str());
        ImGui::Text("Start %.3f ms  Duration %.3f ms", event.start_ms, event.duration_ms);
        ImGui::EndTooltip();
      }
    }
    ImGui::Dummy(ImVec2(timeline_width + 170.0f, 26.0f));
  }
}

void DrawProfilerGpuTimeline(const GpuTimestampFrameSnapshot& frame) {
  if (!frame.results_available) {
    ImGui::TextDisabled("GPU results are unresolved for this frame.");
    return;
  }
  const float timeline_width = std::max(320.0f, ImGui::GetContentRegionAvail().x - 150.0f);
  const float scale = timeline_width / std::max(0.001f, static_cast<float>(frame.span_milliseconds));
  auto* draw_list = ImGui::GetWindowDrawList();
  for (const auto queue : {GpuTimestampQueue::Graphics, GpuTimestampQueue::Compute, GpuTimestampQueue::Transfer,
                           GpuTimestampQueue::RayTracing, GpuTimestampQueue::Immediate}) {
    std::vector<const GpuTimestampSample*> samples;
    for (const auto& sample : frame.samples) {
      if (sample.metadata.queue == queue)
        samples.emplace_back(&sample);
    }
    if (samples.empty())
      continue;
    std::stable_sort(samples.begin(), samples.end(), [](const auto* left, const auto* right) {
      return std::tie(left->begin_offset_milliseconds, left->end_offset_milliseconds) <
             std::tie(right->begin_offset_milliseconds, right->end_offset_milliseconds);
    });
    std::vector<double> lane_ends;
    std::vector<size_t> lanes;
    for (const auto* sample : samples) {
      size_t lane = 0;
      while (lane < lane_ends.size() && lane_ends[lane] > sample->begin_offset_milliseconds)
        ++lane;
      if (lane == lane_ends.size())
        lane_ends.emplace_back();
      lane_ends[lane] = sample->end_offset_milliseconds;
      lanes.emplace_back(lane);
    }
    const float row_height = 4.0f + 16.0f * static_cast<float>(lane_ends.size());
    const auto row_origin = ImGui::GetCursorScreenPos();
    ImGui::TextUnformatted(GpuTimestampQueueName(queue));
    const float timeline_x = row_origin.x + 140.0f;
    draw_list->AddRectFilled(ImVec2(timeline_x, row_origin.y),
                             ImVec2(timeline_x + timeline_width, row_origin.y + row_height),
                             ImGui::GetColorU32(ImGuiCol_FrameBg), 3.0f);
    for (size_t index = 0; index < samples.size(); ++index) {
      const auto* sample = samples[index];
      const ImVec2 sample_min(timeline_x + static_cast<float>(sample->begin_offset_milliseconds) * scale,
                              row_origin.y + 2.0f + static_cast<float>(lanes[index]) * 16.0f);
      const ImVec2 sample_max(
          std::max(sample_min.x + 1.0f, timeline_x + static_cast<float>(sample->end_offset_milliseconds) * scale),
          sample_min.y + 13.0f);
      const auto key = GpuProfilerPassKey(sample->metadata);
      draw_list->AddRectFilled(sample_min, sample_max, GpuProfilerSeriesColor(key), 2.0f);
      const auto mouse = ImGui::GetIO().MousePos;
      if (ImGui::IsWindowHovered() && mouse.x >= sample_min.x && mouse.x <= sample_max.x && mouse.y >= sample_min.y &&
          mouse.y <= sample_max.y) {
        ImGui::BeginTooltip();
        ImGui::TextUnformatted(sample->metadata.display_name.c_str());
        ImGui::Text("Group: %s  Queue: %s", sample->metadata.group.c_str(), GpuTimestampQueueName(queue));
        ImGui::Text("Offset %.3f ms  Duration %.3f ms", sample->begin_offset_milliseconds,
                    sample->duration_milliseconds);
        ImGui::EndTooltip();
      }
    }
    ImGui::Dummy(ImVec2(timeline_width + 140.0f, row_height + 5.0f));
  }
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
  std::filesystem::path cmake_build_dir;
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

bool UsesLightTheme() {
  return editor_theme::GetCurrentTheme() == editor_theme::Theme::Light;
}

const char* TitleBarLogoIconName() {
  return UsesLightTheme() ? "TitleBarLogoBlack" : "TitleBarLogoWhite";
}

ImU32 TitleBarColor() {
  return UsesLightTheme() ? IM_COL32(246, 247, 248, 255) : kTitleBarColor;
}

ImU32 TitleBarTextColor() {
  return UsesLightTheme() ? IM_COL32(28, 31, 34, 255) : kTitleBarText;
}

ImU32 TitleBarSecondaryTextColor() {
  return UsesLightTheme() ? IM_COL32(98, 104, 110, 255) : kTitleBarTextDarker;
}

ImU32 TitleBarMutedColor() {
  return UsesLightTheme() ? IM_COL32(208, 212, 216, 255) : kTitleBarMuted;
}

ImU32 TitleBarBorderColor() {
  return UsesLightTheme() ? IM_COL32(214, 218, 222, 255) : IM_COL32(40, 40, 40, 255);
}

ImU32 TitleBarMenuPopupBgColor() {
  return UsesLightTheme() ? IM_COL32(255, 255, 255, 255) : kTitleBarMenuPopupBg;
}

ImU32 TitleBarMenuPopupBorderColor() {
  return UsesLightTheme() ? IM_COL32(214, 218, 222, 255) : kTitleBarMenuPopupBorder;
}

ImU32 TitleBarMenuItemHoveredColor() {
  return UsesLightTheme() ? IM_COL32(0, 0, 0, 18) : kTitleBarMenuItemHovered;
}

ImU32 TitleBarSearchBgColor() {
  return UsesLightTheme() ? IM_COL32(255, 255, 255, 255) : kTitleBarSearchBg;
}

ImU32 TitleBarSearchHoveredColor() {
  return UsesLightTheme() ? IM_COL32(250, 251, 252, 255) : kTitleBarSearchHovered;
}

ImU32 TitleBarSearchActiveColor() {
  return UsesLightTheme() ? IM_COL32(255, 255, 255, 255) : kTitleBarSearchActive;
}

ImU32 TitleBarSearchBorderColor() {
  return UsesLightTheme() ? IM_COL32(194, 200, 206, 255) : kTitleBarSearchBorder;
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
  const ImU32 title_bar_text = TitleBarTextColor();
  ImU32 tint = close_button ? title_bar_text : MultiplyColor(title_bar_text, 0.9f);
  if (ImGui::IsItemActive()) {
    tint = TitleBarSecondaryTextColor();
  } else if (ImGui::IsItemHovered()) {
    tint = close_button ? MultiplyColor(title_bar_text, 1.4f) : MultiplyColor(title_bar_text, 1.2f);
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
  ImGui::PushStyleColor(ImGuiCol_PopupBg, ImGui::ColorConvertU32ToFloat4(TitleBarMenuPopupBgColor()));
  ImGui::PushStyleColor(ImGuiCol_Border, ImGui::ColorConvertU32ToFloat4(TitleBarMenuPopupBorderColor()));
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
    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImGui::ColorConvertU32ToFloat4(TitleBarMenuItemHoveredColor()));
    ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImGui::ColorConvertU32ToFloat4(TitleBarMenuItemHoveredColor()));
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

bool DrawViewportImageButton(const char* id, const std::shared_ptr<Texture2D>& icon, const char* tooltip,
                             const bool selected = false, const ImVec2 button_size = ImVec2(18.0f, 18.0f)) {
  const ImVec2 button_min = ImGui::GetCursorScreenPos();
  const ImRect button_rect(button_min, ImVec2(button_min.x + button_size.x, button_min.y + button_size.y));
  const bool clicked = ImGui::InvisibleButton(id, button_size);
  ImU32 tint =
      selected ? ImGui::ColorConvertFloat4ToU32(ImGui::GetStyleColorVec4(ImGuiCol_CheckMark)) : TitleBarTextColor();
  if (ImGui::IsItemActive())
    tint = MultiplyColor(tint, 0.75f);
  else if (ImGui::IsItemHovered())
    tint = MultiplyColor(tint, 1.25f);
  DrawFittedImage(ImGui::GetWindowDrawList(), icon, button_rect, tint);
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("%s", tooltip);
  return clicked;
}

const char* ConsoleMessageTypeName(const ConsoleMessageType type) {
  switch (type) {
    case ConsoleMessageType::Log:
      return "Info";
    case ConsoleMessageType::Warning:
      return "Warning";
    case ConsoleMessageType::Error:
      return "Error";
  }
  return "Unknown";
}

ImVec4 ConsoleMessageColor(const ConsoleMessageType type) {
  switch (type) {
    case ConsoleMessageType::Warning:
      return WarningTextColor();
    case ConsoleMessageType::Error:
      return ErrorTextColor();
    case ConsoleMessageType::Log:
      return UsesLightBackground() ? ImVec4(0.0f, 0.32f, 0.72f, 1.0f) : ImVec4(0.0f, 0.58f, 1.0f, 1.0f);
  }
  return ImGui::GetStyleColorVec4(ImGuiCol_Text);
}

std::string ConsoleMessagePreview(const std::string& message) {
  std::string preview;
  preview.reserve(std::min<size_t>(message.size(), 128));
  bool pending_space = false;
  for (const unsigned char character : message) {
    if (std::isspace(character)) {
      pending_space = !preview.empty();
      continue;
    }
    if (pending_space) {
      preview.push_back(' ');
      pending_space = false;
    }
    preview.push_back(static_cast<char>(character));
  }
  if (preview.size() <= 100)
    return preview;
  const size_t end = preview.find(' ', 100);
  preview.resize(end == std::string::npos ? 100 : end);
  preview += "...";
  return preview;
}

std::string FormatConsoleTimestamp(const std::time_t timestamp) {
  if (timestamp == 0)
    return "--:--:--";
  std::tm local_time{};
#ifdef _WIN32
  if (localtime_s(&local_time, &timestamp) != 0)
    return "--:--:--";
#else
  if (!localtime_r(&timestamp, &local_time))
    return "--:--:--";
#endif
  std::array<char, 9> text{};
  if (std::strftime(text.data(), text.size(), "%H:%M:%S", &local_time) == 0)
    return "--:--:--";
  return text.data();
}

bool DrawConsoleFilterButton(const char* id, const std::shared_ptr<Texture2D>& icon, const char* tooltip,
                             const bool selected, const ImVec4& severity_color) {
  constexpr ImVec2 button_size(28.0f, 28.0f);
  const ImVec2 min = ImGui::GetCursorScreenPos();
  const ImRect rect(min, ImVec2(min.x + button_size.x, min.y + button_size.y));
  const bool clicked = ImGui::InvisibleButton(id, button_size);
  const auto& style = ImGui::GetStyle();
  ImVec4 background = selected ? style.Colors[ImGuiCol_Header] : style.Colors[ImGuiCol_FrameBg];
  if (ImGui::IsItemActive())
    background = style.Colors[ImGuiCol_ButtonActive];
  else if (ImGui::IsItemHovered())
    background = style.Colors[ImGuiCol_ButtonHovered];
  ImGui::GetWindowDrawList()->AddRectFilled(rect.Min, rect.Max, ImGui::ColorConvertFloat4ToU32(background), 3.0f);
  const ImRect icon_rect(ImVec2(rect.Min.x + 5.0f, rect.Min.y + 5.0f), ImVec2(rect.Max.x - 5.0f, rect.Max.y - 5.0f));
  DrawFittedImage(icon, icon_rect,
                  ImGui::ColorConvertFloat4ToU32(selected ? severity_color : style.Colors[ImGuiCol_TextDisabled]));
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("%s", tooltip);
  return clicked;
}

bool BeginConsoleTableRow(const void* id, const float row_height) {
  ImGuiWindow* window = ImGui::GetCurrentWindow();
  window->DC.CurrLineSize.y = row_height;
  ImGui::TableNextRow(0, row_height);
  ImGui::TableSetColumnIndex(0);
  const ImVec2 min = ImGui::TableGetCellBgRect(ImGui::GetCurrentTable(), 0).Min;
  const ImVec2 max = {ImGui::TableGetCellBgRect(ImGui::GetCurrentTable(), ImGui::TableGetColumnCount() - 1).Max.x,
                      min.y + row_height};
  bool hovered = false;
  bool held = false;
  const bool clicked = ImGui::ButtonBehavior(ImRect(min, max), ImGui::GetID(id), &hovered, &held);
  if (hovered)
    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, ImGui::GetColorU32(ImGuiCol_HeaderHovered));
  return clicked;
}

struct ComponentHeaderState {
  bool open = false;
  bool enabled_changed = false;
};

ComponentHeaderState DrawComponentHeader(const std::string& name, const std::shared_ptr<Texture2D>& icon,
                                         const std::shared_ptr<Texture2D>& gear_icon, bool* enabled = nullptr,
                                         const bool enabled_mixed = false,
                                         const std::optional<ImU32>& background_color = std::nullopt) {
  constexpr float icon_size = 14.0f;
  constexpr float edge_padding = 6.0f;
  constexpr float gear_size = 18.0f;
  const auto flags = ImGuiTreeNodeFlags_AllowOverlap | ImGuiTreeNodeFlags_DefaultOpen;
  ComponentHeaderState result;
  if (background_color) {
    const bool light_theme = UsesLightTheme();
    ImGui::PushStyleColor(ImGuiCol_Header, ImGui::ColorConvertU32ToFloat4(*background_color));
    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImGui::ColorConvertU32ToFloat4(
                                                      MultiplyColor(*background_color, light_theme ? 0.95f : 1.12f)));
    ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImGui::ColorConvertU32ToFloat4(
                                                     MultiplyColor(*background_color, light_theme ? 0.90f : 1.20f)));
  }
  result.open = ImGui::CollapsingHeader("##ComponentHeader", flags);
  if (background_color)
    ImGui::PopStyleColor(3);
  const bool right_clicked = ImGui::IsItemClicked(ImGuiMouseButton_Right);
  const ImRect header(ImGui::GetItemRectMin(), ImGui::GetItemRectMax());

  const float icon_left = header.Min.x + 28.0f;
  const ImRect icon_rect(ImVec2(icon_left, header.Min.y + (header.GetHeight() - icon_size) * 0.5f),
                         ImVec2(icon_left + icon_size, header.Min.y + (header.GetHeight() + icon_size) * 0.5f));
  DrawFittedImage(icon, icon_rect, ImGui::ColorConvertFloat4ToU32(ImGui::GetStyleColorVec4(ImGuiCol_Text)));

  const float gear_left = header.Max.x - edge_padding - gear_size;
  float text_right = gear_left - edge_padding;
  if (enabled) {
    const float checkbox_size = ImGui::GetFrameHeight();
    const float checkbox_left = gear_left - edge_padding - checkbox_size;
    ImGui::SetCursorScreenPos(ImVec2(checkbox_left, header.Min.y + (header.GetHeight() - checkbox_size) * 0.5f));
    if (enabled_mixed)
      ImGui::PushItemFlag(ImGuiItemFlags_MixedValue, true);
    result.enabled_changed = ImGui::Checkbox("##Enabled", enabled);
    if (enabled_mixed)
      ImGui::PopItemFlag();
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Enabled");
    text_right = checkbox_left - edge_padding;
  }

  auto* draw_list = ImGui::GetWindowDrawList();
  draw_list->PushClipRect(ImVec2(icon_rect.Max.x + edge_padding, header.Min.y), ImVec2(text_right, header.Max.y), true);
  draw_list->AddText(
      ImVec2(icon_rect.Max.x + edge_padding, header.Min.y + (header.GetHeight() - ImGui::GetFontSize()) * 0.5f),
      ImGui::ColorConvertFloat4ToU32(ImGui::GetStyleColorVec4(ImGuiCol_Text)), name.c_str());
  draw_list->PopClipRect();

  ImGui::SetCursorScreenPos(ImVec2(gear_left, header.Min.y + (header.GetHeight() - gear_size) * 0.5f));
  if (DrawViewportImageButton("##ComponentSettings", gear_icon, "Component Settings") || right_clicked)
    ImGui::OpenPopup("ComponentSettings");
  return result;
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
  const auto build_dir = request.cmake_build_dir.empty() ? request.build_dir : request.cmake_build_dir;
  auto command = "cmake --build " + QuoteCommandArgument(build_dir.string());
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
  const auto build_dir = request.cmake_build_dir.empty() ? request.build_dir : request.cmake_build_dir;
  auto command = L"cmake --build " + QuoteWindowsCommandArgument(build_dir.wstring());
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
  const auto build_dir = request.cmake_build_dir.empty() ? request.build_dir : request.cmake_build_dir;
  auto command = "cmake --build " + QuoteShellCommandArgument(build_dir.string());
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
  request.cmake_build_dir = request.build_dir;
  const auto package_project_dir = request.build_dir / "EvoEngine_Packages" / package_name;
  if (std::filesystem::exists(package_project_dir / (request.target_name + ".vcxproj"), ec)) {
    request.cmake_build_dir = package_project_dir;
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

bool IsPackageLayerInspector(const std::shared_ptr<ILayer>& layer) {
  if (!layer) {
    return false;
  }
  const auto* inspector = InspectorRegistry::GetInstance().FindInspector(typeid(*layer));
  return inspector && !inspector->owner_name.empty();
}

bool HasLayerInspector(const std::shared_ptr<ILayer>& layer) {
  return layer && InspectorRegistry::GetInstance().FindInspector(typeid(*layer));
}

void DockLayerInspectionWindows(const ImGuiID built_in_node, const ImGuiID package_node) {
  for (const auto& layer : ApplicationContext::Get().GetLayers()) {
    if (!HasLayerInspector(layer)) {
      continue;
    }
    ImGui::DockBuilderDockWindow(layer->GetLayerName().c_str(),
                                 IsPackageLayerInspector(layer) ? package_node : built_in_node);
  }
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
  ImGui::DockBuilderDockWindow("Plant Visual", center_node);
  ImGui::DockBuilderDockWindow("Entity Explorer", left_node);
  ImGui::DockBuilderDockWindow("Entity Inspector", right_node);
  DockLayerInspectionWindows(left_node, right_node);
  ImGui::DockBuilderDockWindow("Project", bottom_node);
  ImGui::DockBuilderDockWindow("Console", bottom_node);
  ImGui::DockBuilderDockWindow("Resources", bottom_node);
  ImGui::DockBuilderDockWindow("Runtime Package Manager", bottom_node);
  ImGui::DockBuilderFinish(dock_space_id);
}

void BuildCustomEditorDockLayout(const ImGuiID dock_space_id, const ImVec2& dock_size,
                                 const EditorDockLayoutSettings& settings) {
  ImGui::DockBuilderRemoveNode(dock_space_id);
  ImGui::DockBuilderAddNode(dock_space_id, ImGuiDockNodeFlags_DockSpace);
  ImGui::DockBuilderSetNodeSize(dock_space_id, dock_size);

  ImGuiID center_node = dock_space_id;
  const ImGuiID left_node =
      ImGui::DockBuilderSplitNode(center_node, ImGuiDir_Left, settings.left_fraction, nullptr, &center_node);
  ImGuiID right_node =
      ImGui::DockBuilderSplitNode(center_node, ImGuiDir_Right, settings.right_fraction, nullptr, &center_node);
  const ImGuiID bottom_node =
      ImGui::DockBuilderSplitNode(center_node, ImGuiDir_Down, settings.bottom_fraction, nullptr, &center_node);
  const ImGuiID camera_node =
      settings.camera_fraction
          ? ImGui::DockBuilderSplitNode(center_node, ImGuiDir_Right, *settings.camera_fraction, nullptr, &center_node)
          : center_node;
  const ImGuiID plant_visual_node =
      settings.plant_visual_fraction
          ? ImGui::DockBuilderSplitNode(center_node, ImGuiDir_Right, *settings.plant_visual_fraction, nullptr,
                                        &center_node)
          : center_node;

  ImGui::DockBuilderDockWindow("Scene", center_node);
  ImGui::DockBuilderDockWindow("Camera", camera_node);
  ImGui::DockBuilderDockWindow("Plant Visual", plant_visual_node);
  ImGui::DockBuilderDockWindow("Entity Explorer", left_node);
  ImGui::DockBuilderDockWindow("Entity Inspector", right_node);
  DockLayerInspectionWindows(left_node, right_node);
  ImGui::DockBuilderDockWindow("Project", bottom_node);
  ImGui::DockBuilderDockWindow("Console", bottom_node);
  ImGui::DockBuilderFinish(dock_space_id);
}

ImVec2 ResolveFloatingWindowPosition(const EditorFloatingWindowLayout& layout) {
  const auto* viewport = ImGui::GetMainViewport();
  const ImVec2 size(layout.size.x, layout.size.y);
  const ImVec2 margin(layout.margin.x, layout.margin.y);
  switch (layout.anchor) {
    case EditorFloatingWindowLayout::Anchor::UpperRight:
      return {viewport->WorkPos.x + viewport->WorkSize.x - size.x - margin.x, viewport->WorkPos.y + margin.y};
    case EditorFloatingWindowLayout::Anchor::LowerLeft:
      return {viewport->WorkPos.x + margin.x, viewport->WorkPos.y + viewport->WorkSize.y - size.y - margin.y};
    case EditorFloatingWindowLayout::Anchor::LowerRight:
      return {viewport->WorkPos.x + viewport->WorkSize.x - size.x - margin.x,
              viewport->WorkPos.y + viewport->WorkSize.y - size.y - margin.y};
    case EditorFloatingWindowLayout::Anchor::UpperLeft:
    default:
      return {viewport->WorkPos.x + margin.x, viewport->WorkPos.y + margin.y};
  }
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

template <typename T>
void ReadYamlValue(const YAML::Node& in, const char* key, T& value) {
  if (const auto node = in[key]) {
    try {
      value = node.as<T>();
    } catch (const std::exception&) {
    }
  }
}

void SerializeEditorCameraControlKeyBindings(YAML::Emitter& out, const EditorCameraControlKeyBindings& key_bindings) {
  out << YAML::Key << "rotate_mouse_button" << YAML::Value << key_bindings.rotate_mouse_button;
  out << YAML::Key << "focus_selection_key" << YAML::Value << key_bindings.focus_selection_key;
  out << YAML::Key << "move_forward_key" << YAML::Value << key_bindings.move_forward_key;
  out << YAML::Key << "move_backward_key" << YAML::Value << key_bindings.move_backward_key;
  out << YAML::Key << "move_left_key" << YAML::Value << key_bindings.move_left_key;
  out << YAML::Key << "move_right_key" << YAML::Value << key_bindings.move_right_key;
  out << YAML::Key << "move_up_key" << YAML::Value << key_bindings.move_up_key;
  out << YAML::Key << "move_down_key" << YAML::Value << key_bindings.move_down_key;
}

void DeserializeEditorCameraControlKeyBindings(const YAML::Node& in, EditorCameraControlKeyBindings& key_bindings) {
  ReadYamlValue(in, "rotate_mouse_button", key_bindings.rotate_mouse_button);
  ReadYamlValue(in, "focus_selection_key", key_bindings.focus_selection_key);
  ReadYamlValue(in, "move_forward_key", key_bindings.move_forward_key);
  ReadYamlValue(in, "move_backward_key", key_bindings.move_backward_key);
  ReadYamlValue(in, "move_left_key", key_bindings.move_left_key);
  ReadYamlValue(in, "move_right_key", key_bindings.move_right_key);
  ReadYamlValue(in, "move_up_key", key_bindings.move_up_key);
  ReadYamlValue(in, "move_down_key", key_bindings.move_down_key);
}

void SerializeCameraSettings(YAML::Emitter& out, const CameraSettings& settings) {
  out << YAML::Key << "near_distance" << YAML::Value << settings.near_distance;
  out << YAML::Key << "far_distance" << YAML::Value << settings.far_distance;
  out << YAML::Key << "fade_ratio" << YAML::Value << settings.fade_ratio;
  out << YAML::Key << "fade_factor" << YAML::Value << settings.fade_factor;
  out << YAML::Key << "fov" << YAML::Value << settings.fov;
  out << YAML::Key << "background_source" << YAML::Value
      << Camera::GetBackgroundSourceName(Camera::ResolveBackgroundSource(settings));
  out << YAML::Key << "clear_color" << YAML::Value << settings.clear_color;
  out << YAML::Key << "background_intensity" << YAML::Value << settings.background_intensity;
  out << YAML::Key << "sample_size" << YAML::Value << settings.sample_size;
  out << YAML::Key << "bounce" << YAML::Value << settings.bounce;
  out << YAML::Key << "gamma" << YAML::Value << settings.gamma;
  out << YAML::Key << "firefly_clamp_threshold" << YAML::Value << settings.firefly_clamp_threshold;
  out << YAML::Key << "ray_debug_view" << YAML::Value << Camera::GetRayDebugViewName(settings.ray_debug_view);
  out << YAML::Key << "auto_spp_enabled" << YAML::Value << settings.auto_spp_enabled;
  out << YAML::Key << "auto_spp_min_samples" << YAML::Value << settings.auto_spp_min_samples;
  out << YAML::Key << "auto_spp_max_samples" << YAML::Value << settings.auto_spp_max_samples;
  out << YAML::Key << "auto_spp_convergence_threshold" << YAML::Value << settings.auto_spp_convergence_threshold;
  out << YAML::Key << "shader_execution_reordering_mode" << YAML::Value
      << Camera::GetShaderExecutionReorderingModeName(settings.shader_execution_reordering_mode);
}

void DeserializeCameraSettings(const YAML::Node& in, CameraSettings& settings) {
  ReadYamlValue(in, "near_distance", settings.near_distance);
  ReadYamlValue(in, "far_distance", settings.far_distance);
  ReadYamlValue(in, "fade_ratio", settings.fade_ratio);
  ReadYamlValue(in, "fade_factor", settings.fade_factor);
  ReadYamlValue(in, "fov", settings.fov);
  if (const auto source = in["background_source"]) {
    settings.background_source = Camera::ParseBackgroundSource(source.as<std::string>(), settings.background_source);
  }
  ReadYamlValue(in, "clear_color", settings.clear_color);
  ReadYamlValue(in, "background_intensity", settings.background_intensity);
  ReadYamlValue(in, "sample_size", settings.sample_size);
  ReadYamlValue(in, "bounce", settings.bounce);
  ReadYamlValue(in, "gamma", settings.gamma);
  ReadYamlValue(in, "firefly_clamp_threshold", settings.firefly_clamp_threshold);
  if (const auto view = in["ray_debug_view"]) {
    settings.ray_debug_view = Camera::ParseRayDebugView(view.as<std::string>(), settings.ray_debug_view);
  }
  ReadYamlValue(in, "auto_spp_enabled", settings.auto_spp_enabled);
  ReadYamlValue(in, "auto_spp_min_samples", settings.auto_spp_min_samples);
  ReadYamlValue(in, "auto_spp_max_samples", settings.auto_spp_max_samples);
  ReadYamlValue(in, "auto_spp_convergence_threshold", settings.auto_spp_convergence_threshold);
  if (const auto mode = in["shader_execution_reordering_mode"]) {
    settings.shader_execution_reordering_mode =
        Camera::ParseShaderExecutionReorderingMode(mode.as<std::string>(), settings.shader_execution_reordering_mode);
  }
}

bool IsFinite(const glm::vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool IsValid(const Bound& bound) {
  return IsFinite(bound.min) && IsFinite(bound.max) && bound.min.x <= bound.max.x && bound.min.y <= bound.max.y &&
         bound.min.z <= bound.max.z;
}

bool WorldUpCameraRotation(const glm::vec3& front, const glm::quat& fallback_rotation, glm::quat& camera_rotation) {
  if (!IsFinite(front) || glm::length(front) <= glm::epsilon<float>())
    return false;
  const auto front_direction = glm::normalize(front);
  auto right = glm::cross(front_direction, glm::vec3(0.0f, 1.0f, 0.0f));
  if (!IsFinite(right) || glm::length(right) <= glm::epsilon<float>()) {
    right = fallback_rotation * glm::vec3(1.0f, 0.0f, 0.0f);
    right.y = 0.0f;
  }
  if (!IsFinite(right) || glm::length(right) <= glm::epsilon<float>())
    right = glm::vec3(1.0f, 0.0f, 0.0f);
  right = glm::normalize(right);
  const auto up = glm::normalize(glm::cross(right, front_direction));
  camera_rotation = glm::normalize(glm::quatLookAt(front_direction, up));
  return std::isfinite(camera_rotation.x) && std::isfinite(camera_rotation.y) && std::isfinite(camera_rotation.z) &&
         std::isfinite(camera_rotation.w);
}

bool LookAtCenter(const glm::vec3& center, const glm::vec3& camera_position, const glm::quat& current_rotation,
                  glm::quat& camera_rotation) {
  return WorldUpCameraRotation(center - camera_position, current_rotation, camera_rotation);
}

bool CalculateFocusCameraTransform(const Camera& camera, const Bound& bound, const glm::quat& current_rotation,
                                   const glm::vec3& current_position, glm::quat& target_rotation,
                                   glm::vec3& target_position) {
  if (!IsValid(bound))
    return false;
  const auto center = bound.Center();
  const auto extents = glm::max(bound.Size() * 0.5f, glm::vec3(0.5f));
  auto view_side = current_position - center;
  if (!IsFinite(view_side) || glm::length(view_side) <= glm::epsilon<float>())
    view_side = -(current_rotation * glm::vec3(0.0f, 0.0f, -1.0f));
  if (!IsFinite(view_side) || glm::length(view_side) <= glm::epsilon<float>())
    view_side = glm::vec3(0.0f, 0.0f, 1.0f);
  view_side = glm::normalize(view_side);

  if (!LookAtCenter(center, center + view_side, current_rotation, target_rotation))
    return false;
  const auto projection = camera.GetProjection();
  const float tan_half_horizontal = 1.0f / std::abs(projection[0][0]);
  const float tan_half_vertical = 1.0f / std::abs(projection[1][1]);
  if (!std::isfinite(tan_half_horizontal) || !std::isfinite(tan_half_vertical) ||
      tan_half_horizontal <= glm::epsilon<float>() || tan_half_vertical <= glm::epsilon<float>())
    return false;

  const auto front = target_rotation * glm::vec3(0.0f, 0.0f, -1.0f);
  const auto right = target_rotation * glm::vec3(1.0f, 0.0f, 0.0f);
  const auto up = target_rotation * glm::vec3(0.0f, 1.0f, 0.0f);
  float distance = 0.0f;
  for (int x = -1; x <= 1; x += 2)
    for (int y = -1; y <= 1; y += 2)
      for (int z = -1; z <= 1; z += 2) {
        const glm::vec3 offset = extents * glm::vec3(x, y, z);
        const float forward_offset = glm::dot(offset, front);
        distance = std::max(distance, std::abs(glm::dot(offset, right)) / tan_half_horizontal - forward_offset);
        distance = std::max(distance, std::abs(glm::dot(offset, up)) / tan_half_vertical - forward_offset);
        distance = std::max(distance, camera.camera_settings.near_distance - forward_offset);
      }
  distance = std::max(distance * 1.15f, camera.camera_settings.near_distance + 0.01f);
  target_position = center - front * distance;
  return IsFinite(target_position);
}
}  // namespace

bool EditorLayer::HasUsableImGuiDockLayout(const std::string& ini_settings) {
  return ini_settings.find("[Docking][Data]") != std::string::npos;
}

void EditorLayer::OnCreate() {
  enable_inspection = false;
  const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>();
  if (!window_layer) {
    throw std::runtime_error("WindowLayer not present!");
  }
  editor_theme::ApplyDefault();

  basic_entity_archetype_ = Entities::CreateEntityArchetype("General", GlobalTransform(), Transform());
  RegisterComponentDataBatchInspector<Transform>(
      [this](const std::shared_ptr<Scene>& scene, const std::vector<Entity>& targets) {
        return DrawBatchTransformInspector(scene, targets);
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
  scene_camera->camera_settings.background_source = Camera::BackgroundSource::Cubemap;
  scene_camera->OnCreate();
  scene_camera->post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
  RegisterEditorCamera(scene_camera);
  scene_camera_handle_ = scene_camera->GetHandle();
  auto& editor_camera = editor_cameras_[scene_camera_handle_];
  editor_camera.position = default_scene_camera_position;
  editor_camera.rotation = default_scene_camera_rotation;
}

void EditorLayer::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "enable_inspection" << YAML::Value << enable_inspection;
  out << YAML::Key << "show_console_window" << YAML::Value << show_console_window;
  out << YAML::Key << "show_scene_window" << YAML::Value << show_scene_window;
  out << YAML::Key << "show_camera_window" << YAML::Value << show_camera_window;
  out << YAML::Key << "show_camera_info" << YAML::Value << show_camera_info;
  out << YAML::Key << "show_play_buttons" << YAML::Value << show_play_buttons;
  out << YAML::Key << "show_scene_info" << YAML::Value << show_scene_info;
  out << YAML::Key << "show_entity_explorer_window" << YAML::Value << show_entity_explorer_window;
  out << YAML::Key << "show_entity_inspector_window" << YAML::Value << show_entity_inspector_window;
  out << YAML::Key << "show_package_manager_window" << YAML::Value << show_package_manager_window;
  out << YAML::Key << "main_camera_focus_override" << YAML::Value << main_camera_focus_override;
  out << YAML::Key << "scene_camera_focus_override" << YAML::Value << scene_camera_focus_override;
  out << YAML::Key << "selected_hierarchy_display_mode" << YAML::Value << selected_hierarchy_display_mode;
  out << YAML::Key << "velocity" << YAML::Value << velocity;
  out << YAML::Key << "sensitivity" << YAML::Value << sensitivity;
  out << YAML::Key << "camera_control_acceleration_time" << YAML::Value << camera_control_acceleration_time;
  out << YAML::Key << "camera_control_deceleration_time" << YAML::Value << camera_control_deceleration_time;
  out << YAML::Key << "editor_camera_control_key_bindings" << YAML::Value << YAML::BeginMap;
  SerializeEditorCameraControlKeyBindings(out, editor_camera_control_key_bindings);
  out << YAML::EndMap;
  out << YAML::Key << "apply_transform_to_main_camera" << YAML::Value << apply_transform_to_main_camera;
  out << YAML::Key << "default_scene_camera_rotation" << YAML::Value << default_scene_camera_rotation;
  out << YAML::Key << "default_scene_camera_position" << YAML::Value << default_scene_camera_position;
  out << YAML::Key << "highlight_selection" << YAML::Value << entity_selection_highlight_.IsEnabled();
  out << YAML::Key << "enable_gizmos" << YAML::Value << enable_gizmos;
  out << YAML::Key << "transform_read_only" << YAML::Value << transform_read_only;
  out << YAML::Key << "enable_console_logs" << YAML::Value << enable_console_logs_;
  out << YAML::Key << "enable_console_errors" << YAML::Value << enable_console_errors_;
  out << YAML::Key << "enable_console_warnings" << YAML::Value << enable_console_warnings_;
  out << YAML::Key << "console_clear_on_play" << YAML::Value << console_clear_on_play_;
  out << YAML::Key << "main_camera_resolution_x" << YAML::Value << main_camera_resolution_x;
  out << YAML::Key << "main_camera_resolution_y" << YAML::Value << main_camera_resolution_y;
  out << YAML::Key << "main_camera_allow_auto_resize" << YAML::Value << main_camera_allow_auto_resize;
  out << YAML::Key << "scene_camera_resolution_multiplier" << YAML::Value << scene_camera_resolution_multiplier;
  out << YAML::Key << "main_camera_resolution_multiplier" << YAML::Value << main_camera_resolution_multiplier_;
  out << YAML::Key << "local_position_selected" << YAML::Value << local_position_selected_;
  out << YAML::Key << "local_rotation_selected" << YAML::Value << local_rotation_selected_;
  out << YAML::Key << "local_scale_selected" << YAML::Value << local_scale_selected_;
  out << YAML::Key << "entity_gizmo_pivot_mode" << YAML::Value << static_cast<int>(entity_gizmo_pivot_mode_);
  out << YAML::Key << "entity_gizmo_center_default_migrated" << YAML::Value << true;
  out << YAML::Key << "entity_gizmo_orientation_mode" << YAML::Value
      << static_cast<int>(entity_gizmo_orientation_mode_);

  if (const auto search = editor_cameras_.find(scene_camera_handle_); search != editor_cameras_.end()) {
    const auto& cam = search->second;
    out << YAML::Key << "scene_camera_position" << YAML::Value << cam.position;
    out << YAML::Key << "scene_camera_rotation" << YAML::Value << cam.rotation;
    if (cam.camera) {
      out << YAML::Key << "scene_camera_settings" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "render_mode" << YAML::Value
          << Camera::GetCameraRenderModeName(cam.camera->camera_render_mode);
      SerializeCameraSettings(out, cam.camera->camera_settings);
      out << YAML::EndMap;
    }
  }

  uint64_t first_asset_handle = 0;
  out << YAML::Key << "inspecting_assets" << YAML::Value << YAML::BeginSeq;
  for (const auto& inspector_window : inspecting_assets_) {
    if (inspector_window.asset) {
      const auto asset_handle = inspector_window.asset->GetHandle().GetValue();
      if (first_asset_handle == 0) {
        first_asset_handle = asset_handle;
      }
      out << asset_handle;
    }
  }
  out << YAML::EndSeq;
  out << YAML::Key << "inspecting_asset" << YAML::Value << first_asset_handle;

  if (ImGui::GetCurrentContext()) {
    if (const char* ini_settings = ImGui::SaveIniSettingsToMemory()) {
      out << YAML::Key << "ImGuiIni" << YAML::Value << std::string(ini_settings);
      editor_layout_dirty_ = false;
    }
  }
}

void EditorLayer::DeserializeLayout(const YAML::Node& in) {
  if (!in || !in.IsMap()) {
    RequestDefaultEditorLayout();
    return;
  }

  const auto normalize_rotation = [](const glm::quat& rotation, const glm::quat& fallback) {
    const float length_squared = glm::dot(rotation, rotation);
    if (std::isfinite(length_squared) && length_squared > glm::epsilon<float>()) {
      return glm::normalize(rotation);
    }
    return fallback;
  };

  ReadYamlValue(in, "enable_inspection", enable_inspection);
  ReadYamlValue(in, "show_console_window", show_console_window);
  ReadYamlValue(in, "show_scene_window", show_scene_window);
  ReadYamlValue(in, "show_camera_window", show_camera_window);
  ReadYamlValue(in, "show_camera_info", show_camera_info);
  ReadYamlValue(in, "show_play_buttons", show_play_buttons);
  ReadYamlValue(in, "show_scene_info", show_scene_info);
  ReadYamlValue(in, "show_entity_explorer_window", show_entity_explorer_window);
  ReadYamlValue(in, "show_entity_inspector_window", show_entity_inspector_window);
  ReadYamlValue(in, "show_package_manager_window", show_package_manager_window);
  ReadYamlValue(in, "main_camera_focus_override", main_camera_focus_override);
  ReadYamlValue(in, "scene_camera_focus_override", scene_camera_focus_override);
  ReadYamlValue(in, "selected_hierarchy_display_mode", selected_hierarchy_display_mode);
  ReadYamlValue(in, "velocity", velocity);
  ReadYamlValue(in, "sensitivity", sensitivity);
  ReadYamlValue(in, "camera_control_acceleration_time", camera_control_acceleration_time);
  ReadYamlValue(in, "camera_control_deceleration_time", camera_control_deceleration_time);
  camera_control_acceleration_time = ClampFiniteNonnegative(camera_control_acceleration_time, 0.25f);
  camera_control_deceleration_time = ClampFiniteNonnegative(camera_control_deceleration_time, 0.25f);
  if (const auto node = in["editor_camera_control_key_bindings"]; node && node.IsMap()) {
    DeserializeEditorCameraControlKeyBindings(node, editor_camera_control_key_bindings);
  }
  ReadYamlValue(in, "apply_transform_to_main_camera", apply_transform_to_main_camera);
  bool highlight_selection = entity_selection_highlight_.IsEnabled();
  ReadYamlValue(in, "highlight_selection", highlight_selection);
  entity_selection_highlight_.SetEnabled(highlight_selection, !entity_selection_.Empty());
  ReadYamlValue(in, "enable_gizmos", enable_gizmos);
  ReadYamlValue(in, "transform_read_only", transform_read_only);
  ReadYamlValue(in, "enable_console_logs", enable_console_logs_);
  ReadYamlValue(in, "enable_console_errors", enable_console_errors_);
  ReadYamlValue(in, "enable_console_warnings", enable_console_warnings_);
  ReadYamlValue(in, "console_clear_on_play", console_clear_on_play_);
  ReadYamlValue(in, "main_camera_resolution_x", main_camera_resolution_x);
  ReadYamlValue(in, "main_camera_resolution_y", main_camera_resolution_y);
  ReadYamlValue(in, "main_camera_allow_auto_resize", main_camera_allow_auto_resize);
  ReadYamlValue(in, "scene_camera_resolution_multiplier", scene_camera_resolution_multiplier);
  ReadYamlValue(in, "main_camera_resolution_multiplier", main_camera_resolution_multiplier_);
  ReadYamlValue(in, "local_position_selected", local_position_selected_);
  ReadYamlValue(in, "local_rotation_selected", local_rotation_selected_);
  ReadYamlValue(in, "local_scale_selected", local_scale_selected_);
  if (local_position_selected_)
    SelectLocalTransformGizmoOperation(LocalTransformGizmoOperation::Translate);
  else if (local_rotation_selected_)
    SelectLocalTransformGizmoOperation(LocalTransformGizmoOperation::Rotate);
  else if (local_scale_selected_)
    SelectLocalTransformGizmoOperation(LocalTransformGizmoOperation::Scale);
  else
    SelectLocalTransformGizmoOperation(LocalTransformGizmoOperation::Select);
  int entity_gizmo_pivot_mode = static_cast<int>(entity_gizmo_pivot_mode_);
  int entity_gizmo_orientation_mode = static_cast<int>(entity_gizmo_orientation_mode_);
  bool entity_gizmo_center_default_migrated = false;
  ReadYamlValue(in, "entity_gizmo_center_default_migrated", entity_gizmo_center_default_migrated);
  if (entity_gizmo_center_default_migrated)
    ReadYamlValue(in, "entity_gizmo_pivot_mode", entity_gizmo_pivot_mode);
  else
    entity_gizmo_pivot_mode = static_cast<int>(EntityGizmoPivotMode::Center);
  ReadYamlValue(in, "entity_gizmo_orientation_mode", entity_gizmo_orientation_mode);
  entity_gizmo_pivot_mode_ = entity_gizmo_pivot_mode == 1 ? EntityGizmoPivotMode::Center : EntityGizmoPivotMode::Pivot;
  entity_gizmo_orientation_mode_ =
      entity_gizmo_orientation_mode == 1 ? EntityGizmoOrientationMode::Global : EntityGizmoOrientationMode::Local;

  glm::quat scene_camera_rotation = default_scene_camera_rotation;
  ReadYamlValue(in, "default_scene_camera_rotation", scene_camera_rotation);
  default_scene_camera_rotation = normalize_rotation(scene_camera_rotation, default_scene_camera_rotation);
  ReadYamlValue(in, "default_scene_camera_position", default_scene_camera_position);

  if (const auto search = editor_cameras_.find(scene_camera_handle_); search != editor_cameras_.end()) {
    auto& cam = search->second;
    glm::vec3 scene_camera_position = cam.position;
    if (const auto node = in["scene_camera_position"]) {
      ReadYamlValue(in, "scene_camera_position", scene_camera_position);
      cam.position = scene_camera_position;
      default_scene_camera_position = cam.position;
    }
    if (const auto node = in["scene_camera_rotation"]) {
      ReadYamlValue(in, "scene_camera_rotation", scene_camera_rotation);
      cam.rotation = normalize_rotation(scene_camera_rotation, default_scene_camera_rotation);
      default_scene_camera_rotation = cam.rotation;
    }
    if (const auto node = in["scene_camera_settings"]; node && cam.camera) {
      if (const auto render_mode = node["render_mode"]) {
        cam.camera->camera_render_mode =
            Camera::ParseCameraRenderMode(render_mode.as<std::string>(), cam.camera->camera_render_mode);
      }
      DeserializeCameraSettings(node, cam.camera->camera_settings);
      cam.camera->SetRequireRendering(true);
      cam.camera->ResetFrameCount();
    }
  }

  if (const auto node = in["ImGuiIni"]) {
    ReadYamlValue(in, "ImGuiIni", pending_imgui_ini_settings_);
    has_pending_imgui_ini_settings_ = HasUsableImGuiDockLayout(pending_imgui_ini_settings_);
    if (!has_pending_imgui_ini_settings_) {
      pending_imgui_ini_settings_.clear();
      RequestDefaultEditorLayout();
    } else {
      dock_layout_reset_pending_ = false;
    }
  } else {
    RequestDefaultEditorLayout();
  }
}

void EditorLayer::Deserialize(const YAML::Node& in) {
  DeserializeLayout(in);
  DeserializeSceneState(in);
}

void EditorLayer::DeserializeSceneState(const YAML::Node& in) {
  ClearEntitySelectionState();
  if (!in || !in.IsMap()) {
    return;
  }

  ClearAssetInspectors();
  if (const auto nodes = in["inspecting_assets"]; nodes && nodes.IsSequence()) {
    for (const auto node : nodes) {
      uint64_t handle = 0;
      try {
        handle = node.as<uint64_t>();
      } catch (const std::exception&) {
      }
      if (handle != 0) {
        if (const auto asset = AssetManager::GetAssetImpl(Handle(handle))) {
          OpenAssetInspector(asset);
        }
      }
    }
  } else if (const auto node = in["inspecting_asset"]) {
    uint64_t handle = 0;
    ReadYamlValue(in, "inspecting_asset", handle);
    if (handle != 0) {
      if (const auto asset = AssetManager::GetAssetImpl(Handle(handle))) {
        OpenAssetInspector(asset);
      }
    }
  }
}

void EditorLayer::ClearEntitySelectionState() {
  CancelEntityGizmoSession();
  transform_inspector_drag_session_.reset();
  entity_selection_.Clear(EntitySelection::RequestSource::Lifecycle);
  entity_selection_highlight_.Reset();
  selected_entity_hierarchy_list_.clear();
  entity_explorer_visible_entities_.clear();
  entity_explorer_current_entities_.clear();
  pending_viewport_selection_.reset();
  ClearEnvironmentalLightingGizmoTarget();
}

bool EditorLayer::DefaultEditorLayoutPending() const {
  return dock_layout_reset_pending_;
}

bool EditorLayer::IsPlantVisualSplitLayoutReady() const {
  return !dock_layout_reset_pending_ && custom_layout_settings_ && custom_layout_settings_->dock_layout &&
         custom_layout_settings_->dock_layout->plant_visual_fraction.has_value();
}

void EditorLayer::OnDestroy() {
  CancelEntityGizmoSession();
  transform_inspector_drag_session_.reset();
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
  editor_panel_manager_.DrawExcept(EditorPanelCategory::View, editor_layer, "project");
  DrawLayerInspectionWindows(scene, editor_layer);
  editor_panel_manager_.DrawOnly(EditorPanelCategory::View, editor_layer, "project");
  DrawAssetInspectorWindows();
  DrawProjectLoadingPopup();
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

void EditorLayer::OpenAssetInspector(const Handle& asset_handle) {
  if (asset_handle.GetValue() == 0) {
    return;
  }
  if (const auto asset = AssetManager::PeekAssetImpl(asset_handle)) {
    OpenAssetInspector(asset);
    return;
  }
  const auto file = FileManager::GetFile(asset_handle);
  if (!file || file->GetAssetTypeName() == "Binary") {
    return;
  }
  auto& pending_load = pending_asset_inspector_loads_[asset_handle.GetValue()];
  if (pending_load.valid()) {
    return;
  }
  try {
    pending_load = AssetManager::RequestAssetLoad(asset_handle);
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to queue asset inspector load: " + std::string(e.what()))
    pending_asset_inspector_loads_.erase(asset_handle.GetValue());
  } catch (...) {
    EVOENGINE_ERROR("Failed to queue asset inspector load.")
    pending_asset_inspector_loads_.erase(asset_handle.GetValue());
  }
}

void EditorLayer::ClearAssetInspectors() {
  inspecting_assets_.clear();
  pending_asset_inspector_loads_.clear();
}

void EditorLayer::PollPendingAssetInspectorLoads() {
  for (auto it = pending_asset_inspector_loads_.begin(); it != pending_asset_inspector_loads_.end();) {
    auto& future = it->second;
    if (!future.valid()) {
      it = pending_asset_inspector_loads_.erase(it);
      continue;
    }
    if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      ++it;
      continue;
    }
    try {
      OpenAssetInspector(future.get());
    } catch (const std::exception& e) {
      EVOENGINE_ERROR("Failed to open queued asset inspector: " + std::string(e.what()))
    } catch (...) {
      EVOENGINE_ERROR("Failed to open queued asset inspector.")
    }
    it = pending_asset_inspector_loads_.erase(it);
  }
}

std::string EditorLayer::GetAssetRefDisplayName(const AssetRef& target) {
  if (const auto asset = target.Peek<IAsset>()) {
    return asset->GetTitle();
  }
  const auto asset_handle = target.GetAssetHandle();
  if (asset_handle.GetValue() == 0) {
    return "none";
  }
  if (const auto file = FileManager::GetFile(asset_handle)) {
    const auto stem = file->GetAssetsFolderRelativePath().stem().string();
    if (!stem.empty()) {
      return stem;
    }
    return file->GetAssetFileName() + file->GetAssetExtension();
  }
  const auto type_name = target.GetAssetTypeName();
  return type_name.empty() ? "Missing asset" : type_name;
}

std::string EditorLayer::GetAssetRefImGuiTag(const AssetRef& target) {
  auto type_name = target.GetAssetTypeName();
  if (type_name.empty()) {
    type_name = "Asset";
  }
  return "##" + type_name + std::to_string(target.GetAssetHandle().GetValue());
}

void EditorLayer::DraggableAssetRef(const AssetRef& target) {
  if (const auto asset = target.Peek<IAsset>()) {
    DraggableAsset(asset);
    return;
  }
  const auto asset_handle = target.GetAssetHandle();
  if (asset_handle.GetValue() == 0) {
    return;
  }
  if (ImGui::BeginDragDropSource()) {
    ImGui::SetDragDropPayload("Asset", &asset_handle, sizeof(Handle));
    ImGui::TextColored(ImGui::GetStyleColorVec4(ImGuiCol_TextLink), GetAssetRefDisplayName(target).c_str());
    ImGui::EndDragDropSource();
  }
}

void EditorLayer::DrawAssetInspectorWindows() {
  PollPendingAssetInspectorLoads();
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
    if (custom_layout_settings_ && custom_layout_settings_->asset_inspector_window &&
        asset_inspector_window_layout_pending_) {
      const auto& window_layout = *custom_layout_settings_->asset_inspector_window;
      ImGui::SetNextWindowPos(ResolveFloatingWindowPosition(window_layout), ImGuiCond_Always);
      ImGui::SetNextWindowSize(ImVec2(window_layout.size.x, window_layout.size.y), ImGuiCond_Always);
      asset_inspector_window_layout_pending_ = false;
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
      if (!open) {
        ClearEnvironmentalLightingGizmoTarget(Handle(asset_handle));
      }
    }
  }

  inspecting_assets_.erase(std::remove_if(inspecting_assets_.begin(), inspecting_assets_.end(),
                                          [](const AssetInspectorWindow& inspector) {
                                            return !inspector.open || !inspector.asset;
                                          }),
                           inspecting_assets_.end());
}

void EditorLayer::DrawProjectLoadingPopup() {
  auto& project_manager = ProjectManager::GetInstance();
  const auto asset_load_snapshot = AssetManager::GetAssetLoadSnapshot();
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  const bool scene_ready = project_manager.start_scene_ != nullptr;
  const bool pending_scene_load = !project_manager.new_project_path_.empty() && !scene_ready &&
                                  ApplicationContext::Get().GetApplicationInfo().load_project_start_scene;
  if (project_manager.scene_loading_popup_visible_ && scene_ready) {
    if (const auto scene_camera = GetSceneCamera()) {
      scene_camera->SetRequireRendering(true);
    }
    if (!render_layer || render_layer->HasPresentedScene(scene)) {
      project_manager.scene_loading_popup_visible_ = false;
      project_manager.loading_status_ = "Scene ready.";
      EVOENGINE_LOG("Scene is ready.")
    }
  }
  constexpr ImGuiWindowFlags modal_flags = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings;

  if (project_manager.scan_assets_pending && !scene_ready) {
    ImGui::OpenPopup("Scanning assets...");
  } else if (asset_load_snapshot.Active() && !scene_ready) {
    ImGui::OpenPopup("Loading assets...");
  } else if (project_manager.scene_loading_popup_visible_ || pending_scene_load) {
    ImGui::OpenPopup("Loading Scene...");
  } else if (!project_manager.new_project_path_.empty() && !scene_ready) {
    ImGui::OpenPopup("Loading Project...");
  }

  const auto draw_loading_status = [&]() {
    ImGui::Text("%s", project_manager.loading_status_.empty() ? "Busy..." : project_manager.loading_status_.c_str());
  };

  if (ImGui::BeginPopupModal("Loading Scene...", nullptr, modal_flags)) {
    ImGui::TextUnformatted("Scene is loading.");
    draw_loading_status();
    ImGui::SetItemDefaultFocus();
    if (!project_manager.scene_loading_popup_visible_ && !pending_scene_load) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  if (ImGui::BeginPopupModal("Loading Project...", nullptr, modal_flags)) {
    draw_loading_status();
    if (project_manager.new_project_path_.empty() || scene_ready || pending_scene_load) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  if (ImGui::BeginPopupModal("Scanning assets...", nullptr, modal_flags)) {
    draw_loading_status();
    if (!project_manager.scan_assets_pending || scene_ready) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  if (ImGui::BeginPopupModal("Loading assets...", nullptr, modal_flags)) {
    if (!project_manager.loading_status_.empty()) {
      ImGui::Text("%s", project_manager.loading_status_.c_str());
    }
    ImGui::TextUnformatted("Progress:");
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
    if (!asset_load_snapshot.Active() || scene_ready) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
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
  register_panel("profiler", "Profiler", show_profiler_window, [this](const std::shared_ptr<EditorLayer>&) {
    DrawProfilerWindow();
  });
  register_panel("resources", "Resources", Resources::GetInstance().show_resources_,
                 [](const std::shared_ptr<EditorLayer>& editor_layer) {
                   Resources::Draw(editor_layer);
                 });
  project_content_browser_panel_ = std::make_shared<ProjectContentBrowserPanel>();
  editor_panel_manager_.RegisterPanel(EditorPanelCategory::View, "project", "Project",
                                      ProjectManager::GetInstance().show_project_window,
                                      project_content_browser_panel_);
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
  const auto interpolated_rotation = glm::mix(previous_rotation_, target_rotation_, a);
  sceneCameraRotation = interpolated_rotation;
  if (transition_preserves_world_up_)
    WorldUpCameraRotation(interpolated_rotation * glm::vec3(0.0f, 0.0f, -1.0f), previous_rotation_,
                          sceneCameraRotation);
  sceneCameraPosition = glm::mix(previous_position_, target_position_, a);
  if (a >= 1.0f) {
    lock_camera = false;
    sceneCameraRotation = target_rotation_;
    sceneCameraPosition = target_position_;
    transition_preserves_world_up_ = false;
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
    ApplySceneCameraPreviewWindowLayout();
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

void EditorLayer::ApplySceneCameraPreviewWindowLayout() {
  if (!scene_camera_preview_window_size_) {
    return;
  }

  const auto* viewport = ImGui::GetMainViewport();
  if (!viewport) {
    scene_camera_preview_window_size_.reset();
    return;
  }

  const auto size = *scene_camera_preview_window_size_;
  ImGui::SetNextWindowViewport(viewport->ID);
  ImGui::SetNextWindowDockID(0, ImGuiCond_Always);
  ImGui::SetNextWindowPos(viewport->WorkPos, ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(static_cast<float>(size.x), static_cast<float>(size.y)), ImGuiCond_Always);
  ImGui::SetNextWindowCollapsed(false, ImGuiCond_Always);
  ImGui::SetNextWindowFocus();
  scene_camera_preview_window_size_.reset();
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
  if (entity_selection_.BindScene(scene) == EntitySelection::Result::Changed) {
    CancelEntityGizmoSession();
    transform_inspector_drag_session_.reset();
    selected_entity_hierarchy_list_.clear();
    pending_viewport_selection_.reset();
  }
  const auto selection_revision = entity_selection_.GetRevision();
  entity_selection_.PruneInvalid();
  if (selection_revision != entity_selection_.GetRevision()) {
    CancelEntityGizmoSession();
    transform_inspector_drag_session_.reset();
  }
  entity_selection_highlight_.Update(!entity_selection_.Empty(),
                                     static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime()));
  if (scene && show_scene_window)
    ResizeCameras();

  if (scene && !main_camera_window_focused_) {
    auto& pressed_keys = scene->pressed_keys_;
    pressed_keys.clear();
  }

  if (scene && apply_transform_to_main_camera && !ApplicationContext::Get().IsPlaying()) {
    if (const auto camera = scene->main_camera.Get<Camera>(); camera && scene->IsEntityValid(camera->GetOwner())) {
      auto& [sceneCameraRotation, sceneCameraPosition, sceneCamera] = editor_cameras_.at(scene_camera_handle_);
      const auto rotation_length_squared = glm::dot(sceneCameraRotation, sceneCameraRotation);
      if (IsFiniteVector(sceneCameraPosition) && std::isfinite(rotation_length_squared) &&
          rotation_length_squared > glm::epsilon<float>()) {
        GlobalTransform global_transform;
        global_transform.SetValue(sceneCameraPosition, glm::normalize(sceneCameraRotation), glm::vec3(1.0f));
        if (!MatricesNear(scene->GetDataComponent<GlobalTransform>(camera->GetOwner()).value, global_transform.value))
          scene->SetDataComponent(camera->GetOwner(), global_transform);
      }
    }
  }
}

void EditorLayer::DrawEntityExplorerWindow(const std::shared_ptr<Scene>& scene) {
  if (!show_entity_explorer_window) {
    entity_explorer_window_focused_ = false;
    entity_explorer_visible_entities_.clear();
    return;
  }
  ImGui::Begin("Entity Explorer");
  entity_explorer_window_focused_ = ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows);
  if (entity_explorer_window_focused_ && !GetLockEntitySelection() &&
      Input::GetKey(GLFW_KEY_ESCAPE) == Input::KeyActionType::Press) {
    SetSelectedEntity({});
  }
  entity_explorer_current_entities_.clear();
  if (scene) {
    if (ImGui::BeginPopupContextWindow("NewEntityPopup")) {
      if (ImGui::Button("Create new entity")) {
        scene->CreateEntity(basic_entity_archetype_);
      }
      ImGui::EndPopup();
    }
    const char* hierarchy_display_mode[]{"Archetype", "Hierarchy"};

    if (ImGui::Combo("Display mode", &selected_hierarchy_display_mode, hierarchy_display_mode,
                     IM_ARRAYSIZE(hierarchy_display_mode))) {
      entity_selection_.ClearAnchor(EntitySelection::RequestSource::Lifecycle);
      entity_explorer_visible_entities_.clear();
    }
    if (selected_hierarchy_display_mode == 0) {
      scene->UnsafeForEachEntityStorage([&](size_t i, const std::string& name, const DataComponentStorage& storage) {
        if (i == 0)
          return;
        ImGui::Separator();
        const std::string title1 = std::to_string(i) + ". " + name;
        if (ImGui::TreeNode(title1.c_str())) {
          for (size_t j = 0; j < storage.entity_alive_count; j++) {
            Entity entity = storage.chunk_array.entity_array.at(j);
            entity_explorer_current_entities_.push_back(entity);
            std::string title2 = std::to_string(entity.GetIndex()) + ": ";
            title2 += scene->GetEntityName(entity);
            const bool enabled = scene->IsEntityEnabled(entity);
            const bool inherited_highlight = IsInheritedSelectionHighlight(entity);
            if (enabled || inherited_highlight) {
              ImGui::PushStyleColor(ImGuiCol_Text, inherited_highlight ? ImVec4(1.0f, 0.75f, 0.0f, 1.0f)
                                                                       : ImGui::GetStyleColorVec4(ImGuiCol_Text));
            }
            const bool selected = entity_selection_.Contains(entity);
            ImGui::TreeNodeEx(
                title2.c_str(),
                ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoAutoOpenOnLog |
                    (selected ? ImGuiTreeNodeFlags_Selected : ImGuiTreeNodeFlags_None) |
                    (GetSelectedEntity() == entity ? ImGuiTreeNodeFlags_Framed : ImGuiTreeNodeFlags_FramePadding));
            if (enabled || inherited_highlight) {
              ImGui::PopStyleColor();
            }
            DrawEntityMenu(enabled, entity);
            if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(0)) {
              HandleEntityExplorerSelection(entity);
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
  entity_explorer_visible_entities_ = entity_explorer_current_entities_;
  ImGui::End();
}

void EditorLayer::DrawEntityInspectorWindow(const std::shared_ptr<Scene>& scene,
                                            const std::shared_ptr<EditorLayer>& editor_layer) {
  if (!show_entity_inspector_window) {
    return;
  }
  ImGui::Begin("Entity Inspector");
  if (scene) {
    const auto selected_entity = GetSelectedEntity();
    if (entity_selection_.GetCount() > 1) {
      DrawBatchEntityInspector(scene, editor_layer);
    } else if (scene->IsEntityValid(selected_entity)) {
      std::string title = std::to_string(selected_entity.GetIndex()) + ": ";
      title += scene->GetEntityName(selected_entity);
      bool enabled = scene->IsEntityEnabled(selected_entity);
      if (ImGui::Checkbox((title + "##EnabledCheckbox").c_str(), &enabled)) {
        if (scene->IsEntityEnabled(selected_entity) != enabled) {
          scene->SetEnable(selected_entity, enabled);
        }
      }
      ImGui::SameLine();
      bool is_static = scene->IsEntityStatic(selected_entity);
      if (ImGui::Checkbox("Static##StaticCheckbox", &is_static)) {
        if (scene->IsEntityStatic(selected_entity) != is_static) {
          scene->SetEntityStatic(selected_entity, is_static);
        }
      }
      const bool deleted = DrawEntityMenu(scene->IsEntityEnabled(selected_entity), selected_entity);
      ImGui::SameLine();
      bool selection_locked = GetLockEntitySelection();
      if (ImGui::Checkbox("Lock", &selection_locked))
        SetLockEntitySelection(selection_locked);
      ImGui::SameLine();
      if (ImGui::Button("Clear"))
        SetSelectedEntity({});

      if (!deleted) {
        const auto snapshot = entity_selection_.GetSnapshot();
        const auto context = EntityBatchInspector::BuildContext(scene, snapshot.entities, snapshot.primary);
        DrawEntityComponentInspectors(scene, editor_layer, context);
      }
    } else if (selected_entity.GetIndex() != 0) {
      SetSelectedEntity(Entity());
    }
  } else {
    ImGui::Text("No Scene!");
  }
  ImGui::End();
}

bool EditorLayer::DrawBatchTransformInspector(const std::shared_ptr<Scene>& scene, const std::vector<Entity>& targets) {
  transform_read_only = false;
  bool edited = false;
  auto draw_field = [&](const char* label, const int field, EntityBatchValue<glm::vec3> value, const float reset_value,
                        const float speed) {
    ImGui::PushID(field);
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    ImGui::TableSetColumnIndex(1);

    static constexpr const char* axis_labels[] = {"X", "Y", "Z"};
    static constexpr ImVec4 axis_colors[] = {ImVec4(0.8f, 0.1f, 0.15f, 1.0f), ImVec4(0.2f, 0.7f, 0.2f, 1.0f),
                                             ImVec4(0.1f, 0.25f, 0.8f, 1.0f)};
    static constexpr ImVec4 axis_hovered_colors[] = {ImVec4(0.9f, 0.2f, 0.2f, 1.0f), ImVec4(0.3f, 0.8f, 0.3f, 1.0f),
                                                     ImVec4(0.2f, 0.35f, 0.9f, 1.0f)};
    const float spacing = 1.0f;
    const float button_width = ImGui::GetFrameHeight();
    const float input_width =
        std::max(24.0f, (ImGui::GetContentRegionAvail().x - button_width * 3.0f - spacing * 5.0f) / 3.0f);
    for (int axis = 0; axis < 3; ++axis) {
      ImGui::PushID(axis);
      ImGui::PushStyleColor(ImGuiCol_Button, axis_colors[axis]);
      ImGui::PushStyleColor(ImGuiCol_ButtonHovered, axis_hovered_colors[axis]);
      ImGui::PushStyleColor(ImGuiCol_ButtonActive, axis_colors[axis]);
      if (ImGui::Button(axis_labels[axis], ImVec2(button_width, 0.0f))) {
        value.value[axis] = reset_value;
        edited |= EntityBatchInspector::WriteLocalTransformField(scene, targets, field, value.value, axis);
      }
      ImGui::PopStyleColor(3);
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Reset %s %s to %.0f", label, axis_labels[axis], reset_value);
      ImGui::SameLine(0.0f, spacing);
      ImGui::SetNextItemWidth(input_width);
      if (value.mixed_axes[axis])
        ImGui::PushItemFlag(ImGuiItemFlags_MixedValue, true);
      const float start_value = value.value[axis];
      const bool changed = ImGui::DragFloat("##Value", &value.value[axis], speed, 0.0f, 0.0f, "%.2f");
      if (ImGui::IsItemActivated() && targets.size() > 1) {
        TransformInspectorDragSession session;
        session.scene = scene;
        session.selection_revision = entity_selection_.GetRevision();
        session.field = field;
        session.axis = axis;
        session.start_value = start_value;
        session.targets = targets;
        session.original_transforms.reserve(targets.size());
        for (const auto target : targets) {
          if (!scene->IsEntityValid(target) || !scene->HasDataComponent<Transform>(target)) {
            session.original_transforms.clear();
            break;
          }
          session.original_transforms.emplace_back(scene->GetDataComponent<Transform>(target));
        }
        transform_inspector_drag_session_ =
            session.original_transforms.size() == targets.size() ? std::optional(std::move(session)) : std::nullopt;
      }
      const bool relative_drag =
          targets.size() > 1 && ImGui::IsMouseDragging(ImGuiMouseButton_Left, 0.0f) &&
          transform_inspector_drag_session_ && transform_inspector_drag_session_->scene.lock() == scene &&
          transform_inspector_drag_session_->selection_revision == entity_selection_.GetRevision() &&
          transform_inspector_drag_session_->field == field && transform_inspector_drag_session_->axis == axis &&
          transform_inspector_drag_session_->targets == targets;
      if (changed) {
        if (relative_drag) {
          const auto& session = *transform_inspector_drag_session_;
          edited |= EntityBatchInspector::WriteRelativeLocalTransformField(
              scene, targets, session.original_transforms, field, axis, session.start_value, value.value[axis]);
        } else {
          edited |= EntityBatchInspector::WriteLocalTransformField(scene, targets, field, value.value, axis);
        }
      }
      if (ImGui::IsItemDeactivated())
        transform_inspector_drag_session_.reset();
      if (value.mixed_axes[axis] && ImGui::IsItemHovered())
        ImGui::SetTooltip("Mixed value");
      if (value.mixed_axes[axis])
        ImGui::PopItemFlag();
      ImGui::PopID();
      if (axis != 2)
        ImGui::SameLine(0.0f, spacing);
    }
    ImGui::PopID();
  };

  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 8.0f));
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4.0f, 4.0f));
  if (ImGui::BeginTable("TransformComponent", 2,
                        ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_NoClip)) {
    ImGui::TableSetupColumn("Labels", ImGuiTableColumnFlags_WidthFixed, 100.0f);
    ImGui::TableSetupColumn("Values", ImGuiTableColumnFlags_WidthStretch);
    draw_field("Translation", 0, EntityBatchInspector::ReadLocalPosition(scene, targets), 0.0f, 0.01f);
    draw_field("Rotation", 1, EntityBatchInspector::ReadLocalRotationDegrees(scene, targets), 0.0f, 1.0f);
    draw_field("Scale", 2, EntityBatchInspector::ReadLocalScale(scene, targets), 1.0f, 0.01f);
    ImGui::EndTable();
  }
  ImGui::PopStyleVar(2);
  if (edited)
    NotifyStaticEntitiesChanged(scene, targets);
  return edited;
}

void EditorLayer::NotifyStaticEntitiesChanged(const std::shared_ptr<Scene>& scene,
                                              const std::vector<Entity>& entities) const {
  if (!scene)
    return;
  if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>()) {
    for (const auto entity : entities)
      render_layer->NotifyStaticEntityChanged(scene, entity);
  }
}

void EditorLayer::DrawBatchEntityInspector(const std::shared_ptr<Scene>& scene,
                                           const std::shared_ptr<EditorLayer>& editor_layer) {
  const auto snapshot = entity_selection_.GetSnapshot();
  const auto context = EntityBatchInspector::BuildContext(scene, snapshot.entities, snapshot.primary);
  if (context.targets.size() < 2)
    return;

  auto draw_mixed_checkbox = [&](const char* label, const auto read, const auto write) {
    bool value = read(context.targets.front());
    const bool mixed = std::any_of(context.targets.begin() + 1, context.targets.end(), [&](const Entity entity) {
      return read(entity) != value;
    });
    if (mixed)
      ImGui::PushItemFlag(ImGuiItemFlags_MixedValue, true);
    const bool changed = ImGui::Checkbox(label, &value);
    if (changed)
      for (const auto entity : context.targets)
        write(entity, value);
    if (mixed)
      ImGui::PopItemFlag();
    return changed;
  };
  if (draw_mixed_checkbox(
          "##EnabledCheckbox",
          [&](const Entity entity) {
            return scene->IsEntityEnabled(entity);
          },
          [&](const Entity entity, const bool value) {
            scene->SetEnableSingle(entity, value);
          }))
    scene->SetUnsaved();
  ImGui::SameLine();
  ImGui::Text("Editing %zu entities", context.targets.size());
  ImGui::SameLine();
  draw_mixed_checkbox(
      "Static##StaticCheckbox",
      [&](const Entity entity) {
        return scene->IsEntityStatic(entity);
      },
      [&](const Entity entity, const bool value) {
        scene->SetEntityStatic(entity, value);
      });
  ImGui::SameLine();
  bool selection_locked = GetLockEntitySelection();
  if (ImGui::Checkbox("Lock", &selection_locked))
    SetLockEntitySelection(selection_locked);
  ImGui::SameLine();
  const bool selection_cleared = ImGui::Button("Clear");
  if (selection_cleared)
    SetSelectedEntity({});

  if (!selection_cleared)
    DrawEntityComponentInspectors(scene, editor_layer, context);
}

void EditorLayer::DrawEntityComponentInspectors(const std::shared_ptr<Scene>& scene,
                                                const std::shared_ptr<EditorLayer>& editor_layer,
                                                const EntityBatchInspectionContext& context) {
  if (context.targets.empty())
    return;
  const bool multiple = context.targets.size() > 1;
  const auto generic_component_icon = FindIconInMap(editor_icons_, "ComponentGeneric");
  const auto component_settings_icon = FindIconInMap(editor_icons_, "SceneSettings");
  const auto data_components_icon = FindIconInMap(editor_icons_, "DataComponents");
  const bool light_theme = UsesLightTheme();
  const ImU32 data_section_color = light_theme ? IM_COL32(216, 233, 247, 255) : IM_COL32(58, 110, 154, 255);
  const ImU32 data_component_color = light_theme ? IM_COL32(190, 219, 242, 255) : IM_COL32(38, 82, 120, 255);
  const ImU32 private_section_color = light_theme ? IM_COL32(236, 218, 198, 255) : IM_COL32(138, 103, 72, 255);
  const ImU32 private_component_color = light_theme ? IM_COL32(222, 198, 174, 255) : IM_COL32(112, 83, 58, 255);
  const auto find_component_icon = [&](const size_t type_index, const std::shared_ptr<Texture2D>& fallback) {
    if (const auto search = component_icon_map_.find(type_index); search != component_icon_map_.end())
      return search->second;
    return fallback;
  };

  ImGui::PushID("DataComponentsSection");
  const auto data_components_header = DrawComponentHeader("Data Components", data_components_icon,
                                                          component_settings_icon, nullptr, false, data_section_color);
  if (ImGui::BeginPopup("ComponentSettings")) {
    if (ImGui::BeginMenu("Add Component")) {
      bool has_available_component = false;
      for (const auto& [name, id] : Serialization::GetInstance().data_component_ids_) {
        if (id == typeid(Transform).hash_code() || id == typeid(GlobalTransform).hash_code() ||
            id == typeid(TransformUpdateFlag).hash_code() || id == typeid(UnknownDataComponent).hash_code())
          continue;
        const auto missing = std::count_if(context.targets.begin(), context.targets.end(), [&](const Entity entity) {
          return !scene->HasDataComponent(entity, id);
        });
        if (missing == 0)
          continue;
        has_available_component = true;
        const auto label =
            multiple ? name + " - Add to " + std::to_string(missing) + "/" + std::to_string(context.targets.size())
                     : name;
        if (ImGui::MenuItem(label.c_str())) {
          for (const auto entity : context.targets)
            if (!scene->HasDataComponent(entity, id))
              scene->AddDataComponent(entity, id);
        }
      }
      if (!has_available_component)
        ImGui::MenuItem("No data components available", nullptr, false, false);
      ImGui::EndMenu();
    }
    ImGui::EndPopup();
  }
  if (data_components_header.open) {
    if (context.hidden_data_component_types > 0)
      ImGui::TextDisabled("%zu non-common data component type(s) hidden", context.hidden_data_component_types);
    for (const auto& component : context.common_data_components) {
      ImGui::PushID("DataComponent");
      ImGui::PushID(static_cast<int>(component.type_index));
      const auto header =
          DrawComponentHeader(component.type_name, find_component_icon(component.type_index, data_components_icon),
                              component_settings_icon, nullptr, false, data_component_color);
      bool removed = false;
      if (ImGui::BeginPopup("ComponentSettings")) {
        const bool protected_component = component.type_index == typeid(Transform).hash_code();
        ImGui::BeginDisabled(protected_component);
        const auto remove_label =
            multiple ? "Remove from " + std::to_string(context.targets.size()) + " entities" : "Remove component";
        if (ImGui::MenuItem(remove_label.c_str())) {
          for (const auto entity : context.targets)
            scene->RemoveDataComponent(entity, component.type_index);
          removed = true;
        }
        ImGui::EndDisabled();
        if (protected_component && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
          ImGui::SetTooltip("Transform cannot be removed");
        ImGui::EndPopup();
      }
      if (header.open) {
        ImGui::Indent(8.0f);
        ImGui::Spacing();
        const auto single_adapter = component_data_inspector_map_.find(component.type_index);
        const auto batch_adapter = component_data_batch_inspector_map_.find(component.type_index);
        const auto dispatch = EntityBatchInspector::ResolveInspectorDispatch(
            context.targets.size(), single_adapter != component_data_inspector_map_.end(),
            batch_adapter != component_data_batch_inspector_map_.end(),
            component.type_index == typeid(Transform).hash_code());
        if (!removed && dispatch == EntityInspectorDispatch::Single) {
          const DataComponentType type(component.type_name, component.type_index, component.type_size);
          InspectComponentData(context.targets.front(),
                               static_cast<IDataComponent*>(
                                   scene->GetDataComponentPointer(context.targets.front(), component.type_index)),
                               type, scene->GetParent(context.targets.front()).GetIndex() != 0);
        } else if (!removed && dispatch == EntityInspectorDispatch::Batch) {
          if (batch_adapter->second(scene, context.targets)) {
            scene->SetUnsaved();
            NotifyStaticEntitiesChanged(scene, context.targets);
          }
        } else if (!removed) {
          ImGui::TextDisabled(multiple ? "Multi-object editing not supported" : "Component inspection not supported");
        }
        ImGui::Spacing();
        ImGui::Unindent(8.0f);
      }
      ImGui::PopID();
      ImGui::PopID();
    }
  }
  ImGui::PopID();

  ImGui::PushID("PrivateComponentsSection");
  const auto private_components_header = DrawComponentHeader(
      "Private Components", generic_component_icon, component_settings_icon, nullptr, false, private_section_color);
  if (ImGui::BeginPopup("ComponentSettings")) {
    if (ImGui::BeginMenu("Add Component")) {
      bool has_available_component = false;
      for (const auto& [name, id] : Serialization::GetInstance().private_component_ids_) {
        if (id == typeid(UnknownPrivateComponent).hash_code())
          continue;
        const auto missing = std::count_if(context.targets.begin(), context.targets.end(), [&](const Entity entity) {
          return !scene->HasPrivateComponent(entity, id);
        });
        if (missing == 0)
          continue;
        has_available_component = true;
        const auto label =
            multiple ? name + " - Add to " + std::to_string(missing) + "/" + std::to_string(context.targets.size())
                     : name;
        if (ImGui::MenuItem(label.c_str())) {
          for (const auto entity : context.targets)
            if (!scene->HasPrivateComponent(entity, id))
              scene->AddPrivateComponent(entity, id);
        }
      }
      if (!has_available_component)
        ImGui::MenuItem("No private components available", nullptr, false, false);
      ImGui::EndMenu();
    }
    ImGui::EndPopup();
  }
  if (private_components_header.open) {
    if (context.hidden_private_component_types > 0)
      ImGui::TextDisabled("%zu non-common private component type(s) hidden", context.hidden_private_component_types);
    for (const auto& component : context.common_private_components) {
      std::vector<std::shared_ptr<IPrivateComponent>> instances;
      for (const auto entity : context.targets)
        scene->ForEachPrivateComponent(entity, [&](PrivateComponentElement& element) {
          if (element.type_index == component.type_index)
            instances.emplace_back(element.private_component_data);
        });
      if (instances.size() != context.targets.size())
        continue;
      ImGui::PushID("PrivateComponent");
      ImGui::PushID(static_cast<int>(component.type_index));
      bool enabled = instances.front()->IsEnabled();
      const bool mixed = std::any_of(instances.begin() + 1, instances.end(), [&](const auto& instance) {
        return instance->IsEnabled() != enabled;
      });
      const auto header =
          DrawComponentHeader(component.type_name, find_component_icon(component.type_index, generic_component_icon),
                              component_settings_icon, &enabled, mixed, private_component_color);
      if (header.enabled_changed) {
        for (const auto& instance : instances)
          instance->SetEnabled(enabled);
        scene->SetUnsaved();
        NotifyStaticEntitiesChanged(scene, context.targets);
      }
      bool removed = false;
      if (ImGui::BeginPopup("ComponentSettings")) {
        const auto remove_label =
            multiple ? "Remove from " + std::to_string(context.targets.size()) + " entities" : "Remove component";
        if (ImGui::MenuItem(remove_label.c_str())) {
          for (const auto entity : context.targets)
            scene->RemovePrivateComponent(entity, component.type_index);
          removed = true;
        }
        ImGui::EndPopup();
      }
      if (header.open) {
        ImGui::Indent(8.0f);
        ImGui::Spacing();
        if (!removed && !multiple)
          DraggablePrivateComponent(instances.front());
        const auto& registry = InspectorRegistry::GetInstance();
        const bool single_available = registry.FindInspector(typeid(*instances.front())) != nullptr;
        const bool batch_available = registry.FindBatchInspector(typeid(*instances.front())) != nullptr;
        const auto dispatch =
            EntityBatchInspector::ResolveInspectorDispatch(context.targets.size(), single_available, batch_available);
        if (!removed && dispatch == EntityInspectorDispatch::Single) {
          InspectorContext inspector_context{editor_layer, scene};
          if (registry.Inspect(inspector_context, *instances.front())) {
            scene->SetUnsaved();
            NotifyStaticEntitiesChanged(scene, context.targets);
          }
        } else if (!removed && dispatch == EntityInspectorDispatch::Batch) {
          InspectorContext inspector_context{editor_layer, scene};
          if (registry.InspectBatch(inspector_context, instances)) {
            scene->SetUnsaved();
            NotifyStaticEntitiesChanged(scene, context.targets);
          }
        } else if (!removed) {
          ImGui::TextDisabled(multiple ? "Multi-object editing not supported" : "Component inspection not supported");
        }
        ImGui::Spacing();
        ImGui::Unindent(8.0f);
      }
      ImGui::PopID();
      ImGui::PopID();
    }
  }
  ImGui::PopID();
}

const std::vector<Entity>& EditorLayer::ResolveSelectionGizmoParticipants(const std::shared_ptr<Scene>& scene,
                                                                          const EntitySelection::Snapshot& selection) {
  const uint64_t hierarchy_revision = scene ? scene->GetHierarchyRevision() : 0;
  if (selection_gizmo_cache_scene_.lock() == scene && selection_gizmo_cache_selection_revision_ == selection.revision &&
      selection_gizmo_cache_hierarchy_revision_ == hierarchy_revision) {
    return selection_gizmo_participants_;
  }

  const auto participants_started = std::chrono::steady_clock::now();
  {
    const ProfilerScope profiler_scope("EditorLayer::ResolveSelectionGizmoParticipants", "Editor");
    selection_gizmo_participants_ = EntityBatchInspector::BuildGizmoParticipants(scene, selection.entities);
  }
  if (Platform::Initialized() && !selection.entities.empty()) {
    Platform::RecordCpuTimingSample(
        "Editor Selection / Resolve Gizmo Participants",
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - participants_started).count());
  }

  const auto bound_started = std::chrono::steady_clock::now();
  EntityBatchSelectionBound fallback_bound;
  {
    const ProfilerScope profiler_scope("EditorLayer::ResolveSelectionGizmoBoundFallback", "Editor");
    fallback_bound = EntityBatchInspector::BuildSelectionWorldBound(scene, selection_gizmo_participants_);
  }
  if (Platform::Initialized() && !selection.entities.empty()) {
    Platform::RecordCpuTimingSample(
        "Editor Selection / Resolve Gizmo Bound Fallback",
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - bound_started).count());
  }
  selection_gizmo_cache_scene_ = scene;
  selection_gizmo_cache_selection_revision_ = selection.revision;
  selection_gizmo_cache_hierarchy_revision_ = hierarchy_revision;
  selection_gizmo_fallback_bound_ = fallback_bound.world_bound;
  selection_gizmo_fallback_has_renderable_bounds_ = fallback_bound.has_renderable_bounds;
  selection_gizmo_fallback_valid_ = fallback_bound.valid;
  return selection_gizmo_participants_;
}

EntityBatchSelectionBound EditorLayer::ResolveSelectionGizmoBound(const std::shared_ptr<Scene>& scene,
                                                                  const EntitySelection::Snapshot& selection) {
  ResolveSelectionGizmoParticipants(scene, selection);
  if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>()) {
    if (const auto render_instances = render_layer->GetPreviousRenderInstanceStorage()) {
      const auto& snapshot = render_instances->GetEntitySelectionRenderSnapshot();
      if (snapshot.Matches(scene, selection.revision, selection_gizmo_cache_hierarchy_revision_) &&
          snapshot.has_renderable_bounds) {
        return {snapshot.world_bound, true, true};
      }
    }
  }
  return {selection_gizmo_fallback_bound_, selection_gizmo_fallback_has_renderable_bounds_,
          selection_gizmo_fallback_valid_};
}

bool EditorLayer::BeginEntityGizmoSession(const std::shared_ptr<Scene>& scene, const glm::mat4& handle,
                                          const std::vector<Entity>& participants, const Entity reference,
                                          const int operation) {
  if (!scene || participants.empty() || !scene->IsEntityValid(reference))
    return false;
  EntityGizmoSession session;
  session.scene = scene;
  session.selection_revision = entity_selection_.GetRevision();
  session.application_status = static_cast<int>(ApplicationContext::Get().GetApplicationStatus());
  session.operation = operation;
  session.pivot_mode = entity_gizmo_pivot_mode_;
  session.orientation_mode = entity_gizmo_orientation_mode_;
  session.handle = handle;
  session.manipulated_handle = handle;
  Transform handle_transform;
  handle_transform.value = handle;
  glm::vec3 handle_scale;
  if (!handle_transform.Decompose(session.pivot, session.handle_rotation, handle_scale))
    return false;
  for (const auto entity : participants) {
    if (!scene->IsEntityValid(entity) || !scene->HasDataComponent<Transform>(entity) ||
        !scene->HasDataComponent<GlobalTransform>(entity))
      return false;
    EntityGizmoParticipantState state;
    state.entity = entity;
    state.parent = scene->GetParent(entity);
    state.local = scene->GetDataComponent<Transform>(entity).value;
    state.world = scene->GetDataComponent<GlobalTransform>(entity).value;
    if (scene->IsEntityValid(state.parent))
      state.parent_world = scene->GetDataComponent<GlobalTransform>(state.parent).value;
    session.participants.emplace_back(state);
  }
  entity_gizmo_session_ = std::move(session);
  entity_gizmo_message_.clear();
  return true;
}

bool EditorLayer::ApplyEntityGizmoSession(const std::shared_ptr<Scene>& scene, const glm::mat4& manipulated_handle) {
  if (!entity_gizmo_session_)
    return false;
  auto& session = *entity_gizmo_session_;
  if (session.scene.lock() != scene || session.selection_revision != entity_selection_.GetRevision() ||
      session.application_status != static_cast<int>(ApplicationContext::Get().GetApplicationStatus())) {
    CancelEntityGizmoSession("Entity gizmo drag cancelled because editor state changed.");
    return false;
  }
  session.manipulated_handle = manipulated_handle;

  std::vector<Transform> local_results;
  local_results.reserve(session.participants.size());

  for (const auto& state : session.participants) {
    if (!scene->IsEntityValid(state.entity) || scene->GetParent(state.entity) != state.parent ||
        (scene->IsEntityValid(state.parent) &&
         !MatricesNear(scene->GetDataComponent<GlobalTransform>(state.parent).value, state.parent_world))) {
      CancelEntityGizmoSession("Entity gizmo drag cancelled because the hierarchy changed.");
      return false;
    }
    const auto operation =
        session.operation == static_cast<int>(ImGuizmo::OPERATION::TRANSLATE) ? EntityBatchGizmoOperation::Translate
        : session.operation == static_cast<int>(ImGuizmo::OPERATION::ROTATE)  ? EntityBatchGizmoOperation::Rotate
                                                                              : EntityBatchGizmoOperation::Scale;
    glm::mat4 candidate_world(1.0f);
    if (!EntityBatchInspector::TryApplyGizmoTransform(
            session.handle, manipulated_handle, state.world, operation,
            session.pivot_mode == EntityGizmoPivotMode::Pivot ? EntityBatchGizmoPivot::Pivot
                                                              : EntityBatchGizmoPivot::Center,
            session.orientation_mode == EntityGizmoOrientationMode::Local ? EntityBatchGizmoOrientation::Local
                                                                          : EntityBatchGizmoOrientation::Global,
            candidate_world)) {
      entity_gizmo_message_ = "Transform cannot be represented.";
      return false;
    }

    const auto candidate_local = glm::inverse(state.parent_world) * candidate_world;
    glm::mat4 normalized(1.0f);
    glm::vec3 local_position, local_rotation, local_scale;
    if (!TryNormalizeAuthoringTransform(candidate_local, normalized, local_position, local_rotation, local_scale) ||
        !MatricesNear(candidate_local, normalized)) {
      entity_gizmo_message_ = "Group transform would create shear or a singular transform.";
      return false;
    }
    Transform result;
    result.value = normalized;
    local_results.emplace_back(result);
  }
  for (size_t i = 0; i < session.participants.size(); ++i)
    scene->SetDataComponent(session.participants[i].entity, local_results[i]);
  std::vector<Entity> changed_entities;
  changed_entities.reserve(session.participants.size());
  for (const auto& participant : session.participants)
    changed_entities.emplace_back(participant.entity);
  NotifyStaticEntitiesChanged(scene, changed_entities);
  entity_gizmo_message_.clear();
  return true;
}

void EditorLayer::CancelEntityGizmoSession(const char* reason) {
  if (reason && entity_gizmo_session_)
    EVOENGINE_LOG(reason);
  entity_gizmo_session_.reset();
  entity_gizmo_message_.clear();
}

void EditorLayer::ClearConsoleMessages() {
  uint64_t revision = 0;
  {
    std::lock_guard lock(console_message_mutex_);
    console_messages_.clear();
    revision = ++console_message_revision_;
  }
  console_rendered_revision_ = revision;
  console_detailed_message_.reset();
  console_previous_scroll_y_ = 0.0f;
  console_auto_scroll_ = true;
}

void EditorLayer::ClearConsoleOnRuntimeStart() {
  if (console_clear_on_play_)
    ClearConsoleMessages();
}

void EditorLayer::DrawConsoleWindow() {
  if (!show_console_window)
    return;

  bool open_details = false;
  if (ImGui::Begin("Console")) {
    constexpr float toolbar_height = 28.0f;
    constexpr float filter_width = toolbar_height * 3.0f + 16.0f;
    bool filter_changed = false;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
    if (ImGui::BeginChild("ConsoleToolbar", ImVec2(ImGui::GetContentRegionAvail().x, toolbar_height), false,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) {
      if (ImGui::Button("Clear", ImVec2(75.0f, toolbar_height)))
        ClearConsoleMessages();
      ImGui::SameLine();
      const auto& style = ImGui::GetStyle();
      ImGui::PushStyleColor(ImGuiCol_Button,
                            console_clear_on_play_ ? style.Colors[ImGuiCol_Header] : style.Colors[ImGuiCol_FrameBg]);
      if (ImGui::Button("Clear on Play", ImVec2(110.0f, toolbar_height)))
        console_clear_on_play_ = !console_clear_on_play_;
      ImGui::PopStyleColor();

      ImGui::SameLine(std::max(ImGui::GetCursorPosX(), ImGui::GetWindowContentRegionMax().x - filter_width));
      if (DrawConsoleFilterButton("InfoFilter", FindIconInMap(editor_icons_, "InfoButton"), "Info",
                                  enable_console_logs_, ConsoleMessageColor(ConsoleMessageType::Log))) {
        enable_console_logs_ = !enable_console_logs_;
        filter_changed = true;
      }
      ImGui::SameLine();
      if (DrawConsoleFilterButton("WarningFilter", FindIconInMap(editor_icons_, "WarningButton"), "Warning",
                                  enable_console_warnings_, ConsoleMessageColor(ConsoleMessageType::Warning))) {
        enable_console_warnings_ = !enable_console_warnings_;
        filter_changed = true;
      }
      ImGui::SameLine();
      if (DrawConsoleFilterButton("ErrorFilter", FindIconInMap(editor_icons_, "ErrorButton"), "Error",
                                  enable_console_errors_, ConsoleMessageColor(ConsoleMessageType::Error))) {
        enable_console_errors_ = !enable_console_errors_;
        filter_changed = true;
      }
    }
    ImGui::EndChild();
    ImGui::PopStyleVar(2);

    std::vector<ConsoleMessage> messages;
    uint64_t revision = 0;
    {
      std::lock_guard lock(console_message_mutex_);
      messages = console_messages_;
      revision = console_message_revision_;
    }

    constexpr ImGuiTableFlags table_flags = ImGuiTableFlags_NoPadInnerX | ImGuiTableFlags_Resizable |
                                            ImGuiTableFlags_Reorderable | ImGuiTableFlags_ScrollY |
                                            ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV;
    ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(4.0f, 0.0f));
    if (ImGui::BeginTable("ConsoleMessages", 3, table_flags, ImGui::GetContentRegionAvail())) {
      ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 100.0f);
      ImGui::TableSetupColumn("Timestamp", ImGuiTableColumnFlags_WidthFixed, 90.0f);
      ImGui::TableSetupColumn("Message", ImGuiTableColumnFlags_WidthStretch);
      ImGui::TableSetupScrollFreeze(0, 1);
      ImGui::TableHeadersRow();

      const float scroll_y = ImGui::GetScrollY();
      const float max_scroll_y = ImGui::GetScrollMaxY();
      if (scroll_y < console_previous_scroll_y_ - 0.5f)
        console_auto_scroll_ = false;
      if (scroll_y >= max_scroll_y - 1.0f)
        console_auto_scroll_ = true;
      const bool scroll_to_latest = console_auto_scroll_ && (revision != console_rendered_revision_ || filter_changed);
      console_rendered_revision_ = revision;

      constexpr float row_height = 24.0f;
      for (const auto& message : messages) {
        const bool visible = (message.m_type == ConsoleMessageType::Log && enable_console_logs_) ||
                             (message.m_type == ConsoleMessageType::Warning && enable_console_warnings_) ||
                             (message.m_type == ConsoleMessageType::Error && enable_console_errors_);
        if (!visible)
          continue;

        ImGui::PushID(&message);
        const bool clicked = BeginConsoleTableRow(&message, row_height);
        const ImVec4 severity_color = ConsoleMessageColor(message.m_type);
        const ImRect type_cell = ImGui::TableGetCellBgRect(ImGui::GetCurrentTable(), 0);
        ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(type_cell.Min.x + 4.0f, type_cell.Min.y + 4.0f),
                                                  ImVec2(type_cell.Min.x + 8.0f, type_cell.Max.y - 4.0f),
                                                  ImGui::ColorConvertFloat4ToU32(severity_color), 2.0f);
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 12.0f);
        ImGui::TextUnformatted(ConsoleMessageTypeName(message.m_type));

        ImGui::TableSetColumnIndex(1);
        const std::string timestamp = FormatConsoleTimestamp(message.m_timestamp);
        ImGui::TextColored(ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled), "%s", timestamp.c_str());

        ImGui::TableSetColumnIndex(2);
        const std::string preview = ConsoleMessagePreview(message.m_value);
        ImGui::TextUnformatted(preview.c_str());
        if (clicked) {
          console_detailed_message_ = message;
          open_details = true;
        }
        ImGui::PopID();
      }
      if (scroll_to_latest)
        ImGui::SetScrollY(ImGui::GetScrollMaxY());
      console_previous_scroll_y_ = ImGui::GetScrollY();
      ImGui::EndTable();
    }
    ImGui::PopStyleVar();
  }
  ImGui::End();

  if (open_details)
    ImGui::OpenPopup("Console Message Details");
  if (console_detailed_message_) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x * 0.5f, viewport->WorkSize.y * 0.5f), ImGuiCond_Appearing);
  }
  bool details_open = true;
  if (ImGui::BeginPopupModal("Console Message Details", &details_open,
                             ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize)) {
    if (!console_detailed_message_) {
      ImGui::CloseCurrentPopup();
    } else {
      const auto& message = *console_detailed_message_;
      ImGui::TextColored(ConsoleMessageColor(message.m_type), "%s", ConsoleMessageTypeName(message.m_type));
      ImGui::SameLine();
      const std::string timestamp = FormatConsoleTimestamp(message.m_timestamp);
      ImGui::TextColored(ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled), "%s", timestamp.c_str());
      ImGui::Separator();
      if (ImGui::BeginChild("ConsoleMessageText", ImVec2(0.0f, -36.0f), true)) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextUnformatted(message.m_value.c_str());
        ImGui::PopTextWrapPos();
      }
      ImGui::EndChild();
      if (ImGui::Button("Copy to Clipboard", ImVec2(130.0f, 28.0f)))
        ImGui::SetClipboardText(message.m_value.c_str());
      ImGui::SameLine();
      if (ImGui::Button("Close", ImVec2(75.0f, 28.0f))) {
        ImGui::CloseCurrentPopup();
        details_open = false;
      }
    }
    ImGui::EndPopup();
  }
  if (!details_open)
    console_detailed_message_.reset();
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
  if (custom_layout_settings_ && custom_layout_settings_->runtime_package_manager &&
      runtime_package_manager_layout_pending_) {
    const auto& window_layout = custom_layout_settings_->runtime_package_manager->floating_window;
    ImGui::SetNextWindowPos(ResolveFloatingWindowPosition(window_layout), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(window_layout.size.x, window_layout.size.y), ImGuiCond_Always);
    runtime_package_manager_layout_pending_ = false;
  }
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
    const auto package_list_fraction = custom_layout_settings_ && custom_layout_settings_->runtime_package_manager
                                           ? custom_layout_settings_->runtime_package_manager->list_width_fraction
                                           : 0.36f;
    const auto package_list_min = custom_layout_settings_ && custom_layout_settings_->runtime_package_manager
                                      ? custom_layout_settings_->runtime_package_manager->list_width_min
                                      : 260.0f;
    const auto package_list_max = custom_layout_settings_ && custom_layout_settings_->runtime_package_manager
                                      ? custom_layout_settings_->runtime_package_manager->list_width_max
                                      : 420.0f;
    const float package_list_width =
        std::min(package_list_max, std::max(package_list_min, content_region.x * package_list_fraction));
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

void EditorLayer::DrawProfilerWindow() {
  if (!show_profiler_window) {
    return;
  }

  auto& profiler = Profiler::GetInstance();
  if (!profiler_panel_state_)
    profiler_panel_state_ = std::make_shared<ProfilerPanelState>();
  auto& panel_state = *profiler_panel_state_;
  bool profiler_open = show_profiler_window;
  if (!ImGui::Begin("Profiler", &profiler_open)) {
    show_profiler_window = profiler_open;
    ImGui::End();
    return;
  }
  show_profiler_window = profiler_open;

  bool capture_enabled = profiler.IsEnabled();
  if (ImGui::Checkbox("Capture", &capture_enabled)) {
    profiler.SetEnabled(capture_enabled);
    Platform::SetGpuTimestampCaptureEnabled(capture_enabled);
  }
  ImGui::SameLine();
  if (ImGui::Button(profiler_panel_paused_ ? "Resume" : "Pause")) {
    profiler_panel_paused_ = !profiler_panel_paused_;
  }
  ImGui::Checkbox("Pause on next frame", &profiler_pause_on_next_frame_);
  ImGui::SameLine();
  if (ImGui::Button("Clear")) {
    profiler.ClearFrameHistory();
    Platform::ResetGpuTimestampStats();
    profiler_panel_frames_.clear();
    panel_state = {};
    profiler_selected_frame_index_ = -1;
    profiler_cached_latest_frame_index_ = 0;
  }
  if (capture_enabled) {
    if (Platform::GpuTimestampCaptureAvailable()) {
      ImGui::TextUnformatted("CPU + GPU capture active");
    } else {
      ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.25f, 1.0f), "CPU capture active; GPU timestamps unavailable");
    }
  } else {
    ImGui::TextUnformatted("Capture stopped; history frozen");
  }

  if (ImGui::SliderInt("History", &profiler_history_length_, 30, 2000)) {
    profiler.SetMaxFrameHistory(static_cast<size_t>(profiler_history_length_));
  }

  if (!profiler_panel_paused_) {
    auto frames = profiler.GetFrameStatsHistorySnapshot();
    panel_state.gpu_frames = Platform::GetGpuTimestampFrameHistory();
    if (!frames.empty()) {
      const auto latest_frame_index = frames.back().frame_index;
      profiler_panel_frames_ = std::move(frames);
      profiler_selected_frame_index_ = static_cast<int>(profiler_panel_frames_.size()) - 1;
      std::unordered_set<std::string> resolved_gpu_frames;
      for (const auto& frame : panel_state.gpu_frames) {
        resolved_gpu_frames.emplace(GpuProfilerFrameKey(frame.capture_session_index, frame.application_frame_index));
      }
      for (int i = profiler_selected_frame_index_; i >= 0; --i) {
        const auto& frame = profiler_panel_frames_[i];
        if (resolved_gpu_frames.find(GpuProfilerFrameKey(frame.capture_session_index, frame.application_frame_index)) !=
            resolved_gpu_frames.end()) {
          profiler_selected_frame_index_ = i;
          break;
        }
      }
      if (profiler_pause_on_next_frame_ && latest_frame_index != profiler_cached_latest_frame_index_) {
        profiler_panel_paused_ = true;
        profiler_pause_on_next_frame_ = false;
      }
      profiler_cached_latest_frame_index_ = latest_frame_index;
    }
  }

  UpdateProfilerCpuCatalog(profiler_panel_frames_, panel_state);
  UpdateGpuProfilerCatalog(panel_state.gpu_frames, panel_state);

  if (profiler_panel_frames_.empty()) {
    ImGui::TextUnformatted(capture_enabled ? "Waiting for the first complete profiler frame."
                                           : "No profiler frames captured. Enable Capture to begin.");
    ImGui::End();
    return;
  }

  profiler_selected_frame_index_ =
      std::clamp(profiler_selected_frame_index_, 0, static_cast<int>(profiler_panel_frames_.size()) - 1);
  const auto& selected_frame = profiler_panel_frames_[profiler_selected_frame_index_];
  if (!ImGui::BeginTabBar("ProfilerTabs")) {
    ImGui::End();
    return;
  }

  if (ImGui::BeginTabItem("Overview")) {
    const auto overview_samples = BuildProfilerOverviewSamples(profiler_panel_frames_, panel_state.gpu_frames);
    const int previous_selected_frame = profiler_selected_frame_index_;
    ImGui::TextUnformatted("Frame time: CPU active + synchronization; GPU and Total are overlapping wall-time lines");
    DrawProfilerOverviewPlot("ProfilerOverviewPlot", overview_samples, profiler_panel_frames_,
                             profiler_selected_frame_index_);
    if (profiler_selected_frame_index_ != previous_selected_frame)
      profiler_panel_paused_ = true;

    profiler_panel_detail::FrameOverviewSample average;
    size_t paired_frame_count = 0;
    for (const auto& sample : overview_samples) {
      if (!sample.gpu_available)
        continue;
      average.cpu_active_ms += sample.cpu_active_ms;
      average.synchronization_ms += sample.synchronization_ms;
      average.gpu_ms += sample.gpu_ms;
      average.total_ms += sample.total_ms;
      ++paired_frame_count;
    }
    if (paired_frame_count > 0) {
      const double divisor = static_cast<double>(paired_frame_count);
      average.cpu_active_ms /= divisor;
      average.synchronization_ms /= divisor;
      average.gpu_ms /= divisor;
      average.total_ms /= divisor;
    }
    if (ImGui::BeginTable("ProfilerOverviewAverage", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders)) {
      ImGui::TableSetupColumn("History row", ImGuiTableColumnFlags_WidthStretch);
      ImGui::TableSetupColumn("CPU ms");
      ImGui::TableSetupColumn("Sync ms");
      ImGui::TableSetupColumn("GPU ms");
      ImGui::TableSetupColumn("Total ms");
      ImGui::TableHeadersRow();
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      ImGui::Text("Average (%zu paired frames)", paired_frame_count);
      ImGui::TableNextColumn();
      paired_frame_count ? ImGui::Text("%.3f", average.cpu_active_ms) : ImGui::TextUnformatted("-");
      ImGui::TableNextColumn();
      paired_frame_count ? ImGui::Text("%.3f", average.synchronization_ms) : ImGui::TextUnformatted("-");
      ImGui::TableNextColumn();
      paired_frame_count ? ImGui::Text("%.3f", average.gpu_ms) : ImGui::TextUnformatted("-");
      ImGui::TableNextColumn();
      paired_frame_count ? ImGui::Text("%.3f", average.total_ms) : ImGui::TextUnformatted("-");
      ImGui::EndTable();
    }
    const size_t pending_gpu_frames = overview_samples.size() - paired_frame_count;
    if (!Platform::GpuTimestampCaptureAvailable())
      ImGui::TextDisabled("GPU timestamps unavailable; paired averages require resolved GPU frames.");
    else if (pending_gpu_frames > 0)
      ImGui::TextDisabled("%zu CPU frames are pending or do not have aligned GPU results.", pending_gpu_frames);
    ImGui::EndTabItem();
  }

  if (ImGui::BeginTabItem("Breakdown")) {
    const auto selected_index = static_cast<size_t>(profiler_selected_frame_index_);
    const auto now = std::chrono::steady_clock::now();
    if (panel_state.breakdown_selected_frame_index != profiler_selected_frame_index_ ||
        now >= panel_state.breakdown_next_refresh) {
      panel_state.breakdown_cpu_history = BuildProfilerHistoryStats(profiler_panel_frames_, selected_index);
      panel_state.breakdown_aligned_gpu_frames = AlignGpuProfilerFrames(profiler_panel_frames_, panel_state.gpu_frames);
      panel_state.breakdown_gpu_history = BuildGpuTimestampHistoryStats(panel_state.breakdown_aligned_gpu_frames,
                                                                        profiler_panel_frames_.size(), selected_index);
      panel_state.breakdown_cpu_series = BuildCpuBreakdownSeries(profiler_panel_frames_);
      panel_state.breakdown_gpu_series = BuildGpuGroupBreakdownSeries(panel_state.breakdown_aligned_gpu_frames);
      panel_state.breakdown_selected_frame_index = profiler_selected_frame_index_;
      panel_state.breakdown_next_refresh = now + std::chrono::milliseconds(500);
    }
    const auto& cpu_history = panel_state.breakdown_cpu_history;
    const auto& gpu_history = panel_state.breakdown_gpu_history;
    const auto counter_value = [&](const char* name) {
      const auto* counter = FindProfilerCounter(cpu_history, name);
      return counter ? counter->selected : 0.0;
    };
    const double rolling_fps =
        cpu_history.frame_duration.average_ms > 0.0 ? 1000.0 / cpu_history.frame_duration.average_ms : 0.0;
    ImGui::Text("Selected %.3f ms (%.1f FPS)  |  Rolling %.3f ms (%.1f FPS)", cpu_history.frame_duration.selected_ms,
                cpu_history.frame_duration.selected_ms > 0.0 ? 1000.0 / cpu_history.frame_duration.selected_ms : 0.0,
                cpu_history.frame_duration.average_ms, rolling_fps);
    ImGui::Text("GPU span %.3f ms  |  summed GPU work %.3f ms  |  %.0fx%.0f  |  %.0f cameras  |  %.0f instances",
                gpu_history.span.selected_milliseconds, gpu_history.summed_work.selected_milliseconds,
                counter_value("Scene Camera Width"), counter_value("Scene Camera Height"),
                counter_value("Camera Count"), counter_value("Canonical Instances"));
    ImGui::Text("Shadow views %.0f  |  DDGI active %.0f  updated %.0f  converged %s",
                counter_value("Shadow View Count"), counter_value("Active Probes"), counter_value("Updated Probes"),
                counter_value("Converged") > 0.0 ? "yes" : "no");
    ImGui::TextDisabled("Main-thread wall %.3f ms; worker CPU work %.3f ms is reported separately and is not added.",
                        cpu_history.main_thread_wall.selected_ms, cpu_history.worker_cpu_work.selected_ms);

    ImGui::SeparatorText("CPU frame budget");
    if (ImGui::BeginTable(
            "ProfilerBreakdownCpu", 8,
            ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY,
            ImVec2(0.0f, 300.0f))) {
      ImGui::TableSetupScrollFreeze(1, 1);
      ImGui::TableSetupColumn("Application hierarchy", ImGuiTableColumnFlags_WidthStretch, 3.0f);
      ImGui::TableSetupColumn("Selected inclusive ms");
      ImGui::TableSetupColumn("Selected self ms");
      ImGui::TableSetupColumn("Average ms");
      ImGui::TableSetupColumn("Median ms");
      ImGui::TableSetupColumn("P95 ms");
      ImGui::TableSetupColumn("% frame");
      ImGui::TableSetupColumn("Observed");
      ImGui::TableHeadersRow();
      const auto* main_thread = FindMainThreadHistory(cpu_history);
      const ProfilerHistoryNode* application_loop = nullptr;
      if (main_thread) {
        const auto loop =
            std::find_if(main_thread->hierarchy.begin(), main_thread->hierarchy.end(), [](const auto& node) {
              return node.name == "Application::Loop";
            });
        if (loop != main_thread->hierarchy.end()) {
          application_loop = &*loop;
          DrawProfilerBreakdownCpuNode(*application_loop, cpu_history.frame_duration.selected_ms);
          DrawProfilerBreakdownCpuSummaryRow("Loop self (uncategorized)", application_loop->self,
                                             cpu_history.frame_duration.selected_ms);
        }
      }
      const auto overhead = std::find_if(panel_state.breakdown_cpu_series.begin(),
                                         panel_state.breakdown_cpu_series.end(), [](const auto& item) {
                                           return item.key == "FrameOverhead";
                                         });
      if (overhead != panel_state.breakdown_cpu_series.end()) {
        DrawProfilerBreakdownCpuSummaryRow("Frame overhead (outside Application Loop)",
                                           SummarizeBreakdownSeries(*overhead, selected_index),
                                           cpu_history.frame_duration.selected_ms);
      }
      DrawProfilerBreakdownCpuSummaryRow("Worker CPU work (parallel; excluded from wall)", cpu_history.worker_cpu_work,
                                         cpu_history.frame_duration.selected_ms);
      ImGui::EndTable();
    }

    ImGui::SeparatorText("GPU frame budget");
    if (ImGui::BeginTable(
            "ProfilerBreakdownGpu", 7,
            ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY,
            ImVec2(0.0f, 260.0f))) {
      ImGui::TableSetupScrollFreeze(1, 1);
      ImGui::TableSetupColumn("GPU group / pass", ImGuiTableColumnFlags_WidthStretch, 3.0f);
      ImGui::TableSetupColumn("Selected ms");
      ImGui::TableSetupColumn("Average ms");
      ImGui::TableSetupColumn("Median ms");
      ImGui::TableSetupColumn("P95 ms");
      ImGui::TableSetupColumn("% work");
      ImGui::TableSetupColumn("Duty");
      ImGui::TableHeadersRow();
      for (const auto& group : gpu_history.groups) {
        ImGui::PushID(group.name.c_str());
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        const bool open =
            ImGui::TreeNodeEx("BreakdownGroup", ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_SpanFullWidth, "%s",
                              group.name.c_str());
        ImGui::TableNextColumn();
        ImGui::Text("%.3f", group.duration.selected_milliseconds);
        ImGui::TableNextColumn();
        ImGui::Text("%.3f", group.duration.stats.AverageMilliseconds());
        ImGui::TableNextColumn();
        ImGui::Text("%.3f", group.duration.stats.MedianMilliseconds());
        ImGui::TableNextColumn();
        ImGui::Text("%.3f", group.duration.stats.PercentileMilliseconds(0.95));
        ImGui::TableNextColumn();
        ImGui::Text("%.1f%%",
                    gpu_history.summed_work.selected_milliseconds > 0.0
                        ? group.duration.selected_milliseconds / gpu_history.summed_work.selected_milliseconds * 100.0
                        : 0.0);
        ImGui::TableNextColumn();
        ImGui::Text("%.1f%%", group.duration.duty_cycle * 100.0);
        if (open) {
          for (const auto& pass : group.passes) {
            ImGui::PushID(pass.metadata.stable_pass_id.c_str());
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TreeNodeEx(
                "BreakdownPass",
                ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_SpanFullWidth, "%s",
                pass.metadata.display_name.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%.3f", pass.duration.selected_milliseconds);
            ImGui::TableNextColumn();
            ImGui::Text("%.3f", pass.duration.stats.AverageMilliseconds());
            ImGui::TableNextColumn();
            ImGui::Text("%.3f", pass.duration.stats.MedianMilliseconds());
            ImGui::TableNextColumn();
            ImGui::Text("%.3f", pass.duration.stats.PercentileMilliseconds(0.95));
            ImGui::TableNextColumn();
            ImGui::Text(
                "%.1f%%",
                gpu_history.summed_work.selected_milliseconds > 0.0 && pass.metadata.contributes_to_frame_total
                    ? pass.duration.selected_milliseconds / gpu_history.summed_work.selected_milliseconds * 100.0
                    : 0.0);
            ImGui::TableNextColumn();
            ImGui::Text("%.1f%%", pass.duration.duty_cycle * 100.0);
            ImGui::PopID();
          }
          ImGui::TreePop();
        }
        ImGui::PopID();
      }
      ImGui::EndTable();
    }

    const int previous_selected_frame = profiler_selected_frame_index_;
    ImGui::SeparatorText("CPU phase history");
    DrawProfilerBreakdownStackedPlot("ProfilerBreakdownCpuPlot", panel_state.breakdown_cpu_series,
                                     profiler_panel_frames_, profiler_selected_frame_index_);
    ImGui::SeparatorText("GPU group history");
    DrawProfilerBreakdownStackedPlot("ProfilerBreakdownGpuPlot", panel_state.breakdown_gpu_series,
                                     profiler_panel_frames_, profiler_selected_frame_index_);
    if (profiler_selected_frame_index_ != previous_selected_frame)
      profiler_panel_paused_ = true;
    ImGui::EndTabItem();
  }

  if (ImGui::BeginTabItem("CPU")) {
    ImGui::SetNextItemWidth(320.0f);
    ImGui::InputTextWithHint("##ProfilerFilter", "Filter scope, layer, or category", profiler_filter_.data(),
                             profiler_filter_.size());
    ImGui::SameLine();
    ImGui::TextDisabled("History: average / max / p95 across %zu frames", profiler_panel_frames_.size());

    const auto history_summaries = BuildProfilerHistorySummaries(profiler_panel_frames_);
    if (ImGui::BeginTable(
            "ProfilerCpuHierarchy", 9,
            ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY,
            ImVec2(0.0f, 360.0f))) {
      ImGui::TableSetupScrollFreeze(1, 1);
      ImGui::TableSetupColumn("CPU hierarchy", ImGuiTableColumnFlags_WidthStretch, 3.0f);
      ImGui::TableSetupColumn("Inclusive ms");
      ImGui::TableSetupColumn("Self ms");
      ImGui::TableSetupColumn("Calls");
      ImGui::TableSetupColumn("% Parent");
      ImGui::TableSetupColumn("% Frame");
      ImGui::TableSetupColumn("Avg ms");
      ImGui::TableSetupColumn("Max ms");
      ImGui::TableSetupColumn("P95 ms");
      ImGui::TableHeadersRow();
      const std::string filter = profiler_filter_.data();
      std::unordered_map<uint64_t, const ProfilerThreadLane*> current_lanes;
      for (const auto& lane : selected_frame.thread_lanes)
        current_lanes[lane.thread_id] = &lane;
      for (const auto group :
           {profiler_panel_detail::CpuExecutorGroup::MainThread, profiler_panel_detail::CpuExecutorGroup::Worker,
            profiler_panel_detail::CpuExecutorGroup::AssetIo, profiler_panel_detail::CpuExecutorGroup::GpuSubmission,
            profiler_panel_detail::CpuExecutorGroup::Render, profiler_panel_detail::CpuExecutorGroup::Background,
            profiler_panel_detail::CpuExecutorGroup::Other}) {
        std::vector<const ProfilerPanelState::CpuThread*> group_threads;
        const bool group_matches =
            TextContainsCaseInsensitive(profiler_panel_detail::CpuExecutorGroupName(group), filter);
        for (const auto& thread : panel_state.cpu_threads) {
          if (profiler_panel_detail::ClassifyCpuExecutor(thread.name) != group)
            continue;
          const bool thread_matches = TextContainsCaseInsensitive(thread.name, filter);
          const bool hierarchy_matches = std::any_of(thread.roots.begin(), thread.roots.end(), [&](const auto& node) {
            return ProfilerHierarchyMatchesFilter(node, filter);
          });
          if (group_matches || thread_matches || hierarchy_matches)
            group_threads.emplace_back(&thread);
        }
        if (group_threads.empty())
          continue;
        double group_root_ms = 0.0;
        size_t group_event_count = 0;
        for (const auto* thread : group_threads) {
          const auto lane_search = current_lanes.find(thread->id);
          if (lane_search == current_lanes.end())
            continue;
          group_event_count += lane_search->second->events.size();
          for (const auto& root : lane_search->second->hierarchy)
            group_root_ms += root.inclusive_ms;
        }
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::PushID(static_cast<int>(group));
        ImGuiTreeNodeFlags group_flags = ImGuiTreeNodeFlags_SpanFullWidth | ImGuiTreeNodeFlags_OpenOnArrow;
        if (group == profiler_panel_detail::CpuExecutorGroup::MainThread)
          group_flags |= ImGuiTreeNodeFlags_DefaultOpen;
        const bool group_open =
            ImGui::TreeNodeEx("ExecutorGroup", group_flags, "%s (%zu)",
                              profiler_panel_detail::CpuExecutorGroupName(group), group_threads.size());
        ImGui::TableNextColumn();
        ImGui::Text("%.3f", group_root_ms);
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("-");
        ImGui::TableNextColumn();
        ImGui::Text("%zu", group_event_count);
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("-");
        ImGui::TableNextColumn();
        ImGui::Text("%.1f%%",
                    selected_frame.duration_ms > 0.0 ? group_root_ms / selected_frame.duration_ms * 100.0 : 0.0);
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("-");
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("-");
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("-");
        if (group_open) {
          for (const auto* catalog_thread : group_threads) {
            const auto lane_search = current_lanes.find(catalog_thread->id);
            const auto* lane = lane_search == current_lanes.end() ? nullptr : lane_search->second;
            double lane_root_ms = 0.0;
            if (lane) {
              for (const auto& root : lane->hierarchy)
                lane_root_ms += root.inclusive_ms;
            }
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::PushID(static_cast<int>(catalog_thread->id));
            const bool lane_open = ImGui::TreeNodeEx("PhysicalThread", ImGuiTreeNodeFlags_SpanFullWidth, "%s",
                                                     catalog_thread->name.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%.3f", lane_root_ms);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("-");
            ImGui::TableNextColumn();
            ImGui::Text("%zu", lane ? lane->events.size() : 0);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("-");
            ImGui::TableNextColumn();
            ImGui::Text("%.1f%%",
                        selected_frame.duration_ms > 0.0 ? lane_root_ms / selected_frame.duration_ms * 100.0 : 0.0);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("-");
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("-");
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("-");
            if (lane_open) {
              std::unordered_map<std::string, const ProfilerHierarchyNode*> current_nodes;
              if (lane)
                CollectProfilerCurrentNodes(lane->thread_id, lane->hierarchy, "", current_nodes);
              for (const auto& node : catalog_thread->roots) {
                DrawProfilerHierarchyNode(node, current_nodes, selected_frame.duration_ms, selected_frame.duration_ms,
                                          history_summaries, filter, group_matches);
              }
              ImGui::TreePop();
            }
            ImGui::PopID();
          }
          ImGui::TreePop();
        }
        ImGui::PopID();
      }
      ImGui::EndTable();
    }

    auto cpu_series = BuildCpuProfilerSeries(profiler_panel_frames_);
    if (!cpu_series.empty()) {
      ImGui::SetNextItemWidth(220.0f);
      ImGui::SliderInt("Top CPU scopes", &panel_state.cpu_top_series_count, 1, 16);
      ImGui::SameLine();
      ImGui::SetNextItemWidth(280.0f);
      ImGui::InputTextWithHint("##CpuPlotFilter", "Filter CPU plot series", panel_state.cpu_plot_filter.data(),
                               panel_state.cpu_plot_filter.size());
      const std::string plot_filter = panel_state.cpu_plot_filter.data();
      std::vector<size_t> ranked;
      for (size_t index = 0; index < cpu_series.size(); ++index) {
        const auto& item = cpu_series[index];
        if (!TextContainsCaseInsensitive(item.name, plot_filter) &&
            !TextContainsCaseInsensitive(profiler_panel_detail::CpuExecutorGroupName(item.group), plot_filter))
          continue;
        if (panel_state.hidden_cpu_scopes.count(item.key) == 0 && panel_state.pinned_cpu_scopes.count(item.key) == 0)
          ranked.emplace_back(index);
      }
      std::stable_sort(ranked.begin(), ranked.end(), [&](const size_t left, const size_t right) {
        return cpu_series[left].average_milliseconds > cpu_series[right].average_milliseconds;
      });
      if (ranked.size() > static_cast<size_t>(panel_state.cpu_top_series_count))
        ranked.resize(static_cast<size_t>(panel_state.cpu_top_series_count));
      const std::unordered_set<size_t> automatic(ranked.begin(), ranked.end());
      std::vector<size_t> visible;
      for (size_t index = 0; index < cpu_series.size(); ++index) {
        const auto& item = cpu_series[index];
        if (panel_state.hidden_cpu_scopes.count(item.key) != 0)
          continue;
        if (automatic.count(index) != 0 || panel_state.pinned_cpu_scopes.count(item.key) != 0)
          visible.emplace_back(index);
      }
      if (ImGui::CollapsingHeader("CPU plot series and legend")) {
        if (ImGui::BeginTable("CpuPlotSeries", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV,
                              ImVec2(0.0f, 180.0f))) {
          ImGui::TableSetupColumn("Show");
          ImGui::TableSetupColumn("Pin");
          ImGui::TableSetupColumn("Scope");
          ImGui::TableSetupColumn("Executor");
          ImGui::TableSetupColumn("Average ms");
          ImGui::TableHeadersRow();
          for (const auto& item : cpu_series) {
            if (!TextContainsCaseInsensitive(item.name, plot_filter) &&
                !TextContainsCaseInsensitive(profiler_panel_detail::CpuExecutorGroupName(item.group), plot_filter))
              continue;
            ImGui::PushID(item.key.c_str());
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            bool shown = panel_state.hidden_cpu_scopes.count(item.key) == 0;
            if (ImGui::Checkbox("##Show", &shown)) {
              if (shown)
                panel_state.hidden_cpu_scopes.erase(item.key);
              else
                panel_state.hidden_cpu_scopes.insert(item.key);
            }
            ImGui::TableNextColumn();
            bool pinned = panel_state.pinned_cpu_scopes.count(item.key) != 0;
            if (ImGui::Checkbox("##Pin", &pinned)) {
              if (pinned) {
                panel_state.pinned_cpu_scopes.insert(item.key);
                panel_state.hidden_cpu_scopes.erase(item.key);
              } else {
                panel_state.pinned_cpu_scopes.erase(item.key);
              }
            }
            ImGui::TableNextColumn();
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(GpuProfilerSeriesColor(item.key)), "%s",
                               item.name.c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(profiler_panel_detail::CpuExecutorGroupName(item.group));
            ImGui::TableNextColumn();
            ImGui::Text("%.3f", item.average_milliseconds);
            ImGui::PopID();
          }
          ImGui::EndTable();
        }
      }
      const int previous_selected_frame = profiler_selected_frame_index_;
      ImGui::TextUnformatted("Per-scope CPU history");
      DrawCpuProfilerHistoryPlot("CpuScopeHistoryPlot", cpu_series, visible, profiler_panel_frames_,
                                 profiler_selected_frame_index_);
      ImGui::TextUnformatted("Non-overlapping main-thread phase composition");
      const auto cpu_phase_series = BuildCpuBreakdownSeries(profiler_panel_frames_);
      DrawProfilerBreakdownStackedPlot("CpuPhaseStackedPlot", cpu_phase_series, profiler_panel_frames_,
                                       profiler_selected_frame_index_);
      if (profiler_selected_frame_index_ != previous_selected_frame)
        profiler_panel_paused_ = true;
    }
    ImGui::EndTabItem();
  }

  if (ImGui::BeginTabItem("GPU")) {
    ImGui::SeparatorText("GPU render passes");
    std::unordered_map<std::string, const GpuTimestampFrameSnapshot*> gpu_by_frame;
    for (const auto& frame : panel_state.gpu_frames) {
      gpu_by_frame[GpuProfilerFrameKey(frame.capture_session_index, frame.application_frame_index)] = &frame;
    }
    const auto selected_gpu_search = gpu_by_frame.find(
        GpuProfilerFrameKey(selected_frame.capture_session_index, selected_frame.application_frame_index));
    const auto* selected_gpu_frame = selected_gpu_search == gpu_by_frame.end() ? nullptr : selected_gpu_search->second;
    if (!Platform::GpuTimestampCaptureAvailable()) {
      ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.25f, 1.0f),
                         "GPU timestamps are unavailable on the active graphics queue; CPU history remains valid.");
    } else if (!selected_gpu_frame) {
      ImGui::TextDisabled(
          "No GPU result for this application frame. GPU queries resolve after submission and may still "
          "be delayed, or this frame was captured without GPU work.");
    } else if (!selected_gpu_frame->results_available) {
      ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.25f, 1.0f), "GPU result is delayed: %u scopes remain unresolved.",
                         selected_gpu_frame->unresolved_scope_count);
    }
    {
      const auto gpu_aggregates = selected_gpu_frame && selected_gpu_frame->results_available
                                      ? BuildGpuTimestampPassAggregates(*selected_gpu_frame)
                                      : std::vector<GpuTimestampPassAggregate>{};
      const auto gpu_history_summaries = BuildGpuProfilerHistorySummaries(panel_state.gpu_frames, panel_state);
      double summed_pass_work = 0.0;
      for (const auto& aggregate : gpu_aggregates) {
        if (aggregate.metadata.contributes_to_frame_total)
          summed_pass_work += aggregate.total_milliseconds;
      }
      if (selected_gpu_frame && selected_gpu_frame->results_available) {
        ImGui::Text("GPU span %.3f ms  |  summed pass work %.3f ms  |  queries %u/%u",
                    selected_gpu_frame->span_milliseconds, summed_pass_work, selected_gpu_frame->query_used,
                    selected_gpu_frame->query_capacity);
        if (ImGui::IsItemHovered()) {
          ImGui::SetTooltip(
              "GPU span is elapsed device-clock time from the first scope begin to the last scope end.\n"
              "Summed pass work adds non-summary logical pass durations and can differ when work overlaps.");
        }
        if (selected_gpu_frame->skipped_scope_count > 0 || selected_gpu_frame->unresolved_scope_count > 0) {
          ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.25f, 1.0f),
                             "Incomplete GPU capture: %u capacity-skipped and %u unresolved scopes.",
                             selected_gpu_frame->skipped_scope_count, selected_gpu_frame->unresolved_scope_count);
        }
      }

      ImGui::SetNextItemWidth(320.0f);
      ImGui::InputTextWithHint("##GpuProfilerFilter", "Filter GPU group or pass", panel_state.gpu_filter.data(),
                               panel_state.gpu_filter.size());
      const std::string gpu_filter_text = panel_state.gpu_filter.data();

      std::unordered_map<std::string, const GpuTimestampPassAggregate*> selected_passes;
      std::unordered_map<std::string, double> selected_group_totals;
      std::unordered_set<std::string> selected_groups;
      for (const auto& aggregate : gpu_aggregates) {
        selected_passes[GpuProfilerPassKey(aggregate.metadata)] = &aggregate;
        selected_groups.emplace(aggregate.metadata.group);
        if (aggregate.metadata.contributes_to_frame_total)
          selected_group_totals[aggregate.metadata.group] += aggregate.total_milliseconds;
      }

      if (ImGui::BeginTable(
              "ProfilerGpuHierarchy", 9,
              ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY,
              ImVec2(0.0f, 340.0f))) {
        ImGui::TableSetupScrollFreeze(1, 1);
        ImGui::TableSetupColumn("GPU group / pass / instance", ImGuiTableColumnFlags_WidthStretch, 3.0f);
        ImGui::TableSetupColumn("Duration ms");
        ImGui::TableSetupColumn("% Group");
        ImGui::TableSetupColumn("% Work");
        ImGui::TableSetupColumn("Calls");
        ImGui::TableSetupColumn("Avg ms");
        ImGui::TableSetupColumn("Median ms");
        ImGui::TableSetupColumn("Max ms");
        ImGui::TableSetupColumn("P95 ms");
        ImGui::TableHeadersRow();
        for (const auto& group : panel_state.gpu_groups) {
          const auto group_total_search = selected_group_totals.find(group.name);
          const bool group_present = selected_groups.count(group.name) != 0;
          const double group_total =
              group_total_search == selected_group_totals.end() ? 0.0 : group_total_search->second;
          const bool group_matches = TextContainsCaseInsensitive(group.name, gpu_filter_text);
          const bool pass_matches = std::any_of(group.passes.begin(), group.passes.end(), [&](const auto& pass) {
            return TextContainsCaseInsensitive(pass.metadata.display_name, gpu_filter_text);
          });
          if (!group_matches && !pass_matches)
            continue;
          ImGui::PushID(group.name.c_str());
          if (!group_present)
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          const bool group_open = ImGui::TreeNodeEx(
              "Group", ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_SpanFullWidth, "%s", group.name.c_str());
          ImGui::TableNextColumn();
          group_present ? ImGui::Text("%.3f", group_total) : ImGui::TextUnformatted("-");
          ImGui::TableNextColumn();
          ImGui::TextUnformatted("-");
          ImGui::TableNextColumn();
          group_present ? ImGui::Text("%.1f%%", summed_pass_work > 0.0 ? group_total / summed_pass_work * 100.0 : 0.0)
                        : ImGui::TextUnformatted("-");
          ImGui::TableNextColumn();
          if (group_present) {
            const auto call_count = std::count_if(group.passes.begin(), group.passes.end(), [&](const auto& pass) {
              return selected_passes.count(pass.key) != 0;
            });
            ImGui::Text("%zu", call_count);
          } else {
            ImGui::TextUnformatted("-");
          }
          for (int column = 5; column < 9; ++column) {
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("-");
          }
          if (group_open) {
            for (const auto& catalog_pass : group.passes) {
              if (!group_matches && !TextContainsCaseInsensitive(catalog_pass.metadata.display_name, gpu_filter_text))
                continue;
              const auto selected_pass_search = selected_passes.find(catalog_pass.key);
              const auto* pass = selected_pass_search == selected_passes.end() ? nullptr : selected_pass_search->second;
              const auto summary_search = gpu_history_summaries.find(catalog_pass.key);
              const GpuTimestampStats* stats =
                  summary_search == gpu_history_summaries.end() ? nullptr : &summary_search->second.stats;
              ImGui::PushID(catalog_pass.key.c_str());
              if (!pass)
                ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
              ImGui::TableNextRow();
              ImGui::TableNextColumn();
              ImGuiTreeNodeFlags pass_flags = ImGuiTreeNodeFlags_SpanFullWidth;
              if (!pass || pass->instances.empty())
                pass_flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
              const bool pass_open = ImGui::TreeNodeEx(
                  "Pass", pass_flags, "%s%s", catalog_pass.metadata.display_name.c_str(),
                  catalog_pass.metadata.contributes_to_frame_total ? "" : "  (summary; excluded from sum)");
              ImGui::TableNextColumn();
              pass ? ImGui::Text("%.3f", pass->total_milliseconds) : ImGui::TextUnformatted("-");
              ImGui::TableNextColumn();
              pass ? ImGui::Text("%.1f%%", group_total > 0.0 && catalog_pass.metadata.contributes_to_frame_total
                                               ? pass->total_milliseconds / group_total * 100.0
                                               : 0.0)
                   : ImGui::TextUnformatted("-");
              ImGui::TableNextColumn();
              if (!pass)
                ImGui::TextUnformatted("-");
              else if (catalog_pass.metadata.contributes_to_frame_total)
                ImGui::Text("%.1f%%",
                            summed_pass_work > 0.0 ? pass->total_milliseconds / summed_pass_work * 100.0 : 0.0);
              else
                ImGui::TextUnformatted("-");
              ImGui::TableNextColumn();
              pass ? ImGui::Text("%u", pass->call_count) : ImGui::TextUnformatted("-");
              ImGui::TableNextColumn();
              ImGui::Text("%.3f", stats ? stats->AverageMilliseconds() : 0.0);
              ImGui::TableNextColumn();
              ImGui::Text("%.3f", stats ? stats->MedianMilliseconds() : 0.0);
              ImGui::TableNextColumn();
              ImGui::Text("%.3f", stats ? stats->maximum_milliseconds : 0.0);
              ImGui::TableNextColumn();
              ImGui::Text("%.3f", stats ? stats->PercentileMilliseconds(0.95) : 0.0);
              if (pass_open && pass && !pass->instances.empty()) {
                for (size_t instance_index = 0; instance_index < pass->instances.size(); ++instance_index) {
                  const auto& instance = pass->instances[instance_index];
                  ImGui::PushID(static_cast<int>(instance_index));
                  ImGui::TableNextRow();
                  ImGui::TableNextColumn();
                  ImGui::TreeNodeEx(
                      "Instance",
                      ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_SpanFullWidth,
                      "#%zu  view %llu  instance %llu  [%s]", instance_index + 1,
                      static_cast<unsigned long long>(instance.metadata.view_id),
                      static_cast<unsigned long long>(instance.metadata.instance_id),
                      GpuTimestampQueueName(instance.metadata.queue));
                  ImGui::TableNextColumn();
                  ImGui::Text("%.3f", instance.duration_milliseconds);
                  for (int column = 2; column < 9; ++column) {
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted("-");
                  }
                  ImGui::PopID();
                }
                ImGui::TreePop();
              }
              if (!pass)
                ImGui::PopStyleColor();
              ImGui::PopID();
            }
            ImGui::TreePop();
          }
          if (!group_present)
            ImGui::PopStyleColor();
          ImGui::PopID();
        }
        ImGui::EndTable();
      }

      auto gpu_series = BuildGpuProfilerSeries(profiler_panel_frames_, panel_state.gpu_frames, panel_state);
      if (!gpu_series.empty()) {
        ImGui::SetNextItemWidth(220.0f);
        ImGui::SliderInt("Top GPU passes", &panel_state.gpu_top_series_count, 1, 16);
        std::vector<size_t> ranked_series;
        for (size_t series_index = 0; series_index < gpu_series.size(); ++series_index) {
          const auto& pass_series = gpu_series[series_index];
          if (TextContainsCaseInsensitive(pass_series.metadata.display_name, gpu_filter_text) ||
              TextContainsCaseInsensitive(pass_series.metadata.group, gpu_filter_text)) {
            if (panel_state.hidden_gpu_passes.count(pass_series.key) == 0 &&
                panel_state.pinned_gpu_passes.count(pass_series.key) == 0)
              ranked_series.emplace_back(series_index);
          }
        }
        std::stable_sort(ranked_series.begin(), ranked_series.end(), [&](const size_t left, const size_t right) {
          return gpu_series[left].average_milliseconds > gpu_series[right].average_milliseconds;
        });
        if (ranked_series.size() > static_cast<size_t>(panel_state.gpu_top_series_count))
          ranked_series.resize(static_cast<size_t>(panel_state.gpu_top_series_count));
        const std::unordered_set<size_t> automatic_series(ranked_series.begin(), ranked_series.end());
        std::vector<size_t> visible_series;
        for (size_t series_index = 0; series_index < gpu_series.size(); ++series_index) {
          const auto& pass_series = gpu_series[series_index];
          if (!TextContainsCaseInsensitive(pass_series.metadata.display_name, gpu_filter_text) &&
              !TextContainsCaseInsensitive(pass_series.metadata.group, gpu_filter_text))
            continue;
          if (panel_state.hidden_gpu_passes.count(pass_series.key) != 0)
            continue;
          const bool pinned = panel_state.pinned_gpu_passes.count(pass_series.key) != 0;
          if (pinned || automatic_series.count(series_index) != 0) {
            visible_series.emplace_back(series_index);
          }
        }
        if (ImGui::CollapsingHeader("GPU plot series and legend")) {
          if (ImGui::BeginTable("GpuPlotSeries", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV,
                                ImVec2(0.0f, 180.0f))) {
            ImGui::TableSetupColumn("Show");
            ImGui::TableSetupColumn("Pin");
            ImGui::TableSetupColumn("Pass");
            ImGui::TableSetupColumn("Group");
            ImGui::TableSetupColumn("Average ms");
            ImGui::TableHeadersRow();
            for (const auto& pass_series : gpu_series) {
              if (!TextContainsCaseInsensitive(pass_series.metadata.display_name, gpu_filter_text) &&
                  !TextContainsCaseInsensitive(pass_series.metadata.group, gpu_filter_text))
                continue;
              ImGui::PushID(pass_series.key.c_str());
              ImGui::TableNextRow();
              ImGui::TableNextColumn();
              bool shown = panel_state.hidden_gpu_passes.count(pass_series.key) == 0;
              if (ImGui::Checkbox("##Show", &shown)) {
                if (shown)
                  panel_state.hidden_gpu_passes.erase(pass_series.key);
                else
                  panel_state.hidden_gpu_passes.insert(pass_series.key);
              }
              ImGui::TableNextColumn();
              bool pinned = panel_state.pinned_gpu_passes.count(pass_series.key) != 0;
              if (ImGui::Checkbox("##Pin", &pinned)) {
                if (pinned) {
                  panel_state.pinned_gpu_passes.insert(pass_series.key);
                  panel_state.hidden_gpu_passes.erase(pass_series.key);
                } else {
                  panel_state.pinned_gpu_passes.erase(pass_series.key);
                }
              }
              ImGui::TableNextColumn();
              ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(GpuProfilerSeriesColor(pass_series.key)), "%s",
                                 pass_series.metadata.display_name.c_str());
              ImGui::TableNextColumn();
              ImGui::TextUnformatted(pass_series.metadata.group.c_str());
              ImGui::TableNextColumn();
              ImGui::Text("%.3f", pass_series.average_milliseconds);
              ImGui::PopID();
            }
            ImGui::EndTable();
          }
        }
        ImGui::TextUnformatted("Per-pass GPU history (click a point to select its application frame)");
        const int previous_selected_frame = profiler_selected_frame_index_;
        DrawGpuProfilerHistoryPlot("GpuPassHistoryPlot", gpu_series, visible_series, profiler_panel_frames_,
                                   profiler_selected_frame_index_);
        ImGui::TextUnformatted("Stacked GPU pass composition (unselected passes are combined as Other)");
        DrawGpuProfilerStackedPlot("GpuPassStackedPlot", gpu_series, visible_series, profiler_panel_frames_,
                                   profiler_selected_frame_index_);
        if (profiler_selected_frame_index_ != previous_selected_frame)
          profiler_panel_paused_ = true;
      }
    }
    ImGui::EndTabItem();
  }

  if (ImGui::BeginTabItem("Diagnostics")) {
    const auto pin_frame = [&](const int index) {
      const auto& frame = profiler_panel_frames_[index];
      panel_state.diagnostics_cpu_frame = frame;
      const auto gpu_frame =
          std::find_if(panel_state.gpu_frames.begin(), panel_state.gpu_frames.end(), [&](const auto& candidate) {
            return candidate.capture_session_index == frame.capture_session_index &&
                   candidate.application_frame_index == frame.application_frame_index;
          });
      if (gpu_frame == panel_state.gpu_frames.end())
        panel_state.diagnostics_gpu_frame.reset();
      else
        panel_state.diagnostics_gpu_frame = *gpu_frame;
    };
    const auto find_pinned_index = [&]() {
      if (!panel_state.diagnostics_cpu_frame)
        return -1;
      const auto& pinned = *panel_state.diagnostics_cpu_frame;
      for (size_t i = 0; i < profiler_panel_frames_.size(); ++i) {
        const auto& candidate = profiler_panel_frames_[i];
        if (candidate.capture_session_index == pinned.capture_session_index &&
            candidate.frame_index == pinned.frame_index)
          return static_cast<int>(i);
      }
      return -1;
    };
    if (!panel_state.diagnostics_cpu_frame)
      pin_frame(profiler_selected_frame_index_);
    if (panel_state.diagnostics_follow_latest)
      pin_frame(static_cast<int>(profiler_panel_frames_.size()) - 1);
    auto pinned_index = find_pinned_index();
    if (ImGui::Button("Previous") && pinned_index > 0) {
      pin_frame(pinned_index - 1);
      pinned_index = find_pinned_index();
    }
    ImGui::SameLine();
    if (ImGui::Button("Next")) {
      if (pinned_index >= 0 && pinned_index + 1 < static_cast<int>(profiler_panel_frames_.size()))
        pin_frame(pinned_index + 1);
      else if (pinned_index < 0)
        pin_frame(0);
    }
    ImGui::SameLine();
    if (ImGui::Button("Latest"))
      pin_frame(static_cast<int>(profiler_panel_frames_.size()) - 1);
    ImGui::SameLine();
    if (ImGui::Checkbox("Follow latest", &panel_state.diagnostics_follow_latest) &&
        panel_state.diagnostics_follow_latest)
      pin_frame(static_cast<int>(profiler_panel_frames_.size()) - 1);
    const auto& diagnostics_frame = *panel_state.diagnostics_cpu_frame;
    if (!panel_state.diagnostics_gpu_frame) {
      const auto gpu_frame =
          std::find_if(panel_state.gpu_frames.begin(), panel_state.gpu_frames.end(), [&](const auto& frame) {
            return frame.capture_session_index == diagnostics_frame.capture_session_index &&
                   frame.application_frame_index == diagnostics_frame.application_frame_index;
          });
      if (gpu_frame != panel_state.gpu_frames.end())
        panel_state.diagnostics_gpu_frame = *gpu_frame;
    }
    ImGui::Text("Pinned session %llu  frame %llu  CPU %.3f ms  events %u  dropped %u",
                static_cast<unsigned long long>(diagnostics_frame.capture_session_index),
                static_cast<unsigned long long>(diagnostics_frame.frame_index), diagnostics_frame.duration_ms,
                diagnostics_frame.event_count, diagnostics_frame.dropped_event_count);
    if (!panel_state.diagnostics_gpu_frame) {
      ImGui::TextDisabled("GPU snapshot pending or unavailable for the pinned frame.");
    } else {
      const auto& diagnostics_gpu_frame = *panel_state.diagnostics_gpu_frame;
      ImGui::Text("GPU span %.3f ms  queries %u/%u  skipped %u  unresolved %u", diagnostics_gpu_frame.span_milliseconds,
                  diagnostics_gpu_frame.query_used, diagnostics_gpu_frame.query_capacity,
                  diagnostics_gpu_frame.skipped_scope_count, diagnostics_gpu_frame.unresolved_scope_count);
    }

    ImGui::SeparatorText("CPU timeline (CPU frame-relative clock)");
    if (ImGui::BeginChild("ProfilerCpuTimeline", ImVec2(0.0f, 220.0f), true, ImGuiWindowFlags_HorizontalScrollbar))
      DrawProfilerCpuTimeline(diagnostics_frame);
    ImGui::EndChild();
    ImGui::SeparatorText("GPU timeline (GPU snapshot-relative clock; not aligned to CPU)");
    if (ImGui::BeginChild("ProfilerGpuTimeline", ImVec2(0.0f, 220.0f), true, ImGuiWindowFlags_HorizontalScrollbar)) {
      if (!panel_state.diagnostics_gpu_frame)
        ImGui::TextDisabled("No aligned GPU snapshot.");
      else
        DrawProfilerGpuTimeline(*panel_state.diagnostics_gpu_frame);
    }
    ImGui::EndChild();

    if (ImGui::CollapsingHeader("CPU category totals", ImGuiTreeNodeFlags_DefaultOpen)) {
      std::vector<const ProfilerAggregateTotal*> rows;
      for (const auto& row : diagnostics_frame.category_totals)
        rows.emplace_back(&row);
      if (ImGui::BeginTable("ProfilerCategoryTotals", 4,
                            ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Sortable)) {
        ImGui::TableSetupColumn("Category");
        ImGui::TableSetupColumn("Count");
        ImGui::TableSetupColumn("Total ms",
                                ImGuiTableColumnFlags_DefaultSort | ImGuiTableColumnFlags_PreferSortDescending);
        ImGui::TableSetupColumn("Max ms");
        ImGui::TableHeadersRow();
        if (const auto* specs = ImGui::TableGetSortSpecs(); specs && specs->SpecsCount > 0) {
          const auto spec = specs->Specs[0];
          std::stable_sort(rows.begin(), rows.end(), [&](const auto* left, const auto* right) {
            int comparison = 0;
            if (spec.ColumnIndex == 0)
              comparison = left->category.compare(right->category);
            else {
              const double left_value = spec.ColumnIndex == 1   ? left->count
                                        : spec.ColumnIndex == 2 ? left->total_ms
                                                                : left->max_ms;
              const double right_value = spec.ColumnIndex == 1   ? right->count
                                         : spec.ColumnIndex == 2 ? right->total_ms
                                                                 : right->max_ms;
              comparison = left_value < right_value ? -1 : left_value > right_value ? 1 : 0;
            }
            return spec.SortDirection == ImGuiSortDirection_Ascending ? comparison < 0 : comparison > 0;
          });
        }
        for (const auto* row : rows) {
          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(row->category.c_str());
          ImGui::TableNextColumn();
          ImGui::Text("%u", row->count);
          ImGui::TableNextColumn();
          ImGui::Text("%.3f", row->total_ms);
          ImGui::TableNextColumn();
          ImGui::Text("%.3f", row->max_ms);
        }
        ImGui::EndTable();
      }
    }

    if (ImGui::CollapsingHeader("CPU event totals", ImGuiTreeNodeFlags_DefaultOpen)) {
      std::vector<const ProfilerAggregateTotal*> rows;
      for (const auto& row : diagnostics_frame.named_event_totals)
        rows.emplace_back(&row);
      if (ImGui::BeginTable("ProfilerEventTotals", 4,
                            ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Sortable)) {
        ImGui::TableSetupColumn("Name");
        ImGui::TableSetupColumn("Category");
        ImGui::TableSetupColumn("Count");
        ImGui::TableSetupColumn("Total ms",
                                ImGuiTableColumnFlags_DefaultSort | ImGuiTableColumnFlags_PreferSortDescending);
        ImGui::TableHeadersRow();
        if (const auto* specs = ImGui::TableGetSortSpecs(); specs && specs->SpecsCount > 0) {
          const auto spec = specs->Specs[0];
          std::stable_sort(rows.begin(), rows.end(), [&](const auto* left, const auto* right) {
            int comparison = spec.ColumnIndex == 0   ? left->name.compare(right->name)
                             : spec.ColumnIndex == 1 ? left->category.compare(right->category)
                             : spec.ColumnIndex == 2 ? (left->count < right->count   ? -1
                                                        : left->count > right->count ? 1
                                                                                     : 0)
                                                     : (left->total_ms < right->total_ms   ? -1
                                                        : left->total_ms > right->total_ms ? 1
                                                                                           : 0);
            return spec.SortDirection == ImGuiSortDirection_Ascending ? comparison < 0 : comparison > 0;
          });
        }
        for (const auto* row : rows) {
          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(row->name.c_str());
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(row->category.c_str());
          ImGui::TableNextColumn();
          ImGui::Text("%u", row->count);
          ImGui::TableNextColumn();
          ImGui::Text("%.3f", row->total_ms);
        }
        ImGui::EndTable();
      }
    }

    if (panel_state.diagnostics_gpu_frame &&
        ImGui::CollapsingHeader("GPU pass totals", ImGuiTreeNodeFlags_DefaultOpen)) {
      auto rows = BuildGpuTimestampPassAggregates(*panel_state.diagnostics_gpu_frame);
      if (ImGui::BeginTable("ProfilerGpuTotals", 5,
                            ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Sortable)) {
        ImGui::TableSetupColumn("Pass");
        ImGui::TableSetupColumn("Group");
        ImGui::TableSetupColumn("Queue");
        ImGui::TableSetupColumn("Calls");
        ImGui::TableSetupColumn("Total ms",
                                ImGuiTableColumnFlags_DefaultSort | ImGuiTableColumnFlags_PreferSortDescending);
        ImGui::TableHeadersRow();
        if (const auto* specs = ImGui::TableGetSortSpecs(); specs && specs->SpecsCount > 0) {
          const auto spec = specs->Specs[0];
          std::stable_sort(rows.begin(), rows.end(), [&](const auto& left, const auto& right) {
            int comparison = spec.ColumnIndex == 0   ? left.metadata.display_name.compare(right.metadata.display_name)
                             : spec.ColumnIndex == 1 ? left.metadata.group.compare(right.metadata.group)
                             : spec.ColumnIndex == 2 ? std::string(GpuTimestampQueueName(left.metadata.queue))
                                                           .compare(GpuTimestampQueueName(right.metadata.queue))
                             : spec.ColumnIndex == 3 ? (left.call_count < right.call_count   ? -1
                                                        : left.call_count > right.call_count ? 1
                                                                                             : 0)
                                                     : (left.total_milliseconds < right.total_milliseconds   ? -1
                                                        : left.total_milliseconds > right.total_milliseconds ? 1
                                                                                                             : 0);
            return spec.SortDirection == ImGuiSortDirection_Ascending ? comparison < 0 : comparison > 0;
          });
        }
        for (const auto& row : rows) {
          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(row.metadata.display_name.c_str());
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(row.metadata.group.c_str());
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(GpuTimestampQueueName(row.metadata.queue));
          ImGui::TableNextColumn();
          ImGui::Text("%u", row.call_count);
          ImGui::TableNextColumn();
          ImGui::Text("%.3f", row.total_milliseconds);
        }
        ImGui::EndTable();
      }
    }

    if (ImGui::CollapsingHeader("Raw CPU events")) {
      if (ImGui::BeginTable(
              "ProfilerEvents", 6,
              ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY,
              ImVec2(0.0f, 260.0f))) {
        for (const char* column : {"Thread", "Name", "Category", "Start", "Duration", "Depth"})
          ImGui::TableSetupColumn(column);
        ImGui::TableHeadersRow();
        for (const auto& lane : diagnostics_frame.thread_lanes) {
          for (const auto& event : lane.events) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            const auto thread_name = ProfilerThreadDisplayName(lane.thread_name);
            ImGui::TextUnformatted(thread_name.c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(event.name.c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(event.category.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%.3f", event.start_ms);
            ImGui::TableNextColumn();
            ImGui::Text("%.3f", event.duration_ms);
            ImGui::TableNextColumn();
            ImGui::Text("%u", event.depth);
          }
        }
        ImGui::EndTable();
      }
    }

    const auto export_frames = profiler.GetFrameHistorySnapshot();
    ImGui::SeparatorText("Trace export");
    if (ImGui::Button("Quick Export Trace")) {
      std::string error;
      const auto export_path = DefaultProfilerTracePath();
      if (ExportProfilerChromeTrace(export_path, export_frames, &error)) {
        profiler_export_status_ = "Exported " + export_path.string();
      } else {
        profiler_export_status_ = "Export failed: " + error;
      }
    }
    ImGui::SameLine();
    FileUtils::SaveFile(
        "Export Trace...", "Chrome Trace JSON", {".json"},
        [export_frames, this](const std::filesystem::path& path) {
          std::string error;
          if (ExportProfilerChromeTrace(path, export_frames, &error)) {
            profiler_export_status_ = "Exported " + path.string();
          } else {
            profiler_export_status_ = "Export failed: " + error;
          }
        },
        false);
    if (!profiler_export_status_.empty())
      ImGui::TextWrapped("%s", profiler_export_status_.c_str());
    ImGui::EndTabItem();
  }
  ImGui::EndTabBar();

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
  if (!scene || (!scene_camera_window_focused_ && !entity_explorer_window_focused_) || ImGui::GetIO().WantTextInput ||
      Input::GetKey(GLFW_KEY_DELETE) != Input::KeyActionType::Press) {
    return;
  }
  const auto snapshot = entity_selection_.GetSnapshot();
  std::vector<Entity> roots;
  for (const auto& selected : snapshot.entities) {
    if (!scene->IsEntityValid(selected))
      continue;
    bool selected_ancestor = false;
    for (auto parent = scene->GetParent(selected); parent.GetIndex() != 0; parent = scene->GetParent(parent)) {
      if (snapshot.Contains(parent)) {
        selected_ancestor = true;
        break;
      }
    }
    if (!selected_ancestor)
      roots.push_back(selected);
  }
  for (const auto& root : roots) {
    if (scene->IsEntityValid(root))
      scene->DeleteEntity(root);
  }
  entity_selection_.Clear(EntitySelection::RequestSource::Lifecycle);
  selected_entity_hierarchy_list_.clear();
}

bool EditorLayer::IsControlModifierDown() {
  const auto down = [](const int key) {
    const auto state = Input::GetKey(key);
    return state == Input::KeyActionType::Press || state == Input::KeyActionType::Hold;
  };
  return down(GLFW_KEY_LEFT_CONTROL) || down(GLFW_KEY_RIGHT_CONTROL);
}

bool EditorLayer::IsShiftModifierDown() {
  const auto down = [](const int key) {
    const auto state = Input::GetKey(key);
    return state == Input::KeyActionType::Press || state == Input::KeyActionType::Hold;
  };
  return down(GLFW_KEY_LEFT_SHIFT) || down(GLFW_KEY_RIGHT_SHIFT);
}

void EditorLayer::HandleEntityExplorerSelection(const Entity& entity) {
  if (GetLockEntitySelection())
    return;
  ClearEnvironmentalLightingGizmoTarget();
  const bool control = IsControlModifierDown();
  const bool shift = IsShiftModifierDown();
  bool range_handled = false;
  EntitySelection::Result result = EntitySelection::Result::Unchanged;
  if (shift && entity_selection_.GetAnchor().GetIndex() != 0) {
    const auto build_range = [&](const std::vector<Entity>& visible) {
      std::vector<Entity> range;
      const auto anchor = std::find(visible.begin(), visible.end(), entity_selection_.GetAnchor());
      const auto clicked = std::find(visible.begin(), visible.end(), entity);
      if (anchor == visible.end() || clicked == visible.end())
        return range;
      const auto first = std::min(anchor, clicked);
      const auto last = std::max(anchor, clicked);
      range.assign(first, last + 1);
      return range;
    };
    auto range = build_range(entity_explorer_visible_entities_);
    if (range.empty())
      range = build_range(entity_explorer_current_entities_);
    if (!range.empty()) {
      range_handled = true;
      const EntitySelection::RequestOptions options{
          EntitySelection::RequestSource::User, EntitySelection::AnchorPolicy::Preserve, {}};
      result = control ? entity_selection_.AddMany(range, entity, options)
                       : entity_selection_.ReplaceMany(range, entity, options);
    }
  }
  if (!range_handled && !shift) {
    const EntitySelection::RequestOptions options{EntitySelection::RequestSource::User,
                                                  EntitySelection::AnchorPolicy::Set, entity};
    result = control ? entity_selection_.Toggle(entity, options) : entity_selection_.Replace(entity, options);
  } else if (!range_handled && shift) {
    const EntitySelection::RequestOptions options{EntitySelection::RequestSource::User,
                                                  EntitySelection::AnchorPolicy::Set, entity};
    result = entity_selection_.Replace(entity, options);
  }
  if (result == EntitySelection::Result::Changed)
    selected_entity_hierarchy_list_.clear();
}

void EditorLayer::HandleViewportSelection(const Entity& entity, const bool control) {
  if (GetLockEntitySelection())
    return;
  ClearEnvironmentalLightingGizmoTarget();
  EntitySelection::Result result = EntitySelection::Result::Unchanged;
  if (control) {
    if (entity.GetIndex() != 0) {
      result = entity_selection_.Add(entity,
                                     {EntitySelection::RequestSource::User, EntitySelection::AnchorPolicy::Clear, {}});
    }
  } else if (entity.GetIndex() == 0) {
    result = entity_selection_.Clear(EntitySelection::RequestSource::User);
  } else {
    auto target = entity;
    for (auto walker = entity; walker.GetIndex() != 0; walker = GetScene()->GetParent(walker)) {
      if (walker != entity_selection_.GetPrimary())
        continue;
      const auto parent = GetScene()->GetParent(walker);
      target = parent.GetIndex() == 0 ? entity : parent;
      break;
    }
    result = entity_selection_.Replace(
        target, {EntitySelection::RequestSource::User, EntitySelection::AnchorPolicy::Clear, {}});
  }
  if (result == EntitySelection::Result::Changed)
    selected_entity_hierarchy_list_.clear();
}

void EditorLayer::ProcessPendingViewportSelection() {
  if (!pending_viewport_selection_)
    return;
  const auto request = *pending_viewport_selection_;
  pending_viewport_selection_.reset();
  if (const auto camera = request.camera.lock()) {
    HandleViewportSelection(MouseEntitySelection(camera, request.mouse_position), request.control);
  }
}

bool EditorLayer::IsInheritedSelectionHighlight(const Entity& entity) const {
  const auto scene = GetScene();
  if (!scene || entity_selection_.Contains(entity))
    return false;
  for (auto parent = scene->GetParent(entity); parent.GetIndex() != 0; parent = scene->GetParent(parent)) {
    if (entity_selection_.Contains(parent))
      return true;
  }
  return false;
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

void EditorLayer::DrawSceneCameraSettingsContents(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto& [scene_camera_rotation, scene_camera_position, scene_camera] = editor_cameras_.at(scene_camera_handle_);

  ImGui::SeparatorText("Display");
  ImGui::Checkbox("Scene Camera Info", &show_scene_info);

  ImGui::SeparatorText("Camera");
  if (ImGui::Button("Reset camera"))
    MoveCamera(default_scene_camera_rotation, default_scene_camera_position);
  ImGui::SameLine();
  if (ImGui::Button("Set default camera position")) {
    default_scene_camera_position = scene_camera_position;
    default_scene_camera_rotation = scene_camera_rotation;
  }
  ImGui::DragFloat("Max move speed", &velocity, 0.1f, 0.0f, 0.0f, "%.1f");
  ImGui::DragFloat("Max mouse sensitivity", &sensitivity, 0.1f, 0.0f, 0.0f, "%.1f");
  ImGui::DragFloat("Acceleration time", &camera_control_acceleration_time, 0.01f, 0.0f, 5.0f, "%.2f s");
  ImGui::DragFloat("Deceleration time", &camera_control_deceleration_time, 0.01f, 0.0f, 5.0f, "%.2f s");
  ImGui::DragFloat3("Position", &scene_camera_position.x, 0.1f);
  if (ImGui::DragFloat4("Rotation", &scene_camera_rotation.x, 0.01f)) {
    const float length_squared = glm::dot(scene_camera_rotation, scene_camera_rotation);
    if (std::isfinite(length_squared) && length_squared > glm::epsilon<float>())
      scene_camera_rotation = glm::normalize(scene_camera_rotation);
  }
  ImGui::Checkbox("Copy Transform", &apply_transform_to_main_camera);
  ImGui::DragFloat("Resolution", &scene_camera_resolution_multiplier, 0.1f, 0.1f, 4.0f);

  ImGui::SeparatorText("Camera Settings");
  if (scene_camera) {
    InspectorContext context;
    context.editor_layer = editor_layer;
    context.scene = ApplicationContext::Get().GetActiveScene();
    InspectorRegistry::GetInstance().Inspect(context, *scene_camera);
  } else {
    ImGui::Text("No active scene camera!");
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

  if (ImGui::BeginTabBar("EditorLayerInspectionTabs")) {
    auto& [sceneCameraRotation, sceneCameraPosition, sceneCamera] = editor_cameras_.at(scene_camera_handle_);
    if (ImGui::BeginTabItem("Entity Inspector")) {
      bool highlight_selection = entity_selection_highlight_.IsEnabled();
      if (ImGui::Checkbox("Focus", &highlight_selection))
        entity_selection_highlight_.SetEnabled(highlight_selection, !entity_selection_.Empty());
      ImGui::Checkbox("Gizmos", &enable_gizmos);
      int pivot_mode = static_cast<int>(entity_gizmo_pivot_mode_);
      if (ImGui::Combo("Pivot mode", &pivot_mode, "Pivot\0Center\0"))
        entity_gizmo_pivot_mode_ = static_cast<EntityGizmoPivotMode>(pivot_mode);
      int orientation_mode = static_cast<int>(entity_gizmo_orientation_mode_);
      if (ImGui::Combo("Orientation", &orientation_mode, "Local\0Global\0"))
        entity_gizmo_orientation_mode_ = static_cast<EntityGizmoOrientationMode>(orientation_mode);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Key Bindings")) {
      const auto& pressed_keys = Input::GetInstance().pressed_keys_;
      DrawKeyBinding("Rotate mouse button", editor_camera_control_key_bindings.rotate_mouse_button, true, pressed_keys);
      DrawKeyBinding("Focus Selection", editor_camera_control_key_bindings.focus_selection_key, false, pressed_keys);
      DrawKeyBinding("Move forward", editor_camera_control_key_bindings.move_forward_key, false, pressed_keys);
      DrawKeyBinding("Move backward", editor_camera_control_key_bindings.move_backward_key, false, pressed_keys);
      DrawKeyBinding("Move left", editor_camera_control_key_bindings.move_left_key, false, pressed_keys);
      DrawKeyBinding("Move right", editor_camera_control_key_bindings.move_right_key, false, pressed_keys);
      DrawKeyBinding("Move up", editor_camera_control_key_bindings.move_up_key, false, pressed_keys);
      DrawKeyBinding("Move down", editor_camera_control_key_bindings.move_down_key, false, pressed_keys);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Debug")) {
      const auto scene = ApplicationContext::Get().GetActiveScene();
      if (!scene) {
        ImGui::Text("No Scene!");
      } else if (!sceneCamera) {
        ImGui::Text("No active scene camera!");
      } else {
        static float debug_scale = 0.25f;
        ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 1.0f);
        debug_scale = glm::clamp(debug_scale, 0.1f, 1.0f);
        DrawCameraDebugViews(*sceneCamera, debug_scale);
      }
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
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
  ImGui::PushStyleColor(ImGuiCol_FrameBg, ImGui::ColorConvertU32ToFloat4(TitleBarSearchBgColor()));
  ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImGui::ColorConvertU32ToFloat4(TitleBarSearchHoveredColor()));
  ImGui::PushStyleColor(ImGuiCol_FrameBgActive, ImGui::ColorConvertU32ToFloat4(TitleBarSearchActiveColor()));
  ImGui::PushStyleColor(ImGuiCol_Border, ImGui::ColorConvertU32ToFloat4(TitleBarSearchBorderColor()));
  ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(TitleBarTextColor()));
  ImGui::PushStyleColor(ImGuiCol_TextDisabled, ImGui::ColorConvertU32ToFloat4(TitleBarSecondaryTextColor()));
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
  ImGui::PushStyleColor(ImGuiCol_WindowBg, ImGui::ColorConvertU32ToFloat4(TitleBarMenuPopupBgColor()));
  ImGui::PushStyleColor(ImGuiCol_Border, ImGui::ColorConvertU32ToFloat4(TitleBarMenuPopupBorderColor()));
  ImGui::PushStyleColor(ImGuiCol_Header, ImGui::ColorConvertU32ToFloat4(TitleBarMenuItemHoveredColor()));
  ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImGui::ColorConvertU32ToFloat4(TitleBarMenuItemHoveredColor()));
  ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImGui::ColorConvertU32ToFloat4(TitleBarMenuItemHoveredColor()));
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
          ProjectManager::GetInstance().show_project_window = true;
          if (project_content_browser_panel_) {
            project_content_browser_panel_->RevealAsset(result.asset_handle);
          }
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
        draw_list->AddText(label_pos, TitleBarTextColor(), result.label.c_str());
        const std::string detail = std::string(TitleBarSearchResultTypeName(result.type)) + " - " + result.detail;
        draw_list->AddText(detail_pos, TitleBarSecondaryTextColor(), detail.c_str());
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
  const ImU32 title_bar_color = TitleBarColor();
  ImGui::PushStyleColor(ImGuiCol_WindowBg, ImGui::ColorConvertU32ToFloat4(title_bar_color));
  ImGui::PushStyleColor(ImGuiCol_MenuBarBg, ImGui::ColorConvertU32ToFloat4(title_bar_color));
  ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(TitleBarTextColor()));
  if (ImGui::Begin("Editor Custom Title Bar", nullptr, flags)) {
    const auto draw_list = ImGui::GetWindowDrawList();
    const ImVec2 titlebar_min = ImGui::GetWindowPos();
    const ImVec2 titlebar_max(titlebar_min.x + ImGui::GetWindowWidth(), titlebar_min.y + kCustomTitleBarHeight);
    draw_list->AddRectFilled(titlebar_min, titlebar_max, title_bar_color);
    ImU32 titlebar_accent = 0;
    if (CurrentTitleBarAccent(titlebar_accent)) {
      draw_list->AddRectFilledMultiColor(titlebar_min, ImVec2(titlebar_min.x + 380.0f, titlebar_max.y), titlebar_accent,
                                         title_bar_color, title_bar_color, titlebar_accent);
    }

    const ImVec2 logo_min(titlebar_min.x + kTitleBarLogoX,
                          titlebar_min.y + (kCustomTitleBarHeight - kTitleBarLogoSize) * 0.5f);
    DrawFittedImage(FindIconInMap(editor_icons_, TitleBarLogoIconName()),
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
                                 ImVec2(scene_pos.x - 6.0f, separator_y + scene_text_size.y + 1.0f),
                                 TitleBarMutedColor(), 2.0f);
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
        draw_list->AddText(project_pos, TitleBarSecondaryTextColor(), project_name.c_str());
        draw_list->AddRect(ImVec2(project_pos.x - 12.0f, project_pos.y - 5.0f),
                           ImVec2(project_pos.x + project_size.x + 12.0f, project_pos.y + project_size.y + 5.0f),
                           TitleBarBorderColor(), 3.0f);
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
  auto draw_layer_inspection_group = [](const char* label, const bool package_layers) {
    if (!ImGui::BeginMenu(label)) {
      return;
    }
    bool has_items = false;
    for (const auto& layer : ApplicationContext::Get().GetLayers()) {
      if (!HasLayerInspector(layer) || IsPackageLayerInspector(layer) != package_layers) {
        continue;
      }
      has_items = true;
      const auto layer_name = layer->GetLayerName();
      ImGui::Checkbox(layer_name.c_str(), &layer->enable_inspection);
    }
    if (!has_items) {
      ImGui::TextDisabled("No inspectable layers");
    }
    ImGui::EndMenu();
  };
  auto draw_layer_inspection_menu = [&]() {
    if (ImGui::BeginMenu("Layer Inspection")) {
      draw_layer_inspection_group("Built-in/App Layers", false);
      draw_layer_inspection_group("Package Layers", true);
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
      ImGui::BeginDisabled(!show_camera_window);
      panel_menu_item("Main Camera Window Info", show_camera_info);
      ImGui::EndDisabled();
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
    panel_menu_item("Profiler", show_profiler_window);
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
  const float primary_button_left = overlay_pos.x + view_port_size.x * 0.5f - button_size * 0.5f;
  const ImVec2 background_min(primary_button_left - horizontal_padding, overlay_pos.y + 4.0f);
  const ImVec2 background_max(background_min.x + background_width, background_min.y + background_height);
  ImGui::GetWindowDrawList()->AddRectFilled(background_min, background_max, IM_COL32(15, 15, 15, 127), 4.0f);
  if (ImGui::IsMouseHoveringRect(background_min, background_max))
    scene_camera_window_focused_ = false;

  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(item_spacing, 0.0f));
  ImGui::SetCursorScreenPos(
      ImVec2(background_min.x + horizontal_padding, background_min.y + (background_height - button_size) * 0.5f));
  if (DrawPlayControls()) {
    scene_camera_window_focused_ = false;
  }
  ImGui::PopStyleVar();
}

bool EditorLayer::DrawSceneToolsToolbar(const ImVec2& overlay_pos, const ImVec2& view_port_size) {
  constexpr float button_size = 23.0f;
  constexpr float horizontal_padding = 12.0f;
  constexpr float item_spacing = 8.0f;
  constexpr float background_height = 31.0f;
  constexpr float toolbar_gap = 8.0f;
  constexpr float background_width = horizontal_padding * 2.0f + button_size * 4.0f + item_spacing * 3.0f;
  const float playback_left = overlay_pos.x + view_port_size.x * 0.5f - button_size * 0.5f - horizontal_padding;
  const ImVec2 background_min(playback_left - toolbar_gap - background_width, overlay_pos.y + 4.0f);
  const ImVec2 background_max(background_min.x + background_width, background_min.y + background_height);
  ImGui::GetWindowDrawList()->AddRectFilled(background_min, background_max, IM_COL32(15, 15, 15, 127), 4.0f);

  bool interacted = ImGui::IsMouseHoveringRect(background_min, background_max);
  auto draw_button = [&](const char* id, const char* icon, const char* tooltip,
                         const LocalTransformGizmoOperation operation, const bool selected) {
    const bool clicked = DrawViewportImageButton(id, FindIconInMap(editor_icons_, icon), tooltip, selected,
                                                 ImVec2(button_size, button_size));
    interacted |= clicked || ImGui::IsItemHovered() || ImGui::IsItemActive();
    if (clicked) {
      CancelEntityGizmoSession();
      SelectLocalTransformGizmoOperation(operation);
    }
  };

  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(item_spacing, 0.0f));
  ImGui::SetCursorScreenPos(
      ImVec2(background_min.x + horizontal_padding, background_min.y + (background_height - button_size) * 0.5f));
  const bool select_selected = !local_position_selected_ && !local_rotation_selected_ && !local_scale_selected_;
  draw_button("SelectTool", "SelectTool", "Select", LocalTransformGizmoOperation::Select, select_selected);
  ImGui::SameLine();
  draw_button("TranslateTool", "TranslateTool", "Translate", LocalTransformGizmoOperation::Translate,
              local_position_selected_);
  ImGui::SameLine();
  draw_button("RotateTool", "RotateTool", "Rotate", LocalTransformGizmoOperation::Rotate, local_rotation_selected_);
  ImGui::SameLine();
  draw_button("ScaleTool", "ScaleTool", "Scale", LocalTransformGizmoOperation::Scale, local_scale_selected_);
  ImGui::PopStyleVar();
  return interacted;
}

bool EditorLayer::DrawSceneSettingsToolbar(const ImVec2& overlay_pos, const ImVec2& view_port_size) {
  constexpr float button_size = 18.0f;
  constexpr float padding = 4.0f;
  constexpr float background_size = 26.0f;
  const ImVec2 background_min(overlay_pos.x + view_port_size.x - 14.0f - background_size, overlay_pos.y + 4.0f);
  const ImVec2 background_max(background_min.x + background_size, background_min.y + background_size);
  ImGui::GetWindowDrawList()->AddRectFilled(background_min, background_max, IM_COL32(15, 15, 15, 127), 4.0f);
  ImGui::SetCursorScreenPos(ImVec2(background_min.x + padding, background_min.y + padding));
  const bool clicked =
      DrawViewportImageButton("SceneSettings", FindIconInMap(editor_icons_, "SceneSettings"), "Viewport Settings");
  bool interacted = clicked || ImGui::IsMouseHoveringRect(background_min, background_max);
  interacted |= ImGui::IsItemHovered() || ImGui::IsItemActive();
  if (clicked)
    ImGui::OpenPopup("SceneViewportSettings");

  const float popup_width = std::max(1.0f, std::min(360.0f, view_port_size.x - 16.0f));
  const float popup_height = std::max(1.0f, view_port_size.y - background_size - 12.0f);
  ImGui::SetNextWindowPos(ImVec2(std::max(overlay_pos.x + 8.0f, overlay_pos.x + view_port_size.x - 14.0f - popup_width),
                                 background_max.y + 4.0f),
                          ImGuiCond_Appearing);
  ImGui::SetNextWindowSizeConstraints(ImVec2(popup_width, 0.0f), ImVec2(popup_width, popup_height));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));
  if (ImGui::BeginPopup("SceneViewportSettings")) {
    DrawSceneCameraSettingsContents(ApplicationContext::Get().GetLayer<EditorLayer>());
    interacted = true;
    ImGui::EndPopup();
  }
  ImGui::PopStyleVar(2);
  return interacted;
}

void EditorLayer::RequestDefaultEditorLayout() {
  editor_panel_manager_.ResetPanelOpenStatesToDefaults();
  custom_layout_settings_.reset();
  asset_inspector_window_layout_pending_ = false;
  runtime_package_manager_layout_pending_ = false;
  dock_layout_reset_pending_ = true;
}

void EditorLayer::RequestEditorLayout(const EditorLayoutSettings& settings) {
  const auto apply_visibility = [](const std::optional<bool>& value, bool& target) {
    if (value) {
      target = *value;
    }
  };

  apply_visibility(settings.panels.scene, show_scene_window);
  apply_visibility(settings.panels.camera, show_camera_window);
  apply_visibility(settings.panels.scene_info, show_scene_info);
  apply_visibility(settings.panels.camera_info, show_camera_info);
  apply_visibility(settings.panels.entity_explorer, show_entity_explorer_window);
  apply_visibility(settings.panels.entity_inspector, show_entity_inspector_window);
  apply_visibility(settings.panels.console, show_console_window);
  apply_visibility(settings.panels.project, ProjectManager::GetInstance().show_project_window);
  apply_visibility(settings.panels.resources, Resources::GetInstance().show_resources_);
  apply_visibility(settings.panels.profiler, show_profiler_window);
  apply_visibility(settings.panels.runtime_package_manager, show_package_manager_window);
  if (settings.panels.render_layer_inspection) {
    if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>()) {
      render_layer->enable_inspection = *settings.panels.render_layer_inspection;
      if (!*settings.panels.render_layer_inspection) {
      }
    }
  }

  if (project_content_browser_panel_ && settings.project_browser) {
    if (settings.project_browser->hierarchy_width) {
      project_content_browser_panel_->SetHierarchyWidth(*settings.project_browser->hierarchy_width);
    }
    if (settings.project_browser->reveal_folder) {
      project_content_browser_panel_->RevealFolder(*settings.project_browser->reveal_folder);
    }
  }

  custom_layout_settings_ = settings;
  asset_inspector_window_layout_pending_ = settings.asset_inspector_window.has_value();
  runtime_package_manager_layout_pending_ = settings.runtime_package_manager.has_value();
  dock_layout_reset_pending_ = true;
}

void EditorLayer::RequestSceneCameraPreviewWindow(const glm::uvec2& size) {
  show_scene_window = true;
  scene_camera_preview_window_size_ = {std::max(size.x, 1u), std::max(size.y, 1u)};
}

void EditorLayer::SetSceneCameraResolutionOverride(const std::optional<glm::uvec2>& size) {
  scene_camera_resolution_override_ = size;
  if (scene_camera_resolution_override_) {
    scene_camera_resolution_override_->x = std::max(scene_camera_resolution_override_->x, 1u);
    scene_camera_resolution_override_->y = std::max(scene_camera_resolution_override_->y, 1u);
  }
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
    if (custom_layout_settings_ && custom_layout_settings_->dock_layout) {
      BuildCustomEditorDockLayout(dock_space_id, dock_size, *custom_layout_settings_->dock_layout);
    } else {
      BuildDefaultEditorDockLayout(dock_space_id, dock_size);
    }
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
  entity_explorer_current_entities_.push_back(entity);
  std::string title = std::to_string(entity.GetIndex()) + ": ";
  title += scene->GetEntityName(entity);
  const bool enabled = scene->IsEntityEnabled(entity);
  const bool inherited_highlight = IsInheritedSelectionHighlight(entity);
  if (enabled || inherited_highlight) {
    ImGui::PushStyleColor(
        ImGuiCol_Text, inherited_highlight ? ImVec4(1.0f, 0.75f, 0.0f, 1.0f) : ImGui::GetStyleColorVec4(ImGuiCol_Text));
  }
  if (const int index = selected_entity_hierarchy_list_.size() - hierarchy_level - 1;
      !selected_entity_hierarchy_list_.empty() && index >= 0 && index < selected_entity_hierarchy_list_.size() &&
      selected_entity_hierarchy_list_[index] == entity) {
    ImGui::SetNextItemOpen(true);
  }
  const bool opened = ImGui::TreeNodeEx(
      title.c_str(), ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_OpenOnArrow |
                         ImGuiTreeNodeFlags_NoAutoOpenOnLog |
                         (entity_selection_.Contains(entity) ? ImGuiTreeNodeFlags_Selected : ImGuiTreeNodeFlags_None) |
                         (GetSelectedEntity() == entity ? ImGuiTreeNodeFlags_Framed : ImGuiTreeNodeFlags_FramePadding));
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
  if (enabled || inherited_highlight) {
    ImGui::PopStyleColor();
  }
  if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(0) && (IsControlModifierDown() || IsShiftModifierDown())) {
    HandleEntityExplorerSelection(entity);
  } else if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
    HandleEntityExplorerSelection(entity);
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
      NotifyStaticEntitiesChanged(scene, {entity});
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
  suppress_scene_camera_selection_ = false;
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
        if (scene_camera_resolution_override_) {
          scene_camera_resolution_x_ = static_cast<int>(scene_camera_resolution_override_->x);
          scene_camera_resolution_y_ = static_cast<int>(scene_camera_resolution_override_->y);
        } else {
          scene_camera_resolution_x_ = static_cast<int>(view_port_size.x * scene_camera_resolution_multiplier);
          scene_camera_resolution_y_ = static_cast<int>(view_port_size.y * scene_camera_resolution_multiplier);
        }
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
        suppress_scene_camera_selection_ |= DrawSceneToolsToolbar(overlay_pos, view_port_size);
        suppress_scene_camera_selection_ |= DrawSceneSettingsToolbar(overlay_pos, view_port_size);
        constexpr float scene_info_top_offset = 34.0f;
        const auto window_pos =
            ImVec2((corner & 1) ? (overlay_pos.x + view_port_size.x) : overlay_pos.x,
                   (corner & 2) ? (overlay_pos.y + view_port_size.y) : (overlay_pos.y + scene_info_top_offset));

        if (show_scene_info) {
          const auto window_pos_pivot = ImVec2((corner & 1) ? 1.0f : 0.0f, (corner & 2) ? 1.0f : 0.0f);
          ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
          ImGui::SetNextWindowBgAlpha(0.35f);
          constexpr ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoDocking |
                                                    ImGuiWindowFlags_NoSavedSettings |
                                                    ImGuiWindowFlags_NoFocusOnAppearing;
          ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.0f, 8.0f));
          if (constexpr ImGuiChildFlags child_flags = ImGuiChildFlags_AutoResizeY;
              ImGui::BeginChild("Info", ImVec2(240.0f, 0.0f), child_flags, window_flags)) {
            ImGui::Text("Info:");
            ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
            const auto current_frame_index = Platform::GetCurrentFrameIndex();
            DrawRenderCounterSummary(graphics, current_frame_index);
            ImGui::Text("Idle: %.3f", graphics.cpu_wait_time);
            ImGui::Separator();
            if (ImGui::IsMousePosValid()) {
              const auto pos = Input::GetMousePosition();
              ImGui::Text("Mouse: [%.0f,%.0f]", pos.x, pos.y);
            } else {
              ImGui::Text("Mouse: <invalid>");
            }
            uint32_t mode = static_cast<uint32_t>(scene_camera->camera_render_mode);
            if (ImGui::Combo("Render Mode", Camera::GetCameraRenderModeNames(), mode)) {
              scene_camera->camera_render_mode = Camera::NormalizeCameraRenderMode(mode);
              scene_camera->ResetFrameCount();
              suppress_scene_camera_selection_ = true;
            }
            suppress_scene_camera_selection_ |= ImGui::IsItemHovered() || ImGui::IsItemActive();
          }
          ImGui::EndChild();
          ImGui::PopStyleVar();
        }
        ApplyEditorCameraFreeFlyControl(scene_camera_handle_, scene_camera_free_fly_state_,
                                        mouse_scene_window_position_, {view_port_size.x, view_port_size.y},
                                        scene_camera_window_focused_);
      }
#pragma region Gizmos and Entity Selection
      gizmo_using_ = false;
      gizmo_displaying_ = false;
      {
        ImGuizmo::SetOrthographic(false);
        ImGuizmo::SetDrawlist();
        ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, view_port_size.x, view_port_size.y);
        glm::mat4 camera_view = glm::inverse(glm::translate(sceneCameraPosition) * glm::mat4_cast(sceneCameraRotation));
        glm::mat4 camera_projection = scene_camera->GetProjection();
        const auto op = local_position_selected_   ? ImGuizmo::OPERATION::TRANSLATE
                        : local_rotation_selected_ ? ImGuizmo::OPERATION::ROTATE
                                                   : ImGuizmo::OPERATION::SCALE;
        const bool transform_gizmo_selected =
            local_position_selected_ || local_rotation_selected_ || local_scale_selected_;
        if ((!enable_gizmos || !transform_gizmo_selected) && entity_gizmo_session_)
          CancelEntityGizmoSession();
        if (enable_gizmos && transform_gizmo_selected) {
          if (environmental_lighting_gizmo_target_) {
            const auto target = *environmental_lighting_gizmo_target_;
            const auto lighting = target.lighting.lock();
            const auto active_lighting = scene->environmental_lighting.Get<EnvironmentalLighting>();
            const auto reflection_pack = lighting ? lighting->GetReflectionProbePack() : nullptr;
            const auto ddgi_pack = lighting ? lighting->GetDdgiVolumePack() : nullptr;
            const bool local_probe_valid = target.type == EnvironmentalLightingGizmoTargetType::LocalReflectionProbe &&
                                           reflection_pack && target.index < reflection_pack->probes.size() &&
                                           reflection_pack->probes[target.index].stable_id == target.stable_id;
            const bool ddgi_volume_valid = target.type == EnvironmentalLightingGizmoTargetType::DdgiVolume &&
                                           ddgi_pack && target.index < ddgi_pack->volumes.size() &&
                                           ddgi_pack->volumes[target.index].stable_id == target.stable_id;
            if (!lighting || active_lighting != lighting || (!local_probe_valid && !ddgi_volume_valid)) {
              ClearEnvironmentalLightingGizmoTarget();
            } else {
              auto& transform = local_probe_valid ? reflection_pack->probes[target.index].transform
                                                  : ddgi_pack->volumes[target.index].transform;
              const auto authored_pivot =
                  ddgi_volume_valid ? ddgi_pack->volumes[target.index].volume_origin : glm::vec3(0.0f);
              const auto pivot = IsFiniteVector(authored_pivot) ? authored_pivot : glm::vec3(0.0f);
              auto gizmo_transform = CreateAuthoringGizmoTransform(transform, pivot);
              ImGuizmo::Manipulate(glm::value_ptr(camera_view), glm::value_ptr(camera_projection), op, ImGuizmo::LOCAL,
                                   glm::value_ptr(gizmo_transform));
              gizmo_displaying_ = true;
              if (ImGuizmo::IsUsing()) {
                glm::mat4 normalized(1.0f);
                if (TryConvertAuthoringGizmoTransform(gizmo_transform, pivot, normalized) &&
                    !MatricesNear(transform, normalized)) {
                  transform = normalized;
                  if (local_probe_valid)
                    reflection_pack->SetUnsaved();
                  else
                    ddgi_pack->SetUnsaved();
                }
                gizmo_using_ = true;
              }
            }
          }
          const auto selection = entity_selection_.GetSnapshot();
          const auto& participants = ResolveSelectionGizmoParticipants(scene, selection);
          const auto reference = EntityBatchInspector::FindGizmoReference(scene, participants, selection.primary);
          if (entity_gizmo_session_ && (environmental_lighting_gizmo_target_ || !scene->IsEntityValid(reference)))
            CancelEntityGizmoSession("Entity gizmo drag cancelled because its target changed.");
          if (!environmental_lighting_gizmo_target_ && scene->IsEntityValid(reference)) {
            const auto reference_world = scene->GetDataComponent<GlobalTransform>(reference).value;
            Transform reference_transform;
            reference_transform.value = reference_world;
            glm::vec3 reference_position(0.0f), reference_scale(1.0f);
            glm::quat reference_rotation(1.0f, 0.0f, 0.0f, 0.0f);
            reference_transform.Decompose(reference_position, reference_rotation, reference_scale);
            const auto pivot_bound = ResolveSelectionGizmoBound(scene, selection);
            const glm::vec3 pivot = pivot_bound.valid ? pivot_bound.world_bound.Center() : reference_position;
            const auto basis = entity_gizmo_orientation_mode_ == EntityGizmoOrientationMode::Local
                                   ? glm::mat4_cast(reference_rotation)
                                   : glm::mat4(1.0f);
            const glm::mat4 initial_handle =
                entity_gizmo_session_ ? entity_gizmo_session_->handle : glm::translate(pivot) * basis;
            auto manipulated_handle =
                entity_gizmo_session_ ? entity_gizmo_session_->manipulated_handle : initial_handle;
            ImGuizmo::Manipulate(
                glm::value_ptr(camera_view), glm::value_ptr(camera_projection), op,
                entity_gizmo_orientation_mode_ == EntityGizmoOrientationMode::Local ? ImGuizmo::LOCAL : ImGuizmo::WORLD,
                glm::value_ptr(manipulated_handle));
            gizmo_displaying_ = true;
            if (ImGuizmo::IsUsing()) {
              if (!entity_gizmo_session_)
                BeginEntityGizmoSession(scene, initial_handle, participants, reference, static_cast<int>(op));
              ApplyEntityGizmoSession(scene, manipulated_handle);
              gizmo_using_ = true;
            } else if (entity_gizmo_session_) {
              CancelEntityGizmoSession();
            }
          }
          if (!entity_gizmo_message_.empty())
            ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.2f, 1.0f), "%s", entity_gizmo_message_.c_str());
        }
        const ImVec2 view_gizmo_position = ImGui::GetWindowPos();
        ImGuizmo::ViewManipulate(glm::value_ptr(camera_view), 1.0f, view_gizmo_position, ImVec2(96, 96), 0);
        suppress_scene_camera_selection_ |= ImGuizmo::IsOver() || ImGuizmo::IsUsing();
        GlobalTransform gl;
        gl.value = glm::inverse(camera_view);
        sceneCameraRotation = gl.GetRotation();
      }
#pragma endregion

      if (scene_camera_window_focused_ && !ImGui::GetIO().WantTextInput && !gizmo_using_ &&
          Input::GetKey(editor_camera_control_key_bindings.focus_selection_key) == Input::KeyActionType::Press)
        FocusSceneCameraOnSelection(scene, scene_camera);

      if (scene_camera_window_focused_ && !GetLockEntitySelection() &&
          Input::GetKey(GLFW_KEY_ESCAPE) == Input::KeyActionType::Press) {
        SetSelectedEntity({});
      }
      if (scene_camera_window_focused_ && !GetLockEntitySelection() && !gizmo_using_ &&
          !suppress_scene_camera_selection_ && Input::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press &&
          !(mouse_scene_window_position_.x < 0 || mouse_scene_window_position_.y < 0 ||
            mouse_scene_window_position_.x > view_port_size.x || mouse_scene_window_position_.y > view_port_size.y)) {
        pending_viewport_selection_ =
            PendingViewportSelection{scene_camera, mouse_scene_window_position_, IsControlModifierDown()};
      }

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
        AspectFitRect main_camera_fit_rect;
        if (main_camera && main_camera->Rendered()) {
          main_camera_fit_rect = CalculateAspectFitRect(view_port_size, main_camera->GetSize());
          ImGui::SetCursorScreenPos(
              {overlay_pos.x + main_camera_fit_rect.offset.x, overlay_pos.y + main_camera_fit_rect.offset.y});
          ImGui::Image(main_camera->GetRenderTexture()->GetColorImTextureId(),
                       ImVec2(main_camera_fit_rect.size.x, main_camera_fit_rect.size.y), ImVec2(0, 1), ImVec2(1, 0));
          CameraWindowDragAndDrop();
          ImGui::SetCursorScreenPos(overlay_pos);
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
          ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.0f, 8.0f));
          if (constexpr ImGuiChildFlags child_flags = ImGuiChildFlags_AutoResizeY;
              ImGui::BeginChild("Render Info", ImVec2(340.0f, 0.0f), child_flags, window_flags)) {
            ImGui::Text("Info & Settings");
            ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
            ImGui::PushItemWidth(100);
            ImGui::Checkbox("Auto resize", &main_camera_allow_auto_resize);
            if (main_camera_allow_auto_resize) {
              ImGui::DragFloat("Resolution multiplier", &main_camera_resolution_multiplier_, 0.1f, 0.1f, 4.0f);
            }
            ImGui::PopItemWidth();
            const auto current_frame_index = Platform::GetCurrentFrameIndex();
            DrawRenderCounterSummary(graphics, current_frame_index);
            ImGui::Separator();
            if (ImGui::IsMousePosValid()) {
              const auto pos = Input::GetMousePosition();
              ImGui::Text("Mouse Pos: (%.1f,%.1f)", pos.x, pos.y);
            } else {
              ImGui::Text("Mouse Pos: <invalid>");
            }
            uint32_t mode = static_cast<uint32_t>(main_camera->camera_render_mode);
            if (ImGui::Combo("Render Mode", Camera::GetCameraRenderModeNames(), mode)) {
              main_camera->camera_render_mode = Camera::NormalizeCameraRenderMode(mode);
              main_camera->ResetFrameCount();
            }
          }
          ImGui::EndChild();
          ImGui::PopStyleVar();
        }

        if (main_camera_window_focused_ && !GetLockEntitySelection() &&
            Input::GetKey(GLFW_KEY_ESCAPE) == Input::KeyActionType::Press) {
          SetSelectedEntity(Entity());
        }
        if (!ApplicationContext::Get().IsPlaying() && main_camera_window_focused_ && !GetLockEntitySelection() &&
            Input::GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press && main_camera) {
          glm::vec2 texture_mouse_position;
          if (TryMapAspectFitMouseToTexture(main_camera_fit_rect, main_camera->GetSize(), mouse_camera_window_position_,
                                            texture_mouse_position)) {
            pending_viewport_selection_ =
                PendingViewportSelection{main_camera, texture_mouse_position, IsControlModifierDown()};
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

void EditorLayer::SetConsoleMessageFilters(const bool info, const bool warning, const bool error) {
  enable_console_logs_ = info;
  enable_console_warnings_ = warning;
  enable_console_errors_ = error;
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
  if (scene_camera_handle_.GetValue() == 0) {
    scene_camera_handle_ = camera->GetHandle();
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

bool EditorLayer::ApplyEditorCameraFreeFlyControl(const Handle& camera_handle, EditorCameraFreeFlyState& state,
                                                  const glm::vec2& mouse_position, const glm::vec2& viewport_size,
                                                  const bool window_focused) {
  if (!window_focused) {
    ResetEditorCameraFreeFlyState(state);
    return false;
  }

  const auto search = editor_cameras_.find(camera_handle);
  if (search == editor_cameras_.end()) {
    ResetEditorCameraFreeFlyState(state);
    return false;
  }

  if (lock_camera) {
    ResetEditorCameraFreeFlyState(state);
    return false;
  }

  const auto& key_bindings = editor_camera_control_key_bindings;
  const bool mouse_in_viewport = mouse_position.x >= 0.0f && mouse_position.y >= 0.0f &&
                                 mouse_position.x <= viewport_size.x && mouse_position.y <= viewport_size.y;
  const bool mouse_drag = mouse_in_viewport && GetKey(key_bindings.rotate_mouse_button) == Input::KeyActionType::Hold;

  float x_offset = 0.0f;
  float y_offset = 0.0f;
  if (!state.was_dragging) {
    state.previous_mouse_x = mouse_position.x;
    state.previous_mouse_y = mouse_position.y;
  }
  if (mouse_drag) {
    x_offset = mouse_position.x - state.previous_mouse_x;
    y_offset = mouse_position.y - state.previous_mouse_y;
    state.previous_mouse_x = mouse_position.x;
    state.previous_mouse_y = mouse_position.y;
    state.was_dragging = true;
  } else {
    state.was_dragging = false;
  }

  auto& editor_camera = search->second;
  glm::vec3 front = editor_camera.rotation * glm::vec3(0, 0, -1);
  const glm::vec3 right = editor_camera.rotation * glm::vec3(1, 0, 0);
  const float raw_delta_time = static_cast<float>(ApplicationContext::Get().GetTimes().DeltaTime());
  const float delta_time = std::isfinite(raw_delta_time) && raw_delta_time > 0.0f ? raw_delta_time : 0.0f;
  const float max_move_speed = ClampFiniteNonnegative(velocity, 0.0f);
  const float acceleration_time = ClampFiniteNonnegative(camera_control_acceleration_time, 0.25f);
  const float deceleration_time = ClampFiniteNonnegative(camera_control_deceleration_time, 0.25f);
  bool changed = false;

  glm::vec3 target_direction(0.0f);
  if (mouse_drag) {
    if (GetKey(key_bindings.move_forward_key) == Input::KeyActionType::Hold) {
      target_direction += front;
    }
    if (GetKey(key_bindings.move_backward_key) == Input::KeyActionType::Hold) {
      target_direction -= front;
    }
    if (GetKey(key_bindings.move_left_key) == Input::KeyActionType::Hold) {
      target_direction -= right;
    }
    if (GetKey(key_bindings.move_right_key) == Input::KeyActionType::Hold) {
      target_direction += right;
    }
    if (GetKey(key_bindings.move_up_key) == Input::KeyActionType::Hold) {
      target_direction.y += 1.0f;
    }
    if (GetKey(key_bindings.move_down_key) == Input::KeyActionType::Hold) {
      target_direction.y -= 1.0f;
    }
  }

  glm::vec3 target_move_velocity(0.0f);
  if (glm::dot(target_direction, target_direction) > glm::epsilon<float>() && max_move_speed > 0.0f) {
    target_move_velocity = glm::normalize(target_direction) * max_move_speed;
  }

  state.smoothed_move_velocity = SanitizeVelocity(state.smoothed_move_velocity);
  const float current_speed = glm::length(state.smoothed_move_velocity);
  const float target_speed = glm::length(target_move_velocity);
  const float move_response_time = target_speed > current_speed ? acceleration_time : deceleration_time;
  const float reference_speed = std::max({max_move_speed, current_speed, target_speed});
  if (move_response_time <= glm::epsilon<float>()) {
    state.smoothed_move_velocity = target_move_velocity;
  } else if (reference_speed > glm::epsilon<float>()) {
    state.smoothed_move_velocity = MoveTowards(state.smoothed_move_velocity, target_move_velocity,
                                               reference_speed * delta_time / move_response_time);
  }

  if (delta_time > 0.0f &&
      glm::dot(state.smoothed_move_velocity, state.smoothed_move_velocity) > glm::epsilon<float>()) {
    editor_camera.position += state.smoothed_move_velocity * delta_time;
    changed = true;
  }

  const float target_look_response = mouse_drag ? 1.0f : 0.0f;
  if (!std::isfinite(state.look_response)) {
    state.look_response = 0.0f;
  }
  const float look_response_time = target_look_response > state.look_response ? acceleration_time : deceleration_time;
  state.look_response =
      MoveTowards(state.look_response, target_look_response, CameraControlResponseStep(look_response_time, delta_time));

  const float look_sensitivity = ClampFiniteNonnegative(sensitivity, 0.0f) * state.look_response;
  if ((x_offset != 0.0f || y_offset != 0.0f) && look_sensitivity > glm::epsilon<float>()) {
    front = glm::rotate(front, glm::radians(-x_offset * look_sensitivity), glm::vec3(0, 1, 0));
    const glm::vec3 camera_right = glm::normalize(glm::cross(front, glm::vec3(0.0f, 1.0f, 0.0f)));
    if ((front.y < 0.99f && y_offset < 0.0f) || (front.y > -0.99f && y_offset > 0.0f)) {
      front = glm::rotate(front, glm::radians(-y_offset * look_sensitivity), camera_right);
    }
    const glm::vec3 up = glm::normalize(glm::cross(camera_right, front));
    editor_camera.rotation = glm::quatLookAt(front, up);
    changed = true;
  }
  return changed;
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
  return entity_selection_.GetPrimary();
}

const EntitySelection& EditorLayer::GetEntitySelection() const {
  return entity_selection_;
}

EntitySelection::Snapshot EditorLayer::GetEntitySelectionSnapshot() const {
  return entity_selection_.GetSnapshot();
}

EntitySelectionHighlight::Snapshot EditorLayer::GetEntitySelectionHighlightSnapshot() const {
  return entity_selection_highlight_.GetSnapshot();
}

glm::mat4 EditorLayer::ComposeAuthoringTransform(const glm::vec3& position, const glm::vec3& rotation_degrees,
                                                 const glm::vec3& scale) {
  return glm::translate(position) * glm::mat4_cast(glm::quat(glm::radians(rotation_degrees))) * glm::scale(scale);
}

bool EditorLayer::TryNormalizeAuthoringTransform(const glm::mat4& transform, glm::mat4& normalized, glm::vec3& position,
                                                 glm::vec3& rotation_degrees, glm::vec3& scale) {
  for (glm::length_t column = 0; column < 4; ++column) {
    for (glm::length_t row = 0; row < 4; ++row) {
      if (!std::isfinite(transform[column][row])) {
        return false;
      }
    }
  }
  const auto determinant = glm::determinant(glm::mat3(transform));
  Transform decomposed_transform;
  decomposed_transform.value = transform;
  if (!std::isfinite(determinant) || glm::abs(determinant) <= 1.0e-8f ||
      !decomposed_transform.Decompose(position, rotation_degrees, scale) || !IsFiniteVector(position) ||
      !IsFiniteVector(rotation_degrees) || !IsFiniteVector(scale) ||
      !glm::all(glm::greaterThan(glm::abs(scale), glm::vec3(1.0e-8f)))) {
    return false;
  }
  rotation_degrees = glm::degrees(rotation_degrees);
  normalized = ComposeAuthoringTransform(position, rotation_degrees, scale);
  return true;
}

glm::mat4 EditorLayer::CreateAuthoringGizmoTransform(const glm::mat4& transform, const glm::vec3& local_pivot) {
  return transform * glm::translate(local_pivot);
}

bool EditorLayer::TryConvertAuthoringGizmoTransform(const glm::mat4& gizmo_transform, const glm::vec3& local_pivot,
                                                    glm::mat4& transform) {
  glm::vec3 position(0.0f);
  glm::vec3 rotation_degrees(0.0f);
  glm::vec3 scale(1.0f);
  return TryNormalizeAuthoringTransform(gizmo_transform * glm::translate(-local_pivot), transform, position,
                                        rotation_degrees, scale);
}

void EditorLayer::SetEnvironmentalLightingGizmoTarget(const std::shared_ptr<EnvironmentalLighting>& lighting,
                                                      const EnvironmentalLightingGizmoTargetType type,
                                                      const size_t index, const uint64_t stable_id) {
  if (!lighting) {
    ClearEnvironmentalLightingGizmoTarget();
    return;
  }
  SetSelectedEntity({});
  environmental_lighting_gizmo_target_ =
      EnvironmentalLightingGizmoTarget{lighting, lighting->GetHandle(), type, index, stable_id};
}

bool EditorLayer::IsEnvironmentalLightingGizmoTarget(const EnvironmentalLighting& lighting,
                                                     const EnvironmentalLightingGizmoTargetType type,
                                                     const size_t index, const uint64_t stable_id) const {
  if (!environmental_lighting_gizmo_target_) {
    return false;
  }
  const auto owner = environmental_lighting_gizmo_target_->lighting.lock();
  return owner.get() == &lighting && environmental_lighting_gizmo_target_->type == type &&
         environmental_lighting_gizmo_target_->index == index &&
         environmental_lighting_gizmo_target_->stable_id == stable_id;
}

bool EditorLayer::IsEnvironmentalLightingGizmoTarget(const EnvironmentalLightingGizmoTargetType type,
                                                     const Handle& asset_handle, const uint64_t stable_id) const {
  return environmental_lighting_gizmo_target_ && environmental_lighting_gizmo_target_->type == type &&
         environmental_lighting_gizmo_target_->asset_handle == asset_handle &&
         environmental_lighting_gizmo_target_->stable_id == stable_id &&
         !environmental_lighting_gizmo_target_->lighting.expired();
}

void EditorLayer::ClearEnvironmentalLightingGizmoTarget() {
  environmental_lighting_gizmo_target_.reset();
}

void EditorLayer::ClearEnvironmentalLightingGizmoTarget(const Handle& asset_handle) {
  if (environmental_lighting_gizmo_target_ && environmental_lighting_gizmo_target_->asset_handle == asset_handle) {
    ClearEnvironmentalLightingGizmoTarget();
  }
}

void EditorLayer::SetSelectedEntity(const Entity& entity, const bool open_menu) {
  ClearEnvironmentalLightingGizmoTarget();
  const auto scene = GetScene();
  entity_selection_.BindScene(scene);
  const EntitySelection::RequestOptions options{
      EntitySelection::RequestSource::Programmatic, EntitySelection::AnchorPolicy::Clear, {}};
  const auto result = entity.GetIndex() == 0 ? entity_selection_.Clear(EntitySelection::RequestSource::Programmatic)
                                             : entity_selection_.Replace(entity, options);
  if (result != EntitySelection::Result::Changed)
    return;
  selected_entity_hierarchy_list_.clear();
  if (!scene) {
    return;
  }
  if (entity_selection_.Empty()) {
    return;
  }
  if (!open_menu)
    return;
  auto walker = entity_selection_.GetPrimary();
  while (walker.GetIndex() != 0) {
    selected_entity_hierarchy_list_.push_back(walker);
    walker = scene->GetParent(walker);
  }
}

bool EditorLayer::GetLockEntitySelection() const {
  return entity_selection_.IsLocked();
}

void EditorLayer::SetLockEntitySelection(const bool value) {
  entity_selection_.SetLocked(value, EntitySelection::RequestSource::Programmatic);
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
      SetSelectedEntity(entity);
    }
  } else {
    const std::string none_title = "None##" + name;
    ImGui::Button(none_title.c_str());
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

Entity EditorLayer::MouseEntitySelection(const std::shared_ptr<Camera>& target_camera,
                                         const glm::vec2& mouse_position) const {
  Entity ret_val;
  const auto& g_buffer_utility = target_camera->GetGBufferUtilityImage();
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
    Platform::WaitForFrameSubmissions("Entity Picking Readback Fence Wait");
    entity_index_read_buffer_->CopyFromImage(*g_buffer_utility, image_copy);
    float val = -1;
    switch (Platform::Constants::g_buffer_utility) {
      case VK_FORMAT_R32G32B32A32_SFLOAT: {
        const auto* ptr = static_cast<float*>(mapped_entity_index_data_);
        val = glm::round(ptr[0]);
        break;
      }
      case VK_FORMAT_R16G16B16A16_SFLOAT: {
        const auto* ptr = static_cast<glm::detail::hdata*>(mapped_entity_index_data_);
        val = glm::round(glm::detail::toFloat32(ptr[0]));
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
  const auto ptr = target.Peek<IAsset>();
  const auto asset_handle = target.GetAssetHandle();
  bool status_changed = false;
  ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_Header));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyleColorVec4(ImGuiCol_HeaderHovered));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetStyleColorVec4(ImGuiCol_HeaderActive));
  if (ptr || asset_handle.GetValue() != 0) {
    ImGui::Button((GetAssetRefDisplayName(target) + GetAssetRefImGuiTag(target)).c_str());
    const bool open_requested = ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0);
    DraggableAssetRef(target);
    if (modifiable) {
      status_changed = ptr ? RenameAsset(ptr) : false;
      status_changed = Remove(target) || status_changed;
    }
    if (!status_changed && open_requested) {
      if (ptr) {
        OpenAssetInspector(ptr);
      } else {
        OpenAssetInspector(asset_handle);
      }
    }
  } else {
    const std::string none_title = "None##" + name;
    ImGui::Button(none_title.c_str());
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
  const bool light_theme = UsesLightTheme();
  const ImU32 slot_color = light_theme ? IM_COL32(236, 218, 198, 255) : IM_COL32(138, 103, 72, 255);
  ImGui::PushStyleColor(ImGuiCol_Button, ImGui::ColorConvertU32ToFloat4(slot_color));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                        ImGui::ColorConvertU32ToFloat4(MultiplyColor(slot_color, light_theme ? 0.95f : 1.12f)));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                        ImGui::ColorConvertU32ToFloat4(MultiplyColor(slot_color, light_theme ? 0.90f : 1.20f)));
  if (ptr) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    ImGui::Button(scene->GetEntityName(ptr->GetOwner()).c_str());
    const std::string tag = "##" + ptr->GetTypeName() + std::to_string(ptr->GetHandle());
    DraggablePrivateComponent(ptr);
    if (modifiable) {
      status_changed = Remove(target);
    }
  } else {
    const std::string none_title = "None##" + name;
    ImGui::Button(none_title.c_str());
  }
  ImGui::PopStyleColor(3);
  status_changed = UnsafeDroppablePrivateComponent(target, acceptable_type_names) || status_changed;
  return status_changed;
}

void EditorLayer::LoadIcons() {
  const auto default_resources = Resources::GetDefaultResourcesPath();
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
  load_icon("TitleBarLogoWhite", default_resources / "Editor/TitleBar/EvoEngine64White.png");
  load_icon("TitleBarLogoBlack", default_resources / "Icons/EvoEngine64.png");
  load_icon("WindowMinimize", default_resources / "Editor/Window/Minimize.png");
  load_icon("WindowMaximize", default_resources / "Editor/Window/Maximize.png");
  load_icon("WindowRestore", default_resources / "Editor/Window/Restore.png");
  load_icon("WindowClose", default_resources / "Editor/Window/Close.png");
  load_icon("PlayButton", default_resources / "Editor/Viewport/Play.png");
  load_icon("PauseButton", default_resources / "Editor/Viewport/Pause.png");
  load_icon("StopButton", default_resources / "Editor/Viewport/Stop.png");
  load_icon("StepButton", default_resources / "Editor/Viewport/Simulate.png");
  load_icon("SelectTool", default_resources / "Editor/Generic/Pointer.png");
  load_icon("TranslateTool", default_resources / "Editor/Viewport/MoveTool.png");
  load_icon("RotateTool", default_resources / "Editor/Viewport/RotateTool.png");
  load_icon("ScaleTool", default_resources / "Editor/Viewport/ScaleTool.png");
  load_icon("SceneSettings", default_resources / "Editor/Generic/Gear.png");
  load_icon("ComponentGeneric", default_resources / "Editor/Components/Generic.png");
  load_icon("ComponentTransform", default_resources / "Editor/Components/Transform.png");
  load_icon("DataComponents", default_resources / "Editor/Components/DataComponents.png");
  RegisterComponentIcon<Transform>(FindIconInMap(editor_icons_, "ComponentTransform"));
  load_icon("BackButton", default_resources / "Editor/Navigation/back.png");
  load_icon("LeftButton", default_resources / "Editor/Navigation/left.png");
  load_icon("RightButton", default_resources / "Editor/Navigation/right.png");
  load_icon("RefreshButton", default_resources / "Editor/Navigation/refresh.png");
  load_icon("InfoButton", default_resources / "Editor/Console/InfoButton.png");
  load_icon("ErrorButton", default_resources / "Editor/Console/ErrorButton.png");
  load_icon("WarningButton", default_resources / "Editor/Console/WarningButton.png");
}

void EditorLayer::CameraWindowDragAndDrop() const {
  if (AssetRef asset_ref; UnsafeDroppableAsset(
          asset_ref, {"Scene", "Prefab", "Mesh", "Strands", "GaussianSplat", "Cubemap", "EnvironmentalMap"})) {
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
    } else if (asset->GetTypeName() == "GaussianSplat") {
      const auto entity = scene->CreateEntity(asset->GetTitle());
      const auto gaussian_splat_renderer = scene->GetOrSetPrivateComponent<GaussianSplatRenderer>(entity).lock();
      gaussian_splat_renderer->gaussian_splat.Set<GaussianSplat>(std::dynamic_pointer_cast<GaussianSplat>(asset));
    } else if (asset->GetTypeName() == "EnvironmentalMap") {
      auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>();
      if (!lighting || !lighting->IsTemporary()) {
        lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
        scene->environmental_lighting = lighting;
      }
      lighting->indirect_environment_source.kind =
          EnvironmentalLighting::IndirectEnvironmentSourceKind::EnvironmentalMap;
      lighting->indirect_environment_source.environmental_map = std::dynamic_pointer_cast<EnvironmentalMap>(asset);
    } else if (asset->GetTypeName() == "Cubemap") {
      const auto main_camera = scene->main_camera.Get<Camera>();
      main_camera->camera_settings.background_source = Camera::BackgroundSource::Cubemap;
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
  transition_preserves_world_up_ = false;
  lock_camera = true;
}

bool EditorLayer::FocusSceneCameraOnSelection(const std::shared_ptr<Scene>& scene,
                                              const std::shared_ptr<Camera>& scene_camera) {
  if (!scene || !scene_camera)
    return false;
  const auto selection = entity_selection_.GetSnapshot();
  ResolveSelectionGizmoParticipants(scene, selection);
  const auto selection_bound = ResolveSelectionGizmoBound(scene, selection);
  if (!selection_bound.valid)
    return false;

  const auto& editor_camera = editor_cameras_.at(scene_camera_handle_);
  glm::quat target_rotation;
  glm::vec3 target_position;
  if (!CalculateFocusCameraTransform(*scene_camera, selection_bound.world_bound, editor_camera.rotation,
                                     editor_camera.position, target_rotation, target_position))
    return false;
  MoveCamera(target_rotation, target_position);
  transition_preserves_world_up_ = true;
  return true;
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

void EditorLayer::SelectLocalTransformGizmoOperation(const LocalTransformGizmoOperation operation) {
  local_position_selected_ = operation == LocalTransformGizmoOperation::Translate;
  local_rotation_selected_ = operation == LocalTransformGizmoOperation::Rotate;
  local_scale_selected_ = operation == LocalTransformGizmoOperation::Scale;
}
