#include "RuntimeGuiContext.hpp"
#include <imgui_internal.h>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <sstream>

using namespace evo_engine;

namespace {
std::string EncodeId(const char* value) {
  std::string result;
  const auto* identity = std::strstr(value, "###");
  value = identity ? identity + 3 : value;
  for (const unsigned char c : std::string(value)) {
    if (std::isalnum(c) || c == ' ' || c == '_' || c == '-')
      result += c;
    else {
      result += '%';
      result += "0123456789ABCDEF"[c >> 4];
      result += "0123456789ABCDEF"[c & 15];
    }
  }
  return result;
}
void ClipCommands(ImDrawList& list, const ImRect& bounds) {
  for (auto& command : list.CmdBuffer) {
    command.ClipRect.x = std::max(command.ClipRect.x, bounds.Min.x);
    command.ClipRect.y = std::max(command.ClipRect.y, bounds.Min.y);
    command.ClipRect.z = std::max(command.ClipRect.x, std::min(command.ClipRect.z, bounds.Max.x));
    command.ClipRect.w = std::max(command.ClipRect.y, std::min(command.ClipRect.w, bounds.Max.y));
  }
}
}  // namespace

void RuntimeGuiContext::PrepareFrame() {
  for (auto* window : suppressed_mouse_)
    window->Flags &= ~ImGuiWindowFlags_NoMouseInputs;
  suppressed_mouse_.clear();
  // FindHoveredWindow adds resize padding outside OuterRectClipped. Suppress those hits before NewFrame.
  auto mouse = ImGui::GetIO().MousePos;
  for (const auto& event : GImGui->InputEventsQueue)
    if (event.Type == ImGuiInputEventType_MousePos)
      mouse = {event.MousePos.PosX, event.MousePos.PosY};
  const bool inside = ImRect(origin_, {origin_.x + size_.x, origin_.y + size_.y}).Contains(mouse);
  bool occluded = false;
  bool above_host = false;
  for (const auto* candidate : GImGui->Windows) {
    if (host_ && candidate == host_->RootWindow)
      above_host = true;
    if (above_host && host_ && candidate->RootWindow != host_->RootWindow && !Owns(candidate) && candidate->WasActive &&
        !candidate->Hidden && !(candidate->Flags & ImGuiWindowFlags_NoMouseInputs) &&
        candidate->OuterRectClipped.Contains(mouse))
      occluded = true;
  }
  if (!inside || occluded || active_frame_ != ImGui::GetFrameCount())
    for (auto* window : owned_)
      if (!(window->Flags & ImGuiWindowFlags_NoMouseInputs)) {
        suppressed_mouse_.insert(window);
        window->Flags |= ImGuiWindowFlags_NoMouseInputs;
      }
}

void RuntimeGuiContext::BeginView(const ImVec2 origin, const ImVec2 size, const uint64_t owner) {
  previous_origin_ = origin_;
  origin_ = origin;
  size_ = size;
  host_ = ImGui::GetCurrentWindow();
  viewport_ = host_->Viewport;
  owner_ = std::to_string(owner);
  asset_.clear();
  window_paths_.clear();
  roots_.clear();
  owned_.clear();
  active_frame_ = ImGui::GetFrameCount();
  camera_draw_frames_[0] = camera_draw_frames_[1] = -1;
}

ImDrawList* RuntimeGuiContext::GetCameraDrawList(const int layer) {
  IM_ASSERT(active_frame_ == ImGui::GetFrameCount() && GImGui->WithinFrameScope);
  auto& list = camera_draw_lists_[layer];
  if (!list)
    list = std::make_unique<ImDrawList>(ImGui::GetDrawListSharedData());
  if (camera_draw_frames_[layer] != active_frame_) {
    list->_ResetForNewFrame();
    list->PushTexture(ImGui::GetIO().Fonts->TexRef);
    list->PushClipRect(origin_, {origin_.x + size_.x, origin_.y + size_.y});
    camera_draw_frames_[layer] = active_frame_;
  }
  return list.get();
}

bool RuntimeGuiContext::BeginWindow(const char* title, bool* open, ImGuiWindowFlags flags) {
  const auto path = asset_ + EncodeId(title);
  auto& layout = layouts_[owner_][path];
  const std::string name = std::string(title) + "###RuntimeGui/" + owner_ + "/" + path;
  const auto window_origin = ImTrunc(origin_);
  ImVec2 position{window_origin.x + 12, window_origin.y + 12};
  if (const auto* existing = ImGui::FindWindowByName(name.c_str())) {
    position = existing->LastFrameActive == ImGui::GetFrameCount() - 1
                   ? ImVec2(existing->Pos.x + window_origin.x - ImTrunc(previous_origin_.x),
                            existing->Pos.y + window_origin.y - ImTrunc(previous_origin_.y))
                   : ImVec2(window_origin.x + layout.position.x, window_origin.y + layout.position.y);
    position.x = std::clamp(position.x, origin_.x, origin_.x + std::max(0.0f, size_.x - existing->Size.x));
    position.y = std::clamp(position.y, origin_.y, origin_.y + std::max(0.0f, size_.y - existing->Size.y));
  }
  if (layout.apply && layout.size.x > 0 && layout.size.y > 0) {
    position = {window_origin.x + layout.position.x, window_origin.y + layout.position.y};
    ImGui::SetNextWindowSize(layout.size);
    ImGui::SetNextWindowCollapsed(layout.collapsed);
  }
  position.x = std::clamp(position.x, origin_.x, origin_.x + std::max(0.0f, size_.x - 32));
  position.y = std::clamp(position.y, origin_.y, origin_.y + std::max(0.0f, size_.y - 32));
  ImGui::SetNextWindowViewport(viewport_->ID);
  ImGui::SetNextWindowPos(position);
  if (!(layout.apply && layout.size.x > 0 && layout.size.y > 0))
    ImGui::SetNextWindowSize({std::min(size_.x, 300.0f), std::min(size_.y, 220.0f)}, ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSizeConstraints({1, 1}, size_);
  const bool visible =
      ImGui::Begin(name.c_str(), open, flags | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings);
  layout.window = ImGui::GetCurrentWindow();
  layout.apply = false;
  window_paths_.push_back(path);
  roots_.insert(ImGui::GetCurrentWindow()->ID);
  ImGui::PushClipRect(origin_, {origin_.x + size_.x, origin_.y + size_.y}, true);
  return visible;
}

void RuntimeGuiContext::EndWindow() {
  window_paths_.pop_back();
  ImGui::PopClipRect();
  ImGui::End();
}

void RuntimeGuiContext::PrepareTransient() {
  ImGui::SetNextWindowViewport(viewport_->ID);
  ImGui::SetNextWindowSizeConstraints({1, 1}, size_);
}

void RuntimeGuiContext::ConstrainTransient() {
  auto* window = ImGui::GetCurrentWindow();
  const ImRect bounds(origin_, {origin_.x + size_.x, origin_.y + size_.y});
  const ImVec2 position{std::clamp(window->Pos.x, bounds.Min.x, std::max(bounds.Min.x, bounds.Max.x - window->Size.x)),
                        std::clamp(window->Pos.y, bounds.Min.y, std::max(bounds.Min.y, bounds.Max.y - window->Size.y))};
  const ImVec2 previous = window->Pos;
  ImGui::SetWindowPos(window, position, ImGuiCond_Always);
  const ImVec2 delta{window->Pos.x - previous.x, window->Pos.y - previous.y};
  // Begin has emitted decorations, but no caller widgets or children have run yet.
  for (auto* rect : {&window->OuterRectClipped, &window->InnerRect, &window->InnerClipRect, &window->WorkRect,
                     &window->ParentWorkRect, &window->ContentRegionRect})
    rect->Translate(delta);
  window->DC.CursorPosPrevLine.x += delta.x;
  window->DC.CursorPosPrevLine.y += delta.y;
  for (auto& vertex : window->DrawList->VtxBuffer) {
    vertex.pos.x += delta.x;
    vertex.pos.y += delta.y;
  }
  for (auto& command : window->DrawList->CmdBuffer) {
    command.ClipRect.x += delta.x;
    command.ClipRect.y += delta.y;
    command.ClipRect.z += delta.x;
    command.ClipRect.w += delta.y;
  }
  ImGui::PopClipRect();
  ImGui::PushClipRect(window->InnerClipRect.Min, window->InnerClipRect.Max, false);
  ImGui::PushClipRect(bounds.Min, bounds.Max, true);
}

std::string RuntimeGuiContext::PopupName(const char* name) const {
  return std::string(name) + "###RuntimeGui/" + owner_ + "/" + asset_ + "Popup/" + std::to_string(ImGui::GetID(name));
}

void RuntimeGuiContext::OpenPopup(const char* name, const ImGuiPopupFlags flags) {
  ImGui::OpenPopup(PopupName(name).c_str(), flags);
}

bool RuntimeGuiContext::BeginMenu(const char* label, const bool enabled) {
  PrepareTransient();
  if (!ImGui::BeginMenu(PopupName(label).c_str(), enabled))
    return false;
  ConstrainTransient();
  window_paths_.push_back((window_paths_.empty() ? asset_ : window_paths_.back()) + "/Menu/" + EncodeId(label));
  return true;
}

void RuntimeGuiContext::EndMenu() {
  window_paths_.pop_back();
  ImGui::PopClipRect();
  ImGui::EndMenu();
}

bool RuntimeGuiContext::BeginPopup(const char* id, const ImGuiWindowFlags flags) {
  PrepareTransient();
  if (!ImGui::BeginPopup(PopupName(id).c_str(), flags))
    return false;
  roots_.insert(ImGui::GetCurrentWindow()->ID);
  ConstrainTransient();
  window_paths_.push_back((window_paths_.empty() ? asset_ : window_paths_.back()) + "/Popup/" + EncodeId(id));
  return true;
}

bool RuntimeGuiContext::BeginPopupModal(const char* title, bool* open, const ImGuiWindowFlags flags) {
  const auto name = PopupName(title);
  PrepareTransient();
  if (const auto* window = ImGui::FindWindowByName(name.c_str()))
    ImGui::SetNextWindowPos(
        {std::clamp(window->Pos.x, origin_.x, origin_.x + std::max(0.0f, size_.x - window->Size.x)),
         std::clamp(window->Pos.y, origin_.y, origin_.y + std::max(0.0f, size_.y - window->Size.y))});
  else
    ImGui::SetNextWindowPos({origin_.x + size_.x / 2, origin_.y + size_.y / 2}, ImGuiCond_Always, {0.5f, 0.5f});
  if (!ImGui::BeginPopupModal(name.c_str(), open, flags | ImGuiWindowFlags_NoSavedSettings))
    return false;
  roots_.insert(ImGui::GetCurrentWindow()->ID);
  ConstrainTransient();
  window_paths_.push_back((window_paths_.empty() ? asset_ : window_paths_.back()) + "/Modal/" + EncodeId(title));
  return true;
}

void RuntimeGuiContext::EndPopup() {
  window_paths_.pop_back();
  ImGui::PopClipRect();
  ImGui::EndPopup();
}

bool RuntimeGuiContext::BeginTooltip() {
  PrepareTransient();
  if (!ImGui::BeginTooltip())
    return false;
  ConstrainTransient();
  window_paths_.push_back((window_paths_.empty() ? asset_ : window_paths_.back()) + "/Tooltip");
  return true;
}

void RuntimeGuiContext::EndTooltip() {
  window_paths_.pop_back();
  ImGui::PopClipRect();
  ImGui::EndTooltip();
}

bool RuntimeGuiContext::Owns(const ImGuiWindow* window) const {
  for (auto* candidate = window; candidate; candidate = candidate->ParentWindowInBeginStack)
    if (roots_.find(candidate->ID) != roots_.end())
      return true;
  return false;
}

void RuntimeGuiContext::FinishView() {
  if (active_frame_ != ImGui::GetFrameCount())
    return;
  for (auto& [owner, layouts] : layouts_)
    for (auto& [path, layout] : layouts)
      if (layout.window && !layout.apply && layout.window->LastFrameActive == ImGui::GetFrameCount()) {
        layout.position = {layout.window->Pos.x - ImTrunc(origin_.x), layout.window->Pos.y - ImTrunc(origin_.y)};
        layout.size = layout.window->SizeFull;
        layout.collapsed = layout.window->Collapsed;
      }
  const ImRect bounds(origin_, {origin_.x + size_.x, origin_.y + size_.y});
  for (auto* window : GImGui->Windows) {
    if (!window->Active || !Owns(window))
      continue;
    owned_.insert(window);
    window->OuterRectClipped.ClipWith(bounds);
    ClipCommands(*window->DrawList, bounds);
  }
}

void RuntimeGuiContext::PartitionDrawData() {
  draw_data_.Clear();
  if (active_frame_ != ImGui::GetFrameCount() || size_.x <= 0 || size_.y <= 0)
    return;
  draw_data_.Valid = true;
  draw_data_.DisplayPos = origin_;
  draw_data_.DisplaySize = size_;
  draw_data_.FramebufferScale = ImGui::GetIO().DisplayFramebufferScale;
  draw_data_.OwnerViewport = viewport_;
  draw_data_.Textures = &ImGui::GetPlatformIO().Textures;
  const auto append_camera_list = [&](const int layer) {
    if (camera_draw_frames_[layer] == active_frame_) {
      auto& list = *camera_draw_lists_[layer];
      ClipCommands(list, ImRect(origin_, {origin_.x + size_.x, origin_.y + size_.y}));
      draw_data_.AddDrawList(&list);
    }
  };
  append_camera_list(0);
  for (auto* viewport : ImGui::GetPlatformIO().Viewports) {
    auto* data = viewport->DrawData;
    if (!data)
      continue;
    for (int index = 0; index < data->CmdLists.Size;) {
      auto* list = data->CmdLists[index];
      const bool owned = std::any_of(owned_.begin(), owned_.end(), [list, this](const auto* window) {
        return window->DrawList == list && Owns(window);
      });
      if (!owned) {
        ++index;
        continue;
      }
      // Render() may append modal dimming after FinishView().
      ClipCommands(*list, ImRect(origin_, {origin_.x + size_.x, origin_.y + size_.y}));
      draw_data_.AddDrawList(list);
      data->TotalVtxCount -= list->VtxBuffer.Size;
      data->TotalIdxCount -= list->IdxBuffer.Size;
      data->CmdLists.erase(data->CmdLists.Data + index);
      data->CmdListsCount = data->CmdLists.Size;
    }
  }
  append_camera_list(1);
}

bool RuntimeGuiContext::CapturesMouse() const {
  return active_frame_ == ImGui::GetFrameCount() &&
         ((ImRect(origin_, {origin_.x + size_.x, origin_.y + size_.y}).Contains(ImGui::GetIO().MousePos) &&
           Owns(GImGui->HoveredWindow)) ||
          Owns(GImGui->ActiveIdWindow) || (ImGui::GetTopMostPopupModal() && Owns(ImGui::GetTopMostPopupModal())));
}

bool RuntimeGuiContext::CapturesKeyboard() const {
  return active_frame_ == ImGui::GetFrameCount() && Owns(GImGui->NavWindow);
}

ImDrawData* RuntimeGuiContext::GetDrawData() {
  return draw_data_.Valid ? &draw_data_ : nullptr;
}

void RuntimeGuiContext::SetOwner(std::string owner, std::string asset) {
  owner_ = std::move(owner);
  asset_ = std::move(asset) + "/";
}

bool RuntimeGuiContext::BeginChild(const char* id, ImVec2 size, const ImGuiChildFlags child_flags,
                                   const ImGuiWindowFlags window_flags) {
  IM_ASSERT(!window_paths_.empty());
  const auto path = window_paths_.back() + "/Child/" + EncodeId(id);
  auto& layout = layouts_[owner_][path];
  layout.child = true;
  if (layout.apply && layout.size.x > 0 && layout.size.y > 0) {
    if (child_flags & ImGuiChildFlags_ResizeX)
      size.x = layout.size.x;
    if (child_flags & ImGuiChildFlags_ResizeY)
      size.y = layout.size.y;
    if (layout.window) {
      auto restored = layout.window->SizeFull;
      if (child_flags & ImGuiChildFlags_ResizeX)
        restored.x = size.x;
      if (child_flags & ImGuiChildFlags_ResizeY)
        restored.y = size.y;
      ImGui::SetWindowSize(layout.window, restored);
    }
  }
  const auto name = owner_ + "/" + path;
  const bool visible =
      ImGui::BeginChild(ImHashStr(name.c_str()), size, child_flags, window_flags | ImGuiWindowFlags_NoSavedSettings);
  layout.window = ImGui::GetCurrentWindow();
  layout.apply = false;
  window_paths_.push_back(path);
  return visible;
}

void RuntimeGuiContext::EndChild() {
  window_paths_.pop_back();
  ImGui::EndChild();
}

void RuntimeGuiContext::LoadLayout(const std::string& owner, const std::string& ini) {
  auto& layouts = layouts_[owner];
  for (auto& [path, layout] : layouts) {
    layout.position = {};
    layout.size = {};
    layout.collapsed = false;
    layout.apply = true;
  }
  std::istringstream input(ini);
  std::string line;
  WindowLayout* current = nullptr;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    if (line.compare(0, 9, "[Window][") == 0 && line.back() == ']')
      current = &layouts[line.substr(9, line.size() - 10)];
    else if (current) {
      int x, y, value;
      if (std::sscanf(line.c_str(), "Pos=%d,%d", &x, &y) == 2)
        current->position = {float(x), float(y)};
      else if (std::sscanf(line.c_str(), "Size=%d,%d", &x, &y) == 2)
        current->size = {float(x), float(y)};
      else if (std::sscanf(line.c_str(), "Collapsed=%d", &value) == 1)
        current->collapsed = value != 0;
      else if (std::sscanf(line.c_str(), "IsChild=%d", &value) == 1)
        current->child = value != 0;
    }
  }
}

std::string RuntimeGuiContext::SaveLayout(const std::string& owner) const {
  const auto found = layouts_.find(owner);
  if (found == layouts_.end())
    return {};
  std::ostringstream out;
  for (const auto& [path, saved] : found->second) {
    auto layout = saved;
    if (layout.window && !layout.apply && layout.window->LastFrameActive == active_frame_) {
      layout.position = {layout.window->Pos.x - ImTrunc(origin_.x), layout.window->Pos.y - ImTrunc(origin_.y)};
      layout.size = layout.window->SizeFull;
      layout.collapsed = layout.window->Collapsed;
    }
    if (layout.size.x <= 0 || layout.size.y <= 0)
      continue;
    out << "[Window][" << path << "]\n";
    if (layout.child)
      out << "IsChild=1\n";
    else
      out << "Pos=" << int(layout.position.x) << ',' << int(layout.position.y) << '\n';
    out << "Size=" << int(layout.size.x) << ',' << int(layout.size.y) << '\n';
    if (!layout.child)
      out << "Collapsed=" << layout.collapsed << '\n';
    out << '\n';
  }
  return out.str();
}
