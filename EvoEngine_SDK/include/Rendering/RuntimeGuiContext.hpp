#pragma once

#include <imgui.h>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>
#include "EvoEngineAPI.hpp"

struct ImGuiWindow;

namespace evo_engine {
class RuntimeGui;
class Scene;
class Camera;
class EVOENGINE_API RuntimeGuiContext {
  struct WindowLayout {
    ImVec2 position{}, size{};
    bool collapsed = false;
    bool child = false;
    bool apply = true;
    ImGuiWindow* window = nullptr;
  };
  std::map<std::string, std::map<std::string, WindowLayout>> layouts_;
  std::vector<std::string> window_paths_;
  std::string asset_;
  std::weak_ptr<RuntimeGui> component_;
  std::weak_ptr<Scene> scene_;
  std::weak_ptr<Camera> camera_;
  ImVec2 origin_{};
  ImVec2 size_{};
  ImVec2 previous_origin_{};
  ImGuiWindow* host_ = nullptr;
  ImGuiViewport* viewport_ = nullptr;
  std::unordered_set<ImGuiID> roots_;
  std::unordered_set<ImGuiWindow*> owned_;
  std::unordered_set<ImGuiWindow*> suppressed_mouse_;
  std::string owner_;
  ImDrawData draw_data_{};
  std::unique_ptr<ImDrawList> camera_draw_lists_[2];
  int camera_draw_frames_[2] = {-1, -1};
  ImDrawList* GetCameraDrawList(int layer);
  int active_frame_ = -1;
  std::string PopupName(const char* name) const;
  void PrepareTransient();
  void ConstrainTransient();

 public:
  ImDrawList* GetBackgroundDrawList() {
    return GetCameraDrawList(0);
  }
  ImDrawList* GetForegroundDrawList() {
    return GetCameraDrawList(1);
  }
  // Release before destroying the shared ImGui context.
  void ClearDrawLists() {
    for (auto& list : camera_draw_lists_)
      list.reset();
    camera_draw_frames_[0] = camera_draw_frames_[1] = -1;
  }
  void PrepareFrame();
  void BeginView(ImVec2 origin, ImVec2 size, uint64_t owner);
  bool BeginWindow(const char* title, bool* open = nullptr, ImGuiWindowFlags flags = 0);
  void EndWindow();
  // Child IDs are stable within the enclosing helper window/child, independent of PushID.
  bool BeginChild(const char* id, ImVec2 size = {}, ImGuiChildFlags child_flags = 0, ImGuiWindowFlags window_flags = 0);
  void EndChild();
  void SetOwner(std::string owner, std::string asset);
  void SetInvocation(const std::shared_ptr<RuntimeGui>& component, const std::shared_ptr<Scene>& scene,
                     const std::shared_ptr<Camera>& camera) {
    component_ = component;
    scene_ = scene;
    camera_ = camera;
  }
  std::shared_ptr<RuntimeGui> GetComponent() const {
    return component_.lock();
  }
  std::shared_ptr<Scene> GetScene() const {
    return scene_.lock();
  }
  std::shared_ptr<Camera> GetCamera() const {
    return camera_.lock();
  }
  void ResetLayouts() {
    layouts_.clear();
  }
  void LoadLayout(const std::string& owner, const std::string& ini);
  std::string SaveLayout(const std::string& owner) const;
  bool BeginMenu(const char* label, bool enabled = true);
  void EndMenu();
  void OpenPopup(const char* name, ImGuiPopupFlags flags = 0);
  bool BeginPopup(const char* id, ImGuiWindowFlags flags = 0);
  bool BeginPopupModal(const char* name, bool* open = nullptr, ImGuiWindowFlags flags = 0);
  void EndPopup();
  bool BeginTooltip();
  void EndTooltip();
  void FinishView();
  // Called after the single shared ImGui::Render(), before either GPU submission.
  void PartitionDrawData();
  bool Owns(const ImGuiWindow* window) const;
  bool CapturesMouse() const;
  bool CapturesKeyboard() const;
  ImDrawData* GetDrawData();
  ImVec2 GetOrigin() const {
    return origin_;
  }
  ImVec2 GetSize() const {
    return size_;
  }
};
}  // namespace evo_engine
