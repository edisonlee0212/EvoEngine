#pragma once
#include <unordered_map>
#include "ILayer.hpp"
#include "RuntimeGuiRenderer.hpp"
namespace evo_engine {
class Camera;
class RuntimeGui;
class EVOENGINE_API RuntimeGuiLayer final : public ILayer {
  RuntimeGuiRenderer renderer_;
  std::unordered_map<std::string, std::string> loaded_layouts_;
  std::weak_ptr<Scene> source_scene_;
  std::unordered_map<uint64_t, std::weak_ptr<RuntimeGui>> source_components_;
  std::map<std::string, std::string> runtime_overrides_;
  bool overrides_dirty_ = false;
  uint64_t scene_epoch_ = 0;
  void StoreLayout(const std::shared_ptr<RuntimeGui>& component, std::string layout);
  void SaveOverrides();
  void FlushLayouts();

 protected:
  void OnRuntimeStart() override;
  void OnBeforeSceneDetach() override;
  void OnDestroy() override {
    FlushLayouts();
    SaveOverrides();
    renderer_.GetContext().ClearDrawLists();
  }

 public:
  void PrepareFrame() {
    renderer_.PrepareFrame();
  }
  void DrawView(const std::shared_ptr<Camera>& camera, ImVec2 origin, ImVec2 size);
  void RenderOverlay() {
    renderer_.RenderOverlay();
  }
  bool CapturesMouse() const {
    return renderer_.CapturesMouse();
  }
  bool CapturesKeyboard() const {
    return renderer_.CapturesKeyboard();
  }
};
}  // namespace evo_engine
