#pragma once

#include <memory>
#include "RuntimeGuiContext.hpp"

namespace evo_engine {
class RenderTexture;
class GraphicsPipeline;

class EVOENGINE_API RuntimeGuiRenderer {
 protected:
  RuntimeGuiContext context_;

 private:
  std::shared_ptr<RenderTexture> overlay_;
  std::shared_ptr<GraphicsPipeline> compositor_;
  ImTextureID texture_id_ = 0;
  int frame_ = -1;
  static void Composite(const ImDrawList* list, const ImDrawCmd* command);

 public:
  void PrepareFrame() {
    context_.PrepareFrame();
  }
  void BeginView(ImVec2 origin, ImVec2 size);
  RuntimeGuiContext& GetContext() {
    return context_;
  }
  void FinishView() {
    context_.FinishView();
  }
  void RenderOverlay();
  bool CapturesMouse() const {
    return context_.CapturesMouse();
  }
  bool CapturesKeyboard() const {
    return context_.CapturesKeyboard();
  }
  const std::shared_ptr<RenderTexture>& GetOverlay() const {
    return overlay_;
  }
};
}  // namespace evo_engine
