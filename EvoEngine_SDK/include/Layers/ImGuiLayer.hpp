#pragma once

#include "EvoEngineAPI.hpp"

#include <memory>
#include "ILayer.hpp"

namespace evo_engine {
class RuntimeGuiProof;
class EVOENGINE_API ImGuiLayer final : public ILayer {
  std::shared_ptr<RuntimeGuiProof> runtime_gui_proof_;
  void DrawRuntimeView();
  std::vector<Input::InputEvent> gameplay_events_;

 public:
  const std::shared_ptr<RuntimeGuiProof>& GetRuntimeGuiProof() const {
    return runtime_gui_proof_;
  }

 protected:
  void OnWindowGraphicsInitialized() override;
  void OnDestroy() override;
  void PreUpdate() override;
  bool OnInputEvent(const Input::InputEvent& event) override;
};
}  // namespace evo_engine
