#pragma once

#include "EvoEngineAPI.hpp"
#include "EvoEngineEditorAPI.hpp"

#include "ILayer.hpp"

namespace evo_engine {
class EVOENGINE_EDITOR_API ImGuiLayer final : public ILayer {
 protected:
  void OnWindowGraphicsInitialized() override;
  void OnDestroy() override;
  void PreUpdate() override;
};
}  // namespace evo_engine
