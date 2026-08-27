#pragma once

#include "ILayer.hpp"

namespace evo_engine {
class EVOENGINE_API ImGuiLayer final : public ILayer {
 protected:
  void OnDestroy() override;
  void PreUpdate() override;
};
}  // namespace evo_engine
