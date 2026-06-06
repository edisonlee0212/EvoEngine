#pragma once

#include "ILayer.hpp"

namespace evo_engine {
class ImGuiLayer final : public ILayer {
 protected:
  void OnDestroy() override;
  void PreUpdate() override;
};
}  // namespace evo_engine
