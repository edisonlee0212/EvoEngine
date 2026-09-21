#pragma once
#include "IAsset.hpp"

namespace evo_engine {
class RuntimeGuiContext;

class EVOENGINE_API IRuntimeGui : public IAsset {
 public:
  virtual void OnGui(RuntimeGuiContext& context) = 0;
};
}  // namespace evo_engine
