#pragma once
#include "RuntimeGuiRenderer.hpp"
namespace evo_engine {
class EVOENGINE_API RuntimeGuiProof : public RuntimeGuiRenderer {
  float value_ = 0.5f;
  char text_[64] = "Shared context";

 public:
  void Draw(ImVec2 origin, ImVec2 size);
};
}  // namespace evo_engine
