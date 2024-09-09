#pragma once

#define GPR_READING true
#include "gpr.h"

namespace gpr_plugin {
  class Gpr : public evo_engine::IAsset {
  protected:
    bool SaveInternal(const std::filesystem::path& path) const override;
    bool LoadInternal(const std::filesystem::path& path) override;

  public:
    void OnCreate() override;
    bool OnInspect(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer) override;
  };
}
