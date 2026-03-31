#pragma once

#include "ILayer.hpp"

namespace l_system_plugin {

class LSystemLayer : public evo_engine::ILayer {
 public:
  void OnCreate() override;
  void OnDestroy() override;
  void Update() override;
  void OnInspect(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer) override;
};

}  // namespace l_system_plugin
