#pragma once
#include "DynamicTreeSkeleton.hpp"
#include "Tree.hpp"

namespace eco_sys_lab_plugin {
class DynamicStrandsDemo : public IPrivateComponent {
public:
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Update() override;
  void LateUpdate() override;
};

}  // namespace eco_sys_lab_plugin