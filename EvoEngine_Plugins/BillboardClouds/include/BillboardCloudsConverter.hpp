#pragma once

#include "BillboardCloud.hpp"
using namespace evo_engine;

namespace billboard_clouds_plugin {
class BillboardCloudsConverter : public IPrivateComponent {
 public:
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
}  // namespace eco_sys_lab_plugin